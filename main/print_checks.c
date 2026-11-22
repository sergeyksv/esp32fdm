#include "print_checks.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "nvs.h"

#include "bedlevel.h"
#include "httpd.h"
#include "hub_client.h"
#include "i18n.h"
#include "maintenance.h"
#include "printer_backend.h"
#include "printer_comm.h"

static const char *TAG = "checks";

#define NVS_NS        "checks"
#define NVS_GOOD_Z    "good_z"      /* Thousandths of a mm */
#define NVS_LANG      "lang"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static bool s_have_good, s_have_cur, s_take_good;
static float s_good_z, s_cur_z;
static bool s_have_busy;            /* An offset read in a print: Z taps answer with it */
static float s_busy_z;
static uint8_t s_lang;              /* Of the last page that asked: for the hub */

static void save_i32(const char *key, int32_t v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i32(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

void print_checks_init(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return;
    int32_t z;
    if (nvs_get_i32(h, NVS_GOOD_Z, &z) == ESP_OK) {
        s_good_z = z / 1000.0f;
        s_have_good = true;
        ESP_LOGI(TAG, "Checked Z offset %.2f", s_good_z);
    }
    int32_t l;
    if (nvs_get_i32(h, NVS_LANG, &l) == ESP_OK && l >= 0 && l < i18n_lang_count()) s_lang = l;
    nvs_close(h);
}

/* ---- Z offset (Marlin) ---- */

void print_checks_z_seen(float z, bool busy)
{
    bool save = false;
    taskENTER_CRITICAL(&s_mux);
    if (busy) {
        s_busy_z = z;
        s_have_busy = true;
        taskEXIT_CRITICAL(&s_mux);
        return;
    }
    s_cur_z = z;
    s_have_cur = true;
    /* The first offset ever seen is all there is to go by.  One set with Z
     * taps in a print, still there after it (also cancelled), was the user's
     * choice. */
    bool tapped = s_have_busy && fabsf(s_busy_z - z) < 0.005f;
    s_have_busy = false;
    if (s_take_good || !s_have_good || tapped) {
        save = !s_have_good || fabsf(s_good_z - z) > 0.0005f;
        s_good_z = z;
        s_have_good = true;
        s_take_good = false;
    }
    taskEXIT_CRITICAL(&s_mux);
    if (save) {
        save_i32(NVS_GOOD_Z, (int32_t)lroundf(z * 1000.0f));
        ESP_LOGI(TAG, "Checked Z offset now %.2f", z);
    }
}

void print_checks_z_confirmed(void)
{
    taskENTER_CRITICAL(&s_mux);
    s_take_good = true;
    taskEXIT_CRITICAL(&s_mux);
    printer_comm_query_probe_offset();
}

/** True with both offsets if the one on the printer is not the checked one */
static bool z_changed(float *cur, float *good)
{
    if (printer_comm_get_backend() != PRINTER_BACKEND_MARLIN) return false;
    taskENTER_CRITICAL(&s_mux);
    bool ch = s_have_cur && s_have_good && !s_take_good && fabsf(s_cur_z - s_good_z) >= PC_Z_EPS;
    *cur = s_cur_z;
    *good = s_good_z;
    taskEXIT_CRITICAL(&s_mux);
    return ch;
}

/* ---- The warnings ---- */

static int append(char *out, size_t size, size_t *n, const char *fmt, ...) __attribute__((format(printf, 4, 5)));
static int append(char *out, size_t size, size_t *n, const char *fmt, ...)
{
    if (*n && *n < size) *n += snprintf(out + *n, size - *n, "\n");
    if (*n < size) {
        va_list ap;
        va_start(ap, fmt);
        *n += vsnprintf(out + *n, size - *n, fmt, ap);
        va_end(ap);
    }
    return 1;
}

/* Built per language with i18n_lookup(): the hub's text is not the request's */
#define TRL(s) i18n_lookup((s), lang)

int print_checks_text(char *out, size_t size, int lang)
{
    size_t n = 0;
    int count = 0;
    out[0] = '\0';
    float cur, good;
    if (z_changed(&cur, &good)) {
        float d = cur - good;
        count += append(out, size, &n,
                        d < 0 ? TRL(NT_("Z offset is %.2f, the checked one %.2f (a bed calibration changes it): the nozzle would go %.2f mm lower, into the bed"))
                              : TRL(NT_("Z offset is %.2f, the checked one %.2f (a bed calibration changes it): the nozzle would go %.2f mm higher, above the bed")),
                        cur, good, fabsf(d));
    }
    bl_print_mesh_t m;
    if (bedlevel_print_mesh_check(&m) && m.off_mm >= PC_MESH_MM) {
        count += append(out, size, &n,
                        TRL(NT_("The printer prints with bed mesh profile %s, which is %.2f mm off the bed's last probe (%dx%d): save the mesh for printing on Bed Leveling")),
                        m.profile, m.off_mm, m.nx, m.ny);
    }
    if (n >= size && size > 4) strcpy(out + size - 4, "...");
    return count;
}

void print_checks_note_print_start(const char *filename)
{
    char text[400];
    if (!print_checks_text(text, sizeof(text), s_lang)) return;
    ESP_LOGW(TAG, "Print %s started without the checks: %s", filename, text);
    hub_client_send_event(HUB_EVENT_PRINT_CHECK, text, filename);
}

void print_checks_note_print_done(bool completed)
{
    /* The first layer was seen and the print went through: that offset is right */
    if (completed && printer_comm_get_backend() == PRINTER_BACKEND_MARLIN) print_checks_z_confirmed();
}

/* ---- HTTP ---- */

static void remember_lang(void)
{
    int lang = i18n_lang();
    if (lang == s_lang || lang < 0 || lang > 255) return;
    s_lang = lang;
    save_i32(NVS_LANG, lang);
}

static esp_err_t send_json(httpd_req_t *req, cJSON *o)
{
    char *json = o ? cJSON_PrintUnformatted(o) : NULL;
    cJSON_Delete(o);
    if (!json) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = httpd_resp_sendstr(req, json);
    cJSON_free(json);
    return ret;
}

/** GET /print/checks: the question before a print.  n and text are the
 *  overdue maintenance (as /maintenance/due), warn the checks, one per line;
 *  z is there when the Z offset differs, for the buttons on /print. */
static esp_err_t checks_handler(httpd_req_t *req)
{
    remember_lang();
    int lang = i18n_lang();
    char maint[200], warn[400];
    int n = maintenance_overdue_text(maint, sizeof(maint), lang);
    print_checks_text(warn, sizeof(warn), lang);
    cJSON *o = cJSON_CreateObject();
    if (o) {
        cJSON_AddNumberToObject(o, "n", n);
        cJSON_AddStringToObject(o, "text", maint);
        cJSON_AddStringToObject(o, "warn", warn);
        float cur, good;
        if (z_changed(&cur, &good)) {
            cJSON *z = cJSON_AddObjectToObject(o, "z");
            /* Two decimals, as Marlin shows them */
            cJSON_AddNumberToObject(z, "now", roundf(cur * 100) / 100);
            cJSON_AddNumberToObject(z, "good", roundf(good * 100) / 100);
        }
    }
    return send_json(req, o);
}

static esp_err_t conflict(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_status(req, "409 Conflict");
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    return httpd_resp_sendstr(req, msg);
}

static esp_err_t raw(const char *gcode)
{
    printer_cmd_t cmd = { .type = PCMD_RAW };
    strlcpy(cmd.gcode, gcode, sizeof(cmd.gcode));
    return printer_comm_send_cmd(&cmd);
}

/** POST /print/zfix a=restore: the checked offset back on the printer
 *  (M851 + M500); a=keep: the printer's offset is the right one */
static esp_err_t zfix_handler(httpd_req_t *req)
{
    char body[32] = "", a[12] = "";
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len > 0) body[len] = '\0';
    httpd_query_key_value(body, "a", a, sizeof(a));

    float cur, good;
    if (!z_changed(&cur, &good)) return httpd_resp_sendstr(req, "OK");
    if (!strcmp(a, "keep")) {
        taskENTER_CRITICAL(&s_mux);
        s_good_z = cur;
        taskEXIT_CRITICAL(&s_mux);
        save_i32(NVS_GOOD_Z, (int32_t)lroundf(cur * 1000.0f));
        ESP_LOGI(TAG, "Z offset %.2f kept as the checked one", cur);
        return httpd_resp_sendstr(req, "OK");
    }
    if (strcmp(a, "restore") != 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "a=restore|keep");

    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate != PRINTER_OPERATIONAL || printer_comm_is_host_printing())
        return conflict(req, _("The printer is not idle"));
    char cmd[24];
    snprintf(cmd, sizeof(cmd), "M851 Z%.2f", good);
    if (raw(cmd) != ESP_OK || raw("M500") != ESP_OK) return conflict(req, _("Not sent, try again"));
    ESP_LOGI(TAG, "Z offset %.2f put back (it was %.2f)", good, cur);
    /* The reading after the commands shows whether the printer took it */
    printer_comm_query_probe_offset();
    return httpd_resp_sendstr(req, "OK");
}

esp_err_t print_checks_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/print/checks", .method = HTTP_GET,  .handler = checks_handler },
        { .uri = "/print/zfix",   .method = HTTP_POST, .handler = zfix_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) HTTPD_REGISTER(server, &uris[i]);
    return ESP_OK;
}
