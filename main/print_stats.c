#include "print_stats.h"
#include "task_util.h"

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "httpd.h"
#include "i18n.h"
#include "print_checks.h"
#include "nvs.h"
#include "printer_comm.h"
#include "printer_comm_klipper.h"
#include "maintenance.h"
#include "url_util.h"

static const char *TAG = "stats";

#define NVS_NAMESPACE   "stats"
#define NVS_KEY         "v1"
#define NVS_KEY_SEEDED  "mr_seeded"   /* Taken from Moonraker's history once */
#define SAMPLE_MS       5000
#define SAVE_EVERY_S    600         /* While printing: a power cut loses at most this */
#define MAX_STEP_US     30000000LL  /* A stalled task must not count a jump as printing */
#define VALID_TIME      1700000000  /* Before this, SNTP has not set the clock yet */

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static print_stats_t s_stats;

static void load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    print_stats_t st;
    size_t len = sizeof(st);
    if (nvs_get_blob(nvs, NVS_KEY, &st, &len) == ESP_OK && len == sizeof(st)) s_stats = st;
    nvs_close(nvs);
}

static void save(void)
{
    print_stats_t st;
    taskENTER_CRITICAL(&s_mux);
    st = s_stats;
    taskEXIT_CRITICAL(&s_mux);

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    esp_err_t err = nvs_set_blob(nvs, NVS_KEY, &st, sizeof(st));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err != ESP_OK) ESP_LOGE(TAG, "Save failed: %s", esp_err_to_name(err));
}

void print_stats_get(print_stats_t *out)
{
    taskENTER_CRITICAL(&s_mux);
    *out = s_stats;
    taskEXIT_CRITICAL(&s_mux);
}

/**
 * A Klipper printer keeps its own job history in Moonraker: the first time the
 * board sees it, it takes the hours and the jobs from there, so a printer that
 * has printed for years does not start from zero.  Maintenance keeps its dates.
 */
static void seed_from_moonraker(void)
{
    static bool s_done;
    uint32_t hist_s, hist_jobs;
    if (s_done || printer_comm_get_backend() != PRINTER_BACKEND_KLIPPER ||
        !klipper_backend_history_totals(&hist_s, &hist_jobs)) {
        return;
    }
    s_done = true;

    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    uint8_t seeded = 0;
    nvs_get_u8(nvs, NVS_KEY_SEEDED, &seeded);
    if (!seeded) {
        nvs_set_u8(nvs, NVS_KEY_SEEDED, 1);
        nvs_commit(nvs);
    }
    nvs_close(nvs);
    if (seeded) return;

    taskENTER_CRITICAL(&s_mux);
    uint32_t old_s = s_stats.print_s;
    bool more = hist_s > s_stats.print_s;
    if (more) {
        s_stats.print_s = hist_s;
        if (hist_jobs > s_stats.started) s_stats.started = hist_jobs;
    }
    taskEXIT_CRITICAL(&s_mux);
    if (!more) return;

    maintenance_shift_print_s((int64_t)hist_s - old_s);
    save();
    ESP_LOGI(TAG, "Taken from Moonraker's history: %.1f h, %lu prints",
             hist_s / 3600.0f, (unsigned long)hist_jobs);
}

static void print_stats_task(void *arg)
{
    bool active = false;        /* A counted print is running */
    bool host = false;          /* ...and it is a host print */
    float last_progress = 0;
    int64_t carry_us = 0;       /* Counted time short of a whole second */
    int64_t last_us = esp_timer_get_time();
    int64_t saved_us = last_us;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
        int64_t now = esp_timer_get_time();
        int64_t step = now - last_us;
        last_us = now;
        if (step > MAX_STEP_US) step = 0;

        seed_from_moonraker();

        printer_state_t st;
        printer_comm_get_state(&st);
        if (printer_comm_is_simulating()) continue;

        bool running = st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED;
        bool counting = st.opstate == PRINTER_PRINTING && !st.holding;

        if (running && !active && !st.builtin_script) {
            active = true;
            /* Klipper says how a print ended, as a host print does */
            host = st.host_printing || printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER;
            last_progress = 0;
            taskENTER_CRITICAL(&s_mux);
            s_stats.started++;
            if (!s_stats.since && time(NULL) > VALID_TIME) s_stats.since = time(NULL);
            taskEXIT_CRITICAL(&s_mux);
            ESP_LOGI(TAG, "Print started: %s", st.filename);
            maintenance_note_print_start(st.filename);
            print_checks_note_print_start(st.filename);
            save();
            saved_us = now;
            continue;
        }

        if (active && running) {
            if (st.progress_pct >= 0) last_progress = st.progress_pct;
            if (counting) {
                carry_us += step;
                uint32_t secs = (uint32_t)(carry_us / 1000000);
                carry_us -= (int64_t)secs * 1000000;
                taskENTER_CRITICAL(&s_mux);
                s_stats.print_s += secs;
                taskEXIT_CRITICAL(&s_mux);
            }
            if (now - saved_us >= SAVE_EVERY_S * 1000000LL) {
                save();
                saved_us = now;
            }
            continue;
        }

        if (active && !running) {
            /* A host print says how it ended.  For the printer's own SD print
             * the board only sees it stop, and a print stopped at 99 % is done
             * for all practical purposes. */
            bool done = host ? st.last_host_completed : last_progress >= 99.0f;
            active = false;
            taskENTER_CRITICAL(&s_mux);
            if (done) s_stats.completed++;
            else s_stats.cancelled++;
            taskEXIT_CRITICAL(&s_mux);
            ESP_LOGI(TAG, "Print %s, %.1f h printed in all", done ? "completed" : "cancelled",
                     s_stats.print_s / 3600.0f);
            print_checks_note_print_done(done);
            save();
            saved_us = now;
        }
    }
}

void print_stats_init(void)
{
    load();
    ESP_LOGI(TAG, "%.1f h printed, %lu prints", s_stats.print_s / 3600.0f,
             (unsigned long)s_stats.started);
    /* 4 KB: at a print start it may write the maintenance log on LittleFS */
    if (task_create_psram(print_stats_task, "stats", 8192, NULL, 2, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start");
    }
}

/* ---- Settings ---- */

void print_stats_render_settings(html_buf_t *p)
{
    print_stats_t st;
    print_stats_get(&st);

    char since[32] = "";
    if (st.since) {
        time_t t = (time_t)st.since;
        struct tm tm;
        localtime_r(&t, &tm);
        strftime(since, sizeof(since), "%d.%m.%Y", &tm);
    }

    html_buf_printf(p,
        i18n_page(
        "<hr><div class='sh'><h2>" N_("Print Statistics") "</h2>"
        "<button type='submit' form='f-stats'>" N_("Save") "</button></div>"
        "<form id='f-stats' method='POST' action='/stats/config'><table>"
        "<tr><td>" N_("Print time") "</td><td><input name='hours' type='number' min='0' max='100000' "
        "step='0.1' value='%.1f' style='width:90px'> " N_("h") "</td></tr>"
        "<tr><td>" N_("Prints") "</td><td><input name='prints' type='number' min='0' max='1000000' "
        "value='%lu' style='width:90px'></td></tr>"
        "</table></form>"
        "<p><a href='/maintenance'>" N_("Maintenance") " &rarr;</a></p>"
        "<p class='hint'>%lu " N_("completed") ", %lu " N_("cancelled") "%s%s. "
        N_("Counted while printing, without pauses; cold pull and filament change do not count. "
           "Enter what the printer had before, if you know it.") "</p>"),
        st.print_s / 3600.0, (unsigned long)st.started,
        (unsigned long)st.completed, (unsigned long)st.cancelled,
        since[0] ? " · " : "", since[0] ? since : "");
}

static esp_err_t stats_config_handler(httpd_req_t *req)
{
    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    /* Hours may have a fraction: read them as text */
    char hours_s[16] = "";
    const char *h = strstr(buf, "hours=");
    if (h) url_decode_field(h + 6, hours_s, sizeof(hours_s));
    char *end;
    double hours = strtod(hours_s, &end);
    int prints;
    if (!hours_s[0] || *end != '\0' || hours < 0 || hours > 100000 ||
        !form_int(buf, "prints", &prints) || prints < 0 || prints > 1000000) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, _("Missing or bad field."));
        return ESP_FAIL;
    }

    taskENTER_CRITICAL(&s_mux);
    uint32_t old_s = s_stats.print_s;
    s_stats.print_s = (uint32_t)(hours * 3600 + 0.5);
    /* Keep the counts adding up: prints entered by hand count as completed */
    if ((uint32_t)prints != s_stats.started) {
        if (s_stats.cancelled > (uint32_t)prints) s_stats.cancelled = prints;
        s_stats.started = prints;
        s_stats.completed = prints - s_stats.cancelled;
    }
    taskEXIT_CRITICAL(&s_mux);
    ESP_LOGI(TAG, "Set by user: %.1f h (was %.1f), %d prints", hours, old_s / 3600.0, prints);
    save();
    /* The past entered is not time since the last maintenance */
    maintenance_shift_print_s((int64_t)(uint32_t)(hours * 3600 + 0.5) - old_s);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

esp_err_t print_stats_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uri = { .uri = "/stats/config", .method = HTTP_POST,
                              .handler = stats_config_handler };
    HTTPD_REGISTER(server, &uri);
    return ESP_OK;
}
