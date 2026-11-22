#include "nozzle_guard_ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "httpd.h"
#include "hub_client.h"
#include "i18n.h"
#include "nvs.h"
#include "plastics.h"
#include "url_util.h"

static const char *TAG = "nozzle";

#define NVS_NS "nozzle"

/* Standby by plastic name, for plastics.c's table; 0 = not guarded.  PLA chars
 * slowly at its temperature and is left alone by default. */
static const struct { const char *name; int16_t c; } DEFAULT_STANDBY[] = {
    { "PETG", 160 }, { "ABS", 160 }, { "Nylon", 170 },
};

static ng_config_t s_cfg;
static SemaphoreHandle_t s_lock;
static int s_abs_idx = -1;

static void defaults(ng_config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->enabled = true;
    c->purge_every_s = 120;
    c->purge_mm = 5;
    c->purge_for_min = 15;
    c->mid_after_min = 10;
    for (int i = 0; i < PLASTIC_COUNT && i < NG_MAX_PLASTICS; i++) {
        for (size_t k = 0; k < sizeof(DEFAULT_STANDBY) / sizeof(DEFAULT_STANDBY[0]); k++) {
            if (strcmp(PLASTICS[i].name, DEFAULT_STANDBY[k].name) == 0) c->standby_c[i] = DEFAULT_STANDBY[k].c;
        }
    }
}

/* NVS keys are at most 15 characters: "sb_" and the plastic's name */
static void standby_key(char *key, size_t size, int i)
{
    snprintf(key, size, "sb_%.11s", PLASTICS[i].name);
}

static void clamp(ng_config_t *c)
{
    if (c->purge_every_s < 30) c->purge_every_s = 30;
    if (c->purge_every_s > 3600) c->purge_every_s = 3600;
    if (c->purge_mm > 30) c->purge_mm = 30;
    if (c->purge_for_min > 240) c->purge_for_min = 240;
    if (c->mid_after_min > 240) c->mid_after_min = 240;
    for (int i = 0; i < NG_MAX_PLASTICS; i++) {
        if (c->standby_c[i] < 0) c->standby_c[i] = 0;
        if (c->standby_c[i] > 250) c->standby_c[i] = 250;
    }
}

void nozzle_guard_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    for (int i = 0; i < PLASTIC_COUNT; i++) {
        if (strcmp(PLASTICS[i].name, "ABS") == 0) s_abs_idx = i;
    }
    defaults(&s_cfg);
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t u8;
        uint16_t u16;
        int16_t i16;
        if (nvs_get_u8(nvs, "en", &u8) == ESP_OK) s_cfg.enabled = u8 != 0;
        if (nvs_get_u16(nvs, "every_s", &u16) == ESP_OK) s_cfg.purge_every_s = u16;
        if (nvs_get_u8(nvs, "mm", &u8) == ESP_OK) s_cfg.purge_mm = u8;
        if (nvs_get_u16(nvs, "for_min", &u16) == ESP_OK) s_cfg.purge_for_min = u16;
        if (nvs_get_u16(nvs, "mid_min", &u16) == ESP_OK) s_cfg.mid_after_min = u16;
        for (int i = 0; i < PLASTIC_COUNT && i < NG_MAX_PLASTICS; i++) {
            char key[16];
            standby_key(key, sizeof(key), i);
            if (nvs_get_i16(nvs, key, &i16) == ESP_OK) s_cfg.standby_c[i] = i16;
        }
        nvs_close(nvs);
    }
    clamp(&s_cfg);
}

void ng_config_get(ng_config_t *out)
{
    if (!s_lock) {     /* Before nozzle_guard_init(): guard off */
        memset(out, 0, sizeof(*out));
        return;
    }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_cfg;
    xSemaphoreGive(s_lock);
}

int ng_standby_c(int target_c)
{
    ng_config_t c;
    ng_config_get(&c);
    return ng_standby_for(&c, plastic_loaded(), s_abs_idx, target_c);
}

void ng_notify_standby(int from_c, int to_c, const char *filename)
{
    char msg[48];
    snprintf(msg, sizeof(msg), "%d° → %d°", from_c, to_c);
    ESP_LOGI(TAG, "Wait: nozzle %s", msg);
    hub_client_send_event(HUB_EVENT_NOZZLE_STANDBY, msg, filename ? filename : "");
}

void ng_notify_runout(int target_c, int standby_c, const char *filename)
{
    char msg[48] = "";
    if (standby_c > 0) {
        ng_config_t c;
        ng_config_get(&c);
        snprintf(msg, sizeof(msg), "%d° → %d°, %u min", target_c, standby_c, c.mid_after_min);
    }
    ESP_LOGW(TAG, "Filament ran out: %s", msg);
    hub_client_send_event(HUB_EVENT_FILAMENT_RUNOUT, msg, filename ? filename : "");
}

static void save(const ng_config_t *c)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cfg = *c;
    clamp(&s_cfg);
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, "en", s_cfg.enabled);
        nvs_set_u16(nvs, "every_s", s_cfg.purge_every_s);
        nvs_set_u8(nvs, "mm", s_cfg.purge_mm);
        nvs_set_u16(nvs, "for_min", s_cfg.purge_for_min);
        nvs_set_u16(nvs, "mid_min", s_cfg.mid_after_min);
        for (int i = 0; i < PLASTIC_COUNT && i < NG_MAX_PLASTICS; i++) {
            char key[16];
            standby_key(key, sizeof(key), i);
            nvs_set_i16(nvs, key, s_cfg.standby_c[i]);
        }
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Nozzle guard %s: purge %u mm every %u s for %u min, standby after %u min in a print",
             s_cfg.enabled ? "on" : "off", s_cfg.purge_mm, s_cfg.purge_every_s,
             s_cfg.purge_for_min, s_cfg.mid_after_min);
    xSemaphoreGive(s_lock);
}

/* ---- Settings page ---- */

void nozzle_guard_render_settings(html_buf_t *p)
{
    ng_config_t c;
    ng_config_get(&c);
    html_buf_printf(p,
        i18n_page(
        "<hr><div class='sh'><h2>" N_("Nozzle guard in a wait") "</h2>"
        "<button type='submit' form='f-nozzle'>" N_("Save") "</button></div>"
        "<form id='f-nozzle' method='POST' action='/nozzle/config'>"
        "<label><input type='checkbox' name='en' value='1'%s> " N_("On") "</label>"
        "<table style='margin-top:8px'>"
        "<tr><td>" N_("Before the first layer: push out") "</td><td><input type='number' name='mm' min='0' max='30' "
        "value='%u' style='width:60px'> " N_("mm every") " <input type='number' name='every' min='0.5' max='60' step='0.5' "
        "value='%g' style='width:60px'> " N_("min") "</td></tr>"
        "<tr><td>" N_("...for, then standby") "</td><td><input type='number' name='for' min='0' max='240' "
        "value='%u' style='width:60px'> " N_("min") "</td></tr>"
        "<tr><td>" N_("During the print: standby after") "</td><td><input type='number' name='mid' min='0' max='240' "
        "value='%u' style='width:60px'> " N_("min") "</td></tr>"),
        c.enabled ? " checked" : "", c.purge_mm, c.purge_every_s / 60.0,
        c.purge_for_min, c.mid_after_min);
    for (int i = 0; i < PLASTIC_COUNT && i < NG_MAX_PLASTICS; i++) {
        html_buf_printf(p,
            "<tr><td>%s %s</td><td><input type='number' name='sb%d' min='0' max='250' value='%d' "
            "style='width:60px'> °C</td></tr>",
            _("Standby"), PLASTICS[i].name, i, c.standby_c[i]);
    }
    html_buf_printf(p, "%s",
        i18n_page(
        "</table></form>"
        "<p class='hint'>" N_("A plastic left hot in a waiting nozzle (ABS above all) turns to char. While the print "
        "waits for you before its first layer, the board pushes plastic out, then takes the nozzle down to the "
        "standby temperature; during the print (the nozzle is over the part) only the standby. A pause from "
        "Obico goes to standby at once. Continue heats back first. Standby 0: that plastic is not guarded; "
        "an unknown plastic printed at 230 °C or more counts as ABS. Pauses made on the printer are left "
        "alone.") "</p>"));
}

static esp_err_t nozzle_config_handler(httpd_req_t *req)
{
    char buf[400];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[len] = '\0';
    ng_config_t c;
    ng_config_get(&c);
    char en[4] = "";
    c.enabled = form_str(buf, "en", en, sizeof(en)) && en[0] == '1';
    int v;
    if (form_int(buf, "mm", &v) && v >= 0) c.purge_mm = (uint8_t)(v > 30 ? 30 : v);
    char every[12];
    if (form_str(buf, "every", every, sizeof(every))) {
        float m = strtof(every, NULL);
        if (m > 0) c.purge_every_s = (uint16_t)(m * 60 + 0.5f);
    }
    if (form_int(buf, "for", &v) && v >= 0) c.purge_for_min = (uint16_t)v;
    if (form_int(buf, "mid", &v) && v >= 0) c.mid_after_min = (uint16_t)v;
    for (int i = 0; i < PLASTIC_COUNT && i < NG_MAX_PLASTICS; i++) {
        char name[8];
        snprintf(name, sizeof(name), "sb%d", i);
        if (form_int(buf, name, &v)) c.standby_c[i] = (int16_t)v;
    }
    save(&c);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- Copied settings: section "nozzle" ---- */

void nozzle_guard_settings_export(cJSON *o)
{
    ng_config_t c;
    ng_config_get(&c);
    cJSON_AddBoolToObject(o, "enabled", c.enabled);
    cJSON_AddNumberToObject(o, "purge_every_s", c.purge_every_s);
    cJSON_AddNumberToObject(o, "purge_mm", c.purge_mm);
    cJSON_AddNumberToObject(o, "purge_for_min", c.purge_for_min);
    cJSON_AddNumberToObject(o, "mid_after_min", c.mid_after_min);
    cJSON *sb = cJSON_AddObjectToObject(o, "standby_c");
    for (int i = 0; sb && i < PLASTIC_COUNT && i < NG_MAX_PLASTICS; i++)
        cJSON_AddNumberToObject(sb, PLASTICS[i].name, c.standby_c[i]);
}

void nozzle_guard_settings_import(const cJSON *o)
{
    ng_config_t c;
    ng_config_get(&c);
    const cJSON *it;
    if (cJSON_IsBool(it = cJSON_GetObjectItem(o, "enabled"))) c.enabled = cJSON_IsTrue(it);
    if (cJSON_IsNumber(it = cJSON_GetObjectItem(o, "purge_every_s"))) c.purge_every_s = (uint16_t)it->valueint;
    if (cJSON_IsNumber(it = cJSON_GetObjectItem(o, "purge_mm"))) c.purge_mm = (uint8_t)it->valueint;
    if (cJSON_IsNumber(it = cJSON_GetObjectItem(o, "purge_for_min"))) c.purge_for_min = (uint16_t)it->valueint;
    if (cJSON_IsNumber(it = cJSON_GetObjectItem(o, "mid_after_min"))) c.mid_after_min = (uint16_t)it->valueint;
    const cJSON *sb = cJSON_GetObjectItem(o, "standby_c");
    for (int i = 0; cJSON_IsObject(sb) && i < PLASTIC_COUNT && i < NG_MAX_PLASTICS; i++) {
        if (cJSON_IsNumber(it = cJSON_GetObjectItem(sb, PLASTICS[i].name))) c.standby_c[i] = (int16_t)it->valueint;
    }
    save(&c);
}

esp_err_t nozzle_guard_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uri = { .uri = "/nozzle/config", .method = HTTP_POST, .handler = nozzle_config_handler };
    HTTPD_REGISTER(server, &uri);
    return ESP_OK;
}
