#include "board_led.h"
#include "task_util.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "httpd.h"
#include "i18n.h"
#include "nvs.h"
#include "printer_comm.h"
#include "printer_comm_klipper.h"
#include "url_util.h"
#include "cJSON.h"

static const char *TAG = "led";

/* Free on the Freenove ESP32-S3 CAM board: the camera has 4-18, USB 19-20,
 * the SD card 38-40.  Freenove's docs put the single LED on IO2, the board
 * reads IO1, so both are driven. */
#define LED_GPIO_A      1
#define LED_GPIO_B      2
#define RGB_GPIO        48      /* WS2812 */
#define RMT_HZ          10000000    /* 0.1 us ticks */
#define WATCH_MS        2000

static rmt_channel_handle_t s_rmt;
static rmt_encoder_handle_t s_enc;
static bool s_io1, s_io2;
static uint8_t s_rgb[3];
static SemaphoreHandle_t s_lock;    /* The print watcher and /api/led both set them */

/* Which light goes on while a print runs, and the printer's own commands for
 * its light ("" = the backend's default, see default_cmd()) */
#define NVS_NS          "led"
#define CMD_LEN         48
static uint8_t s_mode = LIGHT_BOARD;
static char s_on_cmd[CMD_LEN], s_off_cmd[CMD_LEN];
static SemaphoreHandle_t s_cfg_lock;

static esp_err_t rgb_init(void)
{
    rmt_tx_channel_config_t ch = {
        .gpio_num = RGB_GPIO,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 2,
    };
    esp_err_t err = rmt_new_tx_channel(&ch, &s_rmt);
    if (err != ESP_OK) return err;

    /* WS2812: 0 is 0.3 us high, 0.9 us low; 1 is 0.9 high, 0.3 low */
    rmt_bytes_encoder_config_t enc = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 9, .level1 = 0, .duration1 = 3 },
        .flags.msb_first = 1,
    };
    err = rmt_new_bytes_encoder(&enc, &s_enc);
    if (err != ESP_OK) return err;
    return rmt_enable(s_rmt);
}

static void rgb_send(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_rmt) return;
    uint8_t grb[3] = { g, r, b };
    rmt_transmit_config_t tx = { .loop_count = 0 };
    if (rmt_transmit(s_rmt, s_enc, grb, sizeof(grb), &tx) == ESP_OK) {
        rmt_tx_wait_all_done(s_rmt, 100);
    }
}

static bool is_klipper(void)
{
    return printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER;
}

/** The printer light commands used when none are set: M355 for Marlin (the
 *  Neptune 3 Pro's light bar answers it, M115 says TOGGLE_LIGHTS), FDM_LIGHT for
 *  Klipper (a macro added to the printer's config: Elegoo's FLASHLIGHT_SWITCH
 *  only toggles, so the board could not know the state) */
static const char *default_cmd(bool on)
{
    if (is_klipper()) return on ? "FDM_LIGHT S=1" : "FDM_LIGHT S=0";
    return on ? "M355 S1 P255" : "M355 S0";
}

/** Switch the printer's own light.  With Klipper the default command is sent
 *  only if the printer has the FDM_LIGHT macro: an unknown command is an
 *  error in Klipper's console */
static void printer_light(bool on)
{
    printer_cmd_t cmd = { .type = PCMD_RAW };
    xSemaphoreTake(s_cfg_lock, portMAX_DELAY);
    strlcpy(cmd.gcode, on ? s_on_cmd : s_off_cmd, sizeof(cmd.gcode));
    xSemaphoreGive(s_cfg_lock);
    if (!cmd.gcode[0]) {
        if (is_klipper() && !klipper_backend_has_light()) {
            ESP_LOGW(TAG, "The printer has no FDM_LIGHT macro: its light is not switched");
            return;
        }
        strlcpy(cmd.gcode, default_cmd(on), sizeof(cmd.gcode));
    }
    esp_err_t err = printer_comm_send_cmd(&cmd);
    if (err != ESP_OK) ESP_LOGW(TAG, "Printer light: %s failed (%s)", cmd.gcode, esp_err_to_name(err));
}

/* The light for the camera while a print runs (white plastic shows well): the
 * board's RGB LED, the printer's own light, both or none.  Switched only when
 * a print starts or ends, so /api/led and the printer's screen can override it
 * in between. */
static void print_light_task(void *arg)
{
    bool was_printing = false;
    bool board_lit = false, printer_lit = false;   /* What the start switched on */
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(WATCH_MS));
        printer_state_t st;
        printer_comm_get_state(&st);
        bool printing = st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED;
        if (printing == was_printing) continue;
        was_printing = printing;
        uint8_t mode = s_mode;
        bool board, printer;
        if (printing) {
            /* The mode as it is at the start... */
            board_lit = board = mode == LIGHT_BOARD || mode == LIGHT_BOTH;
            printer_lit = printer = mode == LIGHT_PRINTER || mode == LIGHT_BOTH;
        } else {
            /* ...and the end switches off what the start switched on, even if
             * the mode was changed in between */
            board = board_lit;
            printer = printer_lit;
            board_lit = printer_lit = false;
        }
        ESP_LOGI(TAG, "Print %s: light %s%s%s", printing ? "started" : "ended", printing ? "on" : "off",
                 board ? ", board" : "", printer ? ", printer" : "");
        if (board) {
            uint8_t v = printing ? 255 : 0;
            board_led_set(s_io1, s_io2, v, v, v);
        }
        if (printer) printer_light(printing);
    }
}

static void load_config(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS, NVS_READONLY, &nvs) != ESP_OK) return;
    uint8_t m;
    if (nvs_get_u8(nvs, "mode", &m) == ESP_OK && m <= LIGHT_NONE) s_mode = m;
    size_t len = sizeof(s_on_cmd);
    if (nvs_get_str(nvs, "on", s_on_cmd, &len) != ESP_OK) s_on_cmd[0] = '\0';
    len = sizeof(s_off_cmd);
    if (nvs_get_str(nvs, "off", s_off_cmd, &len) != ESP_OK) s_off_cmd[0] = '\0';
    nvs_close(nvs);
}

/** A command as typed: one line, spaces trimmed */
static void clean_cmd(char *dst, const char *src, size_t size)
{
    while (*src == ' ') src++;
    size_t n = 0;
    for (; *src && *src != '\r' && *src != '\n' && n + 1 < size; src++) dst[n++] = *src;
    while (n && dst[n - 1] == ' ') n--;
    dst[n] = '\0';
}

esp_err_t board_led_set_config(light_mode_t mode, const char *on_cmd, const char *off_cmd)
{
    if ((unsigned)mode > LIGHT_NONE) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_cfg_lock, portMAX_DELAY);
    s_mode = (uint8_t)mode;
    if (on_cmd) clean_cmd(s_on_cmd, on_cmd, sizeof(s_on_cmd));
    if (off_cmd) clean_cmd(s_off_cmd, off_cmd, sizeof(s_off_cmd));
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        nvs_set_u8(nvs, "mode", s_mode);
        nvs_set_str(nvs, "on", s_on_cmd);
        nvs_set_str(nvs, "off", s_off_cmd);
        err = nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Print light: mode %d, on '%s', off '%s'", s_mode, s_on_cmd, s_off_cmd);
    xSemaphoreGive(s_cfg_lock);
    return err;
}

light_mode_t board_led_mode(void)
{
    return (light_mode_t)s_mode;
}

void board_led_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_cfg_lock = xSemaphoreCreateMutex();
    load_config();
    const int pins[] = { LED_GPIO_A, LED_GPIO_B };
    for (int i = 0; i < 2; i++) {
        gpio_reset_pin(pins[i]);
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }
    esp_err_t err = rgb_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RGB LED init failed: %s", esp_err_to_name(err));
        s_rmt = NULL;
    }
    rgb_send(0, 0, 0);
    /* Each log line takes over 1 KB of the caller's stack (logbuf's 512-byte
     * buffer, vsnprintf, the UART), and with 2560 a light switch overflowed it
     * (PANIC on fdm05 and fdm23 right after "Print started/ended"); with Klipper
     * the printer light is an HTTP request made from this task */
    if (task_create_psram(print_light_task, "led", 8192, NULL, 2, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start the print light");
    }
}

void board_led_set(bool io1, bool io2, uint8_t r, uint8_t g, uint8_t b)
{
    if (s_lock) xSemaphoreTake(s_lock, portMAX_DELAY);
    s_io1 = io1;
    s_io2 = io2;
    s_rgb[0] = r;
    s_rgb[1] = g;
    s_rgb[2] = b;
    gpio_set_level(LED_GPIO_A, io1);
    gpio_set_level(LED_GPIO_B, io2);
    rgb_send(r, g, b);
    if (s_lock) xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "IO1 %d, IO2 %d, RGB %02x%02x%02x", io1, io2, r, g, b);
}

/* GET /api/led?on=1, or ?io1=1&io2=0&rgb=ff0000: an experiment, no page yet */
static esp_err_t led_handler(httpd_req_t *req)
{
    char q[64], v[8];
    bool io1 = s_io1, io2 = s_io2;
    uint8_t r = s_rgb[0], g = s_rgb[1], b = s_rgb[2];
    if (httpd_req_get_url_query_str(req, q, sizeof(q)) == ESP_OK) {
        if (httpd_query_key_value(q, "on", v, sizeof(v)) == ESP_OK) {
            bool on = v[0] == '1';
            io1 = io2 = on;
            r = g = b = on ? 255 : 0;
        }
        if (httpd_query_key_value(q, "io1", v, sizeof(v)) == ESP_OK) io1 = v[0] == '1';
        if (httpd_query_key_value(q, "io2", v, sizeof(v)) == ESP_OK) io2 = v[0] == '1';
        if (httpd_query_key_value(q, "rgb", v, sizeof(v)) == ESP_OK && strlen(v) == 6) {
            unsigned long c = strtoul(v, NULL, 16);
            r = c >> 16;
            g = c >> 8;
            b = c;
        }
        board_led_set(io1, io2, r, g, b);
    }

    char json[80];
    snprintf(json, sizeof(json), "{\"io1\":%d,\"io2\":%d,\"rgb\":\"%02x%02x%02x\"}",
             s_io1, s_io2, s_rgb[0], s_rgb[1], s_rgb[2]);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

/* ---- Settings ---- */

static const char *const MODE_NAMES[] = { "board", "printer", "both", "none" };

static int mode_by_name(const char *name)
{
    for (int i = 0; i <= LIGHT_NONE; i++) {
        if (strcmp(name, MODE_NAMES[i]) == 0) return i;
    }
    return -1;
}

/** s with &<>"' escaped, for an attribute value */
static void attr_esc(char *dst, size_t size, const char *s)
{
    size_t n = 0;
    for (; *s && n + 7 < size; s++) {
        const char *ent = *s == '&' ? "&amp;" : *s == '<' ? "&lt;" : *s == '>' ? "&gt;" :
                          *s == '"' ? "&quot;" : *s == '\'' ? "&#39;" : NULL;
        if (ent) {
            n += strlcpy(dst + n, ent, size - n);
        } else {
            dst[n++] = *s;
        }
    }
    dst[n] = '\0';
}

void board_led_render_settings(html_buf_t *p)
{
    char on[CMD_LEN * 6], off[CMD_LEN * 6];
    xSemaphoreTake(s_cfg_lock, portMAX_DELAY);
    uint8_t mode = s_mode;
    attr_esc(on, sizeof(on), s_on_cmd);
    attr_esc(off, sizeof(off), s_off_cmd);
    xSemaphoreGive(s_cfg_lock);

    const char *hint;
    if (!is_klipper()) {
        hint = _("Empty: M355, which the Neptune 3 Pro's light bar answers (M115 lists TOGGLE_LIGHTS).");
    } else if (klipper_backend_has_light()) {
        hint = _("The printer has the FDM_LIGHT macro.");
    } else {
        hint = _("The printer has no FDM_LIGHT macro: add it to the printer's config, or enter its own "
                 "commands. Without either the board does not switch the printer's light.");
    }
    html_buf_printf(p,
        i18n_page(
        "<hr><div class='sh'><h2>" N_("Light during a print") "</h2>"
        "<button type='submit' form='f-light'>" N_("Save") "</button></div>"
        "<form id='f-light' method='POST' action='/led/config'>"
        "<select name='mode'>"
        "<option value='board'%s>" N_("The board's LED") "</option>"
        "<option value='printer'%s>" N_("The printer's light") "</option>"
        "<option value='both'%s>" N_("Both") "</option>"
        "<option value='none'%s>" N_("None") "</option>"
        "</select>"
        "<table style='margin-top:8px'>"
        "<tr><td>" N_("Printer light on") "</td><td><input name='on' value='%s' placeholder='%s' "
        "maxlength='47' style='width:160px'></td></tr>"
        "<tr><td>" N_("Printer light off") "</td><td><input name='off' value='%s' placeholder='%s' "
        "maxlength='47' style='width:160px'></td></tr>"
        "</table></form>"
        "<p class='hint'>" N_("On when a print starts, off when it ends.") " %s</p>"),
        mode == LIGHT_BOARD ? " selected" : "", mode == LIGHT_PRINTER ? " selected" : "",
        mode == LIGHT_BOTH ? " selected" : "", mode == LIGHT_NONE ? " selected" : "",
        on, default_cmd(true), off, default_cmd(false), hint);
}

static esp_err_t led_config_handler(httpd_req_t *req)
{
    char buf[256];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[len] = '\0';
    char mode[12] = "", on[CMD_LEN] = "", off[CMD_LEN] = "";
    form_str(buf, "mode", mode, sizeof(mode));
    form_str(buf, "on", on, sizeof(on));
    form_str(buf, "off", off, sizeof(off));
    int m = mode_by_name(mode);
    if (m < 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Bad mode");
        return ESP_FAIL;
    }
    board_led_set_config((light_mode_t)m, on, off);
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

void board_led_settings_export(cJSON *o)
{
    xSemaphoreTake(s_cfg_lock, portMAX_DELAY);
    cJSON_AddStringToObject(o, "mode", MODE_NAMES[s_mode]);
    cJSON_AddStringToObject(o, "on", s_on_cmd);
    cJSON_AddStringToObject(o, "off", s_off_cmd);
    xSemaphoreGive(s_cfg_lock);
}

void board_led_settings_import(const cJSON *o)
{
    const cJSON *m = cJSON_GetObjectItem(o, "mode");
    const cJSON *on = cJSON_GetObjectItem(o, "on");
    const cJSON *off = cJSON_GetObjectItem(o, "off");
    int mode = cJSON_IsString(m) ? mode_by_name(m->valuestring) : -1;
    board_led_set_config(mode >= 0 ? (light_mode_t)mode : board_led_mode(),
                         cJSON_IsString(on) ? on->valuestring : NULL,
                         cJSON_IsString(off) ? off->valuestring : NULL);
}

esp_err_t board_led_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/api/led",   .method = HTTP_GET,  .handler = led_handler },
        { .uri = "/led/config", .method = HTTP_POST, .handler = led_config_handler },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) HTTPD_REGISTER(server, &uris[i]);
    return ESP_OK;
}
