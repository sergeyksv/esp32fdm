#include "printer_comm_klipper.h"

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>

static const char *TAG = "klipper";

#define POLL_INTERVAL_MS    4000
#define HTTP_TIMEOUT_MS     5000
#define HTTP_BUF_SIZE       2048

/* Moonraker connection info */
static char s_host[64];
static uint16_t s_port;

/* State */
static printer_state_t s_state;
static SemaphoreHandle_t s_state_mutex;
static TaskHandle_t s_task_handle;

/* Response buffer for HTTP reads */
static char s_resp_buf[HTTP_BUF_SIZE];

/* ---- HTTP helpers ---- */

/**
 * Perform a GET request and read the response body into s_resp_buf.
 * Returns the HTTP status code, or -1 on connection failure.
 */
static int http_get(const char *path)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%u%s", s_host, s_port, path);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = HTTP_TIMEOUT_MS,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return -1;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "GET %s open failed: %s", path, esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return -1;
    }

    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);

    int total = 0;
    while (total < (int)sizeof(s_resp_buf) - 1) {
        int r = esp_http_client_read(client, s_resp_buf + total,
                                     sizeof(s_resp_buf) - 1 - total);
        if (r <= 0) break;
        total += r;
    }
    s_resp_buf[total] = '\0';

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return status;
}

/**
 * Perform a POST request with no body.
 * Returns the HTTP status code, or -1 on connection failure.
 */
static int http_post(const char *path)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%u%s", s_host, s_port, path);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = HTTP_TIMEOUT_MS,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return -1;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGD(TAG, "POST %s open failed: %s", path, esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return -1;
    }

    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return status;
}

/* ---- JSON parsing ---- */

static cJSON *json_get_nested(cJSON *root, const char *key1, const char *key2)
{
    cJSON *obj = cJSON_GetObjectItem(root, key1);
    if (!obj) return NULL;
    return cJSON_GetObjectItem(obj, key2);
}

static float json_get_float(cJSON *root, const char *key1, const char *key2,
                            float fallback)
{
    cJSON *item = json_get_nested(root, key1, key2);
    if (!item || !cJSON_IsNumber(item)) return fallback;
    return (float)item->valuedouble;
}

static void parse_moonraker_status(const char *json_str)
{
    cJSON *root = cJSON_Parse(json_str);
    if (!root) {
        ESP_LOGW(TAG, "Malformed JSON from Moonraker");
        return;
    }

    cJSON *result = cJSON_GetObjectItem(root, "result");
    cJSON *status = result ? cJSON_GetObjectItem(result, "status") : NULL;
    if (!status) {
        ESP_LOGW(TAG, "No 'result.status' in Moonraker response");
        cJSON_Delete(root);
        return;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);

    /* Temperatures */
    s_state.hotend_actual = json_get_float(status, "extruder", "temperature", s_state.hotend_actual);
    s_state.hotend_target = json_get_float(status, "extruder", "target", s_state.hotend_target);
    s_state.bed_actual = json_get_float(status, "heater_bed", "temperature", s_state.bed_actual);
    s_state.bed_target = json_get_float(status, "heater_bed", "target", s_state.bed_target);

    /* Print state */
    cJSON *ps = cJSON_GetObjectItem(status, "print_stats");
    if (ps) {
        cJSON *st = cJSON_GetObjectItem(ps, "state");
        if (st && cJSON_IsString(st)) {
            const char *state_str = st->valuestring;
            if (strcmp(state_str, "printing") == 0) {
                s_state.opstate = PRINTER_PRINTING;
            } else if (strcmp(state_str, "paused") == 0) {
                s_state.opstate = PRINTER_PAUSED;
            } else if (strcmp(state_str, "error") == 0) {
                s_state.opstate = PRINTER_ERROR;
            } else {
                /* "standby", "complete", etc. */
                s_state.opstate = PRINTER_OPERATIONAL;
            }
        }

        cJSON *fn = cJSON_GetObjectItem(ps, "filename");
        if (fn && cJSON_IsString(fn)) {
            strncpy(s_state.filename, fn->valuestring, sizeof(s_state.filename) - 1);
            s_state.filename[sizeof(s_state.filename) - 1] = '\0';
        }

        cJSON *dur = cJSON_GetObjectItem(ps, "total_duration");
        if (dur && cJSON_IsNumber(dur)) {
            s_state.print_time_s = (int32_t)dur->valuedouble;
        }
    }

    /* Progress from virtual_sdcard */
    cJSON *vsd = cJSON_GetObjectItem(status, "virtual_sdcard");
    if (vsd) {
        cJSON *prog = cJSON_GetObjectItem(vsd, "progress");
        if (prog && cJSON_IsNumber(prog)) {
            s_state.progress_pct = (float)(prog->valuedouble * 100.0);
        }
    }

    /* Position from toolhead */
    cJSON *th = cJSON_GetObjectItem(status, "toolhead");
    if (th) {
        cJSON *pos = cJSON_GetObjectItem(th, "position");
        if (pos && cJSON_IsArray(pos) && cJSON_GetArraySize(pos) >= 3) {
            s_state.x = (float)cJSON_GetArrayItem(pos, 0)->valuedouble;
            s_state.y = (float)cJSON_GetArrayItem(pos, 1)->valuedouble;
            s_state.z = (float)cJSON_GetArrayItem(pos, 2)->valuedouble;
        }
    }

    /* Estimate time left from progress and elapsed */
    if (s_state.progress_pct > 0.1f && s_state.print_time_s > 0) {
        float total_est = (float)s_state.print_time_s / (s_state.progress_pct / 100.0f);
        s_state.print_time_left_s = (int32_t)(total_est - s_state.print_time_s);
        if (s_state.print_time_left_s < 0) s_state.print_time_left_s = 0;
    } else if (s_state.opstate != PRINTER_PRINTING && s_state.opstate != PRINTER_PAUSED) {
        s_state.progress_pct = -1;
        s_state.print_time_s = -1;
        s_state.print_time_left_s = -1;
    }

    s_state.last_update_us = esp_timer_get_time();

    xSemaphoreGive(s_state_mutex);
    cJSON_Delete(root);
}

/* ---- Polling task ---- */

static void klipper_poll_task(void *arg)
{
    ESP_LOGI(TAG, "Klipper polling task started → %s:%u", s_host, s_port);

    static const char *query_path =
        "/printer/objects/query?"
        "extruder&heater_bed&print_stats&virtual_sdcard&toolhead";

    while (true) {
        int status = http_get(query_path);

        if (status < 0) {
            /* Moonraker unreachable */
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.opstate = PRINTER_DISCONNECTED;
            xSemaphoreGive(s_state_mutex);
            ESP_LOGD(TAG, "Moonraker unreachable");
        } else if (status == 503) {
            /* Klipper MCU not connected */
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.opstate = PRINTER_ERROR;
            xSemaphoreGive(s_state_mutex);
            ESP_LOGW(TAG, "Moonraker returned 503 (Klipper MCU not connected?)");
        } else if (status == 200) {
            parse_moonraker_status(s_resp_buf);
        } else {
            ESP_LOGW(TAG, "Moonraker returned HTTP %d", status);
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_INTERVAL_MS));
    }
}

/* ---- Public API ---- */

esp_err_t klipper_backend_start(const char *host, uint16_t port)
{
    strncpy(s_host, host, sizeof(s_host) - 1);
    s_host[sizeof(s_host) - 1] = '\0';
    s_port = port;

    s_state_mutex = xSemaphoreCreateMutex();
    if (!s_state_mutex) return ESP_ERR_NO_MEM;

    memset(&s_state, 0, sizeof(s_state));
    s_state.opstate = PRINTER_DISCONNECTED;
    s_state.progress_pct = -1;
    s_state.print_time_s = -1;
    s_state.print_time_left_s = -1;

    xTaskCreatePinnedToCore(klipper_poll_task, "klipper_poll", 6144,
                            NULL, 8, &s_task_handle, 0);

    ESP_LOGI(TAG, "Klipper/Moonraker backend started → %s:%u", s_host, s_port);
    return ESP_OK;
}

void klipper_backend_stop(void)
{
    if (s_task_handle) {
        vTaskDelete(s_task_handle);
        s_task_handle = NULL;
    }
    if (s_state_mutex) {
        vSemaphoreDelete(s_state_mutex);
        s_state_mutex = NULL;
    }
}

void klipper_backend_get_state(printer_state_t *out)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    memcpy(out, &s_state, sizeof(printer_state_t));
    xSemaphoreGive(s_state_mutex);
}

esp_err_t klipper_backend_send_cmd(const printer_cmd_t *cmd)
{
    int status;

    switch (cmd->type) {
    case PCMD_PAUSE:
        status = http_post("/printer/print/pause");
        break;
    case PCMD_RESUME:
        status = http_post("/printer/print/resume");
        break;
    case PCMD_CANCEL:
        status = http_post("/printer/print/cancel");
        break;
    case PCMD_RAW: {
        /* URL-encode the gcode for query parameter */
        char path[256];
        char encoded[192];
        const char *src = cmd->gcode;
        char *dst = encoded;
        char *end = encoded + sizeof(encoded) - 4;

        while (*src && dst < end) {
            char c = *src++;
            if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
                *dst++ = c;
            } else {
                dst += snprintf(dst, end - dst + 1, "%%%02X", (unsigned char)c);
            }
        }
        *dst = '\0';

        snprintf(path, sizeof(path), "/printer/gcode/script?script=%s", encoded);
        status = http_post(path);
        break;
    }
    default:
        return ESP_ERR_INVALID_ARG;
    }

    if (status < 0) {
        ESP_LOGW(TAG, "Command failed: Moonraker unreachable");
        return ESP_ERR_NOT_FOUND;
    }
    if (status != 200) {
        ESP_LOGW(TAG, "Command returned HTTP %d", status);
        return ESP_FAIL;
    }

    return ESP_OK;
}
