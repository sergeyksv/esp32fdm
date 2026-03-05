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
#include <sys/stat.h>

static const char *TAG = "klipper";

#define POLL_INTERVAL_MS    4000
#define HTTP_TIMEOUT_MS     5000
#define CMD_TIMEOUT_MS      10000
#define HTTP_BUF_SIZE       2048

/* Moonraker connection info */
static char s_host[64];
static uint16_t s_port;

/* Track last file uploaded to Moonraker so we can delete it when uploading a different one */
static char s_last_uploaded[64];

/* State */
static printer_state_t s_state;
static SemaphoreHandle_t s_state_mutex;
static TaskHandle_t s_task_handle;

/* Serialise all HTTP access (ESP32 lwIP struggles with concurrent connections) */
static SemaphoreHandle_t s_http_mutex;

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
        ESP_LOGW(TAG, "GET %s open failed: %s", path, esp_err_to_name(err));
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
static int http_post(const char *path, int timeout_ms)
{
    char url[256];
    snprintf(url, sizeof(url), "http://%s:%u%s", s_host, s_port, path);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = timeout_ms,
    };

    ESP_LOGI(TAG, "POST %s (timeout %dms)", path, timeout_ms);

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return -1;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "POST %s open failed: %s", path, esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return -1;
    }

    esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);

    ESP_LOGI(TAG, "POST %s → HTTP %d", path, status);

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return status;
}

/* ---- File metadata ---- */

/* Track which file we last fetched metadata for */
static char s_meta_filename[64];

/**
 * Fetch slicer metadata from Moonraker for the given gcode file.
 * Must be called with s_http_mutex held.
 * Populates s_state layer_height, first_layer_height, object_height, total_layers.
 */
static void fetch_file_metadata(const char *filename)
{
    if (!filename || !filename[0]) return;

    /* URL-encode filename for query param */
    char encoded[128];
    const char *src = filename;
    char *dst = encoded;
    char *end = encoded + sizeof(encoded) - 4;
    while (*src && dst < end) {
        char c = *src++;
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~' || c == '/') {
            *dst++ = c;
        } else {
            dst += snprintf(dst, end - dst + 1, "%%%02X", (unsigned char)c);
        }
    }
    *dst = '\0';

    char path[256];
    snprintf(path, sizeof(path), "/server/files/metadata?filename=%s", encoded);

    int status = http_get(path);
    if (status != 200) {
        ESP_LOGW(TAG, "File metadata fetch failed (HTTP %d) for '%s'", status, filename);
        return;
    }

    cJSON *root = cJSON_Parse(s_resp_buf);
    if (!root) {
        ESP_LOGW(TAG, "File metadata: malformed JSON");
        return;
    }

    cJSON *result = cJSON_GetObjectItem(root, "result");
    if (!result) {
        ESP_LOGW(TAG, "File metadata: no 'result'");
        cJSON_Delete(root);
        return;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);

    cJSON *lh = cJSON_GetObjectItem(result, "layer_height");
    if (lh && cJSON_IsNumber(lh)) {
        s_state.layer_height = (float)lh->valuedouble;
    }

    cJSON *flh = cJSON_GetObjectItem(result, "first_layer_height");
    if (flh && cJSON_IsNumber(flh)) {
        s_state.first_layer_height = (float)flh->valuedouble;
    }

    cJSON *oh = cJSON_GetObjectItem(result, "object_height");
    if (oh && cJSON_IsNumber(oh)) {
        s_state.object_height = (float)oh->valuedouble;
    }

    /* Compute total layers */
    if (s_state.layer_height > 0 && s_state.object_height > 0) {
        float first = s_state.first_layer_height > 0 ? s_state.first_layer_height : s_state.layer_height;
        s_state.total_layers = 1 + (int)((s_state.object_height - first) / s_state.layer_height + 0.5f);
    }

    ESP_LOGI(TAG, "File metadata: layer=%.2f first=%.2f height=%.1f totalLayers=%d",
             s_state.layer_height, s_state.first_layer_height,
             s_state.object_height, (int)s_state.total_layers);

    xSemaphoreGive(s_state_mutex);
    cJSON_Delete(root);
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
        ESP_LOGD(TAG, "Raw: %.512s", json_str);
        cJSON_Delete(root);
        return;
    }

    bool need_metadata = false;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);

    /* Temperatures */
    s_state.hotend_actual = json_get_float(status, "extruder", "temperature", s_state.hotend_actual);
    s_state.hotend_target = json_get_float(status, "extruder", "target", s_state.hotend_target);
    s_state.bed_actual = json_get_float(status, "heater_bed", "temperature", s_state.bed_actual);
    s_state.bed_target = json_get_float(status, "heater_bed", "target", s_state.bed_target);

    /* Print state */
    cJSON *ps = cJSON_GetObjectItem(status, "print_stats");
    if (ps) {
        char *ps_str = cJSON_PrintUnformatted(ps);
        if (ps_str) {
            ESP_LOGI(TAG, "print_stats: %.512s", ps_str);
            cJSON_free(ps_str);
        }

        cJSON *st = cJSON_GetObjectItem(ps, "state");
        if (st && cJSON_IsString(st)) {
            const char *state_str = st->valuestring;
            ESP_LOGI(TAG, "state=%s", state_str);
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
        } else {
            ESP_LOGW(TAG, "print_stats.state missing or not string");
        }

        cJSON *fn = cJSON_GetObjectItem(ps, "filename");
        if (fn && cJSON_IsString(fn)) {
            ESP_LOGI(TAG, "filename='%s' (len=%d)", fn->valuestring, (int)strlen(fn->valuestring));
            /* Detect filename change → need metadata fetch */
            bool fn_changed = strcmp(s_state.filename, fn->valuestring) != 0;
            strncpy(s_state.filename, fn->valuestring, sizeof(s_state.filename) - 1);
            s_state.filename[sizeof(s_state.filename) - 1] = '\0';
            if (fn_changed && fn->valuestring[0]) {
                need_metadata = true;
            }
        } else {
            ESP_LOGW(TAG, "print_stats.filename missing (fn=%p, isStr=%d)",
                     fn, fn ? cJSON_IsString(fn) : -1);
        }

        cJSON *dur = cJSON_GetObjectItem(ps, "total_duration");
        if (dur && cJSON_IsNumber(dur)) {
            s_state.print_time_s = (int32_t)dur->valuedouble;
        }
    } else {
        ESP_LOGW(TAG, "print_stats not in status response");
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

    /* Record temperature history sample (shared circular buffer in printer_comm.c) */
    printer_comm_record_temp_sample();

    ESP_LOGI(TAG, "=> op=%d temps=%.0f/%.0f bed=%.0f/%.0f prog=%.1f%% file='%s'",
             s_state.opstate,
             s_state.hotend_actual, s_state.hotend_target,
             s_state.bed_actual, s_state.bed_target,
             s_state.progress_pct, s_state.filename);

    char fetch_filename[64] = {0};
    if (need_metadata) {
        strncpy(fetch_filename, s_state.filename, sizeof(fetch_filename) - 1);
    }

    xSemaphoreGive(s_state_mutex);
    cJSON_Delete(root);

    /* Fetch file metadata outside state mutex (but caller holds http mutex) */
    if (fetch_filename[0]) {
        fetch_file_metadata(fetch_filename);
    }
}

/* ---- Polling task ---- */

static void klipper_poll_task(void *arg)
{
    ESP_LOGI(TAG, "Klipper polling task started → %s:%u", s_host, s_port);

    static const char *query_path =
        "/printer/objects/query?"
        "extruder&heater_bed&print_stats&virtual_sdcard&toolhead";

    while (true) {
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        int status = http_get(query_path);
        xSemaphoreGive(s_http_mutex);

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
            ESP_LOGD(TAG, "Raw response (%d bytes): %.512s", (int)strlen(s_resp_buf), s_resp_buf);
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

    s_http_mutex = xSemaphoreCreateMutex();
    if (!s_http_mutex) return ESP_ERR_NO_MEM;

    memset(&s_state, 0, sizeof(s_state));
    s_state.opstate = PRINTER_DISCONNECTED;
    s_state.progress_pct = -1;
    s_state.print_time_s = -1;
    s_state.print_time_left_s = -1;
    s_state.total_layers = -1;
    s_meta_filename[0] = '\0';

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
    if (s_http_mutex) {
        vSemaphoreDelete(s_http_mutex);
        s_http_mutex = NULL;
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

    static const char *cmd_names[] = { "pause", "resume", "cancel", "raw" };
    ESP_LOGI(TAG, "CMD: %s%s%s", cmd_names[cmd->type],
             cmd->type == PCMD_RAW ? " → " : "",
             cmd->type == PCMD_RAW ? cmd->gcode : "");

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);

    switch (cmd->type) {
    case PCMD_PAUSE:
        status = http_post("/printer/print/pause", CMD_TIMEOUT_MS);
        break;
    case PCMD_RESUME:
        status = http_post("/printer/print/resume", CMD_TIMEOUT_MS);
        break;
    case PCMD_CANCEL:
        status = http_post("/printer/print/cancel", CMD_TIMEOUT_MS);
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
        status = http_post(path, CMD_TIMEOUT_MS);
        break;
    }
    default:
        xSemaphoreGive(s_http_mutex);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreGive(s_http_mutex);

    if (status < 0) {
        ESP_LOGW(TAG, "Command failed: Moonraker unreachable");
        return ESP_ERR_NOT_FOUND;
    }
    if (status != 200) {
        ESP_LOGW(TAG, "Command returned HTTP %d", status);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Command OK (HTTP 200)");
    return ESP_OK;
}

/** URL-encode a filename into dst buffer. Returns pointer to dst. */
static char *url_encode_filename(char *dst, size_t dst_size, const char *filename)
{
    const char *src = filename;
    char *d = dst;
    char *end = dst + dst_size - 4;
    while (*src && d < end) {
        char c = *src++;
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~' || c == '/') {
            *d++ = c;
        } else {
            d += snprintf(d, end - d + 1, "%%%02X", (unsigned char)c);
        }
    }
    *d = '\0';
    return dst;
}

/**
 * Check if Moonraker already has this file with the same size.
 * Must be called with s_http_mutex held.
 * Returns true if the file exists and size matches.
 */
static bool moonraker_file_matches(const char *filename, long local_size)
{
    char encoded[128];
    url_encode_filename(encoded, sizeof(encoded), filename);

    char path[256];
    snprintf(path, sizeof(path), "/server/files/metadata?filename=%s", encoded);

    int status = http_get(path);
    if (status != 200) return false;

    cJSON *root = cJSON_Parse(s_resp_buf);
    if (!root) return false;

    cJSON *result = cJSON_GetObjectItem(root, "result");
    if (!result) { cJSON_Delete(root); return false; }

    cJSON *size_item = cJSON_GetObjectItem(result, "size");
    bool match = false;
    if (size_item && cJSON_IsNumber(size_item)) {
        long remote_size = (long)size_item->valuedouble;
        match = (remote_size == local_size);
        ESP_LOGI(TAG, "Moonraker file '%s': remote=%ld local=%ld → %s",
                 filename, remote_size, local_size, match ? "SKIP upload" : "RE-upload");
    }

    cJSON_Delete(root);
    return match;
}

/**
 * Delete a file from Moonraker's gcodes storage.
 * Must be called with s_http_mutex held.
 */
static void moonraker_delete_file(const char *filename)
{
    char encoded[128];
    url_encode_filename(encoded, sizeof(encoded), filename);

    char url[320];
    snprintf(url, sizeof(url), "http://%s:%u/server/files/gcodes/%s",
             s_host, s_port, encoded);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_DELETE,
        .timeout_ms = HTTP_TIMEOUT_MS,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return;

    esp_err_t err = esp_http_client_open(client, 0);
    if (err == ESP_OK) {
        esp_http_client_fetch_headers(client);
        int status = esp_http_client_get_status_code(client);
        ESP_LOGI(TAG, "Delete '%s' from Moonraker → HTTP %d", filename, status);
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

esp_err_t klipper_backend_print_file(const char *filename)
{
    if (!filename || !filename[0]) return ESP_ERR_INVALID_ARG;

    /* Get local file size */
    char filepath[300];
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);

    struct stat st;
    if (stat(filepath, &st) != 0 || st.st_size <= 0) {
        ESP_LOGE(TAG, "Cannot stat %s", filepath);
        return ESP_ERR_NOT_FOUND;
    }
    long file_size = (long)st.st_size;

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);

    /* Delete previous uploaded file if it's different from what we're about to print */
    if (s_last_uploaded[0] && strcmp(s_last_uploaded, filename) != 0) {
        moonraker_delete_file(s_last_uploaded);
        s_last_uploaded[0] = '\0';
    }

    /* Check if Moonraker already has this file with the same size */
    bool already_uploaded = moonraker_file_matches(filename, file_size);

    if (!already_uploaded) {
        FILE *f = fopen(filepath, "r");
        if (!f) {
            ESP_LOGE(TAG, "Cannot open %s", filepath);
            xSemaphoreGive(s_http_mutex);
            return ESP_ERR_NOT_FOUND;
        }

        ESP_LOGI(TAG, "Uploading %s (%ld bytes) to Moonraker", filename, file_size);

        /* Build multipart body */
        static const char *boundary = "----ESP32FDMUpload";

        char part_header[256];
        int hdr_len = snprintf(part_header, sizeof(part_header),
            "--%s\r\n"
            "Content-Disposition: form-data; name=\"file\"; filename=\"%s\"\r\n"
            "Content-Type: application/octet-stream\r\n"
            "\r\n",
            boundary, filename);

        char part_footer[64];
        int ftr_len = snprintf(part_footer, sizeof(part_footer),
            "\r\n--%s--\r\n", boundary);

        int total_len = hdr_len + (int)file_size + ftr_len;

        char url[256];
        snprintf(url, sizeof(url), "http://%s:%u/server/files/upload", s_host, s_port);

        char content_type[64];
        snprintf(content_type, sizeof(content_type),
            "multipart/form-data; boundary=%s", boundary);

        esp_http_client_config_t config = {
            .url = url,
            .method = HTTP_METHOD_POST,
            .timeout_ms = 60000,
        };

        esp_http_client_handle_t client = esp_http_client_init(&config);
        if (!client) {
            xSemaphoreGive(s_http_mutex);
            fclose(f);
            return ESP_FAIL;
        }

        esp_http_client_set_header(client, "Content-Type", content_type);

        esp_err_t err = esp_http_client_open(client, total_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Upload open failed: %s", esp_err_to_name(err));
            esp_http_client_cleanup(client);
            xSemaphoreGive(s_http_mutex);
            fclose(f);
            return err;
        }

        esp_http_client_write(client, part_header, hdr_len);

        char *buf = malloc(4096);
        if (!buf) {
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            xSemaphoreGive(s_http_mutex);
            fclose(f);
            return ESP_ERR_NO_MEM;
        }

        size_t total_written = 0;
        while (total_written < (size_t)file_size) {
            size_t to_read = 4096;
            if (to_read > (size_t)file_size - total_written)
                to_read = (size_t)file_size - total_written;
            size_t rd = fread(buf, 1, to_read, f);
            if (rd == 0) break;
            int wr = esp_http_client_write(client, buf, rd);
            if (wr < 0) {
                ESP_LOGE(TAG, "Upload write failed");
                break;
            }
            total_written += rd;
        }
        free(buf);
        fclose(f);

        esp_http_client_write(client, part_footer, ftr_len);

        esp_http_client_fetch_headers(client);
        int status = esp_http_client_get_status_code(client);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);

        if (status != 201 && status != 200) {
            ESP_LOGE(TAG, "Upload failed: HTTP %d", status);
            xSemaphoreGive(s_http_mutex);
            return ESP_FAIL;
        }

        ESP_LOGI(TAG, "Upload OK (HTTP %d)", status);
    }

    /* Start print */
    char encoded[128];
    url_encode_filename(encoded, sizeof(encoded), filename);

    char path[256];
    snprintf(path, sizeof(path), "/printer/print/start?filename=%s", encoded);
    int status = http_post(path, CMD_TIMEOUT_MS);

    xSemaphoreGive(s_http_mutex);

    if (status != 200) {
        ESP_LOGE(TAG, "Print start failed: HTTP %d", status);
        return ESP_FAIL;
    }

    /* Track this as the last uploaded file */
    strncpy(s_last_uploaded, filename, sizeof(s_last_uploaded) - 1);
    s_last_uploaded[sizeof(s_last_uploaded) - 1] = '\0';

    ESP_LOGI(TAG, "Print started: %s", filename);
    return ESP_OK;
}
