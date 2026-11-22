#include "printer_comm_klipper.h"
#include "task_util.h"

#include "cJSON.h"
#include "hub_client.h"
#include "klipper_holds.h"
#include "nozzle_guard.h"
#include "esp_http_client.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static const char *TAG = "klipper";

#define POLL_INTERVAL_MS    4000
#define POLL_HOLD_MS        2000    /* While waiting for the user: Z and Continue show sooner */
#define POLL_AFTER_CMD_MS   500
#define HTTP_TIMEOUT_MS     5000
#define CMD_TIMEOUT_MS      10000
#define HTTP_BUF_SIZE       16384   /* The bed mesh with its saved profiles (11x11 each) */

/* Stops for the user in the files we send (see klipper_holds.h) */
#define MAX_HOLDS           64
#define HOLD_BEEP_SCRIPT    "M300 S2000 P150\nG4 P450\nM300 S2000 P150\nG4 P450\nM300 S2000 P150"
#define XFORM_BLOCK_MAX     16384   /* SD reads, in internal DMA RAM; halved until it fits */
#define UPLOAD_CHUNK        16384   /* Network writes, in PSRAM: fewer, larger sends */
#define START_WATCH_S       60      /* How long a slow print start is waited for */

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

/* Stops in the last file we sent, and the current one.  Under s_state_mutex. */
static klipper_hold_t *s_holds;         /* MAX_HOLDS, in PSRAM */
static int s_n_holds;
static char s_holds_file[64];           /* The file they belong to... */
static uint32_t s_holds_size;           /* ...as sent, so a file with that name uploaded elsewhere does not match */
static char s_rescan_tried[64];         /* Rebuilt from the SD card after a reboot at most once per file */
static int s_hold_idx = -1;
static uint32_t s_hold_counter;
static int64_t s_hold_since_us;
static int64_t s_next_beep_us;          /* 0: no more beeps for this hold */
static bool s_hold_new;                 /* Not reported to the hub yet */
static uint32_t s_continue_id;          /* Continue asked for this hold */
static uint32_t s_continue_sent_id;     /* M24 already sent for this hold */
static uint32_t s_timeout_sent_id;      /* Cancelled for this hold */

/* Nozzle guard (nozzle_guard.h), under s_state_mutex: the current stop's, and
 * the temperature a pause (from Obico, or for a runout) took the nozzle down from */
static ng_hold_t s_ng;
static int64_t s_ng_next_us;            /* Poll task only: checked once a second */
static int s_pause_restore_c;

/* The filament sensor (filament_switch_sensor or filament_motion_sensor), from
 * the objects list: its name as a key in the status and in the query (%20).
 * Under s_http_mutex; s_filament_out and the runout wait under s_state_mutex. */
static char s_runout_key[64], s_runout_q[80];
static bool s_filament_out;             /* Enabled and showing no filament */
static ng_hold_t s_rng;                 /* A pause for a runout: standby after mid_after_min */
static bool s_runout_new;               /* ...just begun: tell the hub */
static int64_t s_rng_next_us;           /* Poll task only */

/* The guard's own E moves leave the file's extruder mode and position as
 * they were */
#define NG_SAVE    "SAVE_GCODE_STATE NAME=fdm_guard\nM83\n"
#define NG_RESTORE "RESTORE_GCODE_STATE NAME=fdm_guard"
static int s_z_shift_cu, s_z_pending_cu;    /* Hundredths of a mm */
static uint32_t s_vsd_pos, s_vsd_size;  /* virtual_sdcard file position and size */
static bool s_has_beeper;               /* The printer has an M300 macro */
static bool s_has_light;                /* ...and an FDM_LIGHT macro (board_led.c) */
static volatile int s_upload_pct = -1;  /* A file being sent: percent, reading it counts as 0 */

/* The built-in script (filament change, cold pull) sent last.  Under s_state_mutex. */
static char s_script_file[64];          /* Its name on Moonraker, "" if none */
static host_script_done_t s_script_done;
static int s_script_arg;
static bool s_script_running;           /* Started, end not seen yet */
static bool s_script_ended, s_script_completed;  /* For the tick to call s_script_done */
static volatile bool s_script_starting;

/* The mesh profile the printer loads when it starts (on the Neptune 4 its
 * screen does), learnt by watching the profile after Klipper comes back */
#define BOOT_WATCH_US       (3 * 60 * 1000000LL)
#define NVS_NS_KLIPPER      "klipper"
#define NVS_KEY_BOOT_MESH   "boot_mesh"
static char s_mesh_profile[24];         /* Active now */
static char s_boot_mesh[24];
static bool s_was_down;                 /* Klipper or Moonraker was not answering */
static int64_t s_boot_watch_until;
static bool s_boot_mesh_dirty;          /* Learnt, not in NVS yet */

/* The board's own Z offset, put into files at "; @zoffset" (klipper_holds.h) */
#define NVS_KEY_ZOFFSET     "z_offset"      /* Thousandths of a mm, i32 */
static bool s_has_z;

/* Raise Z: where Z is, and the last place it was known while homed */
#define LIFT_FEED        1200    /* mm/min; Klipper caps it at max_z_velocity */
#define LIFT_TOP_MARGIN  2.0f
static float s_toolhead_z, s_last_z, s_zmax;
static bool s_homed_z, s_last_z_ok;
static float s_z_offset;

/* Moonraker's job history totals, fetched once */
static uint32_t s_hist_print_s, s_hist_jobs;
static bool s_hist_ok;
static bool s_objects_checked;
static int64_t s_objects_checked_us;  /* esp_timer time of the last object check */
static bool s_hist_checked;

/* Response buffer for HTTP reads (allocated in PSRAM) */
static char *s_resp_buf;

/* ---- HTTP helpers ---- */

/**
 * Perform a GET request and read the response body into s_resp_buf.
 * Returns the HTTP status code, or -1 on connection failure.
 */
static int http_get(const char *path)
{
    char url[512];   /* The status query alone is near 300 */
    if (snprintf(url, sizeof(url), "http://%s:%u%s", s_host, s_port, path) >= (int)sizeof(url)) {
        /* A cut query loses objects without an error from Moonraker */
        ESP_LOGE(TAG, "URL too long: %.60s...", path);
        return -1;
    }

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
    while (total < (int)HTTP_BUF_SIZE - 1) {
        int r = esp_http_client_read(client, s_resp_buf + total,
                                     HTTP_BUF_SIZE - 1 - total);
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
    char url[512];   /* The status query alone is near 300 */
    if (snprintf(url, sizeof(url), "http://%s:%u%s", s_host, s_port, path) >= (int)sizeof(url)) {
        /* A cut query loses objects without an error from Moonraker */
        ESP_LOGE(TAG, "URL too long: %.60s...", path);
        return -1;
    }

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

/** POST /printer/gcode/script.  Must be called with s_http_mutex held.
 *  Returns the HTTP status, or -1 if the request could not be sent. */
static int gcode_script_locked(const char *script)
{
    char encoded[400];
    const char *src = script;
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

    char path[sizeof(encoded) + 40];
    snprintf(path, sizeof(path), "/printer/gcode/script?script=%s", encoded);
    return http_post(path, CMD_TIMEOUT_MS);
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

/** Whether Klipper is paused at one of our stops, from the file position.
 *  Caller holds s_state_mutex and has updated the state. */
static void update_hold_locked(void)
{
    int idx = -1;
    if (s_state.opstate == PRINTER_PAUSED && s_n_holds > 0 &&
        strcmp(s_state.filename, s_holds_file) == 0 && s_vsd_size == s_holds_size) {
        for (int i = 0; i < s_n_holds; i++) {
            if (s_holds[i].end_pos == s_vsd_pos) { idx = i; break; }
        }
    }

    /* A pause from Obico or for a runout is over (resumed here, or on the
     * printer's screen, whose RESUME heats back by itself on the Neptune 4) */
    if (s_state.opstate != PRINTER_PAUSED) s_pause_restore_c = 0;

    if (idx < 0) {
        if (s_state.holding) ESP_LOGI(TAG, "Wait for the user is over");
        s_state.holding = false;
        s_state.hold_msg[0] = '\0';
        s_hold_idx = -1;
        s_next_beep_us = 0;
        s_continue_id = 0;
        ng_hold_end(&s_ng);
        if (!s_pause_restore_c) s_state.nozzle = NOZZLE_HOT;
        return;
    }
    if (s_state.holding && idx == s_hold_idx) return;

    ng_hold_begin(&s_ng, esp_timer_get_time(), s_holds[idx].before_layers);
    s_state.nozzle = NOZZLE_HOT;

    s_hold_idx = idx;
    s_state.holding = true;
    s_state.hold_id = ++s_hold_counter;
    strlcpy(s_state.hold_msg, s_holds[idx].msg, sizeof(s_state.hold_msg));
    s_hold_since_us = esp_timer_get_time();
    s_next_beep_us = s_hold_since_us;
    s_hold_new = true;
    ESP_LOGI(TAG, "Waiting for user: %s", s_state.hold_msg);
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

    bool was_active = s_state.opstate == PRINTER_PRINTING || s_state.opstate == PRINTER_PAUSED;
    bool file_changed = false;
    char ps_state[16] = "";

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
            ESP_LOGD(TAG, "print_stats: %.512s", ps_str);
            cJSON_free(ps_str);
        }

        cJSON *st = cJSON_GetObjectItem(ps, "state");
        if (st && cJSON_IsString(st)) {
            const char *state_str = st->valuestring;
            ESP_LOGD(TAG, "state=%s", state_str);
            strlcpy(ps_state, state_str, sizeof(ps_state));
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
            ESP_LOGD(TAG, "filename='%s' (len=%d)", fn->valuestring, (int)strlen(fn->valuestring));
            /* Detect filename change → need metadata fetch */
            bool fn_changed = strcmp(s_state.filename, fn->valuestring) != 0;
            file_changed = fn_changed;
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
        cJSON *fp = cJSON_GetObjectItem(vsd, "file_position");
        cJSON *fs = cJSON_GetObjectItem(vsd, "file_size");
        if (cJSON_IsNumber(fp)) s_vsd_pos = (uint32_t)fp->valuedouble;
        if (cJSON_IsNumber(fs)) s_vsd_size = (uint32_t)fs->valuedouble;
    }

    /* The Z shift is per print */
    bool active = s_state.opstate == PRINTER_PRINTING || s_state.opstate == PRINTER_PAUSED;
    if (active && (!was_active || file_changed)) {
        s_z_shift_cu = 0;
        s_z_pending_cu = 0;
    }
    if (!active) s_z_pending_cu = 0;    /* Taps that came too late for the print */
    update_hold_locked();

    /* The filament ran out: the printer paused itself (pause_on_runout) and
     * its sensor shows none.  Read while paused only: loading the new
     * filament does not end the wait, Continue does. */
    cJSON *fs = s_runout_key[0] ? cJSON_GetObjectItem(status, s_runout_key) : NULL;
    if (fs) {
        cJSON *det = cJSON_GetObjectItem(fs, "filament_detected");
        cJSON *en = cJSON_GetObjectItem(fs, "enabled");
        s_filament_out = cJSON_IsFalse(det) && !cJSON_IsFalse(en);
    } else {
        s_filament_out = false;
    }
    if (s_state.opstate != PRINTER_PAUSED) {
        ng_hold_end(&s_rng);
    } else if (!s_rng.active && !s_state.holding && !s_pause_restore_c && s_filament_out) {
        ESP_LOGW(TAG, "Paused, the filament ran out");
        ng_hold_begin(&s_rng, esp_timer_get_time(), false);
        s_rng_next_us = 0;
        s_runout_new = true;
    }

    cJSON *fan = cJSON_GetObjectItem(cJSON_GetObjectItem(status, "fan"), "speed");
    s_state.fan_pct = cJSON_IsNumber(fan) ? (int8_t)(fan->valuedouble * 100.0 + 0.5) : -1;

    /* The gcode offset: the Z offset the printer's screen sets, and our Z taps */
    cJSON *ho = cJSON_GetObjectItem(cJSON_GetObjectItem(status, "gcode_move"), "homing_origin");
    if (cJSON_GetArraySize(ho) >= 3 && cJSON_IsNumber(cJSON_GetArrayItem(ho, 2)))
        s_state.probe_z_offset = (float)cJSON_GetArrayItem(ho, 2)->valuedouble;

    cJSON *bm = cJSON_GetObjectItem(cJSON_GetObjectItem(status, "bed_mesh"), "profile_name");
    if (cJSON_IsString(bm)) {
        strlcpy(s_mesh_profile, bm->valuestring, sizeof(s_mesh_profile));
        /* Klipper itself loads only "default"; anything else soon after it
         * starts is the printer's own choice */
        if (s_boot_watch_until && esp_timer_get_time() < s_boot_watch_until &&
            bm->valuestring[0] && strcmp(bm->valuestring, "default") != 0) {
            if (strcmp(s_boot_mesh, bm->valuestring) != 0) {
                strlcpy(s_boot_mesh, bm->valuestring, sizeof(s_boot_mesh));
                s_boot_mesh_dirty = true;
            }
            s_boot_watch_until = 0;
            ESP_LOGI(TAG, "The printer loads mesh profile '%s' when it starts", s_boot_mesh);
        }
    }

    /* A built-in script ends: completed only if Klipper says so, a cancel
     * (CANCEL_PRINT resets the file) shows as standby */
    bool script_now = s_script_file[0] && strcmp(s_state.filename, s_script_file) == 0;
    if (!s_script_running && s_script_done && active && script_now && !was_active) {
        s_script_running = true;    /* Started later than the start request gave up */
    }
    if (s_script_running && ps_state[0] && !(active && script_now)) {
        s_script_running = false;
        s_script_ended = true;
        s_script_completed = script_now && strcmp(ps_state, "complete") == 0;
        ESP_LOGI(TAG, "%s %s", s_script_file, s_script_completed ? "completed" : "cancelled");
    }
    s_state.builtin_script = active && script_now;

    /* How a print ended, for the print statistics */
    if (was_active && !active && ps_state[0]) {
        s_state.last_host_completed = strcmp(ps_state, "complete") == 0;
    }
    s_state.z_shift = s_z_shift_cu / 100.0f;
    s_state.z_pending = s_z_pending_cu / 100.0f;

    /* Position from toolhead */
    cJSON *th = cJSON_GetObjectItem(status, "toolhead");
    if (th) {
        cJSON *pos = cJSON_GetObjectItem(th, "position");
        if (pos && cJSON_IsArray(pos) && cJSON_GetArraySize(pos) >= 3) {
            s_state.x = (float)cJSON_GetArrayItem(pos, 0)->valuedouble;
            s_state.y = (float)cJSON_GetArrayItem(pos, 1)->valuedouble;
            s_state.z = (float)cJSON_GetArrayItem(pos, 2)->valuedouble;
            s_toolhead_z = s_state.z;
        }
        /* Klipper forgets the homing when the steppers go off (M84), and
         * keeps the last position: what "Raise Z" starts from when unhomed */
        cJSON *ha = cJSON_GetObjectItem(th, "homed_axes");
        if (cJSON_IsString(ha)) {
            s_state.motors = ha->valuestring[0] != '\0';
            s_homed_z = strchr(ha->valuestring, 'z') != NULL;
            if (s_homed_z) {
                s_last_z = s_toolhead_z;
                s_last_z_ok = true;
            }
        }
        cJSON *am = cJSON_GetObjectItem(th, "axis_maximum");
        if (cJSON_GetArraySize(am) >= 3 && cJSON_IsNumber(cJSON_GetArrayItem(am, 2))) {
            s_zmax = (float)cJSON_GetArrayItem(am, 2)->valuedouble;
        }
    }
    /* Z as G-code sees it, without the gcode offset: Z0 is where the offset
     * puts the nozzle, and the dashboard stops Z- there */
    cJSON *gp = cJSON_GetObjectItem(cJSON_GetObjectItem(status, "gcode_move"), "gcode_position");
    if (cJSON_GetArraySize(gp) >= 3 && cJSON_IsNumber(cJSON_GetArrayItem(gp, 2)))
        s_state.z = (float)cJSON_GetArrayItem(gp, 2)->valuedouble;

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
    printer_comm_record_temp_sample(s_state.hotend_actual, s_state.hotend_target,
                                    s_state.bed_actual, s_state.bed_target);

    ESP_LOGD(TAG, "=> op=%d temps=%.0f/%.0f bed=%.0f/%.0f prog=%.1f%% file='%s'",
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

/* ---- Stops in the files we send ---- */

/* What we send: a file on the SD card, or a built-in script in memory */
typedef struct {
    const char *path;
    const char *mem;
    size_t mem_len;
} src_t;

/**
 * Run a file through the stop rewriting (klipper_holds.h).
 * x is set up by the caller: emit NULL to only count the size and find the stops.
 */
static esp_err_t xform_src(const src_t *src, kh_xform_t *x)
{
    if (src->mem) {
        kh_feed(x, src->mem, src->mem_len);
        kh_finish(x);
        return ESP_OK;
    }

    /* read(), not fread(): stdio reads through FATFS's sector cache, which is
     * in PSRAM here and reached through a bounce buffer, about 1.4 MB/s; whole
     * sectors read straight into internal DMA memory go at 4.5 MB/s (16 KB,
     * measured on fdm06; 8 KB 4.2, 4 KB 3.7) */
    int fd = open(src->path, O_RDONLY);
    if (fd < 0) return ESP_ERR_NOT_FOUND;
    size_t size = XFORM_BLOCK_MAX;
    char *buf = NULL;
    while (size >= 4096 && !(buf = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)))
        size /= 2;
    if (!buf) {
        close(fd);
        return ESP_ERR_NO_MEM;
    }
    ssize_t rd;
    while ((rd = read(fd, buf, size)) > 0) kh_feed(x, buf, (size_t)rd);
    kh_finish(x);
    free(buf);
    close(fd);
    return rd < 0 ? ESP_FAIL : ESP_OK;
}

/** The parsing state, in internal RAM: in PSRAM it slowed every line */
static kh_xform_t *xform_alloc(void)
{
    kh_xform_t *x = heap_caps_malloc(sizeof(kh_xform_t), MALLOC_CAP_INTERNAL);
    return x ? x : heap_caps_malloc(sizeof(kh_xform_t), MALLOC_CAP_SPIRAM);
}

/** Make these the stops of the file Klipper prints as filename */
static void install_holds(const char *filename, const klipper_hold_t *holds, int n, uint32_t size)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    memcpy(s_holds, holds, n * sizeof(klipper_hold_t));
    s_n_holds = n;
    strlcpy(s_holds_file, filename, sizeof(s_holds_file));
    s_holds_size = size;
    strlcpy(s_rescan_tried, filename, sizeof(s_rescan_tried));
    xSemaphoreGive(s_state_mutex);
}

/** After a reboot: the stops of the file Klipper is printing, if it is the
 *  one we sent from the SD card (same name, and the size we would send) */
static void rescan_task(void *arg)
{
    char *filename = arg;
    char path[300];
    snprintf(path, sizeof(path), "/sdcard/%s", filename);

    klipper_hold_t *holds = heap_caps_calloc(MAX_HOLDS, sizeof(klipper_hold_t), MALLOC_CAP_SPIRAM);
    kh_xform_t *x = xform_alloc();
    struct stat st;
    if (holds && x && stat(path, &st) == 0) {
        kh_init(x, holds, MAX_HOLDS, NULL, NULL);
        x->has_z = klipper_backend_get_zoffset(&x->z);
        src_t src = { .path = path };
        if (xform_src(&src, x) == ESP_OK) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            bool same = strcmp(s_state.filename, filename) == 0 && s_vsd_size == x->out_pos;
            xSemaphoreGive(s_state_mutex);
            if (same) {
                install_holds(filename, holds, x->n_holds, x->out_pos);
                ESP_LOGI(TAG, "%s: %d stops for the user, read again from SD", filename, x->n_holds);
            } else {
                ESP_LOGI(TAG, "%s: not the file we sent, its stops are unknown", filename);
            }
        }
    }
    free(x);
    free(holds);
    free(filename);
    task_delete(NULL);
}

/* ---- Polling task ---- */

/* Rechecked this often too: a quick RESTART (to load a macro just added) can
 * fall between two polls, so no poll fails and nothing else would notice */
#define OBJECTS_RECHECK_US (60LL * 1000000)

/** The first filament sensor in the objects list (still in its case), for
 *  the poll's query */
static void find_runout_sensor(void)
{
    static const char *types[] = { "\"filament_switch_sensor ", "\"filament_motion_sensor " };
    char key[sizeof(s_runout_key)] = "", q[sizeof(s_runout_q)] = "";
    for (size_t t = 0; t < sizeof(types) / sizeof(types[0]) && !key[0]; t++) {
        const char *p = strstr(s_resp_buf, types[t]);
        if (!p) continue;
        p++;
        const char *e = strchr(p, '"');
        if (!e || e - p >= (int)sizeof(key)) continue;
        memcpy(key, p, e - p);
        key[e - p] = '\0';
    }
    /* Section names are letters, digits and _: only the space needs encoding */
    size_t n = 0;
    for (const char *c = key; *c && n + 4 < sizeof(q); c++) {
        if (*c == ' ') { memcpy(q + n, "%20", 3); n += 3; }
        else if (isalnum((unsigned char)*c) || *c == '_') q[n++] = *c;
        else { key[0] = '\0'; n = 0; break; }
    }
    q[n] = '\0';
    if (strcmp(key, s_runout_key) != 0)
        ESP_LOGI(TAG, "Filament sensor: %s", key[0] ? key : "none");
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    strlcpy(s_runout_key, key, sizeof(s_runout_key));
    strlcpy(s_runout_q, q, sizeof(s_runout_q));
    xSemaphoreGive(s_state_mutex);
}

/** Whether the printer has an M300 macro, so that a hold can beep, and an
 *  FDM_LIGHT macro for its light */
static void check_objects_locked(void)
{
    s_objects_checked_us = esp_timer_get_time();
    if (http_get("/printer/objects/list") != 200) return;
    find_runout_sensor();
    for (char *c = s_resp_buf; *c; c++) *c = (char)tolower((unsigned char)*c);
    bool beeper = strstr(s_resp_buf, "\"gcode_macro m300\"") != NULL;
    bool light = strstr(s_resp_buf, "\"gcode_macro fdm_light\"") != NULL;
    static bool logged;
    if (!logged || beeper != s_has_beeper || light != s_has_light)
        ESP_LOGI(TAG, "Printer %s M300, %s FDM_LIGHT", beeper ? "has" : "has no",
                 light ? "has" : "has no");
    logged = true;
    s_has_beeper = beeper;
    s_has_light = light;
    s_objects_checked = true;
}

/** Moonraker's history totals, for the print statistics of a new board */
static void check_history_locked(void)
{
    s_hist_checked = true;
    if (http_get("/server/history/totals") != 200) return;
    cJSON *root = cJSON_Parse(s_resp_buf);
    cJSON *t = cJSON_GetObjectItem(cJSON_GetObjectItem(root, "result"), "job_totals");
    cJSON *ps = cJSON_GetObjectItem(t, "total_print_time");
    cJSON *jobs = cJSON_GetObjectItem(t, "total_jobs");
    if (cJSON_IsNumber(ps) && cJSON_IsNumber(jobs)) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_hist_print_s = (uint32_t)ps->valuedouble;
        s_hist_jobs = (uint32_t)jobs->valuedouble;
        s_hist_ok = true;
        xSemaphoreGive(s_state_mutex);
        ESP_LOGI(TAG, "Moonraker history: %lu jobs, %.1f h printed",
                 (unsigned long)s_hist_jobs, s_hist_print_s / 3600.0f);
    }
    cJSON_Delete(root);
}

/** Whether a profile's saved points are the matrix */
static bool same_points(const cJSON *pts, const float *data, int max_n, int nx, int ny)
{
    if (cJSON_GetArraySize(pts) != ny) return false;
    for (int y = 0; y < ny; y++) {
        const cJSON *row = cJSON_GetArrayItem(pts, y);
        if (cJSON_GetArraySize(row) != nx) return false;
        for (int x = 0; x < nx; x++) {
            const cJSON *v = cJSON_GetArrayItem(row, x);
            if (!cJSON_IsNumber(v) || fabsf((float)v->valuedouble - data[y * max_n + x]) > 0.0005f)
                return false;
        }
    }
    return true;
}

esp_err_t klipper_backend_get_mesh(float *data, int max_n, int *nx, int *ny, float bounds[4],
                                   klipper_mesh_info_t *info)
{
    if (!s_http_mutex) return ESP_ERR_INVALID_STATE;
    memset(info, 0, sizeof(*info));
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    int status = http_get("/printer/objects/query?bed_mesh=probed_matrix,mesh_min,mesh_max,profile_name,profiles");
    cJSON *root = status == 200 ? cJSON_Parse(s_resp_buf) : NULL;
    xSemaphoreGive(s_http_mutex);
    if (!root) return status < 0 ? ESP_ERR_NOT_FOUND : ESP_FAIL;

    esp_err_t err = ESP_ERR_NOT_FOUND;
    cJSON *bm = cJSON_GetObjectItem(cJSON_GetObjectItem(cJSON_GetObjectItem(root, "result"), "status"),
                                    "bed_mesh");
    cJSON *pm = cJSON_GetObjectItem(bm, "probed_matrix");
    cJSON *mn = cJSON_GetObjectItem(bm, "mesh_min");
    cJSON *mx = cJSON_GetObjectItem(bm, "mesh_max");
    cJSON *pn = cJSON_GetObjectItem(bm, "profile_name");
    cJSON *profiles = cJSON_GetObjectItem(bm, "profiles");
    if (cJSON_IsString(pn)) strlcpy(info->profile, pn->valuestring, sizeof(info->profile));
    cJSON *pr;
    cJSON_ArrayForEach(pr, profiles) {
        if (info->n_names >= KLIPPER_MESH_NAMES_MAX || !pr->string || !strcmp(pr->string, "default")) continue;
        if (strlen(pr->string) >= sizeof(info->names[0])) continue;
        strcpy(info->names[info->n_names++], pr->string);
    }

    int rows = cJSON_GetArraySize(pm);
    int cols = rows > 0 ? cJSON_GetArraySize(cJSON_GetArrayItem(pm, 0)) : 0;
    if (rows >= 2 && cols >= 2 && rows <= max_n && cols <= max_n &&
        cJSON_GetArraySize(mn) == 2 && cJSON_GetArraySize(mx) == 2) {
        err = ESP_OK;
        for (int y = 0; y < rows && err == ESP_OK; y++) {
            cJSON *row = cJSON_GetArrayItem(pm, y);
            if (cJSON_GetArraySize(row) != cols) { err = ESP_FAIL; break; }
            for (int x = 0; x < cols; x++) {
                cJSON *v = cJSON_GetArrayItem(row, x);
                if (!cJSON_IsNumber(v)) { err = ESP_FAIL; break; }
                data[y * max_n + x] = (float)v->valuedouble;
            }
        }
        *nx = cols;
        *ny = rows;
        bounds[0] = (float)cJSON_GetArrayItem(mn, 0)->valuedouble;
        bounds[1] = (float)cJSON_GetArrayItem(mn, 1)->valuedouble;
        bounds[2] = (float)cJSON_GetArrayItem(mx, 0)->valuedouble;
        bounds[3] = (float)cJSON_GetArrayItem(mx, 1)->valuedouble;
        if (err == ESP_OK && info->profile[0]) {
            cJSON *saved = cJSON_GetObjectItem(cJSON_GetObjectItem(profiles, info->profile), "points");
            info->from_saved = saved && same_points(saved, data, max_n, cols, rows);
        }
    }
    cJSON_Delete(root);
    return err;
}

bool klipper_backend_profile_has_mesh(const char *name)
{
    if (!s_http_mutex) return false;
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    int status = http_get("/printer/objects/query?bed_mesh=probed_matrix,profiles");
    cJSON *root = status == 200 ? cJSON_Parse(s_resp_buf) : NULL;
    xSemaphoreGive(s_http_mutex);
    if (!root) return false;
    cJSON *bm = cJSON_GetObjectItem(cJSON_GetObjectItem(cJSON_GetObjectItem(root, "result"), "status"),
                                    "bed_mesh");
    cJSON *pm = cJSON_GetObjectItem(bm, "probed_matrix");
    cJSON *pts = cJSON_GetObjectItem(cJSON_GetObjectItem(cJSON_GetObjectItem(bm, "profiles"), name), "points");
    int ny = cJSON_GetArraySize(pm), nx = ny ? cJSON_GetArraySize(cJSON_GetArrayItem(pm, 0)) : 0;
    bool ok = pts && ny >= 2 && nx >= 2 && ny <= 15 && nx <= 15;
    float *d = ok ? malloc(15 * 15 * sizeof(float)) : NULL;
    if (d) {
        for (int y = 0; y < ny; y++)
            for (int x = 0; x < nx; x++)
                d[y * 15 + x] = (float)cJSON_GetArrayItem(cJSON_GetArrayItem(pm, y), x)->valuedouble;
        ok = same_points(pts, d, 15, nx, ny);
    } else {
        ok = false;
    }
    free(d);
    cJSON_Delete(root);
    return ok;
}

esp_err_t klipper_backend_get_profile(const char *name, float *data, int max_n, int *nx, int *ny,
                                      float bounds[4])
{
    if (!s_http_mutex) return ESP_ERR_INVALID_STATE;
    xSemaphoreTake(s_http_mutex, portMAX_DELAY);
    int status = http_get("/printer/objects/query?bed_mesh=profiles");
    cJSON *root = status == 200 ? cJSON_Parse(s_resp_buf) : NULL;
    xSemaphoreGive(s_http_mutex);
    if (!root) return status < 0 ? ESP_ERR_INVALID_STATE : ESP_FAIL;

    cJSON *pr = cJSON_GetObjectItem(cJSON_GetObjectItem(cJSON_GetObjectItem(cJSON_GetObjectItem(
                    cJSON_GetObjectItem(root, "result"), "status"), "bed_mesh"), "profiles"), name);
    cJSON *pts = cJSON_GetObjectItem(pr, "points");
    cJSON *mp = cJSON_GetObjectItem(pr, "mesh_params");
    const char *keys[4] = { "min_x", "min_y", "max_x", "max_y" };
    esp_err_t err = pts && mp ? ESP_OK : ESP_ERR_NOT_FOUND;
    for (int i = 0; err == ESP_OK && i < 4; i++) {
        cJSON *v = cJSON_GetObjectItem(mp, keys[i]);
        if (!cJSON_IsNumber(v)) err = ESP_FAIL;
        else bounds[i] = (float)v->valuedouble;
    }
    int rows = cJSON_GetArraySize(pts);
    int cols = rows > 0 ? cJSON_GetArraySize(cJSON_GetArrayItem(pts, 0)) : 0;
    if (err == ESP_OK && (rows < 2 || cols < 2 || rows > max_n || cols > max_n)) err = ESP_FAIL;
    for (int y = 0; err == ESP_OK && y < rows; y++) {
        cJSON *row = cJSON_GetArrayItem(pts, y);
        if (cJSON_GetArraySize(row) != cols) { err = ESP_FAIL; break; }
        for (int x = 0; x < cols; x++) {
            cJSON *v = cJSON_GetArrayItem(row, x);
            if (!cJSON_IsNumber(v)) { err = ESP_FAIL; break; }
            data[y * max_n + x] = (float)v->valuedouble;
        }
    }
    *nx = cols;
    *ny = rows;
    cJSON_Delete(root);
    return err;
}

void klipper_backend_boot_mesh(char *out, size_t size)
{
    if (!s_state_mutex) { out[0] = '\0'; return; }
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    strlcpy(out, s_boot_mesh, size);
    xSemaphoreGive(s_state_mutex);
}

bool klipper_backend_history_totals(uint32_t *print_s, uint32_t *jobs)
{
    if (!s_state_mutex) return false;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    bool ok = s_hist_ok;
    *print_s = s_hist_print_s;
    *jobs = s_hist_jobs;
    xSemaphoreGive(s_state_mutex);
    return ok;
}

static void rescan_task(void *arg);

/** Work for the poll task after each poll: tell the hub about a new hold,
 *  send Z, Continue and beeps, cancel a hold nobody came to.
 *  Returns true if it sent something, so the next poll comes sooner. */
static bool klipper_tick(void)
{
    int64_t now = esp_timer_get_time();
    char msg[sizeof(s_state.hold_msg)], fn[sizeof(s_state.filename)];

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    host_script_done_t done = s_script_ended ? s_script_done : NULL;
    bool done_completed = s_script_completed;
    int done_arg = s_script_arg;
    if (s_script_ended) {
        s_script_ended = false;
        s_script_done = NULL;
    }
    bool holding = s_state.holding;
    uint32_t id = s_state.hold_id;
    bool report = s_hold_new;
    s_hold_new = false;
    strlcpy(msg, s_state.hold_msg, sizeof(msg));
    strlcpy(fn, s_state.filename, sizeof(fn));

    bool timed_out = holding && s_timeout_sent_id != id &&
                     now - s_hold_since_us > CONFIG_HOLD_TIMEOUT_MIN * 60 * 1000000LL;
    if (timed_out) s_timeout_sent_id = id;

    bool go = holding && !timed_out && s_continue_id == id && s_continue_sent_id != id;
    if (go) s_continue_sent_id = id;

    /* Not once Continue is sent: heating back can take a minute, and a beep
     * queued behind it would play in the print */
    bool beep = holding && !go && !timed_out && s_has_beeper && s_continue_sent_id != id &&
                s_next_beep_us && now >= s_next_beep_us;
    if (beep) s_next_beep_us = printer_comm_next_beep_series(now, s_hold_since_us);

    bool can_z = s_state.opstate == PRINTER_PRINTING || holding;
    int zcu = can_z && !timed_out ? s_z_pending_cu : 0;

    /* Nozzle guard: what this stop needs; Continue from standby heats first */
    ng_hold_t ng = s_ng;
    ng_hold_t rng = s_rng;
    bool runout_new = s_runout_new;
    s_runout_new = false;
    int target_c = (int)(s_state.hotend_target + 0.5f);
    if (go && ng.standby) s_state.nozzle = NOZZLE_HEATING;

    /* After a reboot the stops of the running file are unknown: read them
     * again from the SD card, once per file */
    bool active = s_state.opstate == PRINTER_PRINTING || s_state.opstate == PRINTER_PAUSED;
    char *rescan = NULL;
    if (active && fn[0] && strcmp(fn, s_holds_file) != 0 && strcmp(fn, s_rescan_tried) != 0) {
        strlcpy(s_rescan_tried, fn, sizeof(s_rescan_tried));
        rescan = strdup(fn);
    }
    xSemaphoreGive(s_state_mutex);

    if (rescan && task_create_psram(rescan_task, "klipper_scan", 8192, rescan,
                                          3, NULL, 0) != pdPASS) {
        free(rescan);
    }

    if (done) done(done_completed, done_arg);
    if (report) hub_client_send_event(HUB_EVENT_HOLD, msg, fn);
    if (runout_new) ng_notify_runout(target_c, ng_standby_c(target_c), fn);

    ng_action_t ng_act = NG_ACT_NONE;
    int ng_sb = 0;
    ng_config_t ngc;
    if (holding && !go && !timed_out && ng.active && !ng.standby && now >= s_ng_next_us) {
        s_ng_next_us = now + 1000000;
        ng_config_get(&ngc);
        ng_sb = ng_standby_c(target_c);
        ng_act = ng_hold_step(&ngc, &ng, now, ng_sb);
    }

    /* A pause for a runout: as a stop during the print, standby after
     * mid_after_min.  The printer's PAUSE has retracted already. */
    int rn_sb = 0;
    if (rng.active && !rng.standby && now >= s_rng_next_us) {
        s_rng_next_us = now + 1000000;
        ng_config_t c;
        ng_config_get(&c);
        rn_sb = ng_standby_c(target_c);
        if (ng_hold_step(&c, &rng, now, rn_sb) != NG_ACT_STANDBY) rn_sb = 0;
    }

    bool sent = false;
    if (rn_sb) {
        char script[24];
        snprintf(script, sizeof(script), "M104 S%d", rn_sb);
        ESP_LOGI(TAG, "Runout: nozzle %d -> %d C", target_c, rn_sb);
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        int status = gcode_script_locked(script);
        xSemaphoreGive(s_http_mutex);
        if (status == 200 || status < 0) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            if (s_rng.active) {
                s_rng.standby = true;
                s_pause_restore_c = target_c;   /* Continue heats back first */
                s_state.nozzle = NOZZLE_STANDBY;
            }
            xSemaphoreGive(s_state_mutex);
            ng_notify_standby(target_c, rn_sb, fn);
        } else {
            ESP_LOGW(TAG, "Standby refused: HTTP %d", status);
            s_rng_next_us = now + 60 * 1000000LL;
        }
        sent = true;
    }

    if (timed_out) {
        ESP_LOGW(TAG, "Nobody continued within %d min -- cancelling print", CONFIG_HOLD_TIMEOUT_MIN);
        hub_client_send_event(HUB_EVENT_HOLD_TIMEOUT, "", fn);
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        http_post("/printer/print/cancel", CMD_TIMEOUT_MS);
        xSemaphoreGive(s_http_mutex);
        return true;
    }

    if (zcu) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "SET_GCODE_OFFSET Z_ADJUST=%.2f MOVE=1", zcu / 100.0f);
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        int status = gcode_script_locked(cmd);
        xSemaphoreGive(s_http_mutex);
        /* As with Marlin, a step that may have reached the printer is never
         * resent: only one that could not be sent at all stays pending */
        if (status >= 0) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_z_pending_cu -= zcu;
            if (status == 200 || status == 0) s_z_shift_cu += zcu;
            s_state.z_shift = s_z_shift_cu / 100.0f;
            s_state.z_pending = s_z_pending_cu / 100.0f;
            xSemaphoreGive(s_state_mutex);
            if (status != 200 && status != 0) ESP_LOGW(TAG, "Z step refused: HTTP %d", status);
        }
        sent = true;
    }

    if (ng_act == NG_ACT_PURGE) {
        char script[96];
        snprintf(script, sizeof(script), NG_SAVE "G1 E%u F180\n" NG_RESTORE, ngc.purge_mm);
        ESP_LOGI(TAG, "Wait: purging %u mm", ngc.purge_mm);
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        gcode_script_locked(script);
        xSemaphoreGive(s_http_mutex);
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        if (s_ng.active && s_state.hold_id == id) s_ng.last_purge_us = esp_timer_get_time();
        xSemaphoreGive(s_state_mutex);
        sent = true;
    } else if (ng_act == NG_ACT_STANDBY) {
        char script[128];
        snprintf(script, sizeof(script), NG_SAVE "G1 E-%.1f F1800\n" NG_RESTORE "\nM104 S%d",
                 NG_RETRACT_MM, ng_sb);
        ESP_LOGI(TAG, "Wait: nozzle %d -> %d C", target_c, ng_sb);
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        int status = gcode_script_locked(script);
        xSemaphoreGive(s_http_mutex);
        if (status == 200 || status < 0) {     /* -1: no answer, it may have run */
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            if (s_ng.active && s_state.hold_id == id) {
                s_ng.standby = true;
                s_ng.retracted = true;
                s_ng.restore_c = target_c;
                s_state.nozzle = NOZZLE_STANDBY;
            }
            xSemaphoreGive(s_state_mutex);
            ng_notify_standby(target_c, ng_sb, fn);
        } else {
            ESP_LOGW(TAG, "Standby refused: HTTP %d", status);
            s_ng_next_us = now + 60 * 1000000LL;
        }
        sent = true;
    }

    if (go) {
        ESP_LOGI(TAG, "Continue after hold %lu", (unsigned long)id);
        char script[200] = "M117\nM24";
        if (ng.standby && ng.restore_c > 0) {
            /* Heat back and wait, then push back what the retract pulled */
            if (ng.retracted)
                snprintf(script, sizeof(script), "M109 S%d\n" NG_SAVE "G1 E%.1f F1800\n" NG_RESTORE "\nM117\nM24",
                         ng.restore_c, NG_RETRACT_MM);
            else
                snprintf(script, sizeof(script), "M109 S%d\nM117\nM24", ng.restore_c);
            ESP_LOGI(TAG, "Heating back to %d C first", ng.restore_c);
        }
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        int status = gcode_script_locked(script);
        xSemaphoreGive(s_http_mutex);
        /* Heating outlasts the request: no answer means it is running, and a
         * second Continue would push the filament back twice */
        if (status < 0 && ng.standby) status = 200;
        if (status != 200) {
            ESP_LOGW(TAG, "Continue: HTTP %d", status);
            /* Allow another tap */
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            if (s_continue_sent_id == id) s_continue_sent_id = 0;
            if (s_continue_id == id) s_continue_id = 0;
            xSemaphoreGive(s_state_mutex);
        }
        sent = true;
    } else if (beep) {
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        gcode_script_locked(HOLD_BEEP_SCRIPT);
        xSemaphoreGive(s_http_mutex);
    }
    return sent;
}

static void klipper_poll_task(void *arg)
{
    ESP_LOGI(TAG, "Klipper polling task started → %s:%u", s_host, s_port);

    static const char *query_path =
        "/printer/objects/query?"
        "extruder=temperature,target&heater_bed=temperature,target"
        "&print_stats=state,filename,total_duration"
        "&virtual_sdcard=progress,file_position,file_size&toolhead=position,homed_axes,axis_maximum"
        "&bed_mesh=profile_name&gcode_move=homing_origin,gcode_position&fan=speed";
    static char path[512];

    while (true) {
        xSemaphoreTake(s_http_mutex, portMAX_DELAY);
        /* With the filament sensor, if the printer has one */
        if (s_runout_q[0])
            snprintf(path, sizeof(path), "%s&%s=filament_detected,enabled", query_path, s_runout_q);
        else
            strlcpy(path, query_path, sizeof(path));
        int status = http_get(path);
        if (status == 200) {
            ESP_LOGD(TAG, "Raw response (%d bytes): %.512s", (int)strlen(s_resp_buf), s_resp_buf);
            parse_moonraker_status(s_resp_buf);
            if (!s_objects_checked ||
                esp_timer_get_time() - s_objects_checked_us > OBJECTS_RECHECK_US)
                check_objects_locked();
            if (!s_hist_checked) check_history_locked();
        }
        xSemaphoreGive(s_http_mutex);

        if (status < 0) {
            /* Moonraker unreachable */
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.opstate = PRINTER_DISCONNECTED;
            xSemaphoreGive(s_state_mutex);
            s_objects_checked = false;
            ESP_LOGD(TAG, "Moonraker unreachable");
        } else if (status == 503) {
            /* Klipper MCU not connected */
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.opstate = PRINTER_ERROR;
            xSemaphoreGive(s_state_mutex);
            ESP_LOGW(TAG, "Moonraker returned 503 (Klipper MCU not connected?)");
        } else if (status != 200) {
            ESP_LOGW(TAG, "Moonraker returned HTTP %d", status);
        }

        if (status != 200) {
            s_was_down = true;
            s_objects_checked = false;      /* A Klipper restart may bring new macros */
            /* A restart puts the position at 0: unhomed, it is not known any more */
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_last_z_ok = false;
            xSemaphoreGive(s_state_mutex);
        } else if (s_was_down) {
            s_was_down = false;
            s_boot_watch_until = esp_timer_get_time() + BOOT_WATCH_US;
        }
        if (s_boot_mesh_dirty) {
            s_boot_mesh_dirty = false;
            nvs_handle_t nvs;
            if (nvs_open(NVS_NS_KLIPPER, NVS_READWRITE, &nvs) == ESP_OK) {
                nvs_set_str(nvs, NVS_KEY_BOOT_MESH, s_boot_mesh);
                nvs_commit(nvs);
                nvs_close(nvs);
            }
        }

        bool sent = status == 200 && klipper_tick();

        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        bool fast = s_state.holding || s_z_pending_cu != 0;
        xSemaphoreGive(s_state_mutex);
        int wait_ms = sent ? POLL_AFTER_CMD_MS : fast ? POLL_HOLD_MS : POLL_INTERVAL_MS;
        /* A Continue or a Z tap wakes the task at once */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(wait_ms));
    }
}

/* ---- Public API ---- */

esp_err_t klipper_backend_start(const char *host, uint16_t port)
{
    strncpy(s_host, host, sizeof(s_host) - 1);
    s_host[sizeof(s_host) - 1] = '\0';
    s_port = port;

    s_resp_buf = heap_caps_calloc(1, HTTP_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!s_resp_buf) return ESP_ERR_NO_MEM;
    s_holds = heap_caps_calloc(MAX_HOLDS, sizeof(klipper_hold_t), MALLOC_CAP_SPIRAM);
    if (!s_holds) return ESP_ERR_NO_MEM;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_KLIPPER, NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(s_boot_mesh);
        nvs_get_str(nvs, NVS_KEY_BOOT_MESH, s_boot_mesh, &len);
        int32_t um;
        if (nvs_get_i32(nvs, NVS_KEY_ZOFFSET, &um) == ESP_OK) {
            s_has_z = true;
            s_z_offset = um / 1000.0f;
        }
        nvs_close(nvs);
    }

    s_state_mutex = xSemaphoreCreateMutex();
    if (!s_state_mutex) return ESP_ERR_NO_MEM;

    s_http_mutex = xSemaphoreCreateMutex();
    if (!s_http_mutex) return ESP_ERR_NO_MEM;

    memset(&s_state, 0, sizeof(s_state));
    s_state.fan_pct = -1;
    s_state.motors = -1;
    s_state.opstate = PRINTER_DISCONNECTED;
    s_state.progress_pct = -1;
    s_state.print_time_s = -1;
    s_state.print_time_left_s = -1;
    s_state.total_layers = -1;
    s_meta_filename[0] = '\0';

    task_create_psram(klipper_poll_task, "klipper_poll", 12288, NULL, 8, &s_task_handle, 0);

    ESP_LOGI(TAG, "Klipper/Moonraker backend started → %s:%u", s_host, s_port);
    return ESP_OK;
}

void klipper_backend_stop(void)
{
    if (s_task_handle) {
        task_delete(s_task_handle);
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

esp_err_t klipper_backend_continue(uint32_t hold_id)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    bool ok = s_state.holding && s_state.hold_id == hold_id;
    if (ok) s_continue_id = hold_id;
    xSemaphoreGive(s_state_mutex);
    if (!ok) {
        ESP_LOGI(TAG, "Ignoring continue for hold %lu", (unsigned long)hold_id);
        return ESP_ERR_INVALID_STATE;
    }
    xTaskNotifyGive(s_task_handle);
    return ESP_OK;
}

esp_err_t klipper_backend_lift_script(float mm, char *buf, size_t size)
{
    esp_err_t err = ESP_OK;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    bool homed = s_homed_z;
    float z = homed ? s_toolhead_z : s_last_z;
    if (!homed && !s_last_z_ok) {
        err = ESP_ERR_INVALID_STATE;
    } else {
        float d = mm;
        if (s_zmax > 0 && z + d > s_zmax - LIFT_TOP_MARGIN) d = s_zmax - LIFT_TOP_MARGIN - z;
        if (d < 1) {
            err = ESP_ERR_INVALID_SIZE;
        } else if (homed) {
            snprintf(buf, size, "G91\nG1 Z%.1f F%d\nG90", d, LIFT_FEED);
        } else {
            /* force_move's SET_KINEMATIC_POSITION marks every axis homed at
             * the old position; M84 afterwards takes that back */
            snprintf(buf, size, "SET_KINEMATIC_POSITION Z=%.3f\nG91\nG1 Z%.1f F%d\nG90\nM84",
                     z, d, LIFT_FEED);
        }
    }
    xSemaphoreGive(s_state_mutex);
    return err;
}

esp_err_t klipper_backend_zstep(float dz)
{
    int cu = (int)lroundf(dz * 100.0f);
    esp_err_t err = ESP_OK;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (s_state.opstate != PRINTER_PRINTING && !s_state.holding) {
        err = ESP_ERR_INVALID_STATE;
    } else if (abs(s_z_shift_cu + s_z_pending_cu + cu) > z_shift_limit_cu(s_state.builtin_script)) {
        err = ESP_ERR_INVALID_SIZE;
    } else {
        s_z_pending_cu += cu;
        s_state.z_pending = s_z_pending_cu / 100.0f;
    }
    xSemaphoreGive(s_state_mutex);
    if (err == ESP_OK) xTaskNotifyGive(s_task_handle);
    return err;
}

void klipper_backend_get_state(printer_state_t *out)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    memcpy(out, &s_state, sizeof(printer_state_t));
    xSemaphoreGive(s_state_mutex);
    out->upload_pct = (int8_t)s_upload_pct;
}

esp_err_t klipper_backend_send_cmd(const printer_cmd_t *cmd)
{
    int status;

    static const char *cmd_names[] = { "pause", "resume", "cancel", "raw" };
    ESP_LOGI(TAG, "CMD: %s%s%s", cmd_names[cmd->type],
             cmd->type == PCMD_RAW ? " → " : "",
             cmd->type == PCMD_RAW ? cmd->gcode : "");

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    int target_c = (int)(s_state.hotend_target + 0.5f);
    int restore_c = s_pause_restore_c;
    xSemaphoreGive(s_state_mutex);

    switch (cmd->type) {
    case PCMD_PAUSE:
        status = http_post("/printer/print/pause", CMD_TIMEOUT_MS);
        if (cmd->standby && (status == 200 || status < 0)) {
            /* A pause from Obico: the nozzle down at once (the Neptune 4's PAUSE
             * has parked and retracted, and saved the temperature for RESUME) */
            int sb = ng_standby_c(target_c);
            if (sb) {
                char script[24];
                snprintf(script, sizeof(script), "M104 S%d", sb);
                ESP_LOGI(TAG, "Pause: nozzle %d -> %d C", target_c, sb);
                gcode_script_locked(script);
                xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                s_pause_restore_c = target_c;
                s_state.nozzle = NOZZLE_STANDBY;
                xSemaphoreGive(s_state_mutex);
            }
        }
        break;
    case PCMD_RESUME:
        if (restore_c > 0) {
            /* Heat back first: not every RESUME macro does.  The answer comes
             * when RESUME is done, after the heating */
            char script[48];
            snprintf(script, sizeof(script), "M109 S%d\nRESUME", restore_c);
            ESP_LOGI(TAG, "Resume: heating back to %d C first", restore_c);
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_state.nozzle = NOZZLE_HEATING;
            xSemaphoreGive(s_state_mutex);
            status = gcode_script_locked(script);
            if (status < 0) status = 200;   /* No answer yet: it is heating */
        } else {
            status = http_post("/printer/print/resume", CMD_TIMEOUT_MS);
        }
        break;
    case PCMD_CANCEL:
        status = http_post("/printer/print/cancel", CMD_TIMEOUT_MS);
        break;
    case PCMD_RAW:
        status = gcode_script_locked(cmd->gcode);
        break;
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

/* Upload body writer for xform_file(): the HTTP client sends small writes
 * as separate TCP segments, so the output is gathered first */
typedef struct {
    esp_http_client_handle_t client;
    char *buf;
    size_t used;
    size_t sent;
    size_t total;
    bool failed;
} upload_out_t;

static void upload_flush(upload_out_t *u)
{
    if (u->used && !u->failed) {
        if (esp_http_client_write(u->client, u->buf, u->used) < 0) {
            ESP_LOGE(TAG, "Upload write failed");
            u->failed = true;
        } else {
            u->sent += u->used;
            if (u->total) s_upload_pct = (int)((uint64_t)u->sent * 100 / u->total);
        }
    }
    u->used = 0;
}

static void upload_emit(const char *data, size_t len, void *ctx)
{
    upload_out_t *u = ctx;
    while (len > 0 && !u->failed) {
        size_t k = UPLOAD_CHUNK - u->used;
        if (k > len) k = len;
        memcpy(u->buf + u->used, data, k);
        u->used += k;
        data += k;
        len -= k;
        if (u->used == UPLOAD_CHUNK) upload_flush(u);
    }
}

/** Upload filepath as filename, size bytes after the stops are rewritten.
 *  Must be called with s_http_mutex held. */
static esp_err_t upload_file_locked(const src_t *src, const char *filename, uint32_t size,
                                    bool has_z, float z)
{
    ESP_LOGI(TAG, "Uploading %s (%lu bytes) to Moonraker", filename, (unsigned long)size);

    static const char *boundary = "----ESP32FDMUpload";

    char part_header[256];
    int hdr_len = snprintf(part_header, sizeof(part_header),
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"file\"; filename=\"%s\"\r\n"
        "Content-Type: application/octet-stream\r\n"
        "\r\n",
        boundary, filename);

    char part_footer[64];
    int ftr_len = snprintf(part_footer, sizeof(part_footer), "\r\n--%s--\r\n", boundary);

    char url[256];
    snprintf(url, sizeof(url), "http://%s:%u/server/files/upload", s_host, s_port);

    char content_type[64];
    snprintf(content_type, sizeof(content_type), "multipart/form-data; boundary=%s", boundary);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 60000,
    };

    upload_out_t u = { .total = size };
    u.buf = heap_caps_malloc(UPLOAD_CHUNK, MALLOC_CAP_SPIRAM);
    kh_xform_t *x = xform_alloc();
    u.client = esp_http_client_init(&config);
    if (!u.buf || !x || !u.client) {
        if (u.client) esp_http_client_cleanup(u.client);
        free(x);
        free(u.buf);
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_header(u.client, "Content-Type", content_type);

    int64_t t0 = esp_timer_get_time();
    esp_err_t err = esp_http_client_open(u.client, hdr_len + (int)size + ftr_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Upload open failed: %s", esp_err_to_name(err));
    } else {
        esp_http_client_write(u.client, part_header, hdr_len);
        kh_init(x, NULL, 0, upload_emit, &u);
        x->has_z = has_z;
        x->z = z;
        err = xform_src(src, x);
        upload_flush(&u);
        if (err == ESP_OK && (u.failed || u.sent != size)) {
            /* The file changed since it was measured, or the connection broke */
            ESP_LOGE(TAG, "Upload sent %u of %lu bytes", (unsigned)u.sent, (unsigned long)size);
            err = ESP_FAIL;
        }
        if (err == ESP_OK) {
            esp_http_client_write(u.client, part_footer, ftr_len);
            esp_http_client_fetch_headers(u.client);
            int status = esp_http_client_get_status_code(u.client);
            if (status != 201 && status != 200) {
                ESP_LOGE(TAG, "Upload failed: HTTP %d", status);
                err = ESP_FAIL;
            } else {
                int64_t ms = (esp_timer_get_time() - t0) / 1000;
                ESP_LOGI(TAG, "Upload OK (HTTP %d): %lu KB in %lld s, %lld KB/s", status,
                         (unsigned long)(size / 1024), ms / 1000, ms ? (int64_t)size * 1000 / 1024 / ms : 0);
            }
        }
        esp_http_client_close(u.client);
    }
    esp_http_client_cleanup(u.client);
    free(x);
    free(u.buf);
    return err;
}

/** Whether a print_stats query response shows filename printing */
static bool print_started(const char *json, const char *filename)
{
    cJSON *root = cJSON_Parse(json);
    cJSON *ps = cJSON_GetObjectItem(cJSON_GetObjectItem(cJSON_GetObjectItem(root, "result"), "status"),
                                    "print_stats");
    cJSON *st = cJSON_GetObjectItem(ps, "state");
    cJSON *fn = cJSON_GetObjectItem(ps, "filename");
    bool ok = cJSON_IsString(st) && cJSON_IsString(fn) && strcmp(fn->valuestring, filename) == 0 &&
              (strcmp(st->valuestring, "printing") == 0 || strcmp(st->valuestring, "paused") == 0);
    cJSON_Delete(root);
    return ok;
}

/**
 * Send src to Moonraker as filename and start it.  A file on the SD card is
 * not sent again if Moonraker has one of that name and size, and replaces
 * the last one sent; a script is always sent.  Slow: reads src twice.
 */
static int s_sending;

static esp_err_t send_and_start_inner(const src_t *src, const char *filename);

static esp_err_t send_and_start(const src_t *src, const char *filename)
{
    __atomic_fetch_add(&s_sending, 1, __ATOMIC_SEQ_CST);
    s_upload_pct = 0;
    esp_err_t err = send_and_start_inner(src, filename);
    s_upload_pct = -1;
    __atomic_fetch_sub(&s_sending, 1, __ATOMIC_SEQ_CST);
    return err;
}

bool klipper_backend_get_zoffset(float *z)
{
    *z = s_z_offset;
    return s_has_z;
}

esp_err_t klipper_backend_set_zoffset(bool set, float z)
{
    if (set && (z < -5.0f || z > 5.0f)) return ESP_ERR_INVALID_ARG;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NS_KLIPPER, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = set ? nvs_set_i32(nvs, NVS_KEY_ZOFFSET, (int32_t)lroundf(z * 1000.0f))
              : nvs_erase_key(nvs, NVS_KEY_ZOFFSET);
    if (err == ESP_ERR_NVS_NOT_FOUND) err = ESP_OK;
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    if (err == ESP_OK) {
        s_has_z = set;
        s_z_offset = set ? lroundf(z * 1000.0f) / 1000.0f : 0.0f;
        if (set) ESP_LOGI(TAG, "Z offset for prints: %.3f", s_z_offset);
        else ESP_LOGI(TAG, "Z offset for prints: the printer's own");
    }
    return err;
}

bool klipper_backend_has_light(void)
{
    return s_has_light;
}

bool klipper_backend_sending(void)
{
    return s_sending > 0 || s_script_starting;
}

static esp_err_t send_and_start_inner(const src_t *src, const char *filename)
{
    bool from_sd = src->mem == NULL;

    /* First pass, before taking the HTTP lock so that polling goes on: the
     * size of the file as sent and where its stops for the user end up */
    klipper_hold_t *holds = heap_caps_calloc(MAX_HOLDS, sizeof(klipper_hold_t), MALLOC_CAP_SPIRAM);
    kh_xform_t *x = xform_alloc();
    if (!holds || !x) {
        free(x);
        free(holds);
        return ESP_ERR_NO_MEM;
    }
    /* The same offset for both passes, or the size would not match */
    float z;
    bool has_z = klipper_backend_get_zoffset(&z);
    kh_init(x, holds, MAX_HOLDS, NULL, NULL);
    x->has_z = has_z;
    x->z = z;
    esp_err_t err = xform_src(src, x);
    uint32_t size = x->out_pos;
    int n_holds = x->n_holds, n_seen = x->n_seen;
    free(x);
    if (err != ESP_OK || size == 0) {
        ESP_LOGE(TAG, "Cannot read %s", from_sd ? src->path : filename);
        free(holds);
        return err != ESP_OK ? err : ESP_ERR_NOT_FOUND;
    }
    if (n_seen > n_holds) {
        ESP_LOGW(TAG, "%s: %d stops for the user, only the first %d are known", filename, n_seen, n_holds);
    } else if (n_seen) {
        ESP_LOGI(TAG, "%s: %d stops for the user", filename, n_seen);
    }

    xSemaphoreTake(s_http_mutex, portMAX_DELAY);

    if (from_sd) {
        /* Delete previous uploaded file if it's different from what we're about to print */
        if (s_last_uploaded[0] && strcmp(s_last_uploaded, filename) != 0) {
            moonraker_delete_file(s_last_uploaded);
            s_last_uploaded[0] = '\0';
        }
    }

    /* Moonraker may have it from an earlier print: compare the size as sent.
     * Not for a script, whose numbers change without changing its size. */
    if (!from_sd || !moonraker_file_matches(filename, (long)size)) {
        err = upload_file_locked(src, filename, size, has_z, z);
        if (err != ESP_OK) {
            xSemaphoreGive(s_http_mutex);
            free(holds);
            return err;
        }
    }

    /* Known before the print starts, since a stop may come right at the start */
    install_holds(filename, holds, n_holds, size);
    free(holds);

    /* Start print */
    char encoded[128];
    url_encode_filename(encoded, sizeof(encoded), filename);

    char path[256];
    snprintf(path, sizeof(path), "/printer/print/start?filename=%s", encoded);
    int status = http_post(path, CMD_TIMEOUT_MS);
    xSemaphoreGive(s_http_mutex);

    /* The Neptune 4 answers the start late (after the file's first commands,
     * or 20 s with the bed heating), and shows the print later still: watch
     * for it a while before calling it failed */
    if (status != 200) {
        ESP_LOGW(TAG, "Print start: HTTP %d, watching for it", status);
        for (int i = 0; i < START_WATCH_S / 2 && status != 200; i++) {
            vTaskDelay(pdMS_TO_TICKS(2000));
            xSemaphoreTake(s_http_mutex, portMAX_DELAY);
            if (http_get("/printer/objects/query?print_stats=state,filename") == 200 &&
                print_started(s_resp_buf, filename)) {
                status = 200;
            }
            xSemaphoreGive(s_http_mutex);
        }
    }

    if (status != 200) {
        ESP_LOGE(TAG, "Print start failed: HTTP %d", status);
        return ESP_FAIL;
    }

    if (from_sd) strlcpy(s_last_uploaded, filename, sizeof(s_last_uploaded));

    ESP_LOGI(TAG, "Print started: %s", filename);
    return ESP_OK;
}

esp_err_t klipper_backend_print_file(const char *filename)
{
    if (!filename || !filename[0]) return ESP_ERR_INVALID_ARG;

    char filepath[300];
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    src_t src = { .path = filepath };
    return send_and_start(&src, filename);
}

/* ---- Built-in scripts ---- */

typedef struct {
    char name[48];
    char *script;
    host_script_done_t done;
    int done_arg;
} script_job_t;


static void script_task(void *arg)
{
    script_job_t *job = arg;
    char filename[sizeof(s_script_file)];
    snprintf(filename, sizeof(filename), "%s.gcode", job->name);

    /* Known before the start, so that the end is seen however soon it comes */
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    strlcpy(s_script_file, filename, sizeof(s_script_file));
    s_script_done = job->done;
    s_script_arg = job->done_arg;
    s_script_running = false;
    xSemaphoreGive(s_state_mutex);

    src_t src = { .mem = job->script, .mem_len = strlen(job->script) };
    esp_err_t err = send_and_start(&src, filename);

    /* Not seen to start: it stays registered, so that a start later still is
     * picked up by its file name in the poll */
    if (err == ESP_OK) {
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_script_running = true;
        xSemaphoreGive(s_state_mutex);
    }

    if (err != ESP_OK) ESP_LOGE(TAG, "%s: not started", job->name);
    free(job->script);
    free(job);
    s_script_starting = false;
    task_delete(NULL);
}

esp_err_t klipper_backend_print_script(const char *name, char *script,
                                       host_script_done_t done, int done_arg)
{
    if (s_script_starting) {
        free(script);
        return ESP_ERR_INVALID_STATE;
    }
    script_job_t *job = calloc(1, sizeof(*job));
    if (!job) {
        free(script);
        return ESP_ERR_NO_MEM;
    }
    strlcpy(job->name, name, sizeof(job->name));
    job->script = script;
    job->done = done;
    job->done_arg = done_arg;

    s_script_starting = true;
    /* Its own task: the upload is quick, but the Neptune 4 answers the start
     * only after the G28 at the top of the script */
    if (task_create_psram(script_task, "klipper_script", 12288, job, 5, NULL, 0) != pdPASS) {
        s_script_starting = false;
        free(script);
        free(job);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
