#include "httpd.h"
#include "task_util.h"

#include <string.h>
#include <stdlib.h>

#include "camera.h"
#include "wifi.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_memory_utils.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "sdcard.h"
#include "sdcard_httpd.h"
#include "printer_comm.h"
#include "plastics.h"
#include "printer_comm_klipper.h"
#include "usb_serial.h"
#include "layout.h"

#include "obico_client.h"
#include "terminal.h"
#include "bedlevel.h"
#include "url_util.h"
#include "logbuf.h"
#include "ota.h"
#include "hub_client.h"
#include "print_page.h"
#include "cold_pull.h"
#include "filament.h"
#include "maintenance.h"
#include "settings_copy.h"
#include "board_led.h"
#include "nozzle_guard_ui.h"
#include "print_checks.h"
#include "wifi_page.h"
#include "print_stats.h"

static const char *TAG = "httpd";

/* Forward declaration — used by api_command_handler and others */
static bool parse_form_field(const char *body, const char *name,
                             char *out, size_t out_size);

/* Baby-step Z offset — seeded from printer (M290 query), updated locally on adjustments.
 * Re-synced from printer_state_t.probe_z_offset when the printer is queried. */
static float s_baby_z = 0.0f;
static bool  s_baby_z_seeded = false;  /* true once we got a real value from the printer */

#define STREAM_TARGET_FPS 5
#define STREAM_FRAME_INTERVAL_US (1000000 / STREAM_TARGET_FPS)

static const char *STREAM_BOUNDARY = "esp32fdm_frame";

/* ---- /capture handler (async) ---- */

static void capture_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;

    camera_frame_t *frame = camera_get_frame();
    if (!frame) {
        httpd_resp_send_500(req);
    } else {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Disposition",
                           "inline; filename=capture.jpg");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        httpd_resp_send(req, (const char *)frame->buf, frame->len);
        camera_release_frame(frame);
    }

    httpd_req_async_handler_complete(req);
    task_delete(NULL);
}

/* Inline SVG favicon: 3D printer hotend silhouette */
static const char FAVICON_SVG[] =
    "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 32 32'>"
    "<rect x='4' y='2' width='24' height='6' rx='1' fill='#555'/>"
    "<rect x='6' y='8' width='20' height='4' fill='#777'/>"
    "<rect x='13' y='12' width='6' height='8' fill='#555'/>"
    "<polygon points='12,20 20,20 18,26 14,26' fill='#d4a017'/>"
    "<circle cx='16' cy='28' r='2' fill='#e74c3c'/>"
    "</svg>";

static esp_err_t favicon_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "image/svg+xml");
    httpd_resp_set_hdr(req, "Cache-Control", "public, max-age=86400");
    return httpd_resp_send(req, FAVICON_SVG, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t capture_handler(httpd_req_t *req)
{
    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (task_create_psram(capture_task, "capture", 8192, async_req,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- /stream handler (MJPEG multipart) ---- */

/* Stream task runs in background, freeing the httpd thread */
static void stream_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;
    int fd = httpd_req_to_sockfd(req);

    ESP_LOGI(TAG, "Stream client connected");

    char part_header[128];
    int64_t last_frame_time = 0;

    while (true) {
        /* Frame rate limiter */
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - last_frame_time;
        if (elapsed < STREAM_FRAME_INTERVAL_US) {
            vTaskDelay(pdMS_TO_TICKS((STREAM_FRAME_INTERVAL_US - elapsed) / 1000));
        }

        camera_frame_t *frame = camera_get_frame();
        last_frame_time = esp_timer_get_time();
        if (!frame) {
            ESP_LOGW(TAG, "Stream: no frame available, retrying...");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int hdr_len = snprintf(part_header, sizeof(part_header),
                               "--%s\r\n"
                               "Content-Type: image/jpeg\r\n"
                               "Content-Length: %u\r\n"
                               "\r\n",
                               STREAM_BOUNDARY, (unsigned)frame->len);

        /* Send part header + JPEG + trailing CRLF via raw socket */
        if (send(fd, part_header, hdr_len, 0) <= 0 ||
            send(fd, (const char *)frame->buf, frame->len, 0) <= 0 ||
            send(fd, "\r\n", 2, 0) <= 0) {
            camera_release_frame(frame);
            break;
        }
        camera_release_frame(frame);
    }

    ESP_LOGI(TAG, "Stream client disconnected");
    httpd_req_async_handler_complete(req);
    task_delete(NULL);
}

static esp_err_t stream_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);

    /* TCP_NODELAY for low-latency frame delivery */
    int nodelay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    /* Send HTTP response header before handing off to background task */
    const char *resp_hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace;boundary=esp32fdm_frame\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "X-Framerate: 5\r\n"
        "\r\n";
    if (httpd_send(req, resp_hdr, strlen(resp_hdr)) <= 0) {
        return ESP_FAIL;
    }

    /* Clone the request for async use and spawn a background task */
    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start async stream handler");
        return ESP_FAIL;
    }

    if (task_create_psram(stream_task, "stream", 8192, async_req,
                                 5, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create stream task");
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- GET /api/status — JSON for status bar + dashboard ---- */

static esp_err_t api_status_handler(httpd_req_t *req)
{
    printer_state_t st;
    printer_comm_get_state(&st);

    const char *state_str;
    switch (st.opstate) {
        case PRINTER_PRINTING: state_str = "printing"; break;
        case PRINTER_PAUSED:   state_str = "paused"; break;
        case PRINTER_ERROR:    state_str = "error"; break;
        case PRINTER_OPERATIONAL: state_str = "operational"; break;
        default: state_str = "disconnected"; break;
    }

    bool usb = usb_serial_is_connected();
    int32_t layer = st.current_layer;
    int32_t total_layers = st.total_layers;

    /* Sync baby Z from printer when available (probe_z_offset is 0 until M290 response).
     * Klipper reports its gcode offset with every poll: always take that. */
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        s_baby_z = st.probe_z_offset;
    } else if (!s_baby_z_seeded && st.probe_z_offset != 0.0f) {
        s_baby_z = st.probe_z_offset;
        s_baby_z_seeded = true;
    }

    char pending[32];
    ota_get_pending(pending, sizeof(pending));
    print_stats_t ps;
    print_stats_get(&ps);

    char json[1024];
    snprintf(json, sizeof(json),
        "{\"state\":\"%s\",\"backend\":\"%s\",\"hotend\":[%.1f,%.1f],\"bed\":[%.1f,%.1f],"
        "\"progress\":%.1f,\"filename\":\"%s\",\"elapsed\":%ld,\"remaining\":%ld,"
        "\"layer\":\"%ld/%ld\",\"z\":%.2f,\"baby_z\":%.2f,\"usb\":%s,"
        "\"name\":\"%s\",\"preparing\":%s,\"hold\":%s,\"hold_id\":%lu,\"hold_msg\":\"%s\","
        "\"z_shift\":%.2f,\"z_pending\":%.2f,\"notify\":\"%s\",\"update_pending\":\"%s\","
        "\"print_hours\":%.1f,\"prints\":%lu,\"upload\":%d,\"fan\":%d,\"motors\":%d,\"plastic\":\"%s\",\"nozzle\":\"%s\",\"version\":\"%s\",\"build\":\"%s\"}",
        state_str, printer_comm_backend_name(),
        st.hotend_actual, st.hotend_target,
        st.bed_actual, st.bed_target,
        st.progress_pct >= 0 ? st.progress_pct : 0.0f,
        st.filename,
        (long)(st.print_time_s >= 0 ? st.print_time_s : 0),
        (long)(st.print_time_left_s >= 0 ? st.print_time_left_s : 0),
        (long)(layer >= 0 ? layer : 0),
        (long)(total_layers >= 0 ? total_layers : 0),
        st.z, s_baby_z,
        usb ? "true" : "false",
        wifi_get_hostname(),
        st.preparing ? "true" : "false",
        st.holding ? "true" : "false",
        (unsigned long)st.hold_id,
        st.hold_msg,
        st.z_shift,
        st.z_pending,
        hub_client_notify_status(),
        pending,
        ps.print_s / 3600.0, (unsigned long)ps.started,
        printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER ? st.upload_pct : -1,
        (int)st.fan_pct, (int)st.motors, plastic_loaded() >= 0 ? PLASTICS[plastic_loaded()].name : "",
        st.nozzle == NOZZLE_STANDBY ? "standby" : st.nozzle == NOZZLE_HEATING ? "heating" : "hot",
        FW_VERSION, FW_BUILD);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
}

/* ---- GET /api/heap — memory, for diagnostics ---- */

static esp_err_t api_heap_handler(httpd_req_t *req)
{
    char json[320];
    snprintf(json, sizeof(json),
             "{\"internal\":{\"free\":%u,\"largest\":%u,\"min\":%u},"
             "\"dma\":{\"free\":%u,\"largest\":%u,\"min\":%u},"
             "\"psram\":{\"free\":%u,\"largest\":%u},\"uptime_s\":%lld}",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_DMA),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
             (long long)(esp_timer_get_time() / 1000000));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

/* ---- GET /api/tasks — tasks and their stacks, for diagnostics ---- */

static esp_err_t api_tasks_handler(httpd_req_t *req)
{
    UBaseType_t n = uxTaskGetNumberOfTasks() + 4;
    TaskStatus_t *ts = heap_caps_malloc(n * sizeof(TaskStatus_t), MALLOC_CAP_SPIRAM);
    if (!ts) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    n = uxTaskGetSystemState(ts, n, NULL);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr_chunk(req, "[");
    for (UBaseType_t i = 0; i < n; i++) {
        char row[160];
        snprintf(row, sizeof(row), "%s{\"name\":\"%s\",\"prio\":%u,\"core\":%d,\"free_min\":%u,\"psram\":%s}",
                 i ? "," : "", ts[i].pcTaskName, (unsigned)ts[i].uxCurrentPriority,
                 xTaskGetCoreID(ts[i].xHandle) == tskNO_AFFINITY ? -1 : (int)xTaskGetCoreID(ts[i].xHandle),
                 (unsigned)ts[i].usStackHighWaterMark,
                 esp_ptr_external_ram(ts[i].pxStackBase) ? "true" : "false");
        httpd_resp_sendstr_chunk(req, row);
    }
    free(ts);
    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_sendstr_chunk(req, NULL);
}

/* ---- GET /api/stats — per-core CPU usage ---- */

/* Accumulated idle ticks from previous call, per core */
static uint32_t s_prev_idle[2];
static uint32_t s_prev_total;
static bool     s_stats_initialized;

static esp_err_t api_stats_handler(httpd_req_t *req)
{
#if defined(CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS)
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_array = malloc(num_tasks * sizeof(TaskStatus_t));
    if (!task_array) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    uint32_t total_runtime;
    UBaseType_t count = uxTaskGetSystemState(task_array, num_tasks, &total_runtime);

    /* Sum idle task runtime per core (IDLE0, IDLE1) */
    uint32_t idle_now[2] = {0, 0};
    for (UBaseType_t i = 0; i < count; i++) {
        const char *name = task_array[i].pcTaskName;
        if (strcmp(name, "IDLE0") == 0 || strcmp(name, "IDLE") == 0)
            idle_now[0] = task_array[i].ulRunTimeCounter;
        else if (strcmp(name, "IDLE1") == 0)
            idle_now[1] = task_array[i].ulRunTimeCounter;
    }

    int cpu0_pct = 0, cpu1_pct = 0;
    if (s_stats_initialized && total_runtime > s_prev_total) {
        uint32_t dt = total_runtime - s_prev_total;
        /* total_runtime is wall-clock timer ticks (same for both cores) */
        if (dt > 0) {
            uint32_t idle0_delta = idle_now[0] - s_prev_idle[0];
            uint32_t idle1_delta = idle_now[1] - s_prev_idle[1];
            cpu0_pct = 100 - (int)(idle0_delta * 100 / dt);
            cpu1_pct = 100 - (int)(idle1_delta * 100 / dt);
            if (cpu0_pct < 0) cpu0_pct = 0;
            if (cpu1_pct < 0) cpu1_pct = 0;
            if (cpu0_pct > 100) cpu0_pct = 100;
            if (cpu1_pct > 100) cpu1_pct = 100;
        }
    }
    s_prev_idle[0] = idle_now[0];
    s_prev_idle[1] = idle_now[1];
    s_prev_total = total_runtime;
    s_stats_initialized = true;

    free(task_array);

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_heap = esp_get_minimum_free_heap_size();

    char json[128];
    snprintf(json, sizeof(json),
        "{\"cpu0\":%d,\"cpu1\":%d,\"heap\":%lu,\"heap_min\":%lu}",
        cpu0_pct, cpu1_pct, (unsigned long)free_heap, (unsigned long)min_heap);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json, strlen(json));
#else
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"error\":\"stats not enabled\"}", HTTPD_RESP_USE_STRLEN);
#endif
}

/* ---- GET /api/temp_history — temperature history for chart ---- */

static esp_err_t api_temp_history_handler(httpd_req_t *req)
{
    /* Allocate on heap — samples are 24 bytes each, 900 * 24 = ~21KB */
    temp_sample_t *samples = malloc(TEMP_HISTORY_MAX * sizeof(temp_sample_t));
    if (!samples) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int count = printer_comm_get_temp_history(samples, TEMP_HISTORY_MAX);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    /* Use chunked response to avoid a huge JSON buffer */
    char chunk[128];
    httpd_resp_sendstr_chunk(req, "{\"samples\":[");

    for (int i = 0; i < count; i++) {
        temp_sample_t *s = &samples[i];
        int64_t ts_sec = s->timestamp_us / 1000000;
        int len = snprintf(chunk, sizeof(chunk),
            "%s[%lld,%.1f,%.0f,%.1f,%.0f]",
            i > 0 ? "," : "",
            (long long)ts_sec,
            s->hotend_actual, s->hotend_target,
            s->bed_actual, s->bed_target);
        httpd_resp_send_chunk(req, chunk, len);
    }

    httpd_resp_sendstr_chunk(req, "]}");
    httpd_resp_send_chunk(req, NULL, 0);  /* finish chunked response */

    free(samples);
    return ESP_OK;
}

/* ---- POST /api/command — send raw GCode ---- */

static esp_err_t api_command_handler(httpd_req_t *req)
{
    char buf[128];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    char cmd[96] = {0};
    if (!parse_form_field(buf, "cmd", cmd, sizeof(cmd)) || cmd[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "cmd required");
        return ESP_FAIL;
    }

    /* Track baby-step Z offset accumulator */
    const char *m290 = strcasestr(cmd, "M290");
    if (m290) {
        const char *zp = strchr(m290, 'Z');
        if (!zp) zp = strchr(m290, 'z');
        if (zp) s_baby_z += strtof(zp + 1, NULL);
    }

    printer_cmd_t pcmd = { .type = PCMD_RAW };
    strncpy(pcmd.gcode, cmd, sizeof(pcmd.gcode) - 1);
    printer_comm_send_cmd(&pcmd);

    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "OK", 2);
}

/* ---- /api/zoffset — the board's Z offset for prints (Klipper) ---- */

static esp_err_t zoffset_reply(httpd_req_t *req)
{
    float z;
    bool saved = klipper_backend_get_zoffset(&z);
    char json[64];
    snprintf(json, sizeof(json), "{\"saved\":%s,\"z\":%.3f}", saved ? "true" : "false", saved ? z : 0.0f);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t api_zoffset_get_handler(httpd_req_t *req)
{
    return zoffset_reply(req);
}

/* save=1: the gcode offset Klipper has now; clear=1: forget it */
static esp_err_t api_zoffset_post_handler(httpd_req_t *req)
{
    if (printer_comm_get_backend() != PRINTER_BACKEND_KLIPPER)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Klipper only");
    char body[32] = "";
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    body[len > 0 ? len : 0] = '\0';
    esp_err_t err = ESP_ERR_INVALID_ARG;
    if (strstr(body, "save=1")) {
        printer_state_t st;
        printer_comm_get_state(&st);
        if (st.opstate == PRINTER_DISCONNECTED || st.opstate == PRINTER_ERROR)
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Klipper not ready");
        err = klipper_backend_set_zoffset(true, st.probe_z_offset);
    } else if (strstr(body, "clear=1")) {
        err = klipper_backend_set_zoffset(false, 0);
    }
    if (err != ESP_OK) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(err));
    return zoffset_reply(req);
}

void httpd_note_babystep(float dz)
{
    s_baby_z += dz;
}

/* ---- / root handler — dashboard ---- */

static esp_err_t root_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);

    layout_html_begin(&p, NT_("ESP32 FDM Dashboard"), "/");

    html_buf_printf(&p,
        "<style>"
        ".ctrl{margin:8px 0}"
        ".ctrl summary{font-weight:bold;cursor:pointer;padding:8px;background:#f5f5f5;border-radius:4px;user-select:none}"
        ".ctrl .cg{padding:8px 0}"
        ".ctrl label{font-size:13px;font-weight:bold;display:block;margin:6px 0 2px}"
        ".bg{display:flex;flex-wrap:wrap;gap:4px;align-items:center}"
        ".bg button{min-width:44px;padding:6px 8px;font-size:13px}"
        ".bg input[type=number]{width:64px;padding:5px;font-size:13px}"
        ".bg .val{min-width:44px;text-align:center;font-weight:bold;font-size:14px}"
        "</style>");

    /* Controls panel */
    html_buf_printf(&p,
        i18n_page(
        "<details class='ctrl'><summary>" N_("Z offset (baby stepping)") "</summary><div class='cg'>"
        "<div class='bg'>"
        "<button onclick=\"zb(-0.05)\">\xe2\x96\xbc 0.05</button>"
        "<button onclick=\"zb(-0.01)\">\xe2\x96\xbc 0.01</button>"
        "<span class='val' id='zof'>%.2f</span>"),
        s_baby_z);
    html_buf_printf(&p,
        i18n_page(
        "<button onclick=\"zb(0.01)\">\xe2\x96\xb2 0.01</button>"
        "<button onclick=\"zb(0.05)\">\xe2\x96\xb2 0.05</button>"));
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        /* No M290 and no M500 in Klipper: the gcode offset, kept until Klipper
         * restarts.  Saved on the board, it goes into each file sent, at
         * "; @zoffset" in the start G-code (klipper_holds.h). */
        html_buf_printf(&p, "%s", i18n_page(
            "<button onclick=\"zsave(1)\" style='margin-left:8px'>" N_("Save for prints") "</button>"
            "<button onclick=\"zsave(0)\">" N_("Forget") "</button>"
            "</div>"
            "<p class='hint' style='margin:4px 0 0'>\xe2\x96\xbc " N_("closer to the bed") " &middot; \xe2\x96\xb2 " N_("farther from the bed") "</p>"
            "<p class='hint' style='margin:4px 0 0' id='zsv'></p>"
            "<p class='hint' style='margin:4px 0 0'>" N_("Saved, it is put into each file the board sends, at the start G-code's ; @zoffset line. "
            "Not saved, the printer's own offset applies: the Neptune 4's screen sets it at power-on.") "</p>"
            "</div></details>"
            "<script>"
            "function zshow(j){document.getElementById('zsv').textContent=j.saved?'" N_("Saved for prints:") " '+j.z.toFixed(2)"
            ":'" N_("Not saved: prints use the offset the printer sets") "'}"
            "function zsave(on){fetch('/api/zoffset',{method:'POST',body:on?'save=1':'clear=1'})"
            ".then(function(r){return r.json()}).then(zshow)}"
            "fetch('/api/zoffset').then(function(r){return r.json()}).then(zshow);"
            "</script>"));
    } else {
        html_buf_printf(&p, "%s", i18n_page(
            "<button onclick=\"gc('M500')\" style='margin-left:8px' title='" N_("Save all settings to EEPROM (M500)") "'>" N_("Save") "</button>"
            "</div>"
            "<p class='hint' style='margin:4px 0 0'>\xe2\x96\xbc " N_("closer to the bed") " &middot; \xe2\x96\xb2 " N_("farther from the bed") " &middot; " N_("Save writes to EEPROM (M500)") "</p>"
            "</div></details>"));
    }

    html_buf_printf(&p,
        i18n_page(
        "<details class='ctrl'><summary>" N_("Temperatures") "</summary><div class='cg'>"
        "<label>" N_("Nozzle") "</label><div class='bg'>"
        "<button onclick=\"gc('M104 S0')\">" N_("Off") "</button>"
        "<button onclick=\"gc('M104 S190')\">190</button>"
        "<button onclick=\"gc('M104 S200')\">200</button>"
        "<button onclick=\"gc('M104 S210')\">210</button>"
        "<button onclick=\"gc('M104 S220')\">220</button>"
        "<input type='number' id='ht' placeholder='&deg;C' min='0' max='300'>"
        "<button onclick=\"gc('M104 S'+document.getElementById('ht').value)\">" N_("Set") "</button>"
        "</div>"
        "<label>" N_("Bed") "</label><div class='bg'>"
        "<button onclick=\"gc('M140 S0')\">" N_("Off") "</button>"
        "<button onclick=\"gc('M140 S50')\">50</button>"
        "<button onclick=\"gc('M140 S55')\">55</button>"
        "<button onclick=\"gc('M140 S60')\">60</button>"
        "<button onclick=\"gc('M140 S70')\">70</button>"
        "<input type='number' id='bt' placeholder='&deg;C' min='0' max='120'>"
        "<button onclick=\"gc('M140 S'+document.getElementById('bt').value)\">" N_("Set") "</button>"
        "</div></div></details>"));

    html_buf_printf(&p,
        i18n_page(
        "<details class='ctrl'><summary>" N_("Move") "</summary><div class='cg'>"
        "<label>" N_("Step (mm)") "</label><div class='bg'>"
        "<button id='s01' onclick=\"setStep(0.1)\" style='font-weight:bold'>0.1</button>"
        "<button id='s1' onclick=\"setStep(1)\">1</button>"
        "<button id='s10' onclick=\"setStep(10)\">10</button>"
        "<button id='s100' onclick=\"setStep(100)\">100</button>"
        "</div>"
        "<label>" N_("Axes") "</label><div class='bg'>"
        "<button id='xm' onclick=\"jog('X',-1)\">X-</button>"
        "<button id='xp' onclick=\"jog('X',1)\">X+</button>"
        "<button id='ym' onclick=\"jog('Y',-1)\">Y-</button>"
        "<button id='yp' onclick=\"jog('Y',1)\">Y+</button>"
        "<button id='zm' onclick=\"jog('Z',-1)\">Z-</button>"
        "<button id='zp' onclick=\"jog('Z',1)\">Z+</button>"
        "<button id='z0' onclick=\"zzero()\" title='" N_("Lower the nozzle to Z0, where the Z offset puts it") "'>Z0</button>"
        "</div>"
        "<p class='hint' style='margin:4px 0 0'>" N_("Z- stops at 0. To set the Z offset: home, Z0, then the Z offset buttons until the paper drags.") "</p>"
        "<label>" N_("Extruder") "</label><div class='bg'>"
        "<button id='er' onclick=\"extrude(-1)\">" N_("Retract") "</button>"
        "<button id='ee' onclick=\"extrude(1)\">" N_("Extrude") "</button>"
        "<button id='esn' onclick=\"setESpd(0)\" style='font-weight:bold'>" N_("Normal") "</button>"
        "<button id='ess' onclick=\"setESpd(1)\">" N_("Slow") "</button>"
        "</div>"
        "<p class='hint' style='margin:2px 0 0'>" N_("Normal: 300 mm/min. Slow: 60 mm/min (for measuring filament length).") "</p>"
        "<label>" N_("Homing") "</label><div class='bg'>"
        "<button id='ha' onclick=\"gc('G28')\">" N_("All") "</button>"
        "<button id='hx' onclick=\"gc('G28 X')\">X</button>"
        "<button id='hy' onclick=\"gc('G28 Y')\">Y</button>"
        "<button id='hz' onclick=\"gc('G28 Z')\">Z</button>"
        "</div></div></details>"));

    html_buf_printf(&p,
        i18n_page(
        "<details class='ctrl'><summary>" N_("Speed / Flow / Fan") "</summary><div class='cg'>"
        "<label>" N_("Speed") "</label><div class='bg'>"
        "<button onclick=\"adj('spd',-10)\">-10%%</button>"
        "<span class='val' id='spd'>100%%</span>"
        "<button onclick=\"adj('spd',10)\">+10%%</button>"
        "</div>"
        "<label>" N_("Flow") "</label><div class='bg'>"
        "<button onclick=\"adj('flw',-5)\">-5%%</button>"
        "<span class='val' id='flw'>100%%</span>"
        "<button onclick=\"adj('flw',5)\">+5%%</button>"
        "</div>"
        "<label>" N_("Fan") "</label><div class='bg'>"
        "<button onclick=\"gc('M106 S0')\">" N_("Off") "</button>"
        "<button onclick=\"gc('M106 S64')\">25%%</button>"
        "<button onclick=\"gc('M106 S128')\">50%%</button>"
        "<button onclick=\"gc('M106 S191')\">75%%</button>"
        "<button onclick=\"gc('M106 S255')\">100%%</button>"
        "</div></div></details>"));

    html_buf_printf(&p,
        i18n_page(
        "<div style='text-align:center;margin:12px 0'>"
        "<img id='cam' src='/capture' style='max-width:100%%;border-radius:4px;background:#000' alt='" N_("Camera") "'>"
        "</div>"
        "<canvas id='tc' width='600' height='200' style='width:100%%;max-width:600px;display:block;margin:8px auto;background:#1a1a2e;border-radius:4px'></canvas>"));

    html_buf_printf(&p,
        "<script>"
        "var cam=document.getElementById('cam');"
        "setInterval(function(){cam.src='/capture?'+Date.now()},5000);"
        /* gc() — send GCode via /api/command */
        "function gc(c){fetch('/api/command',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'cmd='+encodeURIComponent(c)})}"
        "var zoff=%.2f;"
        "function zb(d){zoff=Math.round((zoff+d)*100)/100;"
        "gc(%s);document.getElementById('zof').textContent=zoff.toFixed(2)}",
        s_baby_z,
        /* Klipper: no M290; its gcode offset is what the printer's screen sets too */
        printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER
            ? "'SET_GCODE_OFFSET Z_ADJUST='+d+' MOVE=1'" : "'M290 Z'+d");
    html_buf_printf(&p, i18n_page(
        /* Step selector for jog */
        "var step=0.1;"
        "function setStep(v){step=v;['s01','s1','s10','s100'].forEach(function(id){document.getElementById(id).style.fontWeight='normal'});"
        "document.getElementById(v==0.1?'s01':v==1?'s1':v==10?'s10':'s100').style.fontWeight='bold'}"
        /* Jog function — relative move */
        "var isPrinting=false;"
        /* Z never goes below 0 from here: the status Z lags behind quick taps,
           so the taps since the last status count too */
        "var zt=0,zj=0;"
        "function jog(ax,dir){if(isPrinting)return;"
        "var d=step*dir;var f=ax=='Z'?600:3000;"
        "if(ax=='Z'){var t=Math.round((zt+d)*100)/100;if(t<0)t=0;d=Math.round((t-zt)*100)/100;if(!d)return;zt=t;zj=Date.now()}"
        "gc('G91\\nG0 '+ax+d+' F'+f+'\\nG90')}"
        "function zzero(){if(isPrinting)return;zt=0;zj=Date.now();gc('G90\\nG0 Z0 F600')}"
        /* Extruder — two speed modes: normal 300mm/min, slow 60mm/min */
        "var espd=0;"
        "function setESpd(m){espd=m;document.getElementById('esn').style.fontWeight=m?'normal':'bold';"
        "document.getElementById('ess').style.fontWeight=m?'bold':'normal'}"
        "function extrude(dir){if(isPrinting)return;"
        "var d=step*dir;var f=espd?60:300;gc('M83\\nG1 E'+d+' F'+f+'\\nM82')}"
        /* Speed/flow adjusters */
        "var vals={spd:100,flw:100};"
        "function adj(k,d){vals[k]+=d;"
        "if(k=='spd'){if(vals[k]<50)vals[k]=50;if(vals[k]>200)vals[k]=200;gc('M220 S'+vals[k])}"
        "if(k=='flw'){if(vals[k]<75)vals[k]=75;if(vals[k]>150)vals[k]=150;gc('M221 S'+vals[k])}"
        "document.getElementById(k).textContent=vals[k]+'%%'}"
        /* Status polling — update controls state */
        "function u(){"
        "fetch('/api/status').then(function(r){return r.json()}).then(function(s){"
        "isPrinting=s.state=='printing'||s.state=='paused';"
        "['xm','xp','ym','yp','zm','zp','z0','er','ee','ha','hx','hy','hz'].forEach(function(id){var b=document.getElementById(id);if(b)b.disabled=isPrinting});"
        "if(typeof s.z==='number'&&Date.now()-zj>4000)zt=s.z;"
        "if(typeof s.baby_z==='number'){zoff=s.baby_z;document.getElementById('zof').textContent=zoff.toFixed(2)}"
        "var hi=document.getElementById('ht'),bi=document.getElementById('bt');"
        "if(hi&&document.activeElement!==hi)hi.value=s.hotend[1]>0?s.hotend[1].toFixed(0):'';"
        "if(bi&&document.activeElement!==bi)bi.value=s.bed[1]>0?s.bed[1].toFixed(0):'';"
        "}).catch(function(){})}"
        "u();setInterval(u,2000);"
        "</script>"
        "<script>"
        "var cv=document.getElementById('tc'),cx=cv.getContext('2d');"
        "function drawChart(d){"
        "var W=cv.width,H=cv.height,pad=40,r=pad,b=H-25,t=10,w=W-pad-r;"
        "if(!d.length)return;"
        "cx.clearRect(0,0,W,H);"
        /* compute time range and temp range */
        "var t0=d[0][0],t1=d[d.length-1][0];"
        "if(t1<=t0)t1=t0+1;"
        "var mn=999,mx=0;"
        "for(var i=0;i<d.length;i++){for(var j=1;j<5;j++){if(d[i][j]>0){if(d[i][j]<mn)mn=d[i][j];if(d[i][j]>mx)mx=d[i][j]}}}"
        "if(mn>=mx){mn=0;mx=100;}"
        "mn=Math.floor(mn/10)*10;mx=Math.ceil(mx/10)*10;"
        "if(mx-mn<20)mx=mn+20;"
        /* grid lines */
        "cx.strokeStyle='#333';cx.lineWidth=0.5;cx.font='10px sans-serif';cx.fillStyle='#888';"
        "var steps=5;for(var i=0;i<=steps;i++){"
        "var y=b-(b-t)*i/steps,v=mn+(mx-mn)*i/steps;"
        "cx.beginPath();cx.moveTo(r,y);cx.lineTo(r+w,y);cx.stroke();"
        "cx.fillText(v.toFixed(0),2,y+3);}"
        /* x-axis: minutes ago */
        "cx.fillStyle='#888';"
        "var dur=t1-t0;var xsteps=Math.min(6,Math.floor(dur/60));"
        "if(xsteps<1)xsteps=1;"
        "for(var i=0;i<=xsteps;i++){"
        "var x=r+w*i/xsteps,sec=dur*i/xsteps;"
        "cx.fillText('-'+(dur-sec>=60?Math.round((dur-sec)/60)+'" N_("min") "':Math.round(dur-sec)+'" N_("s") "'),x-8,H-4);}"
        /* draw lines: [he_act=red, he_tgt=red dash, bed_act=blue, bed_tgt=blue dash] */
        "var colors=['#ff4444','#ff4444','#4488ff','#4488ff'];"
        "var dashes=[[],[5,4],[],[5,4]];"
        "for(var li=0;li<4;li++){"
        "cx.strokeStyle=colors[li];cx.lineWidth=li%%2?1:1.5;cx.setLineDash(dashes[li]);"
        "cx.beginPath();"
        "for(var i=0;i<d.length;i++){"
        "var x=r+((d[i][0]-t0)/(t1-t0))*w;"
        "var y=b-((d[i][li+1]-mn)/(mx-mn))*(b-t);"
        "if(i==0)cx.moveTo(x,y);else cx.lineTo(x,y);}"
        "cx.stroke();cx.setLineDash([]);}"
        /* legend */
        "var lx=r+6,ly=t+10;"
        "var lbl=['" N_("Nozzle") "','" N_("Nozzle target") "','" N_("Bed") "','" N_("Bed target") "'];"
        "for(var i=0;i<4;i++){"
        "cx.strokeStyle=colors[i];cx.lineWidth=i%%2?1:1.5;cx.setLineDash(dashes[i]);"
        "cx.beginPath();cx.moveTo(lx,ly);cx.lineTo(lx+16,ly);cx.stroke();cx.setLineDash([]);"
        "cx.fillStyle=colors[i];cx.fillText(lbl[i],lx+19,ly+3);lx+=cx.measureText(lbl[i]).width+30;}"
        "}"
        "function fetchTemp(){"
        "fetch('/api/temp_history').then(function(r){return r.json()}).then(function(j){"
        "drawChart(j.samples)}).catch(function(){})}"
        "fetchTemp();setInterval(fetchTemp,15000);"
        "</script>"));

    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

/* ---- /settings handler ---- */

static esp_err_t settings_get_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);

    layout_html_begin(&p, NT_("Settings"), "/settings");

    const char *ip = wifi_get_ip_str();
    bool is_marlin = (printer_comm_get_backend() == PRINTER_BACKEND_MARLIN);

    /* 1. Printer (backend selection) */
    html_buf_printf(&p,
        i18n_page(
        "<div class='sh' style='margin-top:16px'><h2>" N_("Printer") "</h2>"
        "<button type='submit' form='f-printer'>" N_("Save &amp; Reboot") "</button></div>"));
    printer_config_render_backend(&p);

    /* 2. Marlin (conditionally visible) */
    if (is_marlin) {
        html_buf_printf(&p,
            i18n_page(
            "<hr><div class='sh'><h2>Marlin</h2>"
            "<button type='submit' form='f-marlin'>" N_("Save") "</button></div>"));
        printer_config_render_marlin(&p);
    } else {
        html_buf_printf(&p,
            i18n_page(
            "<hr><div class='sh'><h2>Klipper</h2>"
            "<button type='submit' form='f-marlin'>" N_("Save") "</button></div>"));
        printer_config_render_klipper(&p);
    }

    /* 3. Video (camera + streaming) */
    bool rot = camera_get_rotate180();
    int xclk = camera_get_xclk_mhz();
    const char *jh = obico_get_janus_host();
    uint16_t jp = obico_get_janus_port();
    html_buf_printf(&p,
        i18n_page(
        "<hr><div class='sh'><h2>" N_("Video") "</h2>"
        "<button type='submit' form='f-video'>" N_("Save") "</button></div>"
        "<form id='f-video' method='POST' action='/video/config'>"
        "<label><input type='checkbox' name='rotate180' value='1'%s> " N_("Rotate 180°") "</label>"
        "<div style='margin:8px 0'><label>XCLK: "
        "<select name='xclk'>"
        "<option value='10'%s>10 MHz (" N_("safe") ")</option>"
        "<option value='12'%s>12 MHz</option>"
        "<option value='16'%s>16 MHz</option>"
        "<option value='20'%s>20 MHz (" N_("fast") ")</option>"
        "</select></label>"
        " <span class='hint'>" N_("Reboot to apply") "</span></div>"
        "<p style='margin:12px 0 4px;font-weight:bold'>Janus Proxy (" N_("WebRTC streaming") ")</p>"
        "<label>" N_("Host") "<br><input type='text' name='janus_host' value='%s' placeholder='192.168.1.x' "
        "style='padding:8px;width:100%%;box-sizing:border-box;margin:4px 0'></label>"
        "<label>" N_("Port") "<br><input type='number' name='janus_port' value='%u' min='1' max='65535' "
        "style='padding:8px;width:100%%;box-sizing:border-box;margin:4px 0'></label>"
        "<p class='hint'>" N_("Leave host empty to disable Janus proxy.") "</p>"
        "</form>"),
        rot ? " checked" : "",
        xclk == 10 ? " selected" : "",
        xclk == 12 ? " selected" : "",
        xclk == 16 ? " selected" : "",
        xclk == 20 ? " selected" : "",
        jh, (unsigned)jp);

    /* 4. Obico */
    if (obico_is_linked()) {
        html_buf_printf(&p,
            i18n_page(
            "<hr><div class='sh'><h2>Obico</h2>"
            "<button type='submit' form='f-obico' class='btn-red' "
            "onclick=\"return confirm('" N_("Unlink from Obico?") "')\">" N_("Unlink") "</button></div>"));
    } else {
        html_buf_printf(&p,
            i18n_page(
            "<hr><div class='sh'><h2>Obico</h2>"
            "<button type='submit' form='f-obico' class='btn-green'>" N_("Link") "</button></div>"));
    }
    obico_render_settings(&p);

    hub_client_render_settings(&p);
    print_stats_render_settings(&p);
    board_led_render_settings(&p);
    nozzle_guard_render_settings(&p);

    /* 5. External Endpoints (read-only) */
    html_buf_printf(&p,
        i18n_page(
        "<hr><h2>" N_("External Endpoints") "</h2>"
        "<p class='hint'>"
        N_("MJPEG Stream") ": <a href='/stream'>http://%s/stream</a><br>"
        N_("Snapshot") ": <a href='/capture'>http://%s/capture</a>"),
        ip, ip);
    if (is_marlin) {
        html_buf_printf(&p,
            "<br>RFC 2217: <code>rfc2217://%s:%d</code>",
            ip, CONFIG_RFC2217_PORT);
    }
    html_buf_printf(&p, "</p>");

    /* 6. Device */
    const char *hostname = wifi_get_hostname();
    html_buf_printf(&p,
        i18n_page(
        "<hr><div class='sh'><h2>" N_("Device") "</h2>"
        "<button type='submit' form='f-device'>" N_("Save &amp; Reboot") "</button></div>"
        "<form id='f-device' method='POST' action='/device/config'>"
        "<label>" N_("Hostname") "<br>"
        "<input type='text' name='hostname' value='%s' maxlength='31' "
        "pattern='[a-zA-Z][a-zA-Z0-9-]*' "
        "title='" N_("Letters, digits, hyphens. Must start with a letter.") "' "
        "style='padding:8px;width:100%%;box-sizing:border-box;margin:4px 0'>"
        "</label>"
        "<p class='hint'>" N_("Visible on router and as") " <b>%s.local</b> " N_("via mDNS.") "</p>"
        "</form>"),
        hostname, hostname);

    /* 7. Firmware Update (OTA) */
    ota_render_settings(&p);

    settings_copy_render_settings(&p);

    /* 8. WiFi */
    char ssid[33] = {0};
    html_buf_printf(&p,
        i18n_page(
        "<hr><div class='sh'><h2>WiFi</h2>"
        "<form id='f-wifi' method='POST' action='/wifi/reset'>"
        "<button type='submit' class='btn-red' "
        "onclick=\"return confirm('" N_("Reset WiFi credentials and reboot?") "')\">" N_("Reset WiFi") "</button>"
        "</form></div>"));
    if (wifi_get_ssid(ssid, sizeof(ssid))) {
        html_buf_printf(&p,
            "<p>%s <b>%s</b></p>", _("Connected to:"), ssid);
    }
    html_buf_printf(&p, "<p><a href='/wifi'>%s &rarr;</a></p>", _("Wi-Fi status and access points"));

    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

/* ---- /camera handler ---- */

static esp_err_t utils_page_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, NT_("Utilities"), "/utils");
    html_buf_printf(&p,
        "<h2>%s</h2>"
        "<table>"
        "<tr><td><a href='/bedlevel'><b>%s</b></a></td><td>%s</td></tr>",
        _("Utilities"), _("Bed Leveling"),
        _("Probe bed mesh, visualize, and calculate screw adjustments"));
    html_buf_printf(&p,
        "<tr><td><a href='/maintenance'><b>%s</b></a></td><td>%s</td></tr>",
        _("Maintenance"), _("Jobs due by print hours or days, with reminders"));
    html_buf_printf(&p,
        "<tr><td><a href='/utils/filament'><b>%s</b></a></td><td>%s</td></tr>"
        "<tr><td><a href='/utils/coldpull'><b>%s</b></a></td><td>%s</td></tr>",
        _("Filament Change"), _("Load new filament over the old one, purging at the hotter temperature"),
        _("Cold Pull"), _("Clean the nozzle by pulling out cooled filament"));
    html_buf_printf(&p, "</table>");
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t camera_page_handler(httpd_req_t *req)
{
    int bright, contrast, sat, ae;
    camera_get_image_params(&bright, &contrast, &sat, &ae);

    html_buf_t p;
    html_buf_init(&p);

    layout_html_begin(&p, NT_("Camera"), "/camera");
    html_buf_printf(&p,
        i18n_page(
        "<style>"
        ".cam-ctl{margin:12px 0;font-size:14px}"
        ".cam-ctl label{display:inline-block;width:110px}"
        ".cam-ctl input[type=range]{width:140px;vertical-align:middle}"
        ".cam-ctl .val{display:inline-block;width:24px;text-align:center}"
        "</style>"
        "<div style='text-align:center'>"
        "<img src='/stream' style='max-width:100%%;border-radius:4px;background:#000' alt='" N_("MJPEG Stream") "'>"
        "</div>"
        "<details open>"
        "<summary>" N_("Image Settings") "</summary>"
        "<div class='cam-ctl'><label>" N_("Brightness") "</label>"
        "<input type='range' min='-2' max='2' value='%d' id='s-br' oninput='camUpd(this,\"vbr\")'>"
        "<span class='val' id='vbr'>%d</span></div>"
        "<div class='cam-ctl'><label>" N_("Contrast") "</label>"
        "<input type='range' min='-2' max='2' value='%d' id='s-co' oninput='camUpd(this,\"vco\")'>"
        "<span class='val' id='vco'>%d</span></div>"
        "<div class='cam-ctl'><label>" N_("Saturation") "</label>"
        "<input type='range' min='-2' max='2' value='%d' id='s-sa' oninput='camUpd(this,\"vsa\")'>"
        "<span class='val' id='vsa'>%d</span></div>"
        "<div class='cam-ctl'><label>" N_("AE Level") "</label>"
        "<input type='range' min='-2' max='2' value='%d' id='s-ae' oninput='camUpd(this,\"vae\")'>"
        "<span class='val' id='vae'>%d</span></div>"
        "</details>"
        "<script>"
        "function camUpd(el,vid){"
        "document.getElementById(vid).textContent=el.value;"
        "fetch('/camera/settings',{method:'POST',"
        "headers:{'Content-Type':'application/x-www-form-urlencoded'},"
        "body:'brightness='+document.getElementById('s-br').value"
        "+'&contrast='+document.getElementById('s-co').value"
        "+'&saturation='+document.getElementById('s-sa').value"
        "+'&ae_level='+document.getElementById('s-ae').value})}"
        "</script>"),
        bright, bright, contrast, contrast, sat, sat, ae, ae);
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t camera_settings_post_handler(httpd_req_t *req)
{
    char buf[128];
    int len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    int bright = 0, contrast = 0, sat = 0, ae = 0;
    char val[8];
    if (parse_form_field(buf, "brightness", val, sizeof(val))) bright = atoi(val);
    if (parse_form_field(buf, "contrast", val, sizeof(val)))   contrast = atoi(val);
    if (parse_form_field(buf, "saturation", val, sizeof(val))) sat = atoi(val);
    if (parse_form_field(buf, "ae_level", val, sizeof(val)))   ae = atoi(val);

    camera_set_image_params(bright, contrast, sat, ae);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t video_config_post_handler(httpd_req_t *req)
{
    char buf[256];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len < 0) recv_len = 0;
    buf[recv_len] = '\0';

    /* Camera rotation */
    bool rotate = (strstr(buf, "rotate180=1") != NULL);
    camera_set_rotate180(rotate);

    /* Camera XCLK */
    char xclk_str[4] = {0};
    if (parse_form_field(buf, "xclk", xclk_str, sizeof(xclk_str))) {
        camera_set_xclk_mhz(atoi(xclk_str));
    }

    /* Janus proxy host/port */
    char janus_host[64] = {0};
    char port_str[8] = {0};
    parse_form_field(buf, "janus_host", janus_host, sizeof(janus_host));
    parse_form_field(buf, "janus_port", port_str, sizeof(port_str));

    uint16_t janus_port = 17800;
    if (port_str[0]) {
        int p = atoi(port_str);
        if (p > 0 && p <= 65535) janus_port = (uint16_t)p;
    }
    obico_set_janus_proxy(janus_host, janus_port);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- /device/config handler ---- */

static esp_err_t device_config_post_handler(httpd_req_t *req)
{
    char buf[128];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len < 0) recv_len = 0;
    buf[recv_len] = '\0';

    char hostname[32] = {0};
    parse_form_field(buf, "hostname", hostname, sizeof(hostname));

    if (hostname[0]) {
        wifi_set_hostname(hostname);
        httpd_resp_set_type(req, "text/plain; charset=utf-8");
        httpd_resp_sendstr(req, _("Device name saved. Rebooting..."));
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- /wifi/reset handler ---- */

static esp_err_t wifi_reset_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_sendstr(req, _("WiFi credentials erased. Rebooting..."));
    vTaskDelay(pdMS_TO_TICKS(1000));
    wifi_reset_credentials();
    return ESP_OK;  /* unreachable */
}

/* ---- Captive portal helpers ---- */

static bool parse_form_field(const char *body, const char *name,
                             char *out, size_t out_size)
{
    char key[36];
    snprintf(key, sizeof(key), "%s=", name);
    const char *start = strstr(body, key);
    if (!start) return false;
    start += strlen(key);

    url_decode_field(start, out, out_size);
    return true;
}

/* A page template for i18n_page(), sent as is: % is single */
static const char CAPTIVE_HTML[] =
    "<!DOCTYPE html><html><head>"
    "<meta charset=\"utf-8\">"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>ESP32 FDM " N_("WiFi Setup") "</title>"
    "<style>"
    "body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:0 20px}"
    "input{width:100%;padding:8px;margin:4px 0 12px;box-sizing:border-box;font-size:16px}"
    "button{width:100%;padding:10px;font-size:18px;background:#2196F3;color:white;"
    "border:none;border-radius:4px;cursor:pointer}"
    "label{font-weight:bold}"
    "</style></head><body>"
    "<h2>" N_("WiFi Setup") "</h2>"
    "<form method=\"POST\" action=\"/\">"
    "<label>SSID</label>"
    "<input type=\"text\" name=\"ssid\" required maxlength=\"32\" autocomplete=\"off\">"
    "<label>" N_("Password") "</label>"
    "<input type=\"password\" name=\"password\" maxlength=\"64\">"
    "<button type=\"submit\">" N_("Connect") "</button>"
    "</form></body></html>";

static esp_err_t captive_root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, i18n_page(CAPTIVE_HTML));
}

static esp_err_t captive_root_post_handler(httpd_req_t *req)
{
    char buf[200];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    char ssid[33] = {0};
    char pass[65] = {0};

    if (!parse_form_field(buf, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_FAIL;
    }
    parse_form_field(buf, "password", pass, sizeof(pass));

    /* Save to NVS */
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("wifi", NVS_READWRITE, &nvs));
    nvs_set_str(nvs, "ssid", ssid);
    nvs_set_str(nvs, "password", pass);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "WiFi credentials saved for \"%s\" — rebooting", ssid);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, i18n_page(
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\"></head>"
        "<body style=\"font-family:sans-serif;text-align:center;margin-top:60px\">"
        "<h2>" N_("Saved!") "</h2><p>" N_("Rebooting and connecting to WiFi...") "</p>"
        "</body></html>"));

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;  /* unreachable */
}

static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- Public API ---- */

esp_err_t httpd_start_captive_portal(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.max_uri_handlers = 4;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start captive portal: 0x%x", err);
        return err;
    }

    /* Order matters: exact matches first, wildcard last */
    httpd_uri_t root_get = {
        .uri = "/", .method = HTTP_GET, .handler = captive_root_get_handler,
    };
    HTTPD_REGISTER(server, &root_get);

    httpd_uri_t root_post = {
        .uri = "/", .method = HTTP_POST, .handler = captive_root_post_handler,
    };
    HTTPD_REGISTER(server, &root_post);

    httpd_uri_t wildcard = {
        .uri = "/*", .method = HTTP_GET, .handler = captive_redirect_handler,
    };
    HTTPD_REGISTER(server, &wildcard);

    ESP_LOGI(TAG, "Captive portal started on port 80");
    return ESP_OK;
}

esp_err_t httpd_start_server(void)
{
    /* HTTP server on port 80 (Core 0) */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.stack_size = 10240;  /* TLS (mbedTLS) in /obico/link handler needs ~8KB */
    config.max_uri_handlers = 104;
    config.max_open_sockets = 13;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: 0x%x", err);
        return err;
    }

    httpd_uri_t root_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_handler,
    };
    HTTPD_REGISTER(server, &root_uri);

    httpd_uri_t favicon_uri = {
        .uri      = "/favicon.svg",
        .method   = HTTP_GET,
        .handler  = favicon_handler,
    };
    HTTPD_REGISTER(server, &favicon_uri);

    httpd_uri_t capture_uri = {
        .uri      = "/capture",
        .method   = HTTP_GET,
        .handler  = capture_handler,
    };
    HTTPD_REGISTER(server, &capture_uri);

    httpd_uri_t stream_uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = stream_handler,
    };
    HTTPD_REGISTER(server, &stream_uri);

    httpd_uri_t api_status = {
        .uri      = "/api/status",
        .method   = HTTP_GET,
        .handler  = api_status_handler,
    };
    HTTPD_REGISTER(server, &api_status);

    httpd_uri_t api_stats = {
        .uri      = "/api/stats",
        .method   = HTTP_GET,
        .handler  = api_stats_handler,
    };
    HTTPD_REGISTER(server, &api_stats);

    httpd_uri_t api_temp_history = {
        .uri      = "/api/temp_history",
        .method   = HTTP_GET,
        .handler  = api_temp_history_handler,
    };
    HTTPD_REGISTER(server, &api_temp_history);

    httpd_uri_t api_heap = {
        .uri      = "/api/heap",
        .method   = HTTP_GET,
        .handler  = api_heap_handler,
    };
    HTTPD_REGISTER(server, &api_heap);

    httpd_uri_t api_tasks = {
        .uri      = "/api/tasks",
        .method   = HTTP_GET,
        .handler  = api_tasks_handler,
    };
    HTTPD_REGISTER(server, &api_tasks);

    httpd_uri_t api_command = {
        .uri      = "/api/command",
        .method   = HTTP_POST,
        .handler  = api_command_handler,
    };
    HTTPD_REGISTER(server, &api_command);

    httpd_uri_t api_zoffset_get = {
        .uri      = "/api/zoffset",
        .method   = HTTP_GET,
        .handler  = api_zoffset_get_handler,
    };
    HTTPD_REGISTER(server, &api_zoffset_get);
    httpd_uri_t api_zoffset_post = {
        .uri      = "/api/zoffset",
        .method   = HTTP_POST,
        .handler  = api_zoffset_post_handler,
    };
    HTTPD_REGISTER(server, &api_zoffset_post);

    httpd_uri_t camera_page = {
        .uri      = "/camera",
        .method   = HTTP_GET,
        .handler  = camera_page_handler,
    };
    HTTPD_REGISTER(server, &camera_page);

    httpd_uri_t settings_uri = {
        .uri      = "/settings",
        .method   = HTTP_GET,
        .handler  = settings_get_handler,
    };
    HTTPD_REGISTER(server, &settings_uri);

    httpd_uri_t video_cfg_post = {
        .uri      = "/video/config",
        .method   = HTTP_POST,
        .handler  = video_config_post_handler,
    };
    HTTPD_REGISTER(server, &video_cfg_post);

    httpd_uri_t cam_settings_post = {
        .uri      = "/camera/settings",
        .method   = HTTP_POST,
        .handler  = camera_settings_post_handler,
    };
    HTTPD_REGISTER(server, &cam_settings_post);

    httpd_uri_t device_cfg_post = {
        .uri      = "/device/config",
        .method   = HTTP_POST,
        .handler  = device_config_post_handler,
    };
    HTTPD_REGISTER(server, &device_cfg_post);

    httpd_uri_t wifi_reset = {
        .uri      = "/wifi/reset",
        .method   = HTTP_POST,
        .handler  = wifi_reset_handler,
    };
    HTTPD_REGISTER(server, &wifi_reset);

    httpd_uri_t utils_uri = {
        .uri      = "/utils",
        .method   = HTTP_GET,
        .handler  = utils_page_handler,
    };
    HTTPD_REGISTER(server, &utils_uri);

    sdcard_httpd_register(server);

    terminal_register_httpd(server);
    bedlevel_register_httpd(server);
    cold_pull_register_httpd(server);
    filament_register_httpd(server);
    logbuf_register_httpd(server);
    print_stats_register_httpd(server);
    maintenance_register_httpd(server);
    settings_copy_register_httpd(server);
    board_led_register_httpd(server);
    nozzle_guard_register_httpd(server);
    print_checks_register_httpd(server);
    wifi_page_register_httpd(server);
    obico_register_httpd(server);
    printer_config_register_httpd(server);
    ota_register_httpd(server);
    print_page_register_httpd(server);
    hub_client_register_httpd(server);

    ESP_LOGI(TAG, "HTTP server started on port 80 (stream at /stream)");

    return ESP_OK;
}
