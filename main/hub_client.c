#include "hub_client.h"
#include "task_util.h"
#include "maintenance.h"
#include "esp_wifi.h"
#include "ota.h"
#include "settings_copy.h"
#include "print_stats.h"

#include <stdio.h>
#include <string.h>

#include "cJSON.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "httpd.h"
#include "printer_comm.h"
#include "version.h"
#include "nvs.h"
#include "url_util.h"
#include "wifi.h"

static const char *TAG = "hub";

#define NVS_NAMESPACE     "hub"
#define NVS_KEY_URL       "url"
#define URL_MAX_LEN       96
#define HTTP_TIMEOUT_MS   8000     /* The hub answers after Telegram has */
#define QUEUE_DEPTH       4
#define SEND_ATTEMPTS     3

/* Status for the hub's dashboard: sent every STATUS_INTERVAL_S, and at once
 * when the state changes, so a tile turns orange as soon as a print stops */
#define STATUS_INTERVAL_S   10
#define STATUS_CHECK_MS     1000
#define STATUS_TIMEOUT_MS   3000

typedef struct {
    hub_event_type_t type;
    char event_id[48];
    char message[192];       /* A list of maintenance tasks fits */
    char filename[64];
} hub_event_t;

static QueueHandle_t s_queue;
static char s_device_id[13];
static char s_hub_url[URL_MAX_LEN];      /* From settings, empty = notifications off */
static const char *volatile s_status = "";

static const char *event_type_name(hub_event_type_t type)
{
    switch (type) {
    case HUB_EVENT_HOLD:         return "hold";
    case HUB_EVENT_HOLD_TIMEOUT: return "hold_timeout";
    case HUB_EVENT_UPDATED:      return "updated";
    case HUB_EVENT_UPDATE_FAILED: return "update_failed";
    case HUB_EVENT_MAINTENANCE:  return "maintenance";
    case HUB_EVENT_MAINTENANCE_SKIPPED: return "maintenance_skipped";
    case HUB_EVENT_STACK_LOW:    return "stack_low";
    case HUB_EVENT_OBICO_PAUSE:  return "obico_pause";
    case HUB_EVENT_NOZZLE_STANDBY: return "nozzle_standby";
    case HUB_EVENT_PRINT_CHECK: return "print_check";
    case HUB_EVENT_FILAMENT_RUNOUT: return "filament_runout";
    default:                     return "test";
    }
}

/* ---- Settings ---- */

static void load_hub_url(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    size_t len = sizeof(s_hub_url);
    if (nvs_get_str(nvs, NVS_KEY_URL, s_hub_url, &len) != ESP_OK) {
        s_hub_url[0] = '\0';
    }
    nvs_close(nvs);
}

static esp_err_t save_hub_url(const char *url)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;
    err = nvs_set_str(nvs, NVS_KEY_URL, url);
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);
    return err;
}

/* ---- Sending ---- */

static void add_device(cJSON *root)
{
    cJSON *dev = cJSON_AddObjectToObject(root, "device");
    if (dev) {
        cJSON_AddStringToObject(dev, "id", s_device_id);
        cJSON_AddStringToObject(dev, "name", wifi_get_hostname());
        cJSON_AddStringToObject(dev, "ip", wifi_get_ip_str());
    }
}

static char *event_to_json(const hub_event_t *ev)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;
    cJSON_AddStringToObject(root, "event_id", ev->event_id);
    cJSON_AddStringToObject(root, "type", event_type_name(ev->type));
    add_device(root);
    cJSON_AddStringToObject(root, "message", ev->message);
    cJSON_AddStringToObject(root, "filename", ev->filename);
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return body;
}

/** POST body to base_url + path.  Returns the HTTP status, or -1 if there was none. */
static int post_json(const char *base_url, const char *path, const char *body, int timeout_ms,
                     bool quiet)
{
    char url[URL_MAX_LEN + 24];
    snprintf(url, sizeof(url), "%s%s", base_url, path);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = timeout_ms,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return false;

    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, body, strlen(body));

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) {
        if (!quiet) ESP_LOGW(TAG, "POST %s failed: %s", url, esp_err_to_name(err));
        return -1;
    }
    if (status != 200 && !quiet) {
        ESP_LOGW(TAG, "POST %s returned %d", url, status);
    }
    return status;
}

static void deliver(const hub_event_t *ev)
{
    char *body = event_to_json(ev);
    if (!body) {
        s_status = "failed";
        return;
    }

    bool sent = false;
    for (int attempt = 0; attempt < SEND_ATTEMPTS && !sent; attempt++) {
        if (attempt > 0) vTaskDelay(pdMS_TO_TICKS(attempt * 5000));

        sent = post_json(s_hub_url, "/api/v1/events", body, HTTP_TIMEOUT_MS, false) == 200;
    }
    cJSON_free(body);

    s_status = sent ? "ok" : "failed";
    if (sent) {
        ESP_LOGI(TAG, "Event delivered: %s", event_type_name(ev->type));
    } else {
        ESP_LOGE(TAG, "Event not delivered: %s", event_type_name(ev->type));
    }
}

static void hub_task(void *arg)
{
    hub_event_t ev;
    while (true) {
        if (xQueueReceive(s_queue, &ev, portMAX_DELAY) == pdTRUE) {
            deliver(&ev);
        }
    }
}

void hub_client_send_event(hub_event_type_t type, const char *message, const char *filename)
{
    if (!s_queue) return;

    if (!s_hub_url[0]) {
        s_status = "off";
        return;
    }

    hub_event_t ev = { .type = type };
    /* Random part: a counter would repeat after a reboot and the hub would
     * drop the event as a duplicate. */
    snprintf(ev.event_id, sizeof(ev.event_id), "%s-%08lx%08lx", s_device_id,
             (unsigned long)esp_random(), (unsigned long)esp_random());
    strlcpy(ev.message, message, sizeof(ev.message));
    strlcpy(ev.filename, filename, sizeof(ev.filename));

    s_status = "sending";
    if (xQueueSend(s_queue, &ev, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Event queue full, dropping %s", event_type_name(type));
        s_status = "failed";
    }
}

/* ---- Status ---- */

static const char *state_name(const printer_state_t *st)
{
    switch (st->opstate) {
    case PRINTER_OPERATIONAL: return "idle";
    case PRINTER_PRINTING:    return "printing";
    case PRINTER_PAUSED:      return st->holding ? "hold" : "paused";
    case PRINTER_ERROR:       return "error";
    default:                  return "no_printer";   /* The board is up, the printer is not */
    }
}

static char *status_to_json(const printer_state_t *st)
{
    cJSON *root = cJSON_CreateObject();
    if (!root) return NULL;
    add_device(root);
    cJSON_AddStringToObject(root, "version", FW_VERSION);
    cJSON_AddStringToObject(root, "build", FW_BUILD);
    if (settings_copy_model()[0]) cJSON_AddStringToObject(root, "model", settings_copy_model());
    cJSON_AddNumberToObject(root, "interval_s", STATUS_INTERVAL_S);
    cJSON_AddStringToObject(root, "state", state_name(st));
    print_stats_t ps;
    print_stats_get(&ps);
    cJSON_AddNumberToObject(root, "print_hours", (int)(ps.print_s / 360) / 10.0);
    cJSON_AddNumberToObject(root, "prints", ps.started);
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        char name[26];
        if (!wifi_ap_name(ap.bssid, name, sizeof(name))) {
            snprintf(name, sizeof(name), "%02x:%02x:%02x", ap.bssid[3], ap.bssid[4], ap.bssid[5]);
        }
        cJSON_AddStringToObject(root, "wifi_ap", name);
        cJSON_AddNumberToObject(root, "wifi_rssi", ap.rssi);
    }
    int due = maintenance_due_count();
    if (due) cJSON_AddNumberToObject(root, "maintenance_due", due);
    char pending[32];
    ota_get_pending(pending, sizeof(pending));
    if (pending[0]) cJSON_AddStringToObject(root, "update_pending", pending);

    bool active = st->opstate == PRINTER_PRINTING || st->opstate == PRINTER_PAUSED;
    if (active) {
        cJSON_AddStringToObject(root, "filename", st->filename);
        cJSON_AddBoolToObject(root, "preparing", st->preparing);
        if (st->progress_pct >= 0)
            cJSON_AddNumberToObject(root, "progress", (int)(st->progress_pct * 10) / 10.0);
        if (st->print_time_left_s >= 0)
            cJSON_AddNumberToObject(root, "time_left_s", st->print_time_left_s);
        if (st->holding)
            cJSON_AddStringToObject(root, "hold_msg", st->hold_msg);
    }
    if (st->opstate != PRINTER_DISCONNECTED) {
        cJSON_AddNumberToObject(root, "hotend", (int)st->hotend_actual);
        cJSON_AddNumberToObject(root, "hotend_target", (int)st->hotend_target);
        cJSON_AddNumberToObject(root, "bed", (int)st->bed_actual);
        cJSON_AddNumberToObject(root, "bed_target", (int)st->bed_target);
    }

    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return body;
}

static void status_task(void *arg)
{
    int64_t last_sent_us = 0;
    printer_opstate_t last_opstate = PRINTER_DISCONNECTED;
    uint32_t last_hold_id = 0;
    bool last_holding = false;
    bool last_ok = true;

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(STATUS_CHECK_MS));

        char url[URL_MAX_LEN];
        strlcpy(url, s_hub_url, sizeof(url));
        if (!url[0] || !wifi_get_ip_str()[0]) continue;

        printer_state_t st;
        printer_comm_get_state(&st);

        int64_t now = esp_timer_get_time();
        bool changed = st.opstate != last_opstate || st.holding != last_holding ||
                       (st.holding && st.hold_id != last_hold_id);
        if (!changed && last_sent_us && now - last_sent_us < STATUS_INTERVAL_S * 1000000LL) {
            continue;
        }
        last_sent_us = now;
        last_opstate = st.opstate;
        last_holding = st.holding;
        last_hold_id = st.hold_id;

        char *body = status_to_json(&st);
        if (!body) continue;
        /* Once, not every 10 s: a hub that is down, or too old to know this
         * endpoint, must not flood the log */
        int status = post_json(url, "/api/v1/status", body, STATUS_TIMEOUT_MS, true);
        cJSON_free(body);

        bool ok = status == 200;
        if (ok != last_ok) {
            if (ok) ESP_LOGI(TAG, "Status reaches the hub again");
            else ESP_LOGW(TAG, "Status not accepted by the hub (%d), will keep trying", status);
            last_ok = ok;
        }
    }
}

const char *hub_client_notify_status(void)
{
    return s_status;
}

void hub_client_init(void)
{
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(s_device_id, sizeof(s_device_id), "%02x%02x%02x%02x%02x%02x", MAC2STR(mac));

    load_hub_url();

    s_queue = xQueueCreate(QUEUE_DEPTH, sizeof(hub_event_t));
    if (!s_queue ||
        task_create_psram(hub_task, "hub", 12288, NULL, 3, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start hub client");
        if (s_queue) {
            vQueueDelete(s_queue);
            s_queue = NULL;
        }
    }
    /* 6 KB: 4 KB had 480 bytes left */
    if (task_create_psram(status_task, "hub_status", 12288, NULL, 2, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start hub status");
    }
}

/* ---- Web UI ---- */

void hub_client_render_settings(html_buf_t *p)
{
    html_buf_printf(p,
        i18n_page(
        "<hr><div class='sh'><h2>" N_("Notifications") "</h2>"
        "<button type='submit' form='f-hub'>" N_("Save") "</button></div>"
        "<form id='f-hub' method='POST' action='/hub/config'>"
        "<label>" N_("Hub URL") "<br><input type='url' name='url' value='%s' maxlength='%d' "
        "placeholder='http://192.168.3.2:8787' "
        "style='padding:8px;width:100%%;box-sizing:border-box;margin:4px 0'></label>"
        "</form>"
        "<p class='hint'>" N_("Address of the notification hub. Leave empty to turn notifications off.") "</p>"
        "<button type='button' onclick='hubTest(this)'>" N_("Send test") "</button> <span id='hubr'></span>"
        "<script>"
        "function hubTest(b){var r=document.getElementById('hubr'),n=0;"
        "b.disabled=true;r.textContent='" N_("Sending...") "';"
        "function done(t){r.textContent=t;b.disabled=false}"
        "function poll(){fetch('/api/status').then(function(x){return x.json()}).then(function(s){"
        "if(s.notify=='ok')done('" N_("Delivered") "');"
        "else if(s.notify=='failed')done('" N_("Failed, see /logs") "');"
        "else if(s.notify=='off')done('" N_("Save the hub URL first") "');"
        "else if(++n>30)done('" N_("No answer") "');"
        "else setTimeout(poll,1000)}).catch(function(){done('" N_("No answer") "')})}"
        "fetch('/hub/test',{method:'POST'}).then(function(){setTimeout(poll,1000)})"
        ".catch(function(){done('" N_("No answer") "')})}"
        "</script>"),
        s_hub_url, URL_MAX_LEN - 1);
}

/* An error the browser shows as the page, after the settings form is sent */
static esp_err_t send_form_error(httpd_req_t *req, const char *status, const char *msg)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/plain; charset=utf-8");
    httpd_resp_sendstr(req, msg);
    return ESP_FAIL;
}

const char *hub_client_set_url(const char *in)
{
    char url[URL_MAX_LEN];
    strlcpy(url, in, sizeof(url));
    size_t len = strlen(url);
    while (len > 0 && url[len - 1] == '/') url[--len] = '\0';

    /* The value goes back into an HTML attribute on the settings page */
    if (len > 0 && (strncmp(url, "http://", 7) != 0 || strpbrk(url, "'\"<>& "))) {
        return NT_("URL must start with http://");
    }
    if (save_hub_url(url) != ESP_OK) return NT_("Could not save");
    strlcpy(s_hub_url, url, sizeof(s_hub_url));
    return NULL;
}

const char *hub_client_get_url(void)
{
    return s_hub_url;
}

static esp_err_t hub_config_handler(httpd_req_t *req)
{
    char buf[256];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    char url[URL_MAX_LEN] = {0};
    const char *p = strstr(buf, "url=");
    if (p) url_decode_field(p + 4, url, sizeof(url));

    const char *bad = hub_client_set_url(url);
    if (bad) return send_form_error(req, "400 Bad Request", TR(bad));

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t hub_test_handler(httpd_req_t *req)
{
    hub_client_send_event(HUB_EVENT_TEST, "Test notification", "");
    return httpd_resp_send(req, "OK", 2);
}

esp_err_t hub_client_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/hub/config", .method = HTTP_POST, .handler = hub_config_handler },
        { .uri = "/hub/test",   .method = HTTP_POST, .handler = hub_test_handler },
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }
    return ESP_OK;
}
