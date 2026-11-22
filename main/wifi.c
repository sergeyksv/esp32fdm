#include "wifi.h"
#include "task_util.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_sntp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1     /* Boot only: gave up, start the setup AP */
#define WIFI_DISC_BIT      BIT2     /* A disconnect for the Wi-Fi task to handle */
#define WIFI_SCAN_BIT      BIT3     /* The status page asked for a scan */
#define MAX_RETRY          3        /* At boot, before falling back to the setup AP */

#define NVS_WIFI_NAMESPACE "wifi"
#define NVS_KEY_SSID       "ssid"
#define NVS_KEY_PASSWORD   "password"
#define NVS_KEY_HOSTNAME   "hostname"
#define NVS_KEY_PREFERRED  "pref_bssid"
#define NVS_KEY_AP_NAMES   "ap_names"
#define DEFAULT_HOSTNAME   "esp32fdm"

#define ROAM_RSSI_THRESHOLD   (-70)   /* Look for a better AP below this */
#define ROAM_RSSI_HYSTERESIS  10      /* ...and move only if it is this much better */
#define ROAM_CHECK_US         (30 * 1000000LL)
#define PREF_MIN_RSSI         (-75)   /* The preferred AP is taken only if this strong */
#define PREF_CHECK_US         (5 * 60 * 1000000LL)  /* Look for it this often when elsewhere */
#define PREF_AFTER_BTM_US     (30 * 60 * 1000000LL) /* ...but not this soon after the network moved us */
#define LOST_US               (20 * 1000000LL)      /* Not back by itself after this: reconnect */
#define CONNECT_WAIT_MS       15000
#define BACKOFF_MAX_S         60

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;
static int s_max_retry;
static char s_ip_str[16];
static bool s_sta_netif_created;
static char s_current_ssid[33];
static char s_current_pass[65];
static char s_hostname[32] = DEFAULT_HOSTNAME;

/* After boot the Wi-Fi task owns reconnecting, and never gives up */
static volatile bool s_managed;
/* The task is switching APs itself: its own disconnect is not a loss */
static volatile bool s_switching;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static wifi_stats_t s_stats;
static uint8_t s_preferred[6];
static bool s_has_preferred;
static wifi_ap_name_t s_names[WIFI_AP_NAMES_MAX];
static int s_name_count;

static SemaphoreHandle_t s_scan_lock;
static wifi_seen_ap_t s_seen[WIFI_SEEN_MAX];
static int s_seen_count;
static int64_t s_seen_us;

static bool bssid_eq(const uint8_t *a, const uint8_t *b)
{
    return memcmp(a, b, 6) == 0;
}

/* ---- Names and the preferred AP (NVS) ---- */

static void load_ap_prefs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    size_t len = sizeof(s_preferred);
    s_has_preferred = nvs_get_blob(nvs, NVS_KEY_PREFERRED, s_preferred, &len) == ESP_OK && len == 6;
    len = sizeof(s_names);
    if (nvs_get_blob(nvs, NVS_KEY_AP_NAMES, s_names, &len) == ESP_OK) {
        s_name_count = len / sizeof(wifi_ap_name_t);
    }
    nvs_close(nvs);
}

static void save_ap_prefs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    if (s_has_preferred) nvs_set_blob(nvs, NVS_KEY_PREFERRED, s_preferred, 6);
    else nvs_erase_key(nvs, NVS_KEY_PREFERRED);
    if (s_name_count) nvs_set_blob(nvs, NVS_KEY_AP_NAMES, s_names, s_name_count * sizeof(wifi_ap_name_t));
    else nvs_erase_key(nvs, NVS_KEY_AP_NAMES);
    nvs_commit(nvs);
    nvs_close(nvs);
}

bool wifi_ap_name(const uint8_t bssid[6], char *out, size_t size)
{
    bool found = false;
    taskENTER_CRITICAL(&s_mux);
    for (int i = 0; i < s_name_count; i++) {
        if (bssid_eq(s_names[i].bssid, bssid)) {
            strlcpy(out, s_names[i].name, size);
            found = true;
            break;
        }
    }
    taskEXIT_CRITICAL(&s_mux);
    if (!found && size) out[0] = '\0';
    return found;
}

int wifi_get_ap_names(wifi_ap_name_t *out, int max)
{
    taskENTER_CRITICAL(&s_mux);
    int n = s_name_count < max ? s_name_count : max;
    memcpy(out, s_names, n * sizeof(wifi_ap_name_t));
    taskEXIT_CRITICAL(&s_mux);
    return n;
}

void wifi_set_ap_names(const wifi_ap_name_t *names, int count)
{
    if (count > WIFI_AP_NAMES_MAX) count = WIFI_AP_NAMES_MAX;
    taskENTER_CRITICAL(&s_mux);
    memcpy(s_names, names, count * sizeof(wifi_ap_name_t));
    s_name_count = count;
    taskEXIT_CRITICAL(&s_mux);
    save_ap_prefs();
}

bool wifi_get_preferred(uint8_t out[6])
{
    taskENTER_CRITICAL(&s_mux);
    bool has = s_has_preferred;
    if (has) memcpy(out, s_preferred, 6);
    taskEXIT_CRITICAL(&s_mux);
    return has;
}

void wifi_set_preferred(const uint8_t *bssid)
{
    taskENTER_CRITICAL(&s_mux);
    s_has_preferred = bssid != NULL;
    if (bssid) memcpy(s_preferred, bssid, 6);
    taskEXIT_CRITICAL(&s_mux);
    save_ap_prefs();
}

/* ---- Status ---- */

void wifi_get_stats(wifi_stats_t *out)
{
    taskENTER_CRITICAL(&s_mux);
    *out = s_stats;
    taskEXIT_CRITICAL(&s_mux);
}

int wifi_get_seen(wifi_seen_ap_t *out, int max, int64_t *when_us)
{
    xSemaphoreTake(s_scan_lock, portMAX_DELAY);
    int n = s_seen_count < max ? s_seen_count : max;
    memcpy(out, s_seen, n * sizeof(wifi_seen_ap_t));
    if (when_us) *when_us = s_seen_us;
    xSemaphoreGive(s_scan_lock);
    return n;
}

void wifi_request_scan(void)
{
    if (s_wifi_event_group) xEventGroupSetBits(s_wifi_event_group, WIFI_SCAN_BIT);
}

/* ---- Scanning and choosing ---- */

/** Scan for our SSID, keep what was seen for the status page */
static int scan_ssid(const char *ssid)
{
    wifi_scan_config_t scan_cfg = {
        .ssid = (uint8_t *)ssid,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = { .min = 100, .max = 300 },
    };

    xSemaphoreTake(s_scan_lock, portMAX_DELAY);
    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        xSemaphoreGive(s_scan_lock);
        ESP_LOGW(TAG, "Scan failed: %s", esp_err_to_name(err));
        return 0;
    }

    uint16_t count = WIFI_SEEN_MAX;
    wifi_ap_record_t *list = calloc(count, sizeof(wifi_ap_record_t));
    if (!list) {
        esp_wifi_clear_ap_list();
        xSemaphoreGive(s_scan_lock);
        return 0;
    }
    esp_wifi_scan_get_ap_records(&count, list);

    s_seen_count = 0;
    for (int i = 0; i < count; i++) {
        if (strcmp((char *)list[i].ssid, ssid) != 0) continue;
        wifi_seen_ap_t *a = &s_seen[s_seen_count++];
        memcpy(a->bssid, list[i].bssid, 6);
        a->channel = list[i].primary;
        a->rssi = list[i].rssi;
        ESP_LOGI(TAG, "  AP: " MACSTR "  ch=%d  rssi=%d", MAC2STR(a->bssid), a->channel, a->rssi);
    }
    s_seen_us = esp_timer_get_time();
    int n = s_seen_count;
    xSemaphoreGive(s_scan_lock);
    free(list);
    return n;
}

/** The AP to connect to: the preferred one if it is strong enough, else the
 *  strongest.  False if none of ours is in range. */
static bool choose_ap(const char *ssid, uint8_t *bssid, int8_t *rssi)
{
    if (!scan_ssid(ssid)) return false;
    uint8_t pref[6];
    bool has_pref = wifi_get_preferred(pref);

    xSemaphoreTake(s_scan_lock, portMAX_DELAY);
    int best = -1;
    for (int i = 0; i < s_seen_count; i++) {
        if (has_pref && bssid_eq(s_seen[i].bssid, pref) && s_seen[i].rssi >= PREF_MIN_RSSI) {
            best = i;
            break;
        }
        if (best < 0 || s_seen[i].rssi > s_seen[best].rssi) best = i;
    }
    if (best >= 0) {
        memcpy(bssid, s_seen[best].bssid, 6);
        if (rssi) *rssi = s_seen[best].rssi;
    }
    xSemaphoreGive(s_scan_lock);
    return best >= 0;
}

/** Station config for our network, locked to bssid if given.  Without a
 *  lock the driver scans every channel and takes the strongest AP. */
static void make_config(wifi_config_t *cfg, const uint8_t *bssid)
{
    memset(cfg, 0, sizeof(*cfg));
    strncpy((char *)cfg->sta.ssid, s_current_ssid, sizeof(cfg->sta.ssid) - 1);
    strncpy((char *)cfg->sta.password, s_current_pass, sizeof(cfg->sta.password) - 1);
    cfg->sta.threshold.authmode = s_current_pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    cfg->sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    cfg->sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    cfg->sta.rm_enabled = 1;   /* 802.11k: answer the AP's radio measurement requests */
    cfg->sta.btm_enabled = 1;  /* 802.11v: move when the network (DAWN) asks */
    if (bssid) {
        memcpy(cfg->sta.bssid, bssid, 6);
        cfg->sta.bssid_set = true;
    }
}

/** Connect (again) to bssid, or to the strongest AP if NULL, and wait.
 *  Only from the Wi-Fi task. */
static bool connect_to(const uint8_t *bssid)
{
    wifi_config_t cfg;
    make_config(&cfg, bssid);
    s_switching = true;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_DISC_BIT);
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &cfg);
    esp_wifi_connect();
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT,
                                           pdFALSE, pdFALSE, pdMS_TO_TICKS(CONNECT_WAIT_MS));
    s_switching = false;
    return bits & WIFI_CONNECTED_BIT;
}

/** Connection lost: try until it is back, choosing the AP afresh each time */
static void reconnect(void)
{
    for (int attempt = 0;; attempt++) {
        uint8_t bssid[6];
        int8_t rssi;
        /* After a few misses, let the driver pick: the scan may be what fails */
        bool chosen = attempt < 4 && choose_ap(s_current_ssid, bssid, &rssi);
        if (chosen) ESP_LOGI(TAG, "Reconnecting to " MACSTR " (rssi %d)", MAC2STR(bssid), rssi);
        else ESP_LOGI(TAG, "Reconnecting to the strongest AP");
        if (connect_to(chosen ? bssid : NULL)) {
            taskENTER_CRITICAL(&s_mux);
            s_stats.reconnects++;
            taskEXIT_CRITICAL(&s_mux);
            return;
        }
        int wait_s = 5 << (attempt < 4 ? attempt : 4);
        if (wait_s > BACKOFF_MAX_S) wait_s = BACKOFF_MAX_S;
        ESP_LOGW(TAG, "Reconnect attempt %d failed (reason %u), next in %d s",
                 attempt + 1, s_stats.last_reason, wait_s);
        vTaskDelay(pdMS_TO_TICKS(wait_s * 1000));
        if (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) return;
    }
}

/** Connected: move to a better AP if the signal is poor, or back to the
 *  preferred one when it is in range again */
static void roam_check(int64_t *pref_checked_us)
{
    wifi_ap_record_t cur;
    if (esp_wifi_sta_get_ap_info(&cur) != ESP_OK) return;

    int64_t now = esp_timer_get_time();
    uint8_t pref[6];
    bool has_pref = wifi_get_preferred(pref);
    bool weak = cur.rssi < ROAM_RSSI_THRESHOLD;
    bool want_pref = has_pref && !bssid_eq(cur.bssid, pref) &&
                     now - *pref_checked_us >= PREF_CHECK_US &&
                     (!s_stats.last_btm_us || now - s_stats.last_btm_us >= PREF_AFTER_BTM_US);
    if (!weak && !want_pref) return;
    if (want_pref) *pref_checked_us = now;

    uint8_t best[6];
    int8_t best_rssi;
    if (!choose_ap(s_current_ssid, best, &best_rssi) || bssid_eq(best, cur.bssid)) return;
    bool to_pref = has_pref && bssid_eq(best, pref);
    if (!to_pref && best_rssi < cur.rssi + ROAM_RSSI_HYSTERESIS) return;

    ESP_LOGW(TAG, "Moving to " MACSTR " (rssi %d, was %d)%s", MAC2STR(best), best_rssi, cur.rssi,
             to_pref ? ": the preferred AP" : "");
    if (connect_to(best)) {
        taskENTER_CRITICAL(&s_mux);
        s_stats.roams++;
        taskEXIT_CRITICAL(&s_mux);
    } else {
        ESP_LOGW(TAG, "Move failed, back to the strongest AP");
        reconnect();
    }
}

static void wifi_task(void *arg)
{
    int64_t roam_checked_us = esp_timer_get_time();
    int64_t pref_checked_us = roam_checked_us;

    while (1) {
        EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_DISC_BIT | WIFI_SCAN_BIT,
                                               pdTRUE, pdFALSE, pdMS_TO_TICKS(5000));
        int64_t now = esp_timer_get_time();
        bool connected = xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT;

        /* A disconnect, or a network move (802.11v) that did not come back */
        if (!connected && ((bits & WIFI_DISC_BIT) || now - s_stats.disconnected_us >= LOST_US)) {
            reconnect();
            continue;
        }
        if (bits & WIFI_SCAN_BIT) {
            scan_ssid(s_current_ssid);
        }
        if (connected && now - roam_checked_us >= ROAM_CHECK_US) {
            roam_checked_us = now;
            roam_check(&pref_checked_us);
        }
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *ev = (wifi_event_sta_connected_t *)event_data;
        taskENTER_CRITICAL(&s_mux);
        memcpy(s_stats.bssid, ev->bssid, 6);
        s_stats.channel = ev->channel;
        s_stats.connected_us = esp_timer_get_time();
        taskEXIT_CRITICAL(&s_mux);
        ESP_LOGI(TAG, "Associated with " MACSTR " on channel %d", MAC2STR(ev->bssid), ev->channel);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *ev = (wifi_event_sta_disconnected_t *)event_data;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        int64_t now = esp_timer_get_time();
        taskENTER_CRITICAL(&s_mux);
        s_stats.last_reason = ev->reason;
        s_stats.disconnected_us = now;
        s_stats.disconnects++;
        if (ev->reason == WIFI_REASON_ROAMING) {
            s_stats.btm_roams++;
            s_stats.last_btm_us = now;
        }
        taskEXIT_CRITICAL(&s_mux);

        if (ev->reason == WIFI_REASON_ROAMING) {
            /* The network asked us to move (802.11v): the supplicant connects
             * to the new AP itself, and the task steps in only if it does not */
            ESP_LOGI(TAG, "Moved by the network (802.11v)");
        } else if (s_managed) {
            ESP_LOGW(TAG, "Disconnected, reason %u", ev->reason);
            if (!s_switching) xEventGroupSetBits(s_wifi_event_group, WIFI_DISC_BIT);
        } else if (s_retry_num < s_max_retry) {
            s_retry_num++;
            ESP_LOGW(TAG, "Retrying connection (%d/%d)...", s_retry_num, s_max_retry);
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Connected — IP: %s", s_ip_str);
        s_retry_num = 0;

        /* Start SNTP time sync (non-blocking, runs in background) */
        if (!esp_sntp_enabled()) {
            esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
            esp_sntp_setservername(0, "pool.ntp.org");
            esp_sntp_setservername(1, "time.google.com");
            esp_sntp_init();
            ESP_LOGI(TAG, "SNTP started");
        }

        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool nvs_load_creds(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return false;

    bool ok = (nvs_get_str(nvs, NVS_KEY_SSID, ssid, &ssid_len) == ESP_OK &&
               ssid[0] != '\0');
    if (ok)
        nvs_get_str(nvs, NVS_KEY_PASSWORD, pass, &pass_len);

    nvs_close(nvs);
    return ok;
}

static void nvs_save_creds(const char *ssid, const char *pass)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK)
        return;

    nvs_set_str(nvs, NVS_KEY_SSID, ssid);
    nvs_set_str(nvs, NVS_KEY_PASSWORD, pass);
    nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "WiFi credentials saved to NVS");
}

static void load_hostname(void)
{
    bool loaded = false;
    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        size_t len = sizeof(s_hostname);
        if (nvs_get_str(nvs, NVS_KEY_HOSTNAME, s_hostname, &len) == ESP_OK &&
            s_hostname[0] != '\0') {
            loaded = true;
        }
        nvs_close(nvs);
    }

    if (!loaded) {
        /* Default: esp32fdm-XXYY using last 2 bytes of STA MAC */
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        snprintf(s_hostname, sizeof(s_hostname), "%s-%02x%02x",
                 DEFAULT_HOSTNAME, mac[4], mac[5]);
    }
}

static void apply_hostname(void)
{
    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif) {
        esp_netif_set_hostname(netif, s_hostname);
        ESP_LOGI(TAG, "DHCP hostname set to \"%s\"", s_hostname);
    }
}

static esp_err_t try_sta_connect(const char *ssid, const char *pass)
{
    ESP_LOGI(TAG, "Connecting to \"%s\"...", ssid);

    if (!s_sta_netif_created) {
        esp_netif_create_default_wifi_sta();
        s_sta_netif_created = true;
        apply_hostname();
    }

    strlcpy(s_current_ssid, ssid, sizeof(s_current_ssid));
    strlcpy(s_current_pass, pass, sizeof(s_current_pass));

    wifi_config_t wifi_config;
    make_config(&wifi_config, NULL);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    s_retry_num = 0;
    s_max_retry = MAX_RETRY;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    ESP_ERROR_CHECK(esp_wifi_start());

    /* The preferred AP if it is strong enough, else the strongest */
    ESP_LOGI(TAG, "Scanning for the best AP...");
    uint8_t bssid[6];
    int8_t rssi;
    bool locked = choose_ap(ssid, bssid, &rssi);
    if (locked) {
        ESP_LOGI(TAG, "Chosen AP: " MACSTR " (rssi=%d)", MAC2STR(bssid), rssi);
        make_config(&wifi_config, bssid);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
        return ESP_OK;

    /* If BSSID-locked connect failed, retry without lock */
    if (locked) {
        ESP_LOGW(TAG, "BSSID-locked connect failed, trying any AP...");
        make_config(&wifi_config, NULL);
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        s_retry_num = 0;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
        esp_wifi_connect();

        bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, portMAX_DELAY);
        if (bits & WIFI_CONNECTED_BIT)
            return ESP_OK;
    }

    ESP_LOGW(TAG, "Failed to connect to \"%s\"", ssid);
    esp_wifi_stop();
    return ESP_FAIL;
}

static void start_ap_mode(void)
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);

    char ap_ssid[32];
    snprintf(ap_ssid, sizeof(ap_ssid), "ESP32FDM-%02X%02X", mac[4], mac[5]);

    esp_netif_create_default_wifi_ap();

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.ap.ssid, ap_ssid, sizeof(wifi_config.ap.ssid) - 1);
    wifi_config.ap.ssid_len = strlen(ap_ssid);
    wifi_config.ap.channel = 1;
    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    wifi_config.ap.max_connection = 2;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    snprintf(s_ip_str, sizeof(s_ip_str), "192.168.4.1");
    ESP_LOGW(TAG, "AP mode started: \"%s\" — configure at http://192.168.4.1/", ap_ssid);
}

/** From here on the Wi-Fi task keeps the connection, whatever happens */
static void start_managing(void)
{
    s_managed = true;
    /* 4 KB: 3 KB had 392 bytes left on boards that switch APs, since each
     * line logged (the AP list) takes over 1 KB of stack */
    task_create_psram(wifi_task, "wifi_mgr", 8192, NULL, 2, NULL, 0);
}

wifi_result_t wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    s_scan_lock = xSemaphoreCreateMutex();
    load_hostname();
    load_ap_prefs();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL));

    /* Try NVS credentials first */
    char ssid[33] = {0};
    char pass[65] = {0};

    if (nvs_load_creds(ssid, sizeof(ssid), pass, sizeof(pass))) {
        ESP_LOGI(TAG, "Found WiFi credentials in NVS");
        if (try_sta_connect(ssid, pass) == ESP_OK) {
            start_managing();
            return WIFI_RESULT_STA_CONNECTED;
        }
    }

    /* Try Kconfig credentials (skip if default "myssid") */
    if (strcmp(CONFIG_WIFI_SSID, "myssid") != 0) {
        ESP_LOGI(TAG, "Trying Kconfig credentials...");
        if (try_sta_connect(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD) == ESP_OK) {
            nvs_save_creds(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
            start_managing();
            return WIFI_RESULT_STA_CONNECTED;
        }
    }

    /* All attempts failed — start AP mode */
    start_ap_mode();
    return WIFI_RESULT_AP_MODE;
}

const char *wifi_get_ip_str(void)
{
    return s_ip_str;
}

bool wifi_get_ssid(char *buf, size_t buf_size)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK)
        return false;

    size_t len = buf_size;
    bool ok = (nvs_get_str(nvs, NVS_KEY_SSID, buf, &len) == ESP_OK && buf[0] != '\0');
    nvs_close(nvs);
    return ok;
}

void wifi_reset_credentials(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_key(nvs, NVS_KEY_SSID);
        nvs_erase_key(nvs, NVS_KEY_PASSWORD);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGW(TAG, "WiFi credentials erased — rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}

const char *wifi_get_hostname(void)
{
    return s_hostname;
}

void wifi_set_hostname(const char *name)
{
    if (!name || !name[0])
        return;

    strncpy(s_hostname, name, sizeof(s_hostname) - 1);
    s_hostname[sizeof(s_hostname) - 1] = '\0';

    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_str(nvs, NVS_KEY_HOSTNAME, s_hostname);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGI(TAG, "Hostname saved: \"%s\" (takes effect on reboot)", s_hostname);
}
