#include "wifi.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

static const char *TAG = "wifi";

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define MAX_RETRY          3

#define NVS_WIFI_NAMESPACE "wifi"
#define NVS_KEY_SSID       "ssid"
#define NVS_KEY_PASSWORD   "password"
#define NVS_KEY_HOSTNAME   "hostname"
#define DEFAULT_HOSTNAME   "esp32fdm"

#define ROAM_RSSI_THRESHOLD   (-70)   /* trigger scan when below this */
#define ROAM_RSSI_HYSTERESIS  10      /* only roam if new AP is this much better */
#define ROAM_SCAN_INTERVAL_MS 30000   /* min interval between roam scans */

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;
static int s_max_retry;
static char s_ip_str[16];
static bool s_sta_netif_created;
static char s_current_ssid[33];
static char s_current_pass[65];
static char s_hostname[32] = DEFAULT_HOSTNAME;

/* Scan for all APs matching ssid, return the BSSID of the strongest one.
 * Returns true if a match was found (best_bssid and best_rssi populated). */
static bool scan_best_ap(const char *ssid, uint8_t *best_bssid, int8_t *best_rssi)
{
    wifi_scan_config_t scan_cfg = {
        .ssid = (uint8_t *)ssid,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active = { .min = 100, .max = 300 },
    };

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Scan failed: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t ap_count = 0;
    esp_wifi_scan_get_ap_num(&ap_count);
    if (ap_count == 0) {
        esp_wifi_clear_ap_list();
        return false;
    }

    uint16_t fetch = ap_count > 20 ? 20 : ap_count;
    wifi_ap_record_t *ap_list = calloc(fetch, sizeof(wifi_ap_record_t));
    if (!ap_list) {
        esp_wifi_clear_ap_list();
        return false;
    }

    esp_wifi_scan_get_ap_records(&fetch, ap_list);

    bool found = false;
    int8_t top_rssi = -127;
    for (int i = 0; i < fetch; i++) {
        if (strcmp((char *)ap_list[i].ssid, ssid) != 0)
            continue;
        ESP_LOGI(TAG, "  AP: %02x:%02x:%02x:%02x:%02x:%02x  ch=%d  rssi=%d",
                 ap_list[i].bssid[0], ap_list[i].bssid[1], ap_list[i].bssid[2],
                 ap_list[i].bssid[3], ap_list[i].bssid[4], ap_list[i].bssid[5],
                 ap_list[i].primary, ap_list[i].rssi);
        if (ap_list[i].rssi > top_rssi) {
            top_rssi = ap_list[i].rssi;
            memcpy(best_bssid, ap_list[i].bssid, 6);
            found = true;
        }
    }

    free(ap_list);
    if (found)
        *best_rssi = top_rssi;
    return found;
}

static void roam_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(5000)); /* wait for initial connection to settle */

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(ROAM_SCAN_INTERVAL_MS));

        /* Only roam if connected */
        EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
        if (!(bits & WIFI_CONNECTED_BIT))
            continue;

        /* Check current RSSI */
        wifi_ap_record_t current_ap;
        if (esp_wifi_sta_get_ap_info(&current_ap) != ESP_OK)
            continue;

        if (current_ap.rssi >= ROAM_RSSI_THRESHOLD)
            continue;

        ESP_LOGI(TAG, "RSSI %d below threshold %d, scanning for better AP...",
                 current_ap.rssi, ROAM_RSSI_THRESHOLD);

        uint8_t best_bssid[6];
        int8_t best_rssi;
        if (!scan_best_ap(s_current_ssid, best_bssid, &best_rssi))
            continue;

        /* Only roam if the best AP is significantly better */
        if (best_rssi <= current_ap.rssi + ROAM_RSSI_HYSTERESIS) {
            ESP_LOGI(TAG, "Best AP rssi=%d, current=%d — not worth roaming",
                     best_rssi, current_ap.rssi);
            continue;
        }

        /* Don't roam to the same AP */
        if (memcmp(best_bssid, current_ap.bssid, 6) == 0)
            continue;

        ESP_LOGW(TAG, "Roaming to %02x:%02x:%02x:%02x:%02x:%02x (rssi=%d, was %d)",
                 best_bssid[0], best_bssid[1], best_bssid[2],
                 best_bssid[3], best_bssid[4], best_bssid[5],
                 best_rssi, current_ap.rssi);

        /* Update config to target the better AP */
        wifi_config_t wifi_config = {};
        strncpy((char *)wifi_config.sta.ssid, s_current_ssid, sizeof(wifi_config.sta.ssid) - 1);
        strncpy((char *)wifi_config.sta.password, s_current_pass, sizeof(wifi_config.sta.password) - 1);
        memcpy(wifi_config.sta.bssid, best_bssid, 6);
        wifi_config.sta.bssid_set = true;
        wifi_config.sta.rm_enabled = 1;
        wifi_config.sta.btm_enabled = 1;

        s_max_retry = MAX_RETRY;
        s_retry_num = 0;
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
        esp_wifi_disconnect();
        esp_wifi_connect();

        /* Wait for reconnect */
        xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(15000));

        bits = xEventGroupGetBits(s_wifi_event_group);
        if (!(bits & WIFI_CONNECTED_BIT)) {
            /* Roam failed — reconnect without BSSID lock */
            ESP_LOGW(TAG, "Roam failed, reconnecting to any AP...");
            wifi_config.sta.bssid_set = false;
            esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
            esp_wifi_connect();
        }
    }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        if (s_retry_num < s_max_retry) {
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

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    wifi_config.sta.rm_enabled = 1;   /* 802.11k Radio Measurement */
    wifi_config.sta.btm_enabled = 1;  /* 802.11v BSS Transition Management */

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    s_retry_num = 0;
    s_max_retry = MAX_RETRY;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    ESP_ERROR_CHECK(esp_wifi_start());

    /* Scan for the strongest AP before connecting */
    ESP_LOGI(TAG, "Scanning for best AP...");
    uint8_t best_bssid[6];
    int8_t best_rssi;
    if (scan_best_ap(ssid, best_bssid, &best_rssi)) {
        ESP_LOGI(TAG, "Best AP: %02x:%02x:%02x:%02x:%02x:%02x (rssi=%d)",
                 best_bssid[0], best_bssid[1], best_bssid[2],
                 best_bssid[3], best_bssid[4], best_bssid[5], best_rssi);
        /* Lock to best BSSID */
        memcpy(wifi_config.sta.bssid, best_bssid, 6);
        wifi_config.sta.bssid_set = true;
        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    }

    /* Save for roaming use */
    strncpy(s_current_ssid, ssid, sizeof(s_current_ssid) - 1);
    strncpy(s_current_pass, pass, sizeof(s_current_pass) - 1);

    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
        return ESP_OK;

    /* If BSSID-locked connect failed, retry without lock */
    if (wifi_config.sta.bssid_set) {
        ESP_LOGW(TAG, "BSSID-locked connect failed, trying any AP...");
        wifi_config.sta.bssid_set = false;
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

wifi_result_t wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();
    load_hostname();

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
            xTaskCreatePinnedToCore(roam_task, "roam", 4096, NULL, 2, NULL, 0);
            return WIFI_RESULT_STA_CONNECTED;
        }
    }

    /* Try Kconfig credentials (skip if default "myssid") */
    if (strcmp(CONFIG_WIFI_SSID, "myssid") != 0) {
        ESP_LOGI(TAG, "Trying Kconfig credentials...");
        if (try_sta_connect(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD) == ESP_OK) {
            nvs_save_creds(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
            xTaskCreatePinnedToCore(roam_task, "roam", 4096, NULL, 2, NULL, 0);
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
