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

static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num;
static int s_max_retry;
static char s_ip_str[16];
static bool s_sta_netif_created;

static void event_handler(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
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

static esp_err_t try_sta_connect(const char *ssid, const char *pass)
{
    ESP_LOGI(TAG, "Connecting to \"%s\"...", ssid);

    if (!s_sta_netif_created) {
        esp_netif_create_default_wifi_sta();
        s_sta_netif_created = true;
    }

    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, pass, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = pass[0] ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    s_retry_num = 0;
    s_max_retry = MAX_RETRY;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);

    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
        return ESP_OK;

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
        if (try_sta_connect(ssid, pass) == ESP_OK)
            return WIFI_RESULT_STA_CONNECTED;
    }

    /* Try Kconfig credentials (skip if default "myssid") */
    if (strcmp(CONFIG_WIFI_SSID, "myssid") != 0) {
        ESP_LOGI(TAG, "Trying Kconfig credentials...");
        if (try_sta_connect(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD) == ESP_OK) {
            nvs_save_creds(CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);
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

void wifi_reset_credentials(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_WIFI_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_erase_all(nvs);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    ESP_LOGW(TAG, "WiFi credentials erased — rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
}
