#include "wifi.h"
#include "camera.h"
#include "httpd.h"
#include "usb_serial.h"
#include "rfc2217.h"

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 FDM Printer Bridge starting...");

    /* NVS — required by WiFi */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* WiFi — blocks until connected */
    ESP_ERROR_CHECK(wifi_init_sta());

    /* Camera */
    ESP_ERROR_CHECK(camera_init());

    /* HTTP server (MJPEG stream + snapshot) */
    ESP_ERROR_CHECK(httpd_start_server());

    /* USB Host serial (printer connection) */
    ESP_ERROR_CHECK(usb_serial_init(rfc2217_feed_serial_data, NULL));

    /* RFC 2217 server (OctoPrint serial bridge) */
    ESP_ERROR_CHECK(rfc2217_start());

    /* Ready */
    const char *ip = wifi_get_ip_str();
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32 FDM Bridge READY");
    ESP_LOGI(TAG, "  Webcam stream: http://%s/stream", ip);
    ESP_LOGI(TAG, "  Snapshot:      http://%s/capture", ip);
    ESP_LOGI(TAG, "  Serial port:   rfc2217://%s:%d", ip, CONFIG_RFC2217_PORT);
    ESP_LOGI(TAG, "========================================");
}
