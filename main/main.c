#include "wifi.h"
#include "camera.h"
#include "httpd.h"
#include "dns_server.h"
#include "usb_serial.h"
#include "sdcard.h"
#include "printer_comm.h"

#if CONFIG_OBICO_ENABLED
#include "obico_client.h"
#endif

#if CONFIG_RFC2217_ENABLED
#include "rfc2217.h"
#endif

#include "esp_log.h"
#include "nvs_flash.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 FDM Printer Bridge starting...");

    /* NVS — required by WiFi and Obico token storage */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* WiFi — try STA, fall back to AP */
    wifi_result_t wifi_result = wifi_init();

    if (wifi_result == WIFI_RESULT_AP_MODE) {
        dns_server_start();
        httpd_start_captive_portal();
        ESP_LOGW(TAG, "========================================");
        ESP_LOGW(TAG, "  AP MODE — configure WiFi at");
        ESP_LOGW(TAG, "  http://192.168.4.1/");
        ESP_LOGW(TAG, "========================================");
        return;
    }

    /* Camera */
    ESP_ERROR_CHECK(camera_init());

    /* SD card — optional, non-fatal if no card inserted */
    sdcard_init();

    /* HTTP server (MJPEG stream + snapshot + SD card + Obico endpoints) */
    ESP_ERROR_CHECK(httpd_start_server());

#if CONFIG_OBICO_ENABLED
    /* Obico mode: printer_comm owns the USB serial RX callback */
    ESP_ERROR_CHECK(usb_serial_init(printer_comm_rx_cb, NULL));
    ESP_ERROR_CHECK(printer_comm_init());
    ESP_ERROR_CHECK(obico_client_init());

    const char *ip = wifi_get_ip_str();
    printer_backend_t backend = printer_comm_get_backend();
    ESP_LOGI(TAG, "========================================");
    if (backend == PRINTER_BACKEND_KLIPPER) {
        ESP_LOGI(TAG, "  ESP32 FDM Bridge READY (Obico - Klipper → %s:%u)",
                 printer_comm_get_mr_host(), printer_comm_get_mr_port());
    } else {
        ESP_LOGI(TAG, "  ESP32 FDM Bridge READY (Obico - Marlin)");
    }
    ESP_LOGI(TAG, "  http://%s/", ip);
    ESP_LOGI(TAG, "========================================");

#elif CONFIG_RFC2217_ENABLED
    /* Legacy mode: RFC 2217 owns the USB serial RX callback */
    ESP_ERROR_CHECK(usb_serial_init(rfc2217_feed_serial_data, NULL));
    ESP_ERROR_CHECK(rfc2217_start());
    /* Also init printer_comm for host SD printing */
    ESP_ERROR_CHECK(printer_comm_init());

    const char *ip = wifi_get_ip_str();
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32 FDM Bridge READY (RFC2217 mode)");
    ESP_LOGI(TAG, "  http://%s/", ip);
    ESP_LOGI(TAG, "========================================");

#else
    /* Camera + SD print mode */
    ESP_ERROR_CHECK(usb_serial_init(printer_comm_rx_cb, NULL));
    ESP_ERROR_CHECK(printer_comm_init());

    const char *ip = wifi_get_ip_str();
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32 FDM Bridge READY");
    ESP_LOGI(TAG, "  http://%s/", ip);
    ESP_LOGI(TAG, "========================================");
#endif
}
