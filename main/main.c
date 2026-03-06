#include "wifi.h"
#include "camera.h"
#include "httpd.h"
#include "dns_server.h"
#include "usb_serial.h"
#include "sdcard.h"
#include "printer_comm.h"
#include "obico_client.h"
#include "rfc2217.h"
#include "terminal.h"

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
    camera_start_capture_task();

    /* SD card — optional, non-fatal if no card inserted */
    sdcard_init();

    /* Terminal ring buffer (must init before HTTP server & USB RX) */
    terminal_init();

    /* HTTP server (MJPEG stream + snapshot + SD card + all endpoints) */
    ESP_ERROR_CHECK(httpd_start_server());

    /* Printer comm — always init with USB serial */
    ESP_ERROR_CHECK(usb_serial_init(printer_comm_rx_cb, NULL));
    ESP_ERROR_CHECK(printer_comm_init());

    /* Obico client */
    ESP_ERROR_CHECK(obico_client_init());

    /* RFC 2217 serial bridge */
    ESP_ERROR_CHECK(rfc2217_start());

    const char *ip = wifi_get_ip_str();
    printer_backend_t backend = printer_comm_get_backend();
    ESP_LOGI(TAG, "========================================");
    if (backend == PRINTER_BACKEND_KLIPPER) {
        ESP_LOGI(TAG, "  ESP32 FDM Bridge READY (Klipper → %s:%u)",
                 printer_comm_get_mr_host(), printer_comm_get_mr_port());
    } else {
        ESP_LOGI(TAG, "  ESP32 FDM Bridge READY (Marlin)");
    }
    ESP_LOGI(TAG, "  http://%s/", ip);
    ESP_LOGI(TAG, "========================================");
}
