#include "wifi.h"
#include "task_util.h"
#include "camera.h"
#include "httpd.h"
#include "dns_server.h"
#include "usb_serial.h"
#include "sdcard.h"
#include "printer_comm.h"
#include "obico_client.h"
#include "rfc2217.h"
#include "terminal.h"
#include "logbuf.h"
#include "mdns_service.h"
#include "maintenance.h"
#include "board_led.h"
#include "nozzle_guard.h"
#include "print_checks.h"
#include "ota.h"
#include "settings_copy.h"
#include "print_stats.h"
#include "hub_client.h"
#include "esp_littlefs.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_memory_utils.h"

#include <string.h>

static const char *TAG = "main";

static void log_heap(const char *label)
{
    ESP_LOGI(TAG, "HEAP [%s] DMA: %lu free, %lu min, largest: %lu | Internal: %lu free | PSRAM: %lu free",
             label,
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA),
             (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_DMA),
             (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DMA),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

/* Stack watch: a task that ever had less than this left of its stack is
 * logged and reported to the hub, once per task per boot, so a stack that is
 * too small shows before it overflows (the led task's 2560 bytes cost reboots
 * mid-print until 2.6). ESP-IDF's IDLE and ipc tasks run with ~520 bytes by
 * design and none of our code; the clean-up task of vTaskDeleteWithCaps()
 * lives for a moment with the minimal stack */
#define STACK_WARN_BYTES  768
#define STACK_WARN_MAX    16

static void check_stacks(void)
{
    static char warned[STACK_WARN_MAX][configMAX_TASK_NAME_LEN];
    static int n_warned;

    UBaseType_t n = uxTaskGetNumberOfTasks() + 4;
    TaskStatus_t *ts = heap_caps_malloc(n * sizeof(TaskStatus_t), MALLOC_CAP_SPIRAM);
    if (!ts) return;
    n = uxTaskGetSystemState(ts, n, NULL);
    for (UBaseType_t i = 0; i < n; i++) {
        const char *name = ts[i].pcTaskName;
        unsigned left = ts[i].usStackHighWaterMark;
        if (left >= STACK_WARN_BYTES || !strncmp(name, "IDLE", 4) || !strncmp(name, "ipc", 3) ||
            !strncmp(name, "prvTaskDelete", 13)) continue;
        bool seen = false;
        for (int k = 0; k < n_warned && !seen; k++) seen = !strcmp(warned[k], name);
        if (seen) continue;
        if (n_warned < STACK_WARN_MAX) strlcpy(warned[n_warned++], name, sizeof(warned[0]));

        char msg[96];
        snprintf(msg, sizeof(msg), "task %s: %u bytes of stack left at the least, %s",
                 name, left, esp_ptr_external_ram(ts[i].pxStackBase) ? "PSRAM" : "internal RAM");
        ESP_LOGW(TAG, "Stack low: %s", msg);
        hub_client_send_event(HUB_EVENT_STACK_LOW, msg, NULL);
    }
    free(ts);
}

static void heap_monitor_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(60000));
        log_heap("periodic");
        check_stacks();
    }
}

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

    /* Confirm OTA boot — cancels rollback if this is a newly flashed firmware */
    ota_confirm_boot();

    /* Log buffer — capture boot logs from here on */
    logbuf_init();

    /* WiFi — try STA, fall back to AP */
    wifi_result_t wifi_result = wifi_init();

    if (wifi_result == WIFI_RESULT_STA_CONNECTED) {
        mdns_service_init();
    }

    if (wifi_result == WIFI_RESULT_AP_MODE) {
        dns_server_start();
        httpd_start_captive_portal();
        ESP_LOGW(TAG, "========================================");
        ESP_LOGW(TAG, "  AP MODE — configure WiFi at");
        ESP_LOGW(TAG, "  http://192.168.4.1/");
        ESP_LOGW(TAG, "========================================");
        return;
    }

    log_heap("pre-camera");

    /* Camera */
    ESP_ERROR_CHECK(camera_init());
    camera_start_capture_task();
    log_heap("post-camera");

    /* SD card — optional, non-fatal if no card inserted */
    sdcard_init();

    /* LittleFS cache partition (~12MB) */
    esp_vfs_littlefs_conf_t lfs_conf = {
        .base_path = "/cache",
        .partition_label = "cache",
        .format_if_mount_failed = true,
    };
    ret = esp_vfs_littlefs_register(&lfs_conf);
    if (ret == ESP_OK) {
        size_t total = 0, used = 0;
        esp_littlefs_info("cache", &total, &used);
        ESP_LOGI(TAG, "LittleFS /cache: %u KB used / %u KB total",
                 (unsigned)(used / 1024), (unsigned)(total / 1024));
    } else {
        ESP_LOGW(TAG, "LittleFS mount failed: %s", esp_err_to_name(ret));
    }

    /* Terminal ring buffer (must init before HTTP server & USB RX) */
    terminal_init();

    /* HTTP server (MJPEG stream + snapshot + SD card + all endpoints) */
    ESP_ERROR_CHECK(httpd_start_server());
    log_heap("post-httpd");

    /* Before the backends, which ask it in every wait */
    nozzle_guard_init();
    print_checks_init();

    /* Printer comm — always init with USB serial */
    ESP_ERROR_CHECK(usb_serial_init(printer_comm_rx_cb, NULL));
    ESP_ERROR_CHECK(printer_comm_init());

    /* Events for the notification hub */
    hub_client_init();
    settings_copy_init();
    board_led_init();
    print_stats_init();
    maintenance_init();
    ota_report_update();

    /* Obico client */
    ESP_ERROR_CHECK(obico_client_init());
    log_heap("post-obico");

    /* 4 KB: 3 KB had 580 bytes left */
    task_create_psram(heap_monitor_task, "heap_mon", 8192, NULL, 1, NULL, 0);

    /* RFC 2217 serial bridge: OctoPrint on the USB printer, Marlin only */
    if (printer_comm_get_backend() == PRINTER_BACKEND_MARLIN) ESP_ERROR_CHECK(rfc2217_start());

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
