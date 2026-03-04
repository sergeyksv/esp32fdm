#include "camera.h"

#include "esp_log.h"

static const char *TAG = "camera";

/*
 * Freenove ESP32-S3-WROOM pin mapping for OV2640 (DVP parallel interface).
 * Reference: Freenove schematic + board silkscreen.
 */
#define CAM_PIN_PWDN   -1  /* No power-down pin */
#define CAM_PIN_RESET   -1  /* No hardware reset pin */
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD    4   /* SCCB SDA */
#define CAM_PIN_SIOC    5   /* SCCB SCL */

#define CAM_PIN_D7      16
#define CAM_PIN_D6      17
#define CAM_PIN_D5      18
#define CAM_PIN_D4      12
#define CAM_PIN_D3      10
#define CAM_PIN_D2      8
#define CAM_PIN_D1      9
#define CAM_PIN_D0      11

#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    7
#define CAM_PIN_PCLK    13

esp_err_t camera_init(void)
{
    camera_config_t config = {
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,
        .pin_xclk     = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7       = CAM_PIN_D7,
        .pin_d6       = CAM_PIN_D6,
        .pin_d5       = CAM_PIN_D5,
        .pin_d4       = CAM_PIN_D4,
        .pin_d3       = CAM_PIN_D3,
        .pin_d2       = CAM_PIN_D2,
        .pin_d1       = CAM_PIN_D1,
        .pin_d0       = CAM_PIN_D0,
        .pin_vsync    = CAM_PIN_VSYNC,
        .pin_href     = CAM_PIN_HREF,
        .pin_pclk     = CAM_PIN_PCLK,

        .xclk_freq_hz = 10000000,  /* 10 MHz — less PSRAM bus contention */
        .ledc_timer   = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_SVGA, /* 800x600 */
        .jpeg_quality = 20,             /* 1-63, lower = better quality */
        .fb_count     = 3,              /* 3 for GRAB_LATEST: DMA + ready + consumer */
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    ESP_LOGI(TAG, "Camera initialized (SVGA JPEG, 3 buffers in PSRAM)");
    return ESP_OK;
}

camera_fb_t *camera_capture_frame(void)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGW(TAG, "Frame capture failed");
    }
    return fb;
}
