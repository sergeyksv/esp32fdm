#include "camera.h"

#include <string.h>

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static const char *TAG = "camera";

#define NVS_NS_CAMERA  "camera"
#define NVS_KEY_ROT180 "rotate180"

static bool s_rotate180 = false;

/* ---- Double-buffered PSRAM frames ---- */

#define FRAME_BUF_INIT_SIZE (50 * 1024)  /* 50 KB — covers most SVGA Q20 frames */
#define CAPTURE_FPS         10
#define CAPTURE_INTERVAL_MS (1000 / CAPTURE_FPS)

#define FRAME_COUNT 3

static camera_frame_t s_frames[FRAME_COUNT];
static camera_frame_t *s_latest;       /* consumer-facing (front buffer) */
static SemaphoreHandle_t s_frame_mutex;

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
        .fb_count     = 2,              /* 2 DMA buffers */
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    /* Load rotation setting from NVS and apply */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_CAMERA, NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t val = 0;
        if (nvs_get_u8(nvs, NVS_KEY_ROT180, &val) == ESP_OK) {
            s_rotate180 = (val != 0);
        }
        nvs_close(nvs);
    }
    if (s_rotate180) {
        sensor_t *s = esp_camera_sensor_get();
        if (s) {
            s->set_hmirror(s, 1);
            s->set_vflip(s, 1);
        }
    }

    ESP_LOGI(TAG, "Camera initialized (SVGA JPEG, 2 DMA buffers, rotate180=%d)", s_rotate180);
    return ESP_OK;
}

/* ---- Background capture task ---- */

static void camera_capture_task(void *arg)
{
    ESP_LOGI(TAG, "Capture task started (%d FPS)", CAPTURE_FPS);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(CAPTURE_INTERVAL_MS));

        /* Find a free buffer (refcount == 0, not s_latest) */
        camera_frame_t *back = NULL;
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        for (int i = 0; i < FRAME_COUNT; i++) {
            if (&s_frames[i] != s_latest && s_frames[i].refcount == 0) {
                back = &s_frames[i];
                break;
            }
        }
        xSemaphoreGive(s_frame_mutex);
        if (!back) {
            continue;  /* all buffers held by consumers */
        }

        /* Capture from DMA */
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "Frame capture failed");
            continue;
        }

        /* Grow PSRAM buffer if needed */
        if (fb->len > back->buf_size) {
            size_t new_size = fb->len + 4096;  /* some headroom */
            uint8_t *new_buf = heap_caps_realloc(back->buf, new_size,
                                                  MALLOC_CAP_SPIRAM);
            if (!new_buf) {
                ESP_LOGW(TAG, "PSRAM realloc failed (%u bytes)", (unsigned)new_size);
                esp_camera_fb_return(fb);
                continue;
            }
            back->buf = new_buf;
            back->buf_size = new_size;
        }

        /* Copy JPEG to PSRAM back buffer */
        memcpy(back->buf, fb->buf, fb->len);
        back->len = fb->len;

        /* Return DMA buffer immediately */
        esp_camera_fb_return(fb);

        /* Swap: back buffer becomes the new latest */
        xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
        s_latest = back;
        xSemaphoreGive(s_frame_mutex);
    }
}

void camera_start_capture_task(void)
{
    s_frame_mutex = xSemaphoreCreateMutex();

    /* Allocate PSRAM frame buffers */
    for (int i = 0; i < FRAME_COUNT; i++) {
        s_frames[i].buf = heap_caps_malloc(FRAME_BUF_INIT_SIZE, MALLOC_CAP_SPIRAM);
        s_frames[i].buf_size = FRAME_BUF_INIT_SIZE;
        s_frames[i].len = 0;
        s_frames[i].refcount = 0;
    }

    s_latest = NULL;  /* no frame yet */

    ESP_LOGI(TAG, "Allocated %dx %u KB PSRAM frame buffers",
             FRAME_COUNT, (unsigned)(FRAME_BUF_INIT_SIZE / 1024));

    xTaskCreatePinnedToCore(camera_capture_task, "cam_cap", 3072,
                            NULL, 5, NULL, 0);
}

camera_frame_t *camera_get_frame(void)
{
    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    camera_frame_t *f = s_latest;
    if (f) {
        f->refcount++;
    }
    xSemaphoreGive(s_frame_mutex);
    return f;
}

void camera_release_frame(camera_frame_t *frame)
{
    if (!frame) return;
    xSemaphoreTake(s_frame_mutex, portMAX_DELAY);
    if (frame->refcount > 0) {
        frame->refcount--;
    }
    xSemaphoreGive(s_frame_mutex);
}

void camera_set_rotate180(bool enable)
{
    s_rotate180 = enable;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_hmirror(s, enable ? 1 : 0);
        s->set_vflip(s, enable ? 1 : 0);
    }

    nvs_handle_t nvs;
    if (nvs_open(NVS_NS_CAMERA, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_ROT180, enable ? 1 : 0);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    ESP_LOGI(TAG, "Rotate 180°: %s", enable ? "ON" : "OFF");
}

bool camera_get_rotate180(void)
{
    return s_rotate180;
}
