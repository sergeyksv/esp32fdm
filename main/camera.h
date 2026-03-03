#pragma once

#include "esp_camera.h"
#include "esp_err.h"

/**
 * Initialize the OV2640 camera with Freenove ESP32-S3 pin mapping.
 * JPEG VGA, 2 frame buffers in PSRAM, GRAB_LATEST mode.
 */
esp_err_t camera_init(void);

/**
 * Capture a single JPEG frame.
 * Returns a camera frame buffer — caller MUST call esp_camera_fb_return() when done.
 */
camera_fb_t *camera_capture_frame(void);
