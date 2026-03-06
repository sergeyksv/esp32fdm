#pragma once

#include "esp_camera.h"
#include "esp_err.h"

#include <stdint.h>
#include <stddef.h>

/**
 * Ref-counted PSRAM frame — shared by all consumers.
 * Obtain via camera_get_frame(), release via camera_release_frame().
 */
typedef struct {
    uint8_t *buf;       /* JPEG data (in PSRAM) */
    size_t   len;       /* Actual JPEG size */
    size_t   buf_size;  /* Allocated buffer size */
    int      refcount;  /* Active consumers */
} camera_frame_t;

/**
 * Initialize the OV2640 camera with Freenove ESP32-S3 pin mapping.
 * JPEG VGA, 2 frame buffers in PSRAM, GRAB_LATEST mode.
 */
esp_err_t camera_init(void);

/**
 * Start the background capture task.
 * Allocates double-buffered PSRAM slots and captures at ~10 FPS.
 * Call once after camera_init().
 */
void camera_start_capture_task(void);

/**
 * Get a ref-counted pointer to the latest captured frame.
 * Returns NULL if no frame is available yet.
 * Caller MUST call camera_release_frame() when done.
 */
camera_frame_t *camera_get_frame(void);

/**
 * Release a frame obtained from camera_get_frame().
 */
void camera_release_frame(camera_frame_t *frame);

/**
 * Set 180° rotation (hmirror + vflip via sensor registers). Saved to NVS.
 * Takes effect on the next captured frame — no reboot required.
 */
void camera_set_rotate180(bool enable);

/**
 * Get current 180° rotation state.
 */
bool camera_get_rotate180(void);
