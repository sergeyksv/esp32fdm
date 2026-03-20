#pragma once

#include "esp_http_server.h"
#include "layout.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Mark current firmware as valid (call early in app_main to confirm boot).
 * Cancels automatic rollback if this is a new OTA firmware.
 */
void ota_confirm_boot(void);

/**
 * Register OTA HTTP endpoints on the given server:
 *   POST /ota/upload  — receive firmware .bin and flash to next OTA slot
 */
void ota_register_httpd(httpd_handle_t server);

/**
 * Render the firmware update section for the settings page.
 */
void ota_render_settings(html_buf_t *p);

#ifdef __cplusplus
}
#endif
