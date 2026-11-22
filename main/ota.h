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
 * Version of an update waiting for the printer to be idle, "" if none.
 */
void ota_get_pending(char *out, size_t size);

/**
 * Report a deferred update installed before this boot to the hub.
 * Call after hub_client_init().
 */
void ota_report_update(void);

/**
 * Render the firmware update section for the settings page.
 */
void ota_render_settings(html_buf_t *p);

#ifdef __cplusplus
}
#endif
