#pragma once

#include "esp_err.h"
#include "layout.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the Obico client.
 * Checks NVS for auth token. If linked, starts WebSocket and snapshot tasks.
 * Must be called after wifi_init_sta(), camera_init(), and printer_comm_init().
 */
esp_err_t obico_client_init(void);

/**
 * Returns true if the device is linked to an Obico server (has auth token).
 */
bool obico_is_linked(void);

/**
 * Link device with a 6-digit code from Obico.
 * POSTs to the server to verify the code and stores the token in NVS.
 * Returns ESP_OK on success.
 */
esp_err_t obico_link_with_code(const char *code);

/**
 * Unlink device — clears auth token from NVS and stops tasks.
 */
esp_err_t obico_unlink(void);

/**
 * Register Obico HTTP endpoints on the given server.
 * Called from httpd.c during server setup.
 */
esp_err_t obico_register_httpd(void *server_handle);

/**
 * Render Obico link section HTML into a page buffer.
 * Used by the unified settings page.
 */
void obico_render_settings(html_buf_t *p);

/**
 * Get current Obico server URL.
 */
const char *obico_get_server_url(void);

/**
 * Set Obico server URL. Saves to NVS.
 * Only takes effect on next link or WS reconnect.
 */
esp_err_t obico_set_server_url(const char *url);

/**
 * Get current Janus proxy host (empty string if not configured).
 */
const char *obico_get_janus_host(void);

/**
 * Get current Janus proxy port.
 */
uint16_t obico_get_janus_port(void);

/**
 * Set Janus proxy host and port. Saves to NVS and reconnects if needed.
 * Pass empty/NULL host to disable.
 */
esp_err_t obico_set_janus_proxy(const char *host, uint16_t port);

#ifdef __cplusplus
}
#endif
