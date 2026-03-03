#pragma once

#include "esp_err.h"
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

#ifdef __cplusplus
}
#endif
