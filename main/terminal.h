#pragma once

#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the terminal ring buffer.
 */
void terminal_init(void);

/**
 * Feed raw RX bytes into the terminal ring buffer.
 * Called from printer_comm_rx_cb().
 */
void terminal_feed_rx(const uint8_t *data, size_t len);

/**
 * Register terminal HTTP endpoints on the given server.
 */
esp_err_t terminal_register_httpd(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
