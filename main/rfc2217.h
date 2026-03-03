#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

/**
 * Start the RFC 2217 (Telnet COM-PORT-OPTION) server.
 * Listens on the configured TCP port (CONFIG_RFC2217_PORT).
 * Bridges data between TCP client (OctoPrint) and USB serial (printer).
 */
esp_err_t rfc2217_start(void);

/**
 * Feed data received from the USB serial device (printer) into the RFC 2217 server.
 * This data will be IAC-escaped and forwarded to the connected TCP client.
 * Safe to call from ISR context — uses a StreamBuffer internally.
 */
void rfc2217_feed_serial_data(const uint8_t *data, size_t len, void *user_ctx);
