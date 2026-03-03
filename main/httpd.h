#pragma once

#include "esp_err.h"

/**
 * Start the HTTP server on port 80.
 * Registers /stream (MJPEG) and /capture (single JPEG) endpoints.
 */
esp_err_t httpd_start_server(void);
