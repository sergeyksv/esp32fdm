#pragma once

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Register SD card file management and print control HTTP endpoints.
 * Requires sdcard_init() to have been called first.
 */
esp_err_t sdcard_httpd_register(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
