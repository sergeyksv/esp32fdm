#pragma once

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Allocate PSRAM ring buffer and install esp_log_set_vprintf() hook.
 * Call as early as possible (right after NVS init) to capture boot logs.
 */
void logbuf_init(void);

/**
 * Register /logs HTTP endpoint.
 */
esp_err_t logbuf_register_httpd(httpd_handle_t server);

#ifdef __cplusplus
}
#endif
