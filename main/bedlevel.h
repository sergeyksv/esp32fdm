#pragma once

#include "esp_http_server.h"

/**
 * Register bed leveling HTTP endpoints on the given server.
 * Marlin-only feature — call only when backend is Marlin.
 */
esp_err_t bedlevel_register_httpd(httpd_handle_t server);
