#pragma once

/* The nozzle guard's settings page section, copied settings and HTTP handler
 * (nozzle_guard.c); the decisions are in nozzle_guard.h */

#include "nozzle_guard.h"

#include "cJSON.h"
#include "esp_http_server.h"
#include "layout.h"

void nozzle_guard_render_settings(html_buf_t *p);
void nozzle_guard_settings_export(cJSON *o);
void nozzle_guard_settings_import(const cJSON *o);
esp_err_t nozzle_guard_register_httpd(httpd_handle_t server);
