#pragma once

#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "i18n.h"

/**
 * Register a URI handler with error logging.
 * Logs an error if registration fails (e.g. max_uri_handlers exceeded).
 */
#define HTTPD_REGISTER(server, uri_ptr) do { \
    /* Every handler first picks the page language: see i18n_handler_wrap() */ \
    httpd_uri_t _uri = *(uri_ptr); \
    _uri.user_ctx = (void *)_uri.handler; \
    _uri.handler = i18n_handler_wrap; \
    esp_err_t _err = httpd_register_uri_handler(server, &_uri); \
    if (_err != ESP_OK) { \
        ESP_LOGE("httpd", "Failed to register %s %s: %s (increase max_uri_handlers?)", \
                 (uri_ptr)->method == HTTP_GET ? "GET" : "POST", \
                 (uri_ptr)->uri, esp_err_to_name(_err)); \
    } \
} while(0)

/**
 * Start the HTTP server on port 80.
 * Registers /stream (MJPEG) and /capture (single JPEG) endpoints.
 */
esp_err_t httpd_start_server(void);

/**
 * Start a captive portal HTTP server for WiFi configuration.
 * Only GET / (config form), POST / (save + reboot), and wildcard redirect.
 */
esp_err_t httpd_start_captive_portal(void);

/**
 * Add a babystep made outside /api/command to the Z offset shown in the UI.
 */
void httpd_note_babystep(float dz);
