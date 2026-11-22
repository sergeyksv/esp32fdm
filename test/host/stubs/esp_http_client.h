#pragma once
#include "esp_err.h"
typedef void *esp_http_client_handle_t;
typedef enum { HTTP_METHOD_GET = 0, HTTP_METHOD_POST } esp_http_client_method_t;
typedef struct { const char *url; esp_http_client_method_t method; int timeout_ms; } esp_http_client_config_t;
esp_http_client_handle_t esp_http_client_init(const esp_http_client_config_t *c);
esp_err_t esp_http_client_perform(esp_http_client_handle_t h);
int esp_http_client_get_status_code(esp_http_client_handle_t h);
esp_err_t esp_http_client_cleanup(esp_http_client_handle_t h);
