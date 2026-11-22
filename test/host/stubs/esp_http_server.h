#pragma once
/* Only the types that appear in the project's headers; no handler runs in the tests */
#include <stddef.h>
#include "esp_err.h"
typedef void *httpd_handle_t;
typedef enum { HTTP_GET = 1, HTTP_POST = 3 } httpd_method_t;
typedef struct httpd_req { size_t content_len; const char *uri; int method; void *user_ctx; } httpd_req_t;
typedef struct httpd_uri {
    const char *uri; httpd_method_t method; esp_err_t (*handler)(httpd_req_t *r); void *user_ctx;
} httpd_uri_t;
typedef int httpd_err_code_t;
#define HTTPD_400_BAD_REQUEST 400
#define HTTPD_404_NOT_FOUND 404
#define HTTPD_500_INTERNAL_SERVER_ERROR 500
int httpd_req_recv(httpd_req_t *r, char *buf, size_t len);
esp_err_t httpd_resp_send(httpd_req_t *r, const char *buf, long len);
esp_err_t httpd_resp_send_err(httpd_req_t *r, httpd_err_code_t code, const char *msg);
esp_err_t httpd_resp_set_hdr(httpd_req_t *r, const char *k, const char *v);
esp_err_t httpd_resp_set_status(httpd_req_t *r, const char *s);
esp_err_t httpd_resp_set_type(httpd_req_t *r, const char *t);
esp_err_t httpd_resp_sendstr(httpd_req_t *r, const char *s);
esp_err_t httpd_resp_sendstr_chunk(httpd_req_t *r, const char *s);
esp_err_t httpd_resp_send_chunk(httpd_req_t *r, const char *buf, long len);
esp_err_t httpd_register_uri_handler(httpd_handle_t h, const httpd_uri_t *u);
