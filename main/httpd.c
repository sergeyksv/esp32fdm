#include "httpd.h"

#include <string.h>
#include <stdlib.h>

#include "camera.h"
#include "wifi.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

#include "sdcard.h"
#include "sdcard_httpd.h"
#include "printer_comm.h"

#if CONFIG_OBICO_ENABLED
#include "obico_client.h"
#endif

static const char *TAG = "httpd";

#define STREAM_TARGET_FPS 5
#define STREAM_FRAME_INTERVAL_US (1000000 / STREAM_TARGET_FPS)

static const char *STREAM_BOUNDARY = "esp32fdm_frame";

/* ---- /capture handler ---- */

static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = camera_capture_frame();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

/* ---- /stream handler (MJPEG multipart) ---- */

static esp_err_t stream_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);

    /* TCP_NODELAY for low-latency frame delivery */
    int nodelay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    /* Send HTTP response header directly (not chunked) */
    const char *resp_hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace;boundary=esp32fdm_frame\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "X-Framerate: 5\r\n"
        "\r\n";
    if (httpd_send(req, resp_hdr, strlen(resp_hdr)) <= 0) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Stream client connected");

    char part_header[128];
    int64_t last_frame_time = 0;

    while (true) {
        /* Frame rate limiter */
        int64_t now = esp_timer_get_time();
        int64_t elapsed = now - last_frame_time;
        if (elapsed < STREAM_FRAME_INTERVAL_US) {
            vTaskDelay(pdMS_TO_TICKS((STREAM_FRAME_INTERVAL_US - elapsed) / 1000));
        }

        camera_fb_t *fb = camera_capture_frame();
        last_frame_time = esp_timer_get_time();
        if (!fb) {
            ESP_LOGW(TAG, "Stream: frame capture failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int hdr_len = snprintf(part_header, sizeof(part_header),
                               "--%s\r\n"
                               "Content-Type: image/jpeg\r\n"
                               "Content-Length: %u\r\n"
                               "\r\n",
                               STREAM_BOUNDARY, (unsigned)fb->len);

        /* Send part header + JPEG + trailing CRLF via raw socket */
        if (send(fd, part_header, hdr_len, 0) <= 0 ||
            send(fd, (const char *)fb->buf, fb->len, 0) <= 0 ||
            send(fd, "\r\n", 2, 0) <= 0) {
            esp_camera_fb_return(fb);
            break;
        }
        esp_camera_fb_return(fb);
    }

    ESP_LOGI(TAG, "Stream client disconnected");
    return ESP_OK;
}

/* ---- / root handler ---- */

static esp_err_t root_handler(httpd_req_t *req)
{
    char buf[1200];
#if !CONFIG_OBICO_ENABLED && CONFIG_RFC2217_ENABLED
    const char *ip = wifi_get_ip_str();
#endif

    int len = snprintf(buf, sizeof(buf),
        "<!DOCTYPE html><html><head>"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>ESP32 FDM Bridge</title>"
        "<style>"
        "body{font-family:sans-serif;max-width:600px;margin:2em auto;padding:0 1em}"
        "a{display:block;margin:.5em 0}"
        "code{background:#eee;padding:2px 6px;border-radius:3px}"
        "</style></head><body>"
        "<h1>ESP32 FDM Bridge</h1>"
        "<h2>Camera</h2>"
        "<a href=\"/stream\">MJPEG Stream</a>"
        "<a href=\"/capture\">Snapshot</a>"
        "<a href=\"/camera/config\">Camera Config</a>"
        "%s"
#if CONFIG_OBICO_ENABLED
        "<h2>Obico</h2>"
        "<a href=\"/obico/link\">Link to Obico</a>"
        "<a href=\"/obico/status\">Obico Status</a>"
        "<a href=\"/printer/config\">Printer Config</a>"
#elif CONFIG_RFC2217_ENABLED
        "<h2>Serial</h2>"
        "<p>RFC 2217 port: <code>rfc2217://%s:%d</code></p>"
#endif
        "<h2>WiFi</h2>"
        "<form method=\"POST\" action=\"/wifi/reset\">"
        "<button type=\"submit\" onclick=\"return confirm('Reset WiFi credentials and reboot?')\">"
        "Reset WiFi</button></form>"
        "</body></html>",
        sdcard_is_mounted() ? "<h2>SD Card</h2><a href=\"/sd\">File Manager</a>" : ""
#if !CONFIG_OBICO_ENABLED && CONFIG_RFC2217_ENABLED
        , ip, CONFIG_RFC2217_PORT
#endif
    );

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, buf, len);
}

/* ---- /camera/config handler ---- */

static esp_err_t camera_config_get_handler(httpd_req_t *req)
{
    bool rot = camera_get_rotate180();
    char buf[512];
    int len = snprintf(buf, sizeof(buf),
        "<!DOCTYPE html><html><head>"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>Camera Config</title>"
        "<style>body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:0 20px}"
        "label{font-size:18px}button{font-size:16px;margin-top:12px;padding:8px 24px}</style>"
        "</head><body><h2>Camera Config</h2>"
        "<form method=\"POST\" action=\"/camera/config\">"
        "<label><input type=\"checkbox\" name=\"rotate180\" value=\"1\"%s> Rotate 180&deg;</label><br>"
        "<button type=\"submit\">Save</button></form></body></html>",
        rot ? " checked" : "");

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, buf, len);
}

static esp_err_t camera_config_post_handler(httpd_req_t *req)
{
    char buf[128];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len < 0) recv_len = 0;
    buf[recv_len] = '\0';

    bool enable = (strstr(buf, "rotate180=1") != NULL);
    camera_set_rotate180(enable);

    /* Redirect back to GET to avoid form resubmit */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/camera/config");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- /wifi/reset handler ---- */

static esp_err_t wifi_reset_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req, "WiFi credentials erased. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    wifi_reset_credentials();
    return ESP_OK;  /* unreachable */
}

/* ---- Captive portal helpers ---- */

static void url_decode(char *dst, const char *src, size_t dst_size)
{
    size_t di = 0;
    while (*src && di < dst_size - 1) {
        if (*src == '%' && src[1] && src[2]) {
            char hex[3] = {src[1], src[2], '\0'};
            dst[di++] = (char)strtol(hex, NULL, 16);
            src += 3;
        } else if (*src == '+') {
            dst[di++] = ' ';
            src++;
        } else {
            dst[di++] = *src++;
        }
    }
    dst[di] = '\0';
}

static bool parse_form_field(const char *body, const char *name,
                             char *out, size_t out_size)
{
    char key[36];
    snprintf(key, sizeof(key), "%s=", name);
    const char *start = strstr(body, key);
    if (!start) return false;
    start += strlen(key);

    const char *end = strchr(start, '&');
    size_t len = end ? (size_t)(end - start) : strlen(start);
    if (len >= out_size) len = out_size - 1;

    char encoded[128];
    if (len >= sizeof(encoded)) len = sizeof(encoded) - 1;
    memcpy(encoded, start, len);
    encoded[len] = '\0';

    url_decode(out, encoded, out_size);
    return true;
}

static const char CAPTIVE_HTML[] =
    "<!DOCTYPE html><html><head>"
    "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>ESP32 FDM WiFi Setup</title>"
    "<style>"
    "body{font-family:sans-serif;max-width:400px;margin:40px auto;padding:0 20px}"
    "input{width:100%;padding:8px;margin:4px 0 12px;box-sizing:border-box;font-size:16px}"
    "button{width:100%;padding:10px;font-size:18px;background:#2196F3;color:white;"
    "border:none;border-radius:4px;cursor:pointer}"
    "label{font-weight:bold}"
    "</style></head><body>"
    "<h2>WiFi Setup</h2>"
    "<form method=\"POST\" action=\"/\">"
    "<label>SSID</label>"
    "<input type=\"text\" name=\"ssid\" required maxlength=\"32\" autocomplete=\"off\">"
    "<label>Password</label>"
    "<input type=\"password\" name=\"password\" maxlength=\"64\">"
    "<button type=\"submit\">Connect</button>"
    "</form></body></html>";

static esp_err_t captive_root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, CAPTIVE_HTML, sizeof(CAPTIVE_HTML) - 1);
}

static esp_err_t captive_root_post_handler(httpd_req_t *req)
{
    char buf[200];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    char ssid[33] = {0};
    char pass[65] = {0};

    if (!parse_form_field(buf, "ssid", ssid, sizeof(ssid)) || ssid[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "SSID required");
        return ESP_FAIL;
    }
    parse_form_field(buf, "password", pass, sizeof(pass));

    /* Save to NVS */
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("wifi", NVS_READWRITE, &nvs));
    nvs_set_str(nvs, "ssid", ssid);
    nvs_set_str(nvs, "password", pass);
    nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "WiFi credentials saved for \"%s\" — rebooting", ssid);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<!DOCTYPE html><html><body style=\"font-family:sans-serif;text-align:center;margin-top:60px\">"
        "<h2>Saved!</h2><p>Rebooting and connecting to WiFi...</p>"
        "</body></html>");

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;  /* unreachable */
}

static esp_err_t captive_redirect_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- Public API ---- */

esp_err_t httpd_start_captive_portal(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.max_uri_handlers = 4;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start captive portal: 0x%x", err);
        return err;
    }

    /* Order matters: exact matches first, wildcard last */
    httpd_uri_t root_get = {
        .uri = "/", .method = HTTP_GET, .handler = captive_root_get_handler,
    };
    httpd_register_uri_handler(server, &root_get);

    httpd_uri_t root_post = {
        .uri = "/", .method = HTTP_POST, .handler = captive_root_post_handler,
    };
    httpd_register_uri_handler(server, &root_post);

    httpd_uri_t wildcard = {
        .uri = "/*", .method = HTTP_GET, .handler = captive_redirect_handler,
    };
    httpd_register_uri_handler(server, &wildcard);

    ESP_LOGI(TAG, "Captive portal started on port 80");
    return ESP_OK;
}

esp_err_t httpd_start_server(void)
{
    /* HTTP server on port 80 (Core 0) */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.stack_size = 10240;  /* TLS (mbedTLS) in /obico/link handler needs ~8KB */
    config.max_uri_handlers = 24;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: 0x%x", err);
        return err;
    }

    httpd_uri_t root_uri = {
        .uri      = "/",
        .method   = HTTP_GET,
        .handler  = root_handler,
    };
    httpd_register_uri_handler(server, &root_uri);

    httpd_uri_t capture_uri = {
        .uri      = "/capture",
        .method   = HTTP_GET,
        .handler  = capture_handler,
    };
    httpd_register_uri_handler(server, &capture_uri);

    httpd_uri_t stream_uri = {
        .uri      = "/stream",
        .method   = HTTP_GET,
        .handler  = stream_handler,
    };
    httpd_register_uri_handler(server, &stream_uri);

    httpd_uri_t cam_cfg_get = {
        .uri      = "/camera/config",
        .method   = HTTP_GET,
        .handler  = camera_config_get_handler,
    };
    httpd_register_uri_handler(server, &cam_cfg_get);

    httpd_uri_t cam_cfg_post = {
        .uri      = "/camera/config",
        .method   = HTTP_POST,
        .handler  = camera_config_post_handler,
    };
    httpd_register_uri_handler(server, &cam_cfg_post);

    httpd_uri_t wifi_reset = {
        .uri      = "/wifi/reset",
        .method   = HTTP_POST,
        .handler  = wifi_reset_handler,
    };
    httpd_register_uri_handler(server, &wifi_reset);

    sdcard_httpd_register(server);

#if CONFIG_OBICO_ENABLED
    obico_register_httpd(server);
    printer_config_register_httpd(server);
#endif

    ESP_LOGI(TAG, "HTTP server started on port 80 (stream at /stream)");

    return ESP_OK;
}
