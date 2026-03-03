#include "httpd.h"

#include "camera.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static const char *TAG = "httpd";

#define STREAM_TARGET_FPS 10
#define STREAM_FRAME_INTERVAL_US (1000000 / STREAM_TARGET_FPS)
#define STREAM_PORT 81

static const char *STREAM_BOUNDARY = "esp32fdm_frame";
static const char *STREAM_RESP_HEADER =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace;boundary=esp32fdm_frame\r\n"
    "Access-Control-Allow-Origin: *\r\n"
    "X-Framerate: 10\r\n"
    "\r\n";

/* ---- /capture handler (stays on httpd, Core 0) ---- */

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

/* ---- MJPEG stream task (raw socket, Core 1) ---- */

static void stream_client_handler(int client_sock)
{
    char part_header[128];
    int64_t last_frame_time = 0;

    /* Send HTTP response header */
    send(client_sock, STREAM_RESP_HEADER, strlen(STREAM_RESP_HEADER), 0);

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

        /* Send part header */
        if (send(client_sock, part_header, hdr_len, 0) <= 0) {
            esp_camera_fb_return(fb);
            break;
        }

        /* Send JPEG data */
        if (send(client_sock, (const char *)fb->buf, fb->len, 0) <= 0) {
            esp_camera_fb_return(fb);
            break;
        }
        esp_camera_fb_return(fb);

        /* Send trailing CRLF */
        if (send(client_sock, "\r\n", 2, 0) <= 0) {
            break;
        }
    }

    ESP_LOGI(TAG, "Stream client disconnected");
}

static void stream_server_task(void *arg)
{
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Stream: failed to create socket");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(STREAM_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) != 0) {
        ESP_LOGE(TAG, "Stream: bind failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    if (listen(listen_sock, 1) != 0) {
        ESP_LOGE(TAG, "Stream: listen failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "MJPEG stream server listening on port %d", STREAM_PORT);

    while (true) {
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        int client = accept(listen_sock, (struct sockaddr *)&client_addr,
                            &addr_len);
        if (client < 0) {
            ESP_LOGW(TAG, "Stream: accept failed");
            continue;
        }

        /* TCP_NODELAY for low-latency frame delivery */
        int nodelay = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        ESP_LOGI(TAG, "Stream client connected");
        stream_client_handler(client);
        close(client);
    }
}

/* ---- Public API ---- */

esp_err_t httpd_start_server(void)
{
    /* HTTP server on port 80 (Core 0) — /capture only */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.core_id = 0;
    config.stack_size = 4096;
    config.max_uri_handlers = 4;
    config.lru_purge_enable = true;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server: 0x%x", err);
        return err;
    }

    httpd_uri_t capture_uri = {
        .uri      = "/capture",
        .method   = HTTP_GET,
        .handler  = capture_handler,
    };
    httpd_register_uri_handler(server, &capture_uri);

    ESP_LOGI(TAG, "HTTP server started on port 80 (capture)");

    /* MJPEG stream task on Core 1 — raw socket, port 81 */
    xTaskCreatePinnedToCore(stream_server_task, "mjpeg_stream", 8192,
                            NULL, 5, NULL, 1);

    return ESP_OK;
}
