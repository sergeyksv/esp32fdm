#include "dns_server.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"

static const char *TAG = "dns";

/* Captive portal IP: 192.168.4.1 */
static const uint8_t CAPTIVE_IP[4] = {192, 168, 4, 1};

static void dns_server_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create DNS socket");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in addr = {
        .sin_family = AF_INET,
        .sin_port = htons(53),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "Failed to bind DNS socket");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "DNS server started on port 53");

    uint8_t buf[512];
    while (true) {
        struct sockaddr_in client;
        socklen_t client_len = sizeof(client);
        int len = recvfrom(sock, buf, sizeof(buf), 0,
                           (struct sockaddr *)&client, &client_len);
        if (len < 12) continue;  /* Too short for DNS header */

        /* Build response: copy header, set response flags */
        buf[2] = 0x81;  /* QR=1, OPCODE=0, AA=1, TC=0, RD=1 */
        buf[3] = 0x80;  /* RA=1, RCODE=0 (no error) */
        /* ANCOUNT = 1 */
        buf[6] = 0x00;
        buf[7] = 0x01;

        /* Find end of question section (skip QNAME + QTYPE + QCLASS) */
        int pos = 12;
        while (pos < len && buf[pos] != 0) pos += 1 + buf[pos];
        pos += 5;  /* null terminator + QTYPE(2) + QCLASS(2) */

        if (pos > len) continue;

        /* Append answer: pointer to name in question, type A, class IN, TTL 60, IP */
        uint8_t answer[] = {
            0xC0, 0x0C,             /* Name pointer to offset 12 */
            0x00, 0x01,             /* Type A */
            0x00, 0x01,             /* Class IN */
            0x00, 0x00, 0x00, 0x3C, /* TTL 60s */
            0x00, 0x04,             /* RDLENGTH 4 */
            CAPTIVE_IP[0], CAPTIVE_IP[1], CAPTIVE_IP[2], CAPTIVE_IP[3],
        };

        if (pos + sizeof(answer) <= sizeof(buf)) {
            memcpy(buf + pos, answer, sizeof(answer));
            sendto(sock, buf, pos + sizeof(answer), 0,
                   (struct sockaddr *)&client, client_len);
        }
    }
}

void dns_server_start(void)
{
    xTaskCreatePinnedToCore(dns_server_task, "dns_srv", 3072, NULL, 5, NULL, 0);
}
