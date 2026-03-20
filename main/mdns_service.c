#include "mdns_service.h"

#include <string.h>

#include "esp_log.h"
#include "mdns.h"
#include "wifi.h"

static const char *TAG = "mdns_svc";

void mdns_service_init(void)
{
    const char *hostname = wifi_get_hostname();

    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "mdns_init failed: %s", esp_err_to_name(err));
        return;
    }

    mdns_hostname_set(hostname);
    mdns_instance_name_set(hostname);

    /* Advertise HTTP service so the device is discoverable */
    mdns_service_add(hostname, "_http", "_tcp", 80, NULL, 0);

    ESP_LOGI(TAG, "mDNS started: %s.local", hostname);
}
