#pragma once

#include "esp_err.h"

typedef enum {
    WIFI_RESULT_STA_CONNECTED,
    WIFI_RESULT_AP_MODE,
} wifi_result_t;

/**
 * Initialize WiFi: try STA with NVS/Kconfig creds, fall back to AP mode.
 */
wifi_result_t wifi_init(void);

/**
 * Get the current IP address as a string.
 * Returns pointer to a static buffer — valid until next call.
 */
const char *wifi_get_ip_str(void);

/**
 * Erase stored WiFi credentials from NVS and reboot.
 */
void wifi_reset_credentials(void);
