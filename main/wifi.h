#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>

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
 * Get the stored WiFi SSID from NVS.
 * Copies into buf (up to buf_size-1 chars + NUL). Returns true if found.
 */
bool wifi_get_ssid(char *buf, size_t buf_size);

/**
 * Erase stored WiFi credentials from NVS and reboot.
 */
void wifi_reset_credentials(void);

/**
 * Get the device hostname (used for DHCP and mDNS).
 * Returns pointer to a static buffer — always valid, never NULL.
 */
const char *wifi_get_hostname(void);

/**
 * Set the device hostname. Saves to NVS and takes effect on next reboot.
 */
void wifi_set_hostname(const char *name);
