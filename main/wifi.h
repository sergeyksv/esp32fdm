#pragma once

#include "esp_err.h"

/**
 * Initialize WiFi in STA mode and block until connected.
 * Returns ESP_OK on success, or an error if connection failed after retries.
 */
esp_err_t wifi_init_sta(void);

/**
 * Get the current IP address as a string.
 * Returns pointer to a static buffer — valid until next call.
 */
const char *wifi_get_ip_str(void);
