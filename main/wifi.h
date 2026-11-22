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

/* ---- Status, for the Wi-Fi page and the hub ---- */

#include <stdint.h>

typedef struct {
    uint8_t bssid[6];           /* Of the AP last associated with */
    uint8_t channel;
    int64_t connected_us;       /* esp_timer time of the last association */
    int64_t disconnected_us;    /* ...and of the last disconnect, 0 if none */
    uint16_t last_reason;       /* wifi_err_reason_t of the last disconnect */
    uint32_t disconnects;
    uint32_t reconnects;        /* Times the board reconnected after a loss */
    uint32_t roams;             /* Moves the board made itself */
    uint32_t btm_roams;         /* Moves the network asked for (802.11v, DAWN) */
    int64_t last_btm_us;
} wifi_stats_t;

void wifi_get_stats(wifi_stats_t *out);

#define WIFI_SEEN_MAX 16
typedef struct {
    uint8_t bssid[6];
    uint8_t channel;
    int8_t rssi;
} wifi_seen_ap_t;

/** APs with our SSID from the last scan, and when it was (esp_timer us) */
int wifi_get_seen(wifi_seen_ap_t *out, int max, int64_t *when_us);

/** Ask the Wi-Fi task for a scan (about 2 s; Wi-Fi pauses meanwhile) */
void wifi_request_scan(void);

/* Names for the APs, shown on the pages and sent to the hub */
#define WIFI_AP_NAMES_MAX 8
typedef struct {
    uint8_t bssid[6];
    char name[26];
} wifi_ap_name_t;

/** The name given to bssid; false (and "") if it has none */
bool wifi_ap_name(const uint8_t bssid[6], char *out, size_t size);
int wifi_get_ap_names(wifi_ap_name_t *out, int max);
void wifi_set_ap_names(const wifi_ap_name_t *names, int count);

/** The AP chosen first when its signal is at least -75 dBm */
bool wifi_get_preferred(uint8_t out[6]);
/** NULL clears it */
void wifi_set_preferred(const uint8_t *bssid);
