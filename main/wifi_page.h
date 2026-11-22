#pragma once

/*
 * /wifi: the connection (AP, signal, channel, addresses, disconnects and
 * moves), the APs with our SSID, their names and the preferred one.
 */

#include "esp_err.h"
#include "esp_http_server.h"

esp_err_t wifi_page_register_httpd(httpd_handle_t server);

/* Settings copied between boards: the AP names */
struct cJSON;
void wifi_page_settings_export(struct cJSON *obj);
void wifi_page_settings_import(const struct cJSON *obj);
