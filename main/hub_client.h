#pragma once

/*
 * Client for the notification hub, a small service on the local network.
 * The board reports events and knows nothing about where they go: the hub
 * holds the Telegram credentials and turns events into messages.
 */

#include "esp_err.h"
#include "esp_http_server.h"
#include "layout.h"

typedef enum {
    HUB_EVENT_HOLD,          /* print is waiting for the user */
    HUB_EVENT_HOLD_TIMEOUT,  /* nobody came: print cancelled, heaters off */
    HUB_EVENT_TEST,
} hub_event_type_t;

/** Start the sender task.  Events sent before this call are dropped. */
void hub_client_init(void);

/** Queue an event.  Does not block: delivery happens on the sender task. */
void hub_client_send_event(hub_event_type_t type, const char *message, const char *filename);

/** Latest delivery result: "", "off" (no hub URL set), "sending", "ok" or "failed". */
const char *hub_client_notify_status(void);

void hub_client_render_settings(html_buf_t *p);

esp_err_t hub_client_register_httpd(httpd_handle_t server);
