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
    HUB_EVENT_UPDATED,       /* a deferred firmware update was installed */
    HUB_EVENT_UPDATE_FAILED, /* a deferred firmware update could not be installed */
    HUB_EVENT_MAINTENANCE,   /* maintenance jobs fell due; message lists them */
    HUB_EVENT_MAINTENANCE_SKIPPED, /* a print started with maintenance overdue; message lists it */
    HUB_EVENT_STACK_LOW,     /* a task came close to the end of its stack; message names it */
    HUB_EVENT_OBICO_PAUSE,   /* Obico's failure detection paused the print */
    HUB_EVENT_NOZZLE_STANDBY, /* a wait went on: the nozzle went down to standby; message "245° → 160°" */
    HUB_EVENT_PRINT_CHECK,   /* a print started with a check failed (Z offset, mesh profile); message lists them */
    HUB_EVENT_FILAMENT_RUNOUT, /* the filament ran out, the print is paused; message "245° → 160°, 10 min" or "" */
} hub_event_type_t;

/** Start the sender task.  Events sent before this call are dropped. */
void hub_client_init(void);

/** Queue an event.  Does not block: delivery happens on the sender task. */
void hub_client_send_event(hub_event_type_t type, const char *message, const char *filename);

/** Latest delivery result: "", "off" (no hub URL set), "sending", "ok" or "failed". */
const char *hub_client_notify_status(void);

/** Set and save the hub URL ("" turns it off).  NULL, or what is wrong (a msgid). */
const char *hub_client_set_url(const char *url);
const char *hub_client_get_url(void);

void hub_client_render_settings(html_buf_t *p);

esp_err_t hub_client_register_httpd(httpd_handle_t server);
