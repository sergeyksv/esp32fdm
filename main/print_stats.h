#pragma once

/*
 * Print statistics: hours spent printing and the number of prints, kept in
 * NVS.  The Kobra's Marlin has no print counter (M78 is unknown), so the board
 * counts: time in PRINTING, not paused or waiting for the user.  Built-in
 * scripts (cold pull, filament change) do not count.
 */

#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"
#include "layout.h"

typedef struct {
    uint32_t print_s;       /* Time spent printing */
    uint32_t started;
    uint32_t completed;
    uint32_t cancelled;     /* Cancelled, failed, or the connection lost */
    int64_t since;          /* Unix time counting began, 0 if unknown */
} print_stats_t;

/** Load the counters and start counting */
void print_stats_init(void);

/** The counters, including the print running now */
void print_stats_get(print_stats_t *out);

void print_stats_render_settings(html_buf_t *p);

esp_err_t print_stats_register_httpd(httpd_handle_t server);
