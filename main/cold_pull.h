#pragma once

/*
 * Cold pull (Utils, Marlin only): cleans the nozzle by pulling out filament
 * that has cooled to just above solid.  The board builds a short G-code script
 * and runs it as a host print, so the stops, /print, the notifications and the
 * hold timeout all work as they do for a file.
 */

#include <stddef.h>
#include "esp_err.h"
#include "esp_http_server.h"

typedef struct {
    int load_c;     /* Nozzle temperature to melt and push filament through */
    int pull_c;     /* Nozzle temperature to pull it out at */
    int cycles;     /* Pulls in one run */
} cold_pull_params_t;

/** Write the script for p into buf.  Returns its length, or 0 if it did not fit. */
size_t cold_pull_build(const cold_pull_params_t *p, char *buf, size_t size);

esp_err_t cold_pull_register_httpd(httpd_handle_t server);
