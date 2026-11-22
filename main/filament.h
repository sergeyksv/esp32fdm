#pragma once

/*
 * Filament change (Utils, Marlin only), without unloading: the old filament is
 * cut above the extruder and the new one pushed in behind it, then the board
 * heats to the hotter of the two plastics, waits for the user and purges, so
 * the new filament pushes the old one out.  Runs as a host print like the
 * cold pull, and remembers the plastic for the next change.
 */

#include <stddef.h>
#include "esp_err.h"
#include "esp_http_server.h"

typedef struct {
    const char *from;   /* Plastic in the nozzle, or NULL if unknown */
    const char *to;     /* Plastic going in */
    int temp_c;         /* Purge temperature */
    int purge_mm;
} filament_params_t;

/** Write the script for p into buf.  Returns its length, or 0 if it did not fit. */
size_t filament_build(const filament_params_t *p, char *buf, size_t size);

esp_err_t filament_register_httpd(httpd_handle_t server);
