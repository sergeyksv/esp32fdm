#pragma once

#include "esp_http_server.h"

/**
 * Register bed leveling HTTP endpoints on the given server.
 * Marlin: G28 + G29, the mesh parsed from the printer's output.  Klipper: the
 * mesh read from Moonraker, probed by a built-in script.
 */
esp_err_t bedlevel_register_httpd(httpd_handle_t server);

/** Middle of the bed in printer coordinates, from the bed size set on /bedlevel */
void bedlevel_bed_center(float *x, float *y);

/* Settings copied between boards: bed size, screws, origin */
struct cJSON;
void bedlevel_settings_export(struct cJSON *obj);
void bedlevel_settings_import(const struct cJSON *obj);

#include <stdbool.h>
#include <stdint.h>

/* Klipper: the mesh profile the printer prints with against the bed's last probe */
typedef struct {
    char profile[24];
    float off_mm;       /* Max minus min of (probe - profile) at the probe's points: a shift is taken by the Z offset */
    int nx, ny;         /* The probe's grid */
    int64_t ts;         /* ...and when it was seen */
} bl_print_mesh_t;

/** Reads Klipper's mesh first (a probe from the printer's screen is kept too).
 *  False if unknown: not Klipper, no probe with coordinates, no profile for
 *  printing, or Klipper unreachable.  Cached for 30 s. */
bool bedlevel_print_mesh_check(bl_print_mesh_t *out);
