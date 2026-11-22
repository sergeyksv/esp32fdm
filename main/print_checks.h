#pragma once

/*
 * Checks before a print: what a calibration changed behind the user's back.
 *
 *   - Marlin, the Z offset: the Kobra measures it on its pad at every bed
 *     calibration and gets it wrong by tenths of a mm either way, while the
 *     real offset (nozzle to probe) does not move.  The board keeps the offset
 *     of the last print that completed (or set with paper) as the good one;
 *     an offset read while idle that differs is reported, with Restore
 *     (M851 + M500), Keep, or the paper test.
 *   - Klipper, the mesh profile the printer loads to print with: compared in
 *     shape with the bed's last probe; a profile from before the screws were
 *     turned is reported (bedlevel_print_mesh_check()).
 *
 * The board's Print buttons ask first (GET /print/checks, which also carries
 * the overdue maintenance), /print shows them while idle, and a print started
 * anyway (also from the printer's screen) sends the hub `print_check`.
 */

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "esp_http_server.h"

#define PC_Z_EPS    0.015f  /* Marlin answers with 2 decimals */
#define PC_MESH_MM  0.2f    /* About a first layer: past it the nozzle touches the bed */

void print_checks_init(void);

/** An offset read from the printer ("Probe Offset Z-0.97"); busy: printing,
 *  paused or a host print, when Z taps answer with it: such an offset becomes
 *  the checked one if the printer still has it when idle.  Called from the
 *  printer task. */
void print_checks_z_seen(float z, bool busy);

/** The offset now on the printer is right: the next reading becomes the good one
 *  (a print completed, the paper test saved it) */
void print_checks_z_confirmed(void);

/** From the print_stats task */
void print_checks_note_print_start(const char *filename);
void print_checks_note_print_done(bool completed);

/** The warnings, one per line, in language lang; returns how many */
int print_checks_text(char *out, size_t size, int lang);

esp_err_t print_checks_register_httpd(httpd_handle_t server);
