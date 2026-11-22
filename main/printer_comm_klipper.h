#pragma once

#include "printer_comm.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the Klipper/Moonraker backend.
 * Spawns a polling task that queries Moonraker over HTTP every 4 seconds.
 */
esp_err_t klipper_backend_start(const char *host, uint16_t port);

/**
 * Stop the Klipper backend and delete the polling task.
 */
void klipper_backend_stop(void);

/**
 * Thread-safe copy of current printer state from Moonraker.
 */
void klipper_backend_get_state(printer_state_t *out);

/**
 * Send a command to the printer via Moonraker API.
 * PCMD_PAUSE/RESUME/CANCEL use dedicated endpoints.
 * PCMD_RAW uses /printer/gcode/script.
 */
esp_err_t klipper_backend_send_cmd(const printer_cmd_t *cmd);

/**
 * Upload a file from local SD card to Moonraker and start printing.
 * Reads /sdcard/<filename>, uploads via multipart POST, then starts the print.
 * Stops for the user (M0, M1, "; @pause") are rewritten on the way, see
 * klipper_holds.h.  Slow (reads the file twice): call from its own task.
 */
esp_err_t klipper_backend_print_file(const char *filename);

/** Start a built-in script (filament change, cold pull): sent to Moonraker as
 *  "<name>.gcode" and printed, from its own task.  Takes ownership of script.
 *  done is called from the poll task when it ends, not if it fails to start. */
esp_err_t klipper_backend_print_script(const char *name, char *script,
                                       host_script_done_t done, int done_arg);

/** The board's Z offset for prints: written into each file sent, at a
 *  "; @zoffset" line.  False if none is saved (the printer's own applies). */
bool klipper_backend_get_zoffset(float *z);

/**
 * The script that raises Z by mm (less near the top).  Unhomed, it starts from
 * the last position seen while homed, with force_move's SET_KINEMATIC_POSITION.
 * ESP_ERR_INVALID_STATE: the position is not known (Klipper restarted since);
 * ESP_ERR_INVALID_SIZE: already at the top.
 */
esp_err_t klipper_backend_lift_script(float mm, char *buf, size_t size);
/** Save z as the offset for prints, or forget it (set false) */
esp_err_t klipper_backend_set_zoffset(bool set, float z);

/** A file or a built-in script is being sent to Moonraker or started */
bool klipper_backend_sending(void);

/** The printer has an FDM_LIGHT macro (S=1 on, S=0 off), as seen at the last
 *  connection: see board_led.c */
bool klipper_backend_has_light(void);

#define KLIPPER_MESH_NAMES_MAX 8

typedef struct {
    char profile[24];           /* Active profile, "" if none */
    bool from_saved;            /* The matrix is that profile's saved points: loaded, maybe not probed now */
    char names[KLIPPER_MESH_NAMES_MAX][24];  /* Profiles Klipper knows, without "default" */
    int n_names;
} klipper_mesh_info_t;

/** The mesh Klipper has now (bed_mesh.probed_matrix): data[y * max_n + x],
 *  row 0 at the front (mesh_min Y); bounds = min X, min Y, max X, max Y.
 *  ESP_ERR_NOT_FOUND if there is none. */
esp_err_t klipper_backend_get_mesh(float *data, int max_n, int *nx, int *ny, float bounds[4],
                                   klipper_mesh_info_t *info);

/** Whether Klipper's profile name holds the mesh it has now (after
 *  BED_MESH_PROFILE SAVE=name, before SAVE_CONFIG) */
bool klipper_backend_profile_has_mesh(const char *name);

/** A saved profile's points (bed_mesh.profiles[name]), laid out as in
 *  klipper_backend_get_mesh(), bounds from its mesh_params.
 *  ESP_ERR_NOT_FOUND if Klipper has no such profile. */
esp_err_t klipper_backend_get_profile(const char *name, float *data, int max_n, int *nx, int *ny,
                                      float bounds[4]);

/** The mesh profile the printer loads when Klipper starts (its screen on the
 *  Neptune 4), "" until seen once */
void klipper_backend_boot_mesh(char *out, size_t size);

/** Moonraker's job history totals (all jobs, also those started elsewhere).
 *  False until fetched, or if Moonraker keeps no history. */
bool klipper_backend_history_totals(uint32_t *print_s, uint32_t *jobs);

/** Continue after one of our stops; ignored unless hold_id is the current one */
esp_err_t klipper_backend_continue(uint32_t hold_id);

/** Shift Z by dz mm (SET_GCODE_OFFSET), while printing or at one of our stops.
 *  ESP_ERR_INVALID_STATE if not now, ESP_ERR_INVALID_SIZE past 1 mm per print. */
esp_err_t klipper_backend_zstep(float dz);

#ifdef __cplusplus
}
#endif
