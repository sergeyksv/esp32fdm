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
 */
esp_err_t klipper_backend_print_file(const char *filename);

#ifdef __cplusplus
}
#endif
