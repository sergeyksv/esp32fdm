#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PRINTER_BACKEND_MARLIN  = 0,
    PRINTER_BACKEND_KLIPPER = 1,
} printer_backend_t;

typedef enum {
    PRINTER_DISCONNECTED,
    PRINTER_OPERATIONAL,
    PRINTER_PRINTING,
    PRINTER_PAUSED,
    PRINTER_ERROR,
} printer_opstate_t;

typedef struct {
    printer_opstate_t opstate;
    float hotend_actual, hotend_target;
    float bed_actual, bed_target;
    float progress_pct;          /* 0-100, or -1 if unknown */
    int32_t print_time_s;        /* elapsed print time, or -1 */
    int32_t print_time_left_s;   /* estimated remaining, or -1 */
    float x, y, z;
    char filename[64];
    float layer_height;          /* from slicer metadata, 0 if unknown */
    float first_layer_height;    /* from slicer metadata, 0 if unknown */
    float object_height;         /* max Z from slicer metadata, 0 if unknown */
    int32_t total_layers;        /* computed from metadata, -1 if unknown */
    int64_t last_update_us;      /* esp_timer_get_time() of last successful parse */
} printer_state_t;

typedef enum {
    PCMD_PAUSE,      /* M25 (SD) or M0 */
    PCMD_RESUME,     /* M24 */
    PCMD_CANCEL,     /* M524 or abort sequence */
    PCMD_RAW,        /* raw gcode string */
} printer_cmd_type_t;

typedef struct {
    printer_cmd_type_t type;
    char gcode[96];  /* only used for PCMD_RAW */
} printer_cmd_t;

/**
 * Initialize the printer communication module.
 * Starts a polling task that queries the printer over USB serial.
 * Must be called after usb_serial_init().
 */
esp_err_t printer_comm_init(void);

/**
 * USB serial RX callback — pass to usb_serial_init().
 * Feeds received data into the printer_comm parser.
 */
void printer_comm_rx_cb(const uint8_t *data, size_t len, void *user_ctx);

/**
 * Thread-safe copy of current printer state.
 */
void printer_comm_get_state(printer_state_t *out);

/**
 * Queue a command to send to the printer.
 * Non-blocking; returns ESP_ERR_TIMEOUT if queue is full.
 */
esp_err_t printer_comm_send_cmd(const printer_cmd_t *cmd);

/**
 * Enable/disable simulation mode.
 * When enabled, get_state returns fake printer data (printing, temps, progress).
 */
void printer_comm_set_simulate(bool enable);

/**
 * Returns true if simulation mode is active.
 */
bool printer_comm_is_simulating(void);

/**
 * Get the currently active printer backend.
 */
printer_backend_t printer_comm_get_backend(void);

/**
 * Get the configured Moonraker host (empty string if not set).
 */
const char *printer_comm_get_mr_host(void);

/**
 * Get the configured Moonraker port.
 */
uint16_t printer_comm_get_mr_port(void);

/**
 * Save printer backend configuration to NVS.
 * Takes effect after reboot.
 */
esp_err_t printer_comm_save_config(printer_backend_t backend,
                                   const char *mr_host, uint16_t mr_port,
                                   bool pause_abort);

/**
 * Register printer config HTTP endpoints on the given server.
 * Called from httpd.c during server setup.
 */
esp_err_t printer_config_register_httpd(void *server_handle);

#ifdef __cplusplus
}
#endif
