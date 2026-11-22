#pragma once

#include "esp_err.h"
#include "layout.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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
    bool host_printing;          /* true if ESP32 is streaming GCode from SD */
    bool builtin_script;         /* the host print is a built-in script (cold pull...) */
    bool last_host_completed;    /* how the last host print ended: false if cancelled */
    int32_t current_layer;       /* current layer from ;LAYER:N comments */
    float probe_z_offset;        /* Z probe offset from M290 query (Marlin), gcode offset Z (Klipper) */
    bool preparing;              /* host print running, file scan not finished yet */
    int8_t upload_pct;           /* Klipper: file being sent to Moonraker, percent; -1 if not */
    int8_t fan_pct;              /* Part cooling fan, percent; -1 if not known */
    int8_t motors;               /* Steppers on (Klipper: axes homed); -1 if not known (Marlin) */
    bool holding;                /* host print waiting for the user (M0/M1 in file) */
    uint32_t hold_id;            /* identifies the current hold, valid while holding */
    char hold_msg[64];           /* text from M0/M1, safe for JSON and HTML */
    float z_shift;               /* manual Z shift sent in this print, mm */
    float z_pending;             /* tapped, not sent to the printer yet, mm */
    int8_t nozzle;               /* Nozzle guard (nozzle_guard.h): NOZZLE_* */
    int64_t last_update_us;      /* esp_timer_get_time() of last successful parse */
} printer_state_t;

typedef enum {
    PCMD_PAUSE,      /* M25 (SD) or M0 */
    PCMD_RESUME,     /* M24 */
    PCMD_CANCEL,     /* M524 or abort sequence */
    PCMD_RAW,        /* raw gcode string */
} printer_cmd_type_t;

/* printer_state_t.nozzle: what the nozzle guard has done in this pause */
#define NOZZLE_HOT      0   /* At its print temperature */
#define NOZZLE_STANDBY  1   /* Taken down; Continue heats it back first */
#define NOZZLE_HEATING  2   /* Continue asked, heating back */

typedef struct {
    printer_cmd_type_t type;
    char gcode[96];  /* only used for PCMD_RAW */
    bool standby;    /* PCMD_PAUSE: nozzle to its standby temperature (a pause from Obico) */
    bool runout;     /* PCMD_PAUSE: the filament ran out (host print): standby after a while */
} printer_cmd_t;

/* Temperature history sample */
typedef struct {
    int64_t timestamp_us;       /* esp_timer_get_time() when recorded */
    float hotend_actual, hotend_target;
    float bed_actual, bed_target;
} temp_sample_t;

#define TEMP_HISTORY_MAX 900    /* 1 hour at 4s intervals */

/**
 * Record a temperature history sample with explicit values.
 * Called by both Marlin and Klipper backends after updating temps.
 */
void printer_comm_record_temp_sample(float hotend_actual, float hotend_target,
                                     float bed_actual, float bed_target);

/**
 * Copy temperature history samples oldest-first into buf.
 * Returns actual number of samples copied (up to TEMP_HISTORY_MAX).
 */
int printer_comm_get_temp_history(temp_sample_t *buf, int max_count);

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

/** Marlin: read the probe Z offset (M290) again at the next idle poll */
void printer_comm_query_probe_offset(void);

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
 * esp_timer time of the last command sent on behalf of a user (terminal, bed
 * probe, a page button), now while one is still running, or 0 if none yet.
 */
int64_t printer_comm_last_user_cmd_us(void);
bool printer_comm_user_cmd_running(void);

/* Settings copied between boards: the Marlin options (pause command,
 * filament check, echo ping), not the backend or the Moonraker address */
struct cJSON;
/** Seconds between beep series while a print waits for the user, 0 = once */
uint16_t printer_comm_hold_beep_s(void);
/** When the next series of beeps is due for a hold that began at hold_since_us
 *  (esp_timer time), or 0 for none: often at first, then less, and often again
 *  in the last minute before the hold times out */
int64_t printer_comm_next_beep_series(int64_t now, int64_t hold_since_us);
void printer_comm_settings_export(struct cJSON *obj);
void printer_comm_settings_import(const struct cJSON *obj);

/**
 * Get a human-readable name for the current backend ("Marlin" or "Klipper").
 */
const char *printer_comm_backend_name(void);

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

/**
 * Render backend selection section (Marlin/Klipper, Moonraker fields, Save & Reboot).
 */
void printer_config_render_backend(html_buf_t *p);

/**
 * Render Marlin-specific settings section (pause command, Save).
 */
void printer_config_render_marlin(html_buf_t *p);

/**
 * Render the Klipper options (the beep while waiting for the user).
 */
void printer_config_render_klipper(html_buf_t *p);

/* ---- G-code file info extraction (see gcode_scan.h) ---- */
#include "gcode_scan.h"

/**
 * Start host-based GCode printing from a file on /sdcard.
 * The file is streamed line-by-line to the printer via USB serial.
 */
esp_err_t printer_comm_host_print_start(const char *filename);

/**
 * Start a host print of a built-in G-code script, as if it were a file named
 * name: holds, /print, notifications and cancel all work the same.
 * Takes ownership of script (malloc'd, NUL-terminated) whatever the result.
 * done, if set, is called with done_arg when the script ends, not if it fails to start.
 * Returns ESP_ERR_INVALID_STATE if another script is still being started.
 */
/** Manual Z shift (the /print Z taps) allowed in total: 1 mm in a print, where
 *  a wrong tap goes into the part, 3 mm in a built-in script (the Z offset paper
 *  test), where a new bed level after screw turns often needs more. Hundredths of a mm */
#define Z_SHIFT_LIMIT_PRINT_CU   100
#define Z_SHIFT_LIMIT_SCRIPT_CU  300
static inline int z_shift_limit_cu(bool builtin_script)
{
    return builtin_script ? Z_SHIFT_LIMIT_SCRIPT_CU : Z_SHIFT_LIMIT_PRINT_CU;
}

/** Called on the printer task when a script ends: completed is false if it was
 *  cancelled (by the user, the hold timeout or a lost connection). */
typedef void (*host_script_done_t)(bool completed, int arg);

esp_err_t printer_comm_host_print_script(const char *name, char *script,
                                         host_script_done_t done, int done_arg);

/**
 * Pause host print (stops sending lines, heaters stay on).
 */
esp_err_t printer_comm_host_print_pause(void);

/**
 * Resume host print after pause.
 */
esp_err_t printer_comm_host_print_resume(void);

/**
 * Cancel host print (closes file, turns off heaters, disables motors).
 */
esp_err_t printer_comm_host_print_cancel(void);

/**
 * Continue after a wait for the user.  Ignored unless hold_id is the current
 * hold, so a repeated request cannot skip the next M0/M1 in the file.
 * With Klipper, the stops are the ones the board wrote into the file it sent.
 */
esp_err_t printer_comm_host_print_continue(uint32_t hold_id);

/**
 * Add a babystep to the current print.  Does not block: taps accumulate
 * and go to the printer as one M290 when it is ready (SET_GCODE_OFFSET with Klipper).
 * Returns ESP_ERR_INVALID_STATE if Z cannot be adjusted now, and
 * ESP_ERR_INVALID_SIZE if the step would exceed the limit per print.
 */
esp_err_t printer_comm_host_print_zstep(float dz);

/**
 * Returns true if a host print is currently active (printing or paused).
 */
bool printer_comm_is_host_printing(void);

/**
 * Suppress automatic polling (M105/M27/M114/M119).
 * Used by terminal manual mode to avoid response clutter.
 */
void printer_comm_set_polling_suppressed(bool suppress);
bool printer_comm_is_polling_suppressed(void);

/**
 * Set a callback invoked for every line received from the printer.
 * Called from the comm task context.  Pass NULL to clear.
 */
void printer_comm_set_line_callback(void (*cb)(const char *line));

/**
 * Send a raw GCode command and block until the printer responds with "ok".
 * Only one caller at a time.  Returns ESP_ERR_TIMEOUT if no response within timeout_ms.
 */
esp_err_t printer_comm_send_gcode_sync(const char *gcode, int timeout_ms);

#ifdef __cplusplus
}
#endif
