#include "printer_comm.h"
#include "printer_comm_klipper.h"

#include "usb_serial.h"
#include "terminal.h"
#include "url_util.h"

#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "layout.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "printer_comm";

/* ---- Configuration ---- */

#define POLL_M105_INTERVAL_MS           4000
#define POLL_M105_HOSTPRINT_INTERVAL_MS 30000   /* Reduced frequency during host print */
#define POLL_M27_INTERVAL_MS            10000
#define POLL_M114_INTERVAL_MS           30000
#define POLL_M119_INTERVAL_MS           30000   /* Filament sensor check during host print */
#define CMD_RESPONSE_TIMEOUT_MS         6000
#define WAIT_SILENT_TIMEOUT_MS          120000  /* G28/G29: no interim feedback, needs long timeout */
#define RX_LINE_BUF_SIZE        256
#define CMD_QUEUE_SIZE          8

#define NVS_NAMESPACE           "pcomm"
#define NVS_KEY_BACKEND         "backend"
#define NVS_KEY_MR_HOST         "mr_host"
#define NVS_KEY_MR_PORT         "mr_port"
#define NVS_KEY_PAUSE_CMD       "pause_cmd"
#define NVS_KEY_FILAMENT_CHK    "fil_chk"
#define NVS_KEY_ECHO_PING       "echo_ping"

/* ---- Backend config ---- */

typedef enum {
    PAUSE_CMD_M25  = 0,  /* Pause SD print (can resume) */
    PAUSE_CMD_M524 = 1,  /* Abort SD print (workaround for buggy firmware) */
} pause_cmd_t;

static printer_backend_t s_backend = PRINTER_BACKEND_MARLIN;
static char s_mr_host[64] = "";
static uint16_t s_mr_port = 7125;
static pause_cmd_t s_pause_cmd = PAUSE_CMD_M25;
static bool s_filament_check = false;  /* Poll M119 for filament runout during host print */
static char s_echo_ping_cmds[128] = "";  /* Comma-separated GCode prefixes that need CDC flush ping */

/* ---- Temperature history circular buffer ---- */

static temp_sample_t *s_temp_history;  /* allocated in PSRAM */
static int s_temp_head;   /* next write index */
static int s_temp_count;  /* number of valid samples */

/* ---- State (Marlin backend) ---- */

static printer_state_t s_state;
static SemaphoreHandle_t s_state_mutex;

static StreamBufferHandle_t s_rx_stream;
static QueueHandle_t s_cmd_queue;

/* Line buffer for assembling responses */
static char s_line_buf[RX_LINE_BUF_SIZE];
static size_t s_line_len;

/* Simulation mode */
static bool s_simulate = false;
static int64_t s_sim_start_us;
static printer_opstate_t s_sim_opstate;
static int64_t s_sim_pause_elapsed_us;  /* accumulated elapsed time before pause */

/* Tracking which query we're waiting for */
typedef enum {
    QUERY_NONE,
    QUERY_M105,
    QUERY_M27,
    QUERY_M27C,
    QUERY_M114,
    QUERY_M119,
    QUERY_CMD,
} query_type_t;

static query_type_t s_pending_query;

/* Polling suppression (terminal manual mode) */
static bool s_polling_suppressed;

/* Timestamps for polling intervals */
static int64_t s_last_m105_us;
static int64_t s_last_m27_us;
static int64_t s_last_m114_us;
static int64_t s_last_m119_us;
static bool s_filament_sensor_present;  /* true once M119 reports a filament: line */

/* One-shot M290 query on connect */
static bool s_m290_queried;

/* Print start time tracking */
static int64_t s_print_start_us;
static bool s_need_filename;
static bool s_have_m73;        /* true once we've seen M73 — prefer slicer progress over M27 bytes */

/* Cooldown after pause/resume/cancel — let firmware settle */
static int64_t s_cmd_cooldown_until_us;

/* Unsupported command tracking — commands that Marlin reports as unknown.
 * Store just the numeric code (e.g. 73 for M73, 900 for M900). */
#define UNSUPPORTED_CMD_MAX 16
static uint16_t s_unsupported_cmds[UNSUPPORTED_CMD_MAX];
static int s_unsupported_count;

/* ---- Host print state ---- */
static FILE *s_host_file;
static bool s_host_printing, s_host_paused;
static int32_t s_host_lines_sent, s_host_total_lines, s_host_layer;
static bool s_host_has_layer_comments;  /* true if file has ;LAYER: comments */
static int64_t s_host_start_us, s_host_pause_elapsed_us;
static char s_host_filename[64];
static char s_host_pending_line[256]; /* line read but not yet sent */
static bool s_host_has_pending;
static bool s_host_line_truncated;    /* true when fgets didn't reach newline (long line split) */
static float s_park_x, s_park_y, s_park_z; /* saved position before park */

/* Marlin line numbering & checksum for reliable host print transmission */
static int32_t s_host_marlin_line;        /* Next Marlin line number to assign */
static int32_t s_host_resend_line;        /* Line requested by Resend:, or -1 */
static char    s_host_numbered_line[160]; /* Formatted "Nxxx gcode*cs" for resend */
static int     s_host_resend_retries;
#define HOST_RESEND_MAX_RETRIES 5

/* Host print command timeout: must be longer than the M113 keepalive
 * interval (2s) so Marlin has a chance to send "busy: processing"
 * before we give up and retry. */
#define HOST_CMD_TIMEOUT_MS       3000

/* Park position and retract settings */
#define PARK_X       0.0f
#define PARK_Y       0.0f
#define PARK_Z_LIFT  5.0f    /* mm to lift Z on pause */
#define PARK_RETRACT 2.0f    /* mm filament retract on pause */

/* Internal command types for host print (sent via s_cmd_queue) */
#define PCMD_HOST_START  100
#define PCMD_HOST_PAUSE  101
#define PCMD_HOST_RESUME 102
#define PCMD_HOST_CANCEL 103

/** Check if a GCode command matches one of the echo-ping prefixes.
 *  The gcode may be numbered ("N123 G92 E0*XX") — skip the N-prefix.
 *  Matches comma-separated prefixes in s_echo_ping_cmds (e.g. "M119,G92"). */
static bool needs_echo_ping(const char *gcode)
{
    if (s_echo_ping_cmds[0] == '\0') return false;

    /* Skip N-prefix for numbered lines: "N123 G92 E0*XX" → "G92 E0*XX" */
    const char *cmd = gcode;
    if ((cmd[0] == 'N' || cmd[0] == 'n') && cmd[1] >= '0' && cmd[1] <= '9') {
        const char *sp = strchr(cmd, ' ');
        if (sp) cmd = sp + 1;
    }

    char buf[128];
    strncpy(buf, s_echo_ping_cmds, sizeof(buf));
    buf[sizeof(buf) - 1] = '\0';
    char *tok = strtok(buf, ", ");
    while (tok) {
        /* Skip leading spaces */
        while (*tok == ' ') tok++;
        size_t tlen = strlen(tok);
        if (tlen > 0 && strncasecmp(cmd, tok, tlen) == 0) {
            /* Match if prefix ends at command boundary (space, *, end) */
            char next = cmd[tlen];
            if (next == '\0' || next == ' ' || next == '*') return true;
        }
        tok = strtok(NULL, ", ");
    }
    return false;
}

/* ---- Helpers ---- */

/** Compute Marlin XOR checksum: XOR all bytes in the string */
static uint8_t marlin_checksum(const char *line)
{
    uint8_t cs = 0;
    while (*line) cs ^= (uint8_t)*line++;
    return cs;
}

/** Format a numbered line: "Nxxx gcode*cs" into dst */
static int format_numbered_line(char *dst, size_t size, int32_t line_num, const char *gcode)
{
    int n = snprintf(dst, size, "N%ld %s", (long)line_num, gcode);
    if (n < 0 || (size_t)n >= size - 5) return -1; /* no room for *cs */
    uint8_t cs = marlin_checksum(dst);
    n += snprintf(dst + n, size - n, "*%u", cs);
    return n;
}

/** Find value after a key character, e.g. 'T' in "T:205.3 /210.0" */
static bool parse_temp_pair(const char *line, char key,
                            float *actual, float *target)
{
    /* Look for "X:" pattern */
    char pattern[4];
    snprintf(pattern, sizeof(pattern), "%c:", key);
    const char *p = strstr(line, pattern);
    if (!p) return false;

    p += 2; /* skip "X:" */
    *actual = strtof(p, (char **)&p);

    /* Look for " /NNN" target temperature */
    while (*p == ' ') p++;
    if (*p == '/') {
        p++;
        *target = strtof(p, NULL);
    }
    return true;
}

void printer_comm_record_temp_sample(float hotend_actual, float hotend_target,
                                     float bed_actual, float bed_target)
{
    temp_sample_t *sample = &s_temp_history[s_temp_head];
    sample->timestamp_us = esp_timer_get_time();
    sample->hotend_actual = hotend_actual;
    sample->hotend_target = hotend_target;
    sample->bed_actual = bed_actual;
    sample->bed_target = bed_target;
    s_temp_head = (s_temp_head + 1) % TEMP_HISTORY_MAX;
    if (s_temp_count < TEMP_HISTORY_MAX) s_temp_count++;
}

/** Parse M105 response: "ok T:205.3 /210.0 B:60.1 /60.0" */
static void parse_m105(const char *line)
{
    float ha, ht, ba, bt;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);

    if (parse_temp_pair(line, 'T', &ha, &ht)) {
        s_state.hotend_actual = ha;
        s_state.hotend_target = ht;
    }
    if (parse_temp_pair(line, 'B', &ba, &bt)) {
        s_state.bed_actual = ba;
        s_state.bed_target = bt;
    }

    s_state.last_update_us = esp_timer_get_time();

    printer_comm_record_temp_sample(s_state.hotend_actual, s_state.hotend_target,
                                    s_state.bed_actual, s_state.bed_target);

    /* Update opstate from temps if not already printing */
    if (s_state.opstate == PRINTER_DISCONNECTED) {
        s_state.opstate = PRINTER_OPERATIONAL;
    }

    xSemaphoreGive(s_state_mutex);
}

/** Parse M27 response: "SD printing byte 12345/67890" or "Not SD printing" */
static void parse_m27(const char *line)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);

    const char *sd = strstr(line, "SD printing byte");
    if (sd) {
        long pos = 0, total = 0;
        if (sscanf(sd, "SD printing byte %ld/%ld", &pos, &total) == 2 && total > 0) {

            if (s_state.opstate != PRINTER_PAUSED) {
                if (s_state.opstate != PRINTER_PRINTING) {
                    s_print_start_us = esp_timer_get_time();
                    s_need_filename = true;
                    s_have_m73 = false;
                }
                s_state.opstate = PRINTER_PRINTING;
            }

            /* M27 byte-position progress — use as fallback if slicer doesn't embed M73 */
            if (!s_have_m73) {
                s_state.progress_pct = (float)pos / (float)total * 100.0f;

                int64_t elapsed_us = esp_timer_get_time() - s_print_start_us;
                s_state.print_time_s = (int32_t)(elapsed_us / 1000000);
                if (s_state.progress_pct > 0.1f) {
                    float total_est = (float)s_state.print_time_s / (s_state.progress_pct / 100.0f);
                    s_state.print_time_left_s = (int32_t)(total_est - s_state.print_time_s);
                    if (s_state.print_time_left_s < 0) s_state.print_time_left_s = 0;
                }
            } else {
                /* Still track elapsed time from wall clock */
                int64_t elapsed_us = esp_timer_get_time() - s_print_start_us;
                s_state.print_time_s = (int32_t)(elapsed_us / 1000000);
            }

            /* Estimate object height and layers from Z + progress */
            if (s_state.z > 0.1f && s_state.progress_pct > 1.0f) {
                s_state.object_height = s_state.z / (s_state.progress_pct / 100.0f);
                float lh = s_state.layer_height > 0 ? s_state.layer_height : 0.2f;
                s_state.total_layers = (int32_t)(s_state.object_height / lh + 0.5f);
                /* Estimate current layer from Z */
                float flh = s_state.first_layer_height > 0 ? s_state.first_layer_height : lh;
                if (s_state.z <= flh)
                    s_state.current_layer = 1;
                else
                    s_state.current_layer = 1 + (int32_t)((s_state.z - flh) / lh + 0.5f);
            }
        }
    } else if (strstr(line, "Not SD printing")) {
        if (s_state.opstate == PRINTER_PRINTING || s_state.opstate == PRINTER_PAUSED) {
            s_state.opstate = PRINTER_OPERATIONAL;
            s_state.progress_pct = -1;
            s_state.print_time_s = -1;
            s_state.print_time_left_s = -1;
            s_state.filename[0] = '\0';
            s_state.object_height = 0;
            s_state.total_layers = -1;
        }
    }

    xSemaphoreGive(s_state_mutex);
}

/** Parse M27 C response: "Current file: filename.gcode" */
static void parse_m27c(const char *line)
{
    const char *prefix = "Current file: ";
    const char *p = strstr(line, prefix);
    if (!p) return;

    p += strlen(prefix);
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    strncpy(s_state.filename, p, sizeof(s_state.filename) - 1);
    s_state.filename[sizeof(s_state.filename) - 1] = '\0';
    xSemaphoreGive(s_state_mutex);

    ESP_LOGI(TAG, "Current file: %s", s_state.filename);
}

/** Parse M114 response: "X:100.00 Y:50.00 Z:0.30 E:123.45" */
static void parse_m114(const char *line)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);

    const char *p;
    p = strstr(line, "X:");
    if (p) s_state.x = strtof(p + 2, NULL);
    p = strstr(line, "Y:");
    if (p) s_state.y = strtof(p + 2, NULL);
    p = strstr(line, "Z:");
    if (p) s_state.z = strtof(p + 2, NULL);

    xSemaphoreGive(s_state_mutex);
}

/** Parse M119 response line for filament sensor.
 *  Marlin reports: "filament: TRIGGERED" (filament present) or "filament: open" (runout).
 *  Only acts during host printing — triggers auto-pause on runout. */
static void parse_m119(const char *line)
{
    const char *p = strstr(line, "filament:");
    if (!p) return;

    s_filament_sensor_present = true;
    const char *val = p + 9;
    while (*val == ' ') val++;

    if (strncasecmp(val, "open", 4) == 0) {
        /* Filament runout detected */
        if (s_host_printing && !s_host_paused) {
            ESP_LOGW(TAG, "Filament runout detected via M119 — pausing host print");
            printer_cmd_t cmd = { .type = PCMD_PAUSE };
            xQueueSend(s_cmd_queue, &cmd, 0);
        }
    }
}

/** Process one complete line from the printer */
static void process_line(const char *line)
{
    /* Skip empty lines */
    if (line[0] == '\0') return;

    ESP_LOGI(TAG, "RX: %s", line);

    /* Content-based matching — handles unsolicited auto-reports too */
    if (strstr(line, "T:")) {
        parse_m105(line);
    } else if (strstr(line, "SD printing") || strstr(line, "Not SD")) {
        parse_m27(line);
    } else if (strstr(line, "Current file:")) {
        parse_m27c(line);
    } else if (strstr(line, "X:") && strstr(line, "Y:") && strstr(line, "Z:")) {
        parse_m114(line);
    } else if (strstr(line, "filament:")) {
        parse_m119(line);
    }

    /* Parse M290 response: "echo:Probe Offset Z-1.69" */
    const char *poff = strstr(line, "Probe Offset Z");
    if (poff) {
        float z = strtof(poff + 14, NULL);
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        s_state.probe_z_offset = z;
        xSemaphoreGive(s_state_mutex);
        ESP_LOGI(TAG, "Probe Z offset: %.2f", z);
    }

    /* Detect unsupported commands: echo:Unknown command: "M73 P15 R21" */
    const char *unk = strstr(line, "Unknown command:");
    if (unk) {
        /* Find the M/G code in the quoted string */
        const char *q = strchr(unk, '"');
        if (q && (q[1] == 'M' || q[1] == 'G')) {
            uint16_t code = (uint16_t)atoi(q + 2);
            /* Add to unsupported list if not already there */
            bool found = false;
            for (int i = 0; i < s_unsupported_count; i++) {
                if (s_unsupported_cmds[i] == code) { found = true; break; }
            }
            if (!found && s_unsupported_count < UNSUPPORTED_CMD_MAX) {
                s_unsupported_cmds[s_unsupported_count++] = code;
                ESP_LOGW(TAG, "Marking %c%u as unsupported",
                         q[1], code);
            }
        }
    }

    /* M73 from slicer — Marlin echoes "echo:Unknown command: "M73 P15 R21"" */
    const char *m73 = strstr(line, "M73");
    if (m73) {
        int pct = -1;
        int mins = -1;
        const char *pp = strstr(m73, "P");
        if (pp) pct = atoi(pp + 1);
        const char *rp = strstr(m73, "R");
        if (rp) mins = atoi(rp + 1);
        if (pct >= 0 && pct <= 100) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_have_m73 = true;
            s_state.progress_pct = (float)pct;
            if (mins >= 0) {
                s_state.print_time_left_s = mins * 60;
            }
            xSemaphoreGive(s_state_mutex);
            ESP_LOGI(TAG, "M73: progress=%d%% remaining=%dmin", pct, mins);
        }
    }

    /* Marlin action commands: //action:pause, //action:out_of_filament, //action:resume */
    if (strncmp(line, "//action:", 9) == 0) {
        const char *action = line + 9;
        if (strcmp(action, "pause") == 0 || strcmp(action, "out_of_filament") == 0) {
            if (s_host_printing && !s_host_paused) {
                ESP_LOGW(TAG, "Printer requested pause: %s", action);
                printer_cmd_t cmd = { .type = PCMD_PAUSE };
                xQueueSend(s_cmd_queue, &cmd, 0);
            }
        } else if (strcmp(action, "resume") == 0) {
            if (s_host_printing && s_host_paused) {
                ESP_LOGI(TAG, "Printer requested resume");
                printer_cmd_t cmd = { .type = PCMD_RESUME };
                xQueueSend(s_cmd_queue, &cmd, 0);
            }
        }
        return;
    }

    /* Marlin says "busy: processing" when it can't accept commands yet */
    if (strstr(line, "busy:")) {
        s_cmd_cooldown_until_us = esp_timer_get_time() + 4000000LL; /* back off 4s */
        s_pending_query = QUERY_NONE;
        return;
    }

    /* Marlin resend request: "Resend: N123" or "rs N123" */
    if (strncasecmp(line, "resend", 6) == 0 || strncmp(line, "rs ", 3) == 0) {
        const char *p = line;
        while (*p && (*p < '0' || *p > '9')) p++; /* skip to first digit */
        if (*p) {
            s_host_resend_line = (int32_t)atol(p);
            ESP_LOGW(TAG, "Resend requested: N%ld", (long)s_host_resend_line);
        }
        /* Don't return — Marlin sends "ok" after "Resend:" which clears s_pending_query */
    }

    /* "ok" signals end of response for current query.
     * Marlin often sends "ok T:205.3 /210.0 B:60.1 /60.0" — temp data on the ok line. */
    if (strncmp(line, "ok", 2) == 0) {
        if (strstr(line, "T:")) {
            parse_m105(line);
        }
        s_pending_query = QUERY_NONE;
    }
}

/** Drain buffered RX data, processing any complete lines (e.g. auto-reports) */
static void drain_rx(void)
{
    uint8_t rx_byte;
    while (xStreamBufferReceive(s_rx_stream, &rx_byte, 1, 0) > 0) {
        if (rx_byte == '\n' || rx_byte == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                process_line(s_line_buf);
                s_line_len = 0;
            }
        } else if (s_line_len < RX_LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = (char)rx_byte;
        }
    }
}

/** Check if a GCode line is a long-running command that blocks until done.
 *  M109=wait hotend, M190=wait bed, M116=wait all temps — send temp reports while waiting.
 *  G28=homing, G29=bed leveling — produce NO output while running.
 *  These need extended timeouts.
 *  If is_silent is non-NULL, sets it to true for commands that produce no interim feedback. */
static bool is_wait_cmd(const char *gcode, bool *is_silent)
{
    if (is_silent) *is_silent = false;
    /* Skip line number prefix "N123 " */
    const char *p = gcode;
    if (*p == 'N' || *p == 'n') {
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;
    }
    if (*p == 'M') {
        int code = atoi(p + 1);
        return (code == 109 || code == 190 || code == 116);
    }
    if (*p == 'G') {
        int code = atoi(p + 1);
        if (code == 28 || code == 29) {
            if (is_silent) *is_silent = true;
            return true;
        }
    }
    return false;
}

/** Send a gcode command to the printer, wait for response lines. */
static bool send_query(const char *gcode, query_type_t qtype)
{
    if (!usb_serial_is_connected()) return false;

    /* Process any buffered unsolicited data before sending */
    s_pending_query = QUERY_NONE;
    drain_rx();

    /* Abort if drain_rx saw busy: and set cooldown */
    if (esp_timer_get_time() < s_cmd_cooldown_until_us) return false;

    s_pending_query = qtype;
    s_line_len = 0;

    /* Send command with newline */
    char buf[100];
    int len = snprintf(buf, sizeof(buf), "%s\n", gcode);
    ESP_LOGI(TAG, "TX: %s", gcode);
    terminal_feed_tx(gcode);
    esp_err_t err = usb_serial_send((const uint8_t *)buf, len);
    if (err != ESP_OK) {
        s_pending_query = QUERY_NONE;
        return false;
    }

    /* Brief pause after TX — gives STM32 CDC time to process the USB
     * OUT packet before we start polling for the response. */
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Echo ping: some STM32 CDC implementations hold TX data until the
     * next OUT transaction.  Send a harmless M118 to flush the response.
     * The "echo:ping" reply is ignored by process_line. */
    if (needs_echo_ping(gcode)) {
        const char *ping = "M118 E1 ping\n";
        usb_serial_send((const uint8_t *)ping, strlen(ping));
    }

    /* Wait for "ok" or timeout, processing lines as they arrive. */
    bool wait_silent = false;
    bool wait = is_wait_cmd(gcode, &wait_silent);
    /* Host printing uses shorter timeout to detect CDC stalls quickly.
     * Wait commands (M109, M190, G28, G29) and non-host queries use full timeout.
     * Silent wait commands (G28, G29) get extra-long timeout since printer
     * produces no interim feedback at all during homing/leveling. */
    int timeout_ms;
    if (wait_silent)
        timeout_ms = WAIT_SILENT_TIMEOUT_MS;
    else if (s_host_printing && !wait)
        timeout_ms = HOST_CMD_TIMEOUT_MS;
    else
        timeout_ms = CMD_RESPONSE_TIMEOUT_MS;
    int64_t start_us = esp_timer_get_time();
    int64_t deadline = start_us + timeout_ms * 1000LL;
    uint8_t rx_byte;

    while (s_pending_query != QUERY_NONE) {
        int64_t remaining_us = deadline - esp_timer_get_time();
        if (remaining_us <= 0) {
            ESP_LOGW(TAG, "CDC TX stall on %s — no response, continuing", gcode);
            s_pending_query = QUERY_NONE;
            return false;
        }

        TickType_t ticks = pdMS_TO_TICKS(remaining_us / 1000);
        if (ticks == 0) ticks = 1;

        size_t n = xStreamBufferReceive(s_rx_stream, &rx_byte, 1, ticks);
        if (n == 0) continue;

        if (rx_byte == '\n' || rx_byte == '\r') {
            if (s_line_len > 0) {
                s_line_buf[s_line_len] = '\0';
                process_line(s_line_buf);
                s_line_len = 0;

                /* "busy: processing" clears s_pending_query in process_line.
                 * For wait/long commands, restore it — we still need the real "ok".
                 * For all commands, extend deadline since printer is alive. */
                if (s_pending_query == QUERY_NONE
                    && esp_timer_get_time() < s_cmd_cooldown_until_us) {
                    if (wait) {
                        s_pending_query = qtype;
                    }
                    deadline = esp_timer_get_time() + CMD_RESPONSE_TIMEOUT_MS * 1000LL;
                }
                /* Wait commands: printer sends temp reports ("T:... W:N") while
                 * waiting — extend deadline on any received line, not just "busy". */
                else if (wait && s_pending_query != QUERY_NONE) {
                    deadline = esp_timer_get_time() + CMD_RESPONSE_TIMEOUT_MS * 1000LL;
                }
            }
        } else if (s_line_len < RX_LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = (char)rx_byte;
        }
    }

    return true;
}

/** Send a numbered GCode command with checksum, handle Resend: retries.
 *  Used only for host print streaming lines. */
static bool send_query_numbered(const char *gcode)
{
    s_host_resend_line = -1;
    s_host_resend_retries = 0;

    if (format_numbered_line(s_host_numbered_line, sizeof(s_host_numbered_line),
                             s_host_marlin_line, gcode) < 0) {
        ESP_LOGE(TAG, "Line too long for numbering: %s", gcode);
        return send_query(gcode, QUERY_CMD); /* fallback unnumbered */
    }

    if (!send_query(s_host_numbered_line, QUERY_CMD)) {
        return false;
    }

    /* Handle resend requests */
    while (s_host_resend_line >= 0) {
        if (s_host_resend_line == s_host_marlin_line + 1) {
            /* Printer already executed our line and wants the next one.
             * This means our command succeeded — just advance the counter. */
            ESP_LOGW(TAG, "Printer already has N%ld, wants N%ld — command accepted",
                     (long)s_host_marlin_line, (long)s_host_resend_line);
            s_host_resend_line = -1;
            s_host_marlin_line++;
            return true;
        }
        if (s_host_resend_line != s_host_marlin_line) {
            ESP_LOGE(TAG, "Resend line mismatch: expected N%ld, got N%ld — resync",
                     (long)s_host_marlin_line, (long)s_host_resend_line);
            /* Resync Marlin's line counter to our current line */
            char sync[40];
            format_numbered_line(sync, sizeof(sync), s_host_marlin_line, "M110");
            send_query(sync, QUERY_CMD);
            s_host_resend_line = -1;
            return false;
        }

        s_host_resend_retries++;
        if (s_host_resend_retries > HOST_RESEND_MAX_RETRIES) {
            ESP_LOGE(TAG, "Resend retries exhausted for N%ld", (long)s_host_marlin_line);
            s_host_resend_line = -1;
            return false;
        }

        ESP_LOGW(TAG, "Resending N%ld (attempt %d)", (long)s_host_marlin_line, s_host_resend_retries);
        s_host_resend_line = -1;
        if (!send_query(s_host_numbered_line, QUERY_CMD)) {
            return false;
        }
    }

    s_host_marlin_line++;
    return true;
}

/* ---- Host print helpers ---- */

/** Scan GCode file: count lines, find max Z, layer count, layer heights */
typedef struct {
    int32_t total_lines;
    float max_z;              /* highest Z from G0/G1 moves */
    float layer_height;       /* from slicer comment or computed */
    float first_layer_height; /* from slicer comment or first Z */
    int32_t total_layers;     /* from ;LAYER: comments */
} gcode_scan_t;

static void scan_gcode_file(FILE *f, gcode_scan_t *out)
{
    memset(out, 0, sizeof(*out));
    out->total_layers = -1;

    char buf[256];
    int32_t max_layer = -1;
    int32_t layer_change_count = 0;
    float first_z = 0, prev_z = 0;
    bool seen_z = false;

    rewind(f);
    while (fgets(buf, sizeof(buf), f)) {
        out->total_lines++;

        /* Slicer metadata comments */
        if (buf[0] == ';') {
            /* PrusaSlicer/SuperSlicer/OrcaSlicer: ; layer_height = 0.2 */
            const char *lh = strstr(buf, "layer_height");
            if (lh) {
                const char *eq = strchr(lh, '=');
                if (eq) {
                    float v = strtof(eq + 1, NULL);
                    if (v > 0.01f && v < 1.0f) {
                        /* "first_layer_height" vs "layer_height" */
                        if (strstr(buf, "first_layer_height"))
                            out->first_layer_height = v;
                        else
                            out->layer_height = v;
                    }
                }
            }
            /* ;LAYER:N — track highest layer number */
            if (strncmp(buf, ";LAYER:", 7) == 0) {
                int32_t n = atoi(buf + 7);
                if (n > max_layer) max_layer = n;
            }
            /* OrcaSlicer: ;LAYER_CHANGE */
            if (strncmp(buf, ";LAYER_CHANGE", 13) == 0) {
                layer_change_count++;
            }
            /* Cura: ;Layer height: 0.2 */
            const char *clh = strstr(buf, ";Layer height:");
            if (clh) {
                float v = strtof(clh + 14, NULL);
                if (v > 0.01f && v < 1.0f) out->layer_height = v;
            }
            continue;
        }

        /* Track Z from G0/G1 moves */
        if (buf[0] == 'G' && (buf[1] == '0' || buf[1] == '1') && buf[2] == ' ') {
            const char *zp = strstr(buf, "Z");
            if (zp && (zp == buf + 3 || *(zp - 1) == ' ')) {
                float z = strtof(zp + 1, NULL);
                if (z > 0.01f) {
                    if (!seen_z) {
                        first_z = z;
                        seen_z = true;
                    }
                    if (z > out->max_z) out->max_z = z;
                    prev_z = z;
                }
            }
        }
    }
    rewind(f);

    /* Derive values if not found in comments */
    if (max_layer >= 0)
        out->total_layers = max_layer + 1;  /* ;LAYER: is 0-based */
    else if (layer_change_count > 0)
        out->total_layers = layer_change_count;

    if (out->first_layer_height == 0 && first_z > 0)
        out->first_layer_height = first_z;

    if (out->layer_height == 0 && out->total_layers > 1 && out->max_z > 0) {
        /* Estimate: (maxZ - first_layer) / (layers - 1) */
        float flh = out->first_layer_height > 0 ? out->first_layer_height : first_z;
        out->layer_height = (out->max_z - flh) / (float)(out->total_layers - 1);
    }

    /* If still no total_layers, estimate from Z and layer height */
    if (out->total_layers <= 0 && out->max_z > 0 && out->layer_height > 0) {
        float flh = out->first_layer_height > 0 ? out->first_layer_height : out->layer_height;
        out->total_layers = 1 + (int32_t)((out->max_z - flh) / out->layer_height + 0.5f);
    }

    (void)prev_z;
    ESP_LOGI(TAG, "GCode scan: %ld lines, maxZ=%.2f, layers=%ld, lh=%.2f, flh=%.2f",
             (long)out->total_lines, out->max_z, (long)out->total_layers,
             out->layer_height, out->first_layer_height);
}

/* ---- Public file info scan (head + tail for speed) ---- */

void gcode_scan_file_info(const char *path, gcode_file_info_t *out)
{
    memset(out, 0, sizeof(*out));
    out->total_layers = -1;
    out->est_time_s = -1;
    out->filament_used_mm = -1;
    out->filament_used_g = -1;

    FILE *f = fopen(path, "r");
    if (!f) return;

    /* Get file size for tail scanning */
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    rewind(f);

    char buf[256];
    int32_t max_layer = -1;
    int32_t layer_change_count = 0;
    float first_z = 0, prev_z = 0;
    bool seen_z = false;

    /* Thumbnail state */
    bool in_thumbnail = false;
    int best_thumb_pixels = 0;
    int cur_thumb_w = 0, cur_thumb_h = 0;
    size_t thumb_alloc = 0, thumb_len = 0;
    char *thumb_buf = NULL;       /* accumulates current thumbnail */
    char *best_thumb = NULL;      /* best (largest) thumbnail found */
    int best_thumb_w = 0, best_thumb_h = 0;
    #define MAX_THUMB_DECODED 40960  /* 40KB cap */

    /* Parse a block of lines for metadata */
    #define PARSE_LINE() do { \
        if (buf[0] == ';') { \
            /* Layer height */ \
            const char *lh = strstr(buf, "layer_height"); \
            if (lh) { \
                const char *eq = strchr(lh, '='); \
                if (eq) { \
                    float v = strtof(eq + 1, NULL); \
                    if (v > 0.01f && v < 1.0f) { \
                        if (strstr(buf, "first_layer_height")) \
                            out->first_layer_height = v; \
                        else \
                            out->layer_height = v; \
                    } \
                } \
            } \
            /* Cura: ;Layer height: 0.2 */ \
            const char *clh = strstr(buf, ";Layer height:"); \
            if (clh) { \
                float v = strtof(clh + 14, NULL); \
                if (v > 0.01f && v < 1.0f) out->layer_height = v; \
            } \
            /* ;LAYER:N */ \
            if (strncmp(buf, ";LAYER:", 7) == 0) { \
                int32_t n = atoi(buf + 7); \
                if (n > max_layer) max_layer = n; \
            } \
            if (strncmp(buf, ";LAYER_CHANGE", 13) == 0) \
                layer_change_count++; \
            /* Print time: PrusaSlicer/OrcaSlicer */ \
            if (out->est_time_s < 0) { \
                const char *etp = strstr(buf, "estimated printing time"); \
                if (etp) { \
                    const char *eq = strchr(etp, '='); \
                    if (eq) { \
                        eq++; \
                        int h = 0, m = 0, s = 0; \
                        const char *p = eq; \
                        while (*p) { \
                            if (*p >= '0' && *p <= '9') { \
                                int val = atoi(p); \
                                while (*p >= '0' && *p <= '9') p++; \
                                if (*p == 'h') h = val; \
                                else if (*p == 'm') m = val; \
                                else if (*p == 's') s = val; \
                            } \
                            if (*p) p++; \
                        } \
                        out->est_time_s = h * 3600 + m * 60 + s; \
                    } \
                } \
            } \
            /* Print time: Cura ;TIME:5025 */ \
            if (out->est_time_s < 0 && strncmp(buf, ";TIME:", 6) == 0) { \
                int t = atoi(buf + 6); \
                if (t > 0) out->est_time_s = t; \
            } \
            /* Filament: PrusaSlicer */ \
            if (out->filament_used_mm < 0) { \
                const char *fm = strstr(buf, "filament used [mm]"); \
                if (fm) { \
                    const char *eq = strchr(fm, '='); \
                    if (eq) out->filament_used_mm = strtof(eq + 1, NULL); \
                } \
            } \
            if (out->filament_used_g < 0) { \
                const char *fg = strstr(buf, "filament used [g]"); \
                if (fg) { \
                    const char *eq = strchr(fg, '='); \
                    if (eq) out->filament_used_g = strtof(eq + 1, NULL); \
                } \
            } \
            /* Filament: Cura ;Filament used: 1.234m */ \
            if (out->filament_used_mm < 0) { \
                const char *fc = strstr(buf, ";Filament used:"); \
                if (fc) { \
                    float v = strtof(fc + 15, NULL); \
                    if (v > 0) out->filament_used_mm = v * 1000.0f; \
                } \
            } \
            /* Thumbnail parsing */ \
            if (strstr(buf, "; thumbnail begin")) { \
                int tw = 0, th = 0, tsz = 0; \
                if (sscanf(buf, "; thumbnail begin %dx%d %d", &tw, &th, &tsz) >= 2) { \
                    in_thumbnail = true; \
                    cur_thumb_w = tw; \
                    cur_thumb_h = th; \
                    /* Estimate base64 size (4/3 * decoded + padding) */ \
                    size_t est = (tsz > 0 ? (size_t)(tsz * 4 / 3 + 100) : 8192); \
                    if (est > MAX_THUMB_DECODED * 4 / 3 + 100) est = MAX_THUMB_DECODED * 4 / 3 + 100; \
                    free(thumb_buf); \
                    thumb_buf = malloc(est + 1); \
                    thumb_alloc = thumb_buf ? est : 0; \
                    thumb_len = 0; \
                } \
            } else if (in_thumbnail && strstr(buf, "; thumbnail end")) { \
                in_thumbnail = false; \
                if (thumb_buf && thumb_len > 0) { \
                    int pixels = cur_thumb_w * cur_thumb_h; \
                    if (pixels > best_thumb_pixels) { \
                        best_thumb_pixels = pixels; \
                        free(best_thumb); \
                        thumb_buf[thumb_len] = '\0'; \
                        best_thumb = thumb_buf; \
                        thumb_buf = NULL; \
                        best_thumb_w = cur_thumb_w; \
                        best_thumb_h = cur_thumb_h; \
                    } else { \
                        free(thumb_buf); \
                        thumb_buf = NULL; \
                    } \
                } \
            } else if (in_thumbnail && thumb_buf) { \
                /* Strip "; " prefix and append base64 data */ \
                const char *data = buf; \
                if (data[0] == ';' && data[1] == ' ') data += 2; \
                else if (data[0] == ';') data += 1; \
                size_t dlen = strlen(data); \
                /* Trim trailing whitespace */ \
                while (dlen > 0 && (data[dlen-1] == '\n' || data[dlen-1] == '\r' || data[dlen-1] == ' ')) \
                    dlen--; \
                if (thumb_len + dlen < thumb_alloc) { \
                    memcpy(thumb_buf + thumb_len, data, dlen); \
                    thumb_len += dlen; \
                } \
            } \
        } else if (buf[0] == 'G' && (buf[1] == '0' || buf[1] == '1') && buf[2] == ' ') { \
            const char *zp = strstr(buf, "Z"); \
            if (zp && (zp == buf + 3 || *(zp - 1) == ' ')) { \
                float z = strtof(zp + 1, NULL); \
                if (z > 0.01f) { \
                    if (!seen_z) { first_z = z; seen_z = true; } \
                    if (z > out->max_z) out->max_z = z; \
                    prev_z = z; \
                } \
            } \
        } \
    } while(0)

    /* Scan head: first 500 lines for metadata, thumbnails, time, filament */
    int head_lines = 0;
    while (head_lines < 500 && fgets(buf, sizeof(buf), f)) {
        head_lines++;
        PARSE_LINE();
    }

    long middle_start = ftell(f);

    /* Scan tail: last ~32KB for slicer summary comments (time, filament, etc.) */
    if (file_size > 32768) {
        long tail_offset = file_size - 32768;
        if (tail_offset > middle_start) {
            fseek(f, tail_offset, SEEK_SET);
            fgets(buf, sizeof(buf), f);  /* skip partial line */
            while (fgets(buf, sizeof(buf), f)) {
                PARSE_LINE();
            }
        } else {
            /* File small enough that head+tail overlap, scan remainder */
            while (fgets(buf, sizeof(buf), f)) {
                PARSE_LINE();
            }
        }
    } else {
        while (fgets(buf, sizeof(buf), f)) {
            PARSE_LINE();
        }
    }

    #undef PARSE_LINE
    #undef MAX_THUMB_DECODED

    free(thumb_buf);

    /* Fast bulk scan of the middle section for layers and Z moves only.
     * Uses large fread() chunks instead of per-line fgets() for SD card speed. */
    long tail_start = (file_size > 32768) ? (file_size - 32768) : file_size;
    if (middle_start < tail_start) {
        fseek(f, middle_start, SEEK_SET);
        /* Use internal DMA-capable RAM — SDMMC DMA on ESP32-S3 can't access PSRAM,
         * falls back to 512-byte bounce buffer per sector if buffer is in PSRAM */
        #define SCAN_BUF_SIZE 4096
        char *sbuf = heap_caps_malloc(SCAN_BUF_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (!sbuf) sbuf = malloc(SCAN_BUF_SIZE);  /* PSRAM fallback */
        if (sbuf) {
            long bytes_left = tail_start - middle_start;
            /* Carry-over: partial line from end of previous chunk */
            char carry[128];
            int carry_len = 0;

            while (bytes_left > 0) {
                int to_read = bytes_left < SCAN_BUF_SIZE ? (int)bytes_left : SCAN_BUF_SIZE;
                int got = fread(sbuf, 1, to_read, f);
                if (got <= 0) break;
                bytes_left -= got;

                char *p = sbuf;
                char *end = sbuf + got;

                /* If we have carry-over, find end of that line first */
                if (carry_len > 0) {
                    char *nl = memchr(p, '\n', end - p);
                    int copy = nl ? (int)(nl - p) : (int)(end - p);
                    if (carry_len + copy < (int)sizeof(carry) - 1) {
                        memcpy(carry + carry_len, p, copy);
                        carry_len += copy;
                    }
                    if (nl) {
                        carry[carry_len] = '\0';
                        /* Process carried-over line */
                        if (carry[0] == ';') {
                            if (strncmp(carry, ";LAYER:", 7) == 0) {
                                int32_t n = atoi(carry + 7);
                                if (n > max_layer) max_layer = n;
                            } else if (strncmp(carry, ";LAYER_CHANGE", 13) == 0) {
                                layer_change_count++;
                            }
                        } else if (carry[0] == 'G' && (carry[1] == '0' || carry[1] == '1') && carry[2] == ' ') {
                            const char *zp = strstr(carry, "Z");
                            if (zp && (zp == carry + 3 || *(zp - 1) == ' ')) {
                                float z = strtof(zp + 1, NULL);
                                if (z > 0.01f) {
                                    if (!seen_z) { first_z = z; seen_z = true; }
                                    if (z > out->max_z) out->max_z = z;
                                    prev_z = z;
                                }
                            }
                        }
                        p = nl + 1;
                        carry_len = 0;
                    } else {
                        /* Entire chunk was part of one long line, skip it */
                        carry_len = 0;
                        continue;
                    }
                }

                /* Process complete lines within this chunk */
                while (p < end) {
                    char *nl = memchr(p, '\n', end - p);
                    if (!nl) {
                        /* Save partial line as carry-over */
                        int rem = (int)(end - p);
                        if (rem < (int)sizeof(carry)) {
                            memcpy(carry, p, rem);
                            carry_len = rem;
                        }
                        break;
                    }

                    /* Quick first-char filter — skip lines that can't match */
                    if (*p == ';') {
                        if (nl - p >= 7 && strncmp(p, ";LAYER:", 7) == 0) {
                            int32_t n = atoi(p + 7);
                            if (n > max_layer) max_layer = n;
                        } else if (nl - p >= 13 && strncmp(p, ";LAYER_CHANGE", 13) == 0) {
                            layer_change_count++;
                        }
                    } else if (*p == 'G' && nl - p > 3 && (p[1] == '0' || p[1] == '1') && p[2] == ' ') {
                        /* Look for Z parameter — scan for ' Z' or 'G0 Z'/'G1 Z' */
                        const char *zp = p + 3;
                        while (zp < nl - 1) {
                            if (*zp == 'Z' && (zp == p + 3 || *(zp - 1) == ' ')) {
                                float z = strtof(zp + 1, NULL);
                                if (z > 0.01f) {
                                    if (!seen_z) { first_z = z; seen_z = true; }
                                    if (z > out->max_z) out->max_z = z;
                                    prev_z = z;
                                }
                                break;
                            }
                            zp++;
                        }
                    }

                    p = nl + 1;
                }

                taskYIELD();  /* Let other tasks run between chunks */
            }
            free(sbuf);
        }
        #undef SCAN_BUF_SIZE
    }

    fclose(f);

    /* Derive layers */
    if (max_layer >= 0)
        out->total_layers = max_layer + 1;
    else if (layer_change_count > 0)
        out->total_layers = layer_change_count;

    if (out->first_layer_height == 0 && first_z > 0)
        out->first_layer_height = first_z;

    if (out->layer_height == 0 && out->total_layers > 1 && out->max_z > 0) {
        float flh = out->first_layer_height > 0 ? out->first_layer_height : first_z;
        out->layer_height = (out->max_z - flh) / (float)(out->total_layers - 1);
    }

    if (out->total_layers <= 0 && out->max_z > 0 && out->layer_height > 0) {
        float flh = out->first_layer_height > 0 ? out->first_layer_height : out->layer_height;
        out->total_layers = 1 + (int32_t)((out->max_z - flh) / out->layer_height + 0.5f);
    }

    /* Assign thumbnail */
    if (best_thumb) {
        out->thumbnail_base64 = best_thumb;
        out->thumb_w = best_thumb_w;
        out->thumb_h = best_thumb_h;
    }

    (void)prev_z;
    ESP_LOGI(TAG, "File info: layers=%ld, maxZ=%.2f, time=%lds, fil=%.0fmm/%.1fg, thumb=%dx%d",
             (long)out->total_layers, out->max_z, (long)out->est_time_s,
             out->filament_used_mm, out->filament_used_g,
             out->thumb_w, out->thumb_h);
}

/** Query current position and save it, then park the head */
static void host_print_park(void)
{
    /* Get fresh position */
    send_query("M114", QUERY_M114);

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_park_x = s_state.x;
    s_park_y = s_state.y;
    s_park_z = s_state.z;
    xSemaphoreGive(s_state_mutex);

    ESP_LOGI(TAG, "Parking head from (%.1f, %.1f, %.1f)", s_park_x, s_park_y, s_park_z);

    char buf[64];
    send_query("M400", QUERY_CMD);           /* Wait for moves to finish */
    send_query("G91", QUERY_CMD);            /* Relative positioning */
    snprintf(buf, sizeof(buf), "G1 E-%.1f F2400", PARK_RETRACT);
    send_query(buf, QUERY_CMD);              /* Retract filament */
    snprintf(buf, sizeof(buf), "G1 Z%.1f F600", PARK_Z_LIFT);
    send_query(buf, QUERY_CMD);              /* Lift Z */
    send_query("G90", QUERY_CMD);            /* Absolute positioning */
    snprintf(buf, sizeof(buf), "G1 X%.1f Y%.1f F6000", PARK_X, PARK_Y);
    send_query(buf, QUERY_CMD);              /* Park at corner */
}

/** Restore head position after park */
static void host_print_unpark(void)
{
    ESP_LOGI(TAG, "Unparking head to (%.1f, %.1f, %.1f)", s_park_x, s_park_y, s_park_z);

    char buf[64];
    send_query("G90", QUERY_CMD);            /* Absolute positioning */
    snprintf(buf, sizeof(buf), "G1 X%.1f Y%.1f F6000", s_park_x, s_park_y);
    send_query(buf, QUERY_CMD);              /* Move to saved XY */
    snprintf(buf, sizeof(buf), "G1 Z%.1f F600", s_park_z);
    send_query(buf, QUERY_CMD);              /* Lower Z back */
    send_query("G91", QUERY_CMD);            /* Relative positioning */
    snprintf(buf, sizeof(buf), "G1 E%.1f F2400", PARK_RETRACT);
    send_query(buf, QUERY_CMD);              /* Prime filament */
    send_query("G90", QUERY_CMD);            /* Absolute positioning */
    send_query("M83", QUERY_CMD);            /* Extruder relative (most slicers use this) */
}

/** Start host print from within the task context */
static void host_print_start_internal(const char *filename)
{
    if (s_host_printing) {
        ESP_LOGW(TAG, "Host print already active");
        return;
    }

    char path[128];
    snprintf(path, sizeof(path), "/sdcard/%s", filename);

    FILE *f = fopen(path, "r");
    if (!f) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return;
    }

    gcode_scan_t scan;
    scan_gcode_file(f, &scan);

    ESP_LOGI(TAG, "Host print starting: %s (%ld lines)", filename, (long)scan.total_lines);

    s_host_file = f;
    s_host_printing = true;
    s_host_paused = false;
    s_host_lines_sent = 0;
    s_host_total_lines = scan.total_lines;
    s_host_layer = 0;
    s_host_has_layer_comments = false;
    s_host_start_us = esp_timer_get_time();
    s_host_pause_elapsed_us = 0;
    s_host_has_pending = false;
    s_host_line_truncated = false;
    strncpy(s_host_filename, filename, sizeof(s_host_filename) - 1);
    s_host_filename[sizeof(s_host_filename) - 1] = '\0';

    /* Reset Marlin line numbering for reliable transmission */
    s_host_marlin_line = 0;
    s_host_resend_line = -1;
    char sync[40];
    format_numbered_line(sync, sizeof(sync), 0, "M110 N0");
    send_query(sync, QUERY_CMD);
    s_host_marlin_line = 1;

    /* Enable host keepalive — Marlin sends "busy: processing" every 2s
     * during long-running commands (G28, G29, etc.) so we know it's alive. */
    send_query("M113 S2", QUERY_CMD);

    /* Tell Marlin we're printing — starts LCD print timer & enables host mode */
    send_query("M75", QUERY_CMD);

    /* Show filename on LCD */
    char lcd_msg[128];
    snprintf(lcd_msg, sizeof(lcd_msg), "M117 %s", filename);
    send_query(lcd_msg, QUERY_CMD);

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_state.opstate = PRINTER_PRINTING;
    s_state.progress_pct = 0;
    s_state.print_time_s = 0;
    s_state.print_time_left_s = -1;
    s_state.host_printing = true;
    s_state.current_layer = 0;
    s_state.object_height = scan.max_z;
    s_state.total_layers = scan.total_layers;
    s_state.layer_height = scan.layer_height;
    s_state.first_layer_height = scan.first_layer_height;
    strncpy(s_state.filename, filename, sizeof(s_state.filename) - 1);
    s_state.filename[sizeof(s_state.filename) - 1] = '\0';
    xSemaphoreGive(s_state_mutex);
}

/** Finish host print (complete or cancel) */
static void host_print_finish(bool cancelled)
{
    if (s_host_file) {
        fclose(s_host_file);
        s_host_file = NULL;
    }

    /* Reset Marlin's line counter so it stops expecting numbered lines.
     * Must be sent as a numbered command with the expected line number. */
    char reset[40];
    format_numbered_line(reset, sizeof(reset), s_host_marlin_line, "M110 N0");
    send_query(reset, QUERY_CMD);
    s_host_resend_line = -1;

    /* Stop Marlin print timer */
    send_query("M77", QUERY_CMD);
    send_query("M117", QUERY_CMD);  /* Clear LCD message */

    /* Disable host keepalive — restore default (disabled) to reduce chatter */
    send_query("M113 S0", QUERY_CMD);

    if (cancelled) {
        ESP_LOGI(TAG, "Host print cancelled — parking and turning off heaters");
        send_query("M400", QUERY_CMD);       /* Wait for planner to drain */
        send_query("G91", QUERY_CMD);         /* Relative positioning */
        send_query("G1 Z10 F600", QUERY_CMD); /* Raise nozzle 10mm */
        send_query("G90", QUERY_CMD);         /* Absolute positioning */
        send_query("G1 X0 Y200 F3000", QUERY_CMD); /* Park head at rear-left */
        send_query("M104 S0", QUERY_CMD);     /* Hotend off */
        send_query("M140 S0", QUERY_CMD);     /* Bed off */
        send_query("M107", QUERY_CMD);         /* Fan off */
        send_query("M84", QUERY_CMD);          /* Disable steppers */
    } else {
        int64_t elapsed_us = s_host_pause_elapsed_us + (esp_timer_get_time() - s_host_start_us);
        ESP_LOGI(TAG, "Host print complete: %s (%ld lines, %lds)",
                 s_host_filename, (long)s_host_lines_sent,
                 (long)(elapsed_us / 1000000));
        send_query("M117 Print complete", QUERY_CMD);
    }

    s_host_printing = false;
    s_host_paused = false;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_state.opstate = PRINTER_OPERATIONAL;
    s_state.progress_pct = -1;
    s_state.print_time_s = -1;
    s_state.print_time_left_s = -1;
    s_state.host_printing = false;
    s_state.current_layer = 0;
    s_state.filename[0] = '\0';
    xSemaphoreGive(s_state_mutex);
}

/* ---- Polling task ---- */

static void printer_comm_task(void *arg)
{
    ESP_LOGI(TAG, "Printer comm task started");

    /* Initialize timing */
    s_last_m105_us = 0;
    s_last_m27_us = 0;
    s_last_m114_us = 0;

    while (true) {
        /* Wait for USB device to connect */
        if (!usb_serial_is_connected()) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            /* Preserve printing/paused state during USB disconnect so UI
             * keeps showing print status — host print will resume on reconnect */
            if (!s_host_printing) {
                s_state.opstate = PRINTER_DISCONNECTED;
            }
            xSemaphoreGive(s_state_mutex);
            s_m290_queried = false;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int64_t now = esp_timer_get_time();

        /* Drain command queue first */
        printer_cmd_t cmd;
        while (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
            const char *gcode = NULL;

            /* Host print commands */
            if (cmd.type == PCMD_HOST_START) {
                host_print_start_internal(cmd.gcode);
                continue;
            } else if (cmd.type == PCMD_HOST_PAUSE) {
                goto do_host_pause;
            } else if (cmd.type == PCMD_HOST_RESUME) {
                goto do_host_resume;
            } else if (cmd.type == PCMD_HOST_CANCEL) {
                if (s_host_printing) host_print_finish(true);
                continue;
            }

            /* Route pause/resume/cancel to host print if active */
            if (s_host_printing) {
                if (cmd.type == PCMD_PAUSE) goto do_host_pause;
                if (cmd.type == PCMD_RESUME) goto do_host_resume;
                if (cmd.type == PCMD_CANCEL) { host_print_finish(true); continue; }
            }

            if (false) {
            do_host_pause:
                if (s_host_printing && !s_host_paused) {
                    s_host_paused = true;
                    s_host_pause_elapsed_us += esp_timer_get_time() - s_host_start_us;
                    host_print_park();
                    send_query("M76", QUERY_CMD);  /* Pause Marlin print timer */
                    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                    s_state.opstate = PRINTER_PAUSED;
                    xSemaphoreGive(s_state_mutex);
                    ESP_LOGI(TAG, "Host print paused (head parked)");
                }
                continue;
            do_host_resume:
                if (s_host_printing && s_host_paused) {
                    send_query("M75", QUERY_CMD);  /* Resume Marlin print timer */
                    host_print_unpark();
                    s_host_paused = false;
                    s_host_start_us = esp_timer_get_time();
                    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                    s_state.opstate = PRINTER_PRINTING;
                    xSemaphoreGive(s_state_mutex);
                    ESP_LOGI(TAG, "Host print resumed (head restored)");
                }
                continue;
            }

            switch (cmd.type) {
            case PCMD_PAUSE:
                ESP_LOGI(TAG, "Draining planner (M400) before pause...");
                send_query("M400", QUERY_CMD);
                gcode = (s_pause_cmd == PAUSE_CMD_M524) ? "M524" : "M25";
                break;
            case PCMD_RESUME: gcode = "M24"; break;
            case PCMD_CANCEL: gcode = "M524"; break;
            case PCMD_RAW:    gcode = cmd.gcode; break;
            default: break;
            }
            if (gcode) {
                ESP_LOGI(TAG, "Sending command: %s", gcode);
                send_query(gcode, QUERY_CMD);

                if (cmd.type == PCMD_PAUSE) {
                    s_cmd_cooldown_until_us = esp_timer_get_time() + 5000000LL; /* 5s */
                }
            }
        }

        /* ---- Host print: stream GCode lines ---- */
        if (s_host_printing && !s_host_paused) {
            int lines_this_batch = 0;
            char line[256];

            while (lines_this_batch < 4 && s_host_file) {
                if (esp_timer_get_time() < s_cmd_cooldown_until_us) break;

                /* Use pending line from previous failed send, or read new */
                if (s_host_has_pending) {
                    strncpy(line, s_host_pending_line, sizeof(line));
                    s_host_has_pending = false;
                } else {
                    if (!fgets(line, sizeof(line), s_host_file)) {
                        /* EOF — print complete */
                        host_print_finish(false);
                        break;
                    }

                    /* Strip newline and detect truncated (split) long lines.
                     * fgets fills the buffer without a newline when the line
                     * is longer than sizeof(line)-1.  The continuations of
                     * such lines must be skipped until we see a newline. */
                    size_t ln = strlen(line);
                    bool has_newline = (ln > 0 && (line[ln-1] == '\n' || line[ln-1] == '\r'));
                    while (ln > 0 && (line[ln-1] == '\n' || line[ln-1] == '\r'))
                        line[--ln] = '\0';

                    if (s_host_line_truncated) {
                        /* Still consuming a split long line — skip until newline */
                        s_host_line_truncated = !has_newline;
                        s_host_lines_sent++;
                        continue;
                    }
                    if (!has_newline && ln > 0) {
                        /* First chunk of a long line — skip it and mark truncated */
                        s_host_line_truncated = true;
                        /* Still count and check if it was a comment */
                        if (line[0] != ';') {
                            ESP_LOGW(TAG, "Skipping long line (%d chars): %.40s...", (int)ln, line);
                        }
                        s_host_lines_sent++;
                        continue;
                    }

                    /* Parse layer comments for tracking */
                    if (strncmp(line, ";LAYER:", 7) == 0) {
                        s_host_layer = atoi(line + 7);
                        s_host_has_layer_comments = true;
                    } else if (strncmp(line, ";LAYER_CHANGE", 13) == 0) {
                        /* First LAYER_CHANGE = layer 0 (0-based, like ;LAYER:) */
                        if (s_host_has_layer_comments)
                            s_host_layer++;
                        s_host_has_layer_comments = true;
                    }

                    /* Skip empty lines and comments */
                    if (ln == 0 || line[0] == ';') {
                        s_host_lines_sent++;
                        continue;
                    }

                    /* Strip inline comments */
                    char *comment = strchr(line, ';');
                    if (comment) {
                        *comment = '\0';
                        /* Trim trailing spaces */
                        ln = strlen(line);
                        while (ln > 0 && line[ln-1] == ' ') line[--ln] = '\0';
                    }
                    if (ln == 0) {
                        s_host_lines_sent++;
                        continue;
                    }

                    /* Skip lines that aren't valid GCode commands.
                     * Valid GCode starts with G, M, T, N, O, or F.
                     * Slicer annotations like "27 @ 4.6mm" would otherwise
                     * be sent raw, causing response desync. */
                    char first = line[0];
                    if (first != 'G' && first != 'g' &&
                        first != 'M' && first != 'm' &&
                        first != 'T' && first != 't' &&
                        first != 'N' && first != 'n' &&
                        first != 'O' && first != 'o' &&
                        first != 'F' && first != 'f') {
                        ESP_LOGW(TAG, "Skipping non-GCode line: %s", line);
                        s_host_lines_sent++;
                        continue;
                    }
                }

                /* Check if this command was previously reported as unsupported */
                if ((line[0] == 'M' || line[0] == 'G') && s_unsupported_count > 0) {
                    uint16_t code = (uint16_t)atoi(line + 1);
                    bool skip = false;
                    for (int i = 0; i < s_unsupported_count; i++) {
                        if (s_unsupported_cmds[i] == code) { skip = true; break; }
                    }
                    if (skip) {
                        /* Still parse M73 locally for our own progress tracking */
                        if (line[0] == 'M' && code == 73) {
                            const char *pp = strstr(line, "P");
                            const char *rp = strstr(line, "R");
                            int pct = pp ? atoi(pp + 1) : -1;
                            int mins = rp ? atoi(rp + 1) : -1;
                            if (pct >= 0 && pct <= 100) {
                                xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                                s_state.progress_pct = (float)pct;
                                if (mins >= 0)
                                    s_state.print_time_left_s = mins * 60;
                                xSemaphoreGive(s_state_mutex);
                            }
                        }
                        s_host_lines_sent++;
                        continue;
                    }
                }

                /* Track Z from G0/G1 moves for real-time position */
                if ((line[0] == 'G') && (line[1] == '0' || line[1] == '1') && line[2] == ' ') {
                    const char *zp = strstr(line, "Z");
                    if (zp && (zp == line + 3 || *(zp - 1) == ' ')) {
                        float z = strtof(zp + 1, NULL);
                        if (z > 0.01f) {
                            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                            s_state.z = z;
                            xSemaphoreGive(s_state_mutex);
                        }
                    }
                }

                /* Send to printer with line numbering & checksum. */
                if (!send_query_numbered(line)) {
                    if (esp_timer_get_time() < s_cmd_cooldown_until_us) {
                        /* Printer said "busy: processing" — defer */
                        strncpy(s_host_pending_line, line, sizeof(s_host_pending_line));
                        s_host_has_pending = true;
                        break;
                    }
                    /* Wait commands (G28/G29) already had a long timeout in send_query.
                     * If they still failed, it's a real problem — abort. */
                    if (is_wait_cmd(line, NULL)) {
                        ESP_LOGE(TAG, "Wait command timed out, aborting host print: %s", line);
                        goto host_print_end;
                    }
                    /* STM32 CDC TX flush bug: the printer holds "ok" in its
                     * USB TX FIFO until the next OUT transaction triggers a
                     * flush.  Don't waste time retrying — advance immediately.
                     * The NEXT numbered command will flush USB, and if Marlin
                     * disagrees it will send Resend: which we handle. */
                    ESP_LOGW(TAG, "CDC TX stall on %s — advancing to next line", line);
                    s_host_marlin_line++;
                }
                s_host_lines_sent++;
                lines_this_batch++;

                /* Update progress */
                xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                if (s_host_total_lines > 0) {
                    s_state.progress_pct = (float)s_host_lines_sent / (float)s_host_total_lines * 100.0f;
                }
                int64_t elapsed_us = s_host_pause_elapsed_us + (esp_timer_get_time() - s_host_start_us);
                s_state.print_time_s = (int32_t)(elapsed_us / 1000000);
                s_state.current_layer = s_host_layer + 1;  /* ;LAYER: is 0-based, display 1-based */
                /* If no ;LAYER: comments in file, estimate from Z */
                if (!s_host_has_layer_comments && s_state.z > 0 && s_state.layer_height > 0) {
                    float flh = s_state.first_layer_height > 0 ? s_state.first_layer_height : s_state.layer_height;
                    if (s_state.z <= flh)
                        s_state.current_layer = 1;
                    else
                        s_state.current_layer = 1 + (int32_t)((s_state.z - flh) / s_state.layer_height + 0.5f);
                }
                if (s_state.progress_pct > 0.1f) {
                    float total_est = (float)s_state.print_time_s / (s_state.progress_pct / 100.0f);
                    s_state.print_time_left_s = (int32_t)(total_est - s_state.print_time_s);
                    if (s_state.print_time_left_s < 0) s_state.print_time_left_s = 0;
                }
                xSemaphoreGive(s_state_mutex);
            }
        }

        if (0) {
host_print_end:
            host_print_finish(true);
        }

        /* Macro: skip remaining polls if cooldown active (set by busy: handler) */
#define CHECK_COOLDOWN() \
        do { if (esp_timer_get_time() < s_cmd_cooldown_until_us) goto poll_done; } while(0)

        CHECK_COOLDOWN();

        /* Skip all polling when suppressed (terminal manual mode) */
        if (s_polling_suppressed) goto poll_done;

        /* During active host printing, only poll M105 at reduced frequency
         * (for Obico temp display). Also poll M119 for filament sensor.
         * Skip M27/M114 entirely — position is tracked from the GCode stream.
         *
         * CRITICAL: skip polling entirely when a retry is pending.
         * drain_rx() inside send_query would consume the late "ok" for the
         * timed-out command, desynchronising Marlin's line counter and
         * causing an unrecoverable retry loop. */
        if (s_host_printing && !s_host_paused) {
            if (s_host_has_pending) goto poll_done;
            now = esp_timer_get_time();
            if (now - s_last_m105_us >= POLL_M105_HOSTPRINT_INTERVAL_MS * 1000LL) {
                send_query("M105", QUERY_M105);
                s_last_m105_us = esp_timer_get_time();
                CHECK_COOLDOWN();
            }
            /* Poll M119 for filament runout detection (if enabled) */
            if (s_filament_check) {
                now = esp_timer_get_time();
                if (now - s_last_m119_us >= POLL_M119_INTERVAL_MS * 1000LL) {
                    send_query("M119", QUERY_M119);
                    s_last_m119_us = esp_timer_get_time();
                }
            }
            goto poll_done;
        }

        /* Query M290 once on connect to get current probe Z offset */
        if (!s_m290_queried) {
            s_m290_queried = true;
            send_query("M290", QUERY_CMD);
            CHECK_COOLDOWN();
        }

        /* Poll M105 (temperatures) every 4s */
        now = esp_timer_get_time();
        if (now - s_last_m105_us >= POLL_M105_INTERVAL_MS * 1000LL) {
            send_query("M105", QUERY_M105);
            s_last_m105_us = esp_timer_get_time();
            CHECK_COOLDOWN();
        }

        /* Poll M27 (SD progress) every 10s — skip during host print */
        if (!s_host_printing) {
            now = esp_timer_get_time();
            if (now - s_last_m27_us >= POLL_M27_INTERVAL_MS * 1000LL) {
                send_query("M27", QUERY_M27);
                s_last_m27_us = esp_timer_get_time();
                CHECK_COOLDOWN();
            }

            /* Fetch filename once when printing starts */
            if (s_need_filename) {
                s_need_filename = false;
                send_query("M27 C", QUERY_M27C);
                CHECK_COOLDOWN();
            }
        }

        /* Poll M114 (position) every 30s */
        now = esp_timer_get_time();
        if (now - s_last_m114_us >= POLL_M114_INTERVAL_MS * 1000LL) {
            send_query("M114", QUERY_M114);
            s_last_m114_us = esp_timer_get_time();
        }

poll_done:
#undef CHECK_COOLDOWN

        /* Shorter delay during host print for throughput, normal otherwise */
        vTaskDelay(pdMS_TO_TICKS(s_host_printing && !s_host_paused ? 20 : 100));
    }
}

/* ---- NVS config ---- */

static void load_config_from_nvs(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) {
        ESP_LOGI(TAG, "No pcomm NVS namespace, using defaults (Marlin)");
        return;
    }

    uint8_t backend_val = 0;
    if (nvs_get_u8(nvs, NVS_KEY_BACKEND, &backend_val) == ESP_OK) {
        s_backend = (backend_val == 1) ? PRINTER_BACKEND_KLIPPER : PRINTER_BACKEND_MARLIN;
    }

    size_t len = sizeof(s_mr_host);
    nvs_get_str(nvs, NVS_KEY_MR_HOST, s_mr_host, &len);

    nvs_get_u16(nvs, NVS_KEY_MR_PORT, &s_mr_port);
    if (s_mr_port == 0) s_mr_port = 7125;

    uint8_t pause_val = 0;
    if (nvs_get_u8(nvs, NVS_KEY_PAUSE_CMD, &pause_val) == ESP_OK) {
        s_pause_cmd = (pause_val == 1) ? PAUSE_CMD_M524 : PAUSE_CMD_M25;
    }

    uint8_t fil_chk = 0;
    if (nvs_get_u8(nvs, NVS_KEY_FILAMENT_CHK, &fil_chk) == ESP_OK) {
        s_filament_check = (fil_chk == 1);
    }

    size_t ping_len = sizeof(s_echo_ping_cmds);
    if (nvs_get_str(nvs, NVS_KEY_ECHO_PING, s_echo_ping_cmds, &ping_len) != ESP_OK) {
        s_echo_ping_cmds[0] = '\0';
    }

    nvs_close(nvs);

    /* Fall back to Marlin if Klipper selected but no host configured */
    if (s_backend == PRINTER_BACKEND_KLIPPER && s_mr_host[0] == '\0') {
        ESP_LOGW(TAG, "Klipper selected but no Moonraker host — falling back to Marlin");
        s_backend = PRINTER_BACKEND_MARLIN;
    }

    ESP_LOGI(TAG, "Config loaded: backend=%s, moonraker=%s:%u, pause=%s, filament_check=%s, echo_ping=[%s]",
             s_backend == PRINTER_BACKEND_KLIPPER ? "Klipper" : "Marlin",
             s_mr_host, s_mr_port,
             s_pause_cmd == PAUSE_CMD_M524 ? "M524 (abort)" : "M25 (pause)",
             s_filament_check ? "on" : "off",
             s_echo_ping_cmds);
}

/* ---- Public API ---- */

void printer_comm_rx_cb(const uint8_t *data, size_t len, void *user_ctx)
{
    if (s_backend != PRINTER_BACKEND_MARLIN) return;
    if (len > 0) {
        if (s_rx_stream) {
            xStreamBufferSend(s_rx_stream, data, len, 0);
        }
        terminal_feed_rx(data, len);
    }
}

void printer_comm_get_state(printer_state_t *out)
{
    if (s_simulate) {
        /* Compute elapsed print time (freezes when paused/cancelled) */
        int64_t elapsed_us;
        if (s_sim_opstate == PRINTER_PRINTING) {
            elapsed_us = s_sim_pause_elapsed_us + (esp_timer_get_time() - s_sim_start_us);
        } else {
            elapsed_us = s_sim_pause_elapsed_us;
        }
        int32_t elapsed_s = (int32_t)(elapsed_us / 1000000);
        float progress = (float)(elapsed_s % 600) / 6.0f; /* 0-100% over 10 min */

        memset(out, 0, sizeof(*out));
        out->opstate = s_sim_opstate;
        out->hotend_actual = 205.0f + (elapsed_s % 10) * 0.3f;
        out->hotend_target = (s_sim_opstate == PRINTER_OPERATIONAL) ? 0.0f : 210.0f;
        out->bed_actual = 59.5f + (elapsed_s % 5) * 0.2f;
        out->bed_target = (s_sim_opstate == PRINTER_OPERATIONAL) ? 0.0f : 60.0f;
        out->last_update_us = esp_timer_get_time();

        if (s_sim_opstate == PRINTER_PRINTING || s_sim_opstate == PRINTER_PAUSED) {
            out->progress_pct = progress;
            out->print_time_s = elapsed_s;
            out->print_time_left_s = (int32_t)((100.0f - progress) / 100.0f * 3600);
            out->x = 100.0f + (elapsed_s % 20) * 2.5f;
            out->y = 50.0f + (elapsed_s % 15) * 3.0f;
            out->z = 0.2f * (elapsed_s / 30);
            snprintf(out->filename, sizeof(out->filename), "simulated_benchy.gcode");
        } else {
            out->progress_pct = -1;
            out->print_time_s = -1;
            out->print_time_left_s = -1;
        }
        return;
    }

    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        klipper_backend_get_state(out);
        return;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    memcpy(out, &s_state, sizeof(printer_state_t));
    xSemaphoreGive(s_state_mutex);
}

int printer_comm_get_temp_history(temp_sample_t *buf, int max_count)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    int count = s_temp_count < max_count ? s_temp_count : max_count;
    /* Copy oldest-first: start from (head - count) wrapped */
    int start = (s_temp_head - s_temp_count + TEMP_HISTORY_MAX) % TEMP_HISTORY_MAX;
    /* Skip oldest samples if count < s_temp_count */
    start = (start + (s_temp_count - count)) % TEMP_HISTORY_MAX;
    for (int i = 0; i < count; i++) {
        buf[i] = s_temp_history[(start + i) % TEMP_HISTORY_MAX];
    }
    xSemaphoreGive(s_state_mutex);
    return count;
}

void printer_comm_set_simulate(bool enable)
{
    s_simulate = enable;
    if (enable) {
        s_sim_start_us = esp_timer_get_time();
        s_sim_pause_elapsed_us = 0;
        s_sim_opstate = PRINTER_PRINTING;
        ESP_LOGW(TAG, "Simulation mode ENABLED (printing)");
    } else {
        ESP_LOGI(TAG, "Simulation mode disabled");
    }
}

bool printer_comm_is_simulating(void)
{
    return s_simulate;
}

esp_err_t printer_comm_send_cmd(const printer_cmd_t *cmd)
{
    if (s_simulate) {
        switch (cmd->type) {
        case PCMD_PAUSE:
            if (s_sim_opstate == PRINTER_PRINTING) {
                /* Freeze elapsed time */
                s_sim_pause_elapsed_us += esp_timer_get_time() - s_sim_start_us;
                s_sim_opstate = PRINTER_PAUSED;
                ESP_LOGI(TAG, "[SIM] Paused");
            }
            break;
        case PCMD_RESUME:
            if (s_sim_opstate == PRINTER_PAUSED) {
                /* Resume timer from where we left off */
                s_sim_start_us = esp_timer_get_time();
                s_sim_opstate = PRINTER_PRINTING;
                ESP_LOGI(TAG, "[SIM] Resumed");
            }
            break;
        case PCMD_CANCEL:
            if (s_sim_opstate == PRINTER_PRINTING || s_sim_opstate == PRINTER_PAUSED) {
                s_sim_opstate = PRINTER_OPERATIONAL;
                s_sim_pause_elapsed_us = 0;
                ESP_LOGI(TAG, "[SIM] Cancelled");
            }
            break;
        case PCMD_RAW:
            ESP_LOGI(TAG, "[SIM] Raw gcode: %s", cmd->gcode);
            break;
        }
        return ESP_OK;
    }
    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        return klipper_backend_send_cmd(cmd);
    }

    if (xQueueSend(s_cmd_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t printer_comm_init(void)
{
    load_config_from_nvs();

    /* Temp history in PSRAM to save internal SRAM */
    s_temp_history = heap_caps_calloc(TEMP_HISTORY_MAX, sizeof(temp_sample_t),
                                      MALLOC_CAP_SPIRAM);
    if (!s_temp_history) return ESP_ERR_NO_MEM;

    s_state_mutex = xSemaphoreCreateMutex();
    if (!s_state_mutex) return ESP_ERR_NO_MEM;

    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        ESP_LOGI(TAG, "Starting Klipper/Moonraker backend → %s:%u", s_mr_host, s_mr_port);
        return klipper_backend_start(s_mr_host, s_mr_port);
    }

    /* Marlin backend */

    s_rx_stream = xStreamBufferCreate(2048, 1);
    if (!s_rx_stream) return ESP_ERR_NO_MEM;

    s_cmd_queue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(printer_cmd_t));
    if (!s_cmd_queue) return ESP_ERR_NO_MEM;

    /* Initialize state */
    memset(&s_state, 0, sizeof(s_state));
    s_state.opstate = PRINTER_DISCONNECTED;
    s_state.progress_pct = -1;
    s_state.print_time_s = -1;
    s_state.print_time_left_s = -1;
    s_state.total_layers = -1;

    xTaskCreatePinnedToCore(printer_comm_task, "printer_comm", 6144,
                            NULL, 8, NULL, 1);  /* Core 1: with USB host, isolated from WiFi/HTTP */

    ESP_LOGI(TAG, "Printer comm initialized (Marlin backend)");
    return ESP_OK;
}

/* ---- Host print public API ---- */

esp_err_t printer_comm_host_print_start(const char *filename)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_START };
    strncpy(cmd.gcode, filename, sizeof(cmd.gcode) - 1);
    cmd.gcode[sizeof(cmd.gcode) - 1] = '\0';
    if (xQueueSend(s_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t printer_comm_host_print_pause(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_PAUSE };
    if (xQueueSend(s_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t printer_comm_host_print_resume(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_RESUME };
    if (xQueueSend(s_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t printer_comm_host_print_cancel(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_CANCEL };
    if (xQueueSend(s_cmd_queue, &cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

bool printer_comm_is_host_printing(void)
{
    return s_host_printing;
}

void printer_comm_set_polling_suppressed(bool suppress)
{
    s_polling_suppressed = suppress;
}

bool printer_comm_is_polling_suppressed(void)
{
    return s_polling_suppressed;
}

/* ---- Backend getters ---- */

printer_backend_t printer_comm_get_backend(void)
{
    return s_backend;
}

const char *printer_comm_backend_name(void)
{
    return s_backend == PRINTER_BACKEND_KLIPPER ? "Klipper" : "Marlin";
}

const char *printer_comm_get_mr_host(void)
{
    return s_mr_host;
}

uint16_t printer_comm_get_mr_port(void)
{
    return s_mr_port;
}

/* ---- NVS save ---- */

esp_err_t printer_comm_save_config(printer_backend_t backend,
                                   const char *mr_host, uint16_t mr_port,
                                   bool pause_abort)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    nvs_set_u8(nvs, NVS_KEY_BACKEND, (uint8_t)backend);
    nvs_set_str(nvs, NVS_KEY_MR_HOST, mr_host ? mr_host : "");
    nvs_set_u16(nvs, NVS_KEY_MR_PORT, mr_port);
    nvs_set_u8(nvs, NVS_KEY_PAUSE_CMD, pause_abort ? 1 : 0);

    err = nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Config saved: backend=%s, moonraker=%s:%u, pause=%s",
             backend == PRINTER_BACKEND_KLIPPER ? "Klipper" : "Marlin",
             mr_host ? mr_host : "", mr_port,
             pause_abort ? "M524 (abort)" : "M25 (pause)");
    return err;
}

/* ---- Web UI HTTP handlers ---- */

void printer_config_render_backend(html_buf_t *p)
{
    const char *checked_marlin  = (s_backend == PRINTER_BACKEND_MARLIN)  ? "checked" : "";
    const char *checked_klipper = (s_backend == PRINTER_BACKEND_KLIPPER) ? "checked" : "";

    html_buf_printf(p,
        "<form id='f-printer' method='POST' action='/printer/config'>"
        "<div style='margin:10px 0'>"
        "<label style='display:block;margin:8px 0 4px'><input type='radio' name='backend' value='marlin' %s> Marlin (USB serial)</label>"
        "<label style='display:block;margin:8px 0 4px'><input type='radio' name='backend' value='klipper' %s> Klipper (Moonraker HTTP)</label>"
        "</div>"
        "<div style='margin-left:20px'>"
        "<label style='display:block;margin:8px 0 4px'>Moonraker Host/IP:<input type='text' name='mr_host' value='%s' maxlength='63' style='width:100%%;box-sizing:border-box;padding:6px'></label>"
        "<label style='display:block;margin:8px 0 4px'>Moonraker Port:<input type='number' name='mr_port' value='%u' min='1' max='65535' style='width:100%%;box-sizing:border-box;padding:6px'></label>"
        "</div>"
        "<p class='hint'><a href='/printer/config/test'>Test Moonraker connection</a></p>"
        "</form>",
        checked_marlin, checked_klipper, s_mr_host, s_mr_port);
}

void printer_config_render_marlin(html_buf_t *p)
{
    const char *checked_m25  = (s_pause_cmd == PAUSE_CMD_M25)  ? "checked" : "";
    const char *checked_m524 = (s_pause_cmd == PAUSE_CMD_M524) ? "checked" : "";
    const char *checked_fil  = s_filament_check ? "checked" : "";

    html_buf_printf(p,
        "<form id='f-marlin' method='POST' action='/printer/config/marlin'>"
        "<div style='margin:10px 0'>"
        "<label style='display:block;margin:8px 0 4px'><b>Pause Command</b></label>"
        "<label style='display:block;margin:8px 0 4px'><input type='radio' name='pause_cmd' value='m25' %s> M25 &mdash; Pause SD print (can resume)</label>"
        "<label style='display:block;margin:8px 0 4px'><input type='radio' name='pause_cmd' value='m524' %s> M524 &mdash; Abort print (if M25 crashes firmware)</label>"
        "</div>"
        "<div style='margin:10px 0'>"
        "<label style='display:block;margin:8px 0 4px'><b>Filament Sensor</b></label>"
        "<label style='display:block;margin:8px 0 4px'><input type='checkbox' name='filament_chk' value='1' %s> Poll filament sensor (M119) during host print &mdash; auto-pause on runout</label>"
        "</div>"
        "<div style='margin:10px 0'>"
        "<label style='display:block;margin:8px 0 4px'><b>CDC Echo Ping</b></label>"
        "<p style='margin:4px 0;font-size:13px;color:#666'>Some printers hold USB responses until the next command is sent. "
        "Adding commands here sends a harmless M118 ping after them to flush the response. "
        "Typical candidates: <b>M119, G92</b></p>"
        "<input type='text' name='echo_ping' value='%s' placeholder='e.g. M119, G92'"
        " style='width:100%%;padding:6px;font-family:monospace'>"
        "</div>"
        "</form>",
        checked_m25, checked_m524, checked_fil, s_echo_ping_cmds);
}

static esp_err_t printer_config_get_handler(httpd_req_t *req)
{
    /* Redirect to unified settings page */
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t printer_config_post_handler(httpd_req_t *req)
{
    char body[256];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    /* Parse form fields */
    printer_backend_t backend = PRINTER_BACKEND_MARLIN;
    char mr_host[64] = "";
    uint16_t mr_port = 7125;

    if (strstr(body, "backend=klipper")) {
        backend = PRINTER_BACKEND_KLIPPER;
    }

    /* Extract mr_host */
    char *hp = strstr(body, "mr_host=");
    if (hp) {
        hp += 8;
        char *end = strchr(hp, '&');
        size_t hlen = end ? (size_t)(end - hp) : strlen(hp);
        if (hlen >= sizeof(mr_host)) hlen = sizeof(mr_host) - 1;
        memcpy(mr_host, hp, hlen);
        mr_host[hlen] = '\0';
    }

    /* Extract mr_port */
    char *pp = strstr(body, "mr_port=");
    if (pp) {
        mr_port = (uint16_t)atoi(pp + 8);
        if (mr_port == 0) mr_port = 7125;
    }

    bool pause_abort = (s_pause_cmd == PAUSE_CMD_M524);  /* preserve current */

    esp_err_t err = printer_comm_save_config(backend, mr_host, mr_port, pause_abort);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS write failed");
        return ESP_FAIL;
    }

    html_buf_t pg;
    html_buf_init(&pg);
    layout_html_begin(&pg, "Saved", "/settings");
    html_buf_printf(&pg,
        "<h2>Configuration saved!</h2>"
        "<p>Rebooting in 3 seconds...</p>"
        "<script>setTimeout(function(){location.href='/settings'},5000)</script>");
    layout_html_end(&pg);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, pg.data, pg.len);
    html_buf_free(&pg);

    /* Reboot after a short delay so the response can be sent */
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();

    return ESP_OK;
}

static esp_err_t printer_config_marlin_post_handler(httpd_req_t *req)
{
    char body[256];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    pause_cmd_t new_cmd = (strstr(body, "pause_cmd=m524") != NULL)
                           ? PAUSE_CMD_M524 : PAUSE_CMD_M25;
    s_pause_cmd = new_cmd;

    bool new_fil_chk = (strstr(body, "filament_chk=1") != NULL);
    s_filament_check = new_fil_chk;

    /* Parse echo_ping field */
    char new_ping[128] = "";
    char *ep = strstr(body, "echo_ping=");
    if (ep) {
        url_decode_field(ep + 10, new_ping, sizeof(new_ping));
        /* Trim whitespace */
        char *start = new_ping;
        while (*start == ' ') start++;
        if (start != new_ping) memmove(new_ping, start, strlen(start) + 1);
        size_t plen = strlen(new_ping);
        while (plen > 0 && new_ping[plen-1] == ' ') new_ping[--plen] = '\0';
    }
    strncpy(s_echo_ping_cmds, new_ping, sizeof(s_echo_ping_cmds));
    s_echo_ping_cmds[sizeof(s_echo_ping_cmds) - 1] = '\0';

    /* Persist to NVS */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u8(nvs, NVS_KEY_PAUSE_CMD, (uint8_t)new_cmd);
        nvs_set_u8(nvs, NVS_KEY_FILAMENT_CHK, new_fil_chk ? 1 : 0);
        nvs_set_str(nvs, NVS_KEY_ECHO_PING, s_echo_ping_cmds);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t printer_config_test_handler(httpd_req_t *req)
{
    char url[128];
    snprintf(url, sizeof(url), "http://%s:%u/printer/info", s_mr_host, s_mr_port);

    /* Quick connectivity test */
    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_GET,
        .timeout_ms = 5000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        httpd_resp_set_type(req, "application/json");
        const char *fail = "{\"ok\":false,\"error\":\"client init failed\"}";
        return httpd_resp_send(req, fail, strlen(fail));
    }

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    char resp[128];
    if (err == ESP_OK && status == 200) {
        snprintf(resp, sizeof(resp),
                 "{\"ok\":true,\"host\":\"%s\",\"port\":%u}", s_mr_host, s_mr_port);
    } else {
        snprintf(resp, sizeof(resp),
                 "{\"ok\":false,\"error\":\"HTTP %d, err=%s\"}",
                 status, esp_err_to_name(err));
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, resp, strlen(resp));
}

esp_err_t printer_config_register_httpd(void *server_handle)
{
    httpd_handle_t server = (httpd_handle_t)server_handle;

    httpd_uri_t get_cfg = {
        .uri     = "/printer/config",
        .method  = HTTP_GET,
        .handler = printer_config_get_handler,
    };
    httpd_register_uri_handler(server, &get_cfg);

    httpd_uri_t post_cfg = {
        .uri     = "/printer/config",
        .method  = HTTP_POST,
        .handler = printer_config_post_handler,
    };
    httpd_register_uri_handler(server, &post_cfg);

    httpd_uri_t marlin_cfg = {
        .uri     = "/printer/config/marlin",
        .method  = HTTP_POST,
        .handler = printer_config_marlin_post_handler,
    };
    httpd_register_uri_handler(server, &marlin_cfg);

    httpd_uri_t test_cfg = {
        .uri     = "/printer/config/test",
        .method  = HTTP_GET,
        .handler = printer_config_test_handler,
    };
    httpd_register_uri_handler(server, &test_cfg);

    return ESP_OK;
}
