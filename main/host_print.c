#include "marlin_proto.h"
#include "gcode_scan.h"

#include "esp_log.h"
#include "esp_timer.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "host_print";

/* ---- Constants ---- */

#define HOST_RESEND_MAX_RETRIES 5
#define HOST_LINE_HISTORY_SIZE  5

/* Park position and retract settings */
#define PARK_X       0.0f
#define PARK_Y       0.0f
#define PARK_Z_LIFT  5.0f    /* mm to lift Z on pause */
#define PARK_RETRACT 2.0f    /* mm filament retract on pause */

/* ---- Host print state ---- */

typedef struct {
    /* State */
    bool printing, paused;

    /* File I/O */
    FILE *file;
    char filename[64];
    int32_t lines_sent, total_lines;

    /* Pending line (deferred on busy) */
    char pending_line[256];
    bool has_pending;
    bool line_truncated;

    /* Layer tracking */
    int32_t layer;
    bool has_layer_comments;

    /* Timing */
    int64_t start_us, pause_elapsed_us;

    /* Marlin line numbering & checksum */
    int32_t marlin_line;        /* Next Marlin line number to assign */
    int32_t resend_line;        /* Line requested by Resend:, or -1 */
    char numbered_line[160];    /* Formatted "Nxxx gcode*cs" for resend */
    int resend_retries;

    /* Rolling history of recently sent numbered lines */
    char line_history[HOST_LINE_HISTORY_SIZE][160];
    int32_t line_history_num[HOST_LINE_HISTORY_SIZE];
    int line_history_idx;

    /* Park position */
    float park_x, park_y, park_z;
} hp_ctx_t;

static hp_ctx_t s_hp;

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

/** Look up a previously sent numbered line from the rolling history. */
static const char *history_lookup(int32_t line_num)
{
    for (int i = 0; i < HOST_LINE_HISTORY_SIZE; i++) {
        if (s_hp.line_history_num[i] == line_num && s_hp.line_history[i][0])
            return s_hp.line_history[i];
    }
    return NULL;
}

/** Store a numbered line in the rolling history. */
static void history_store(int32_t line_num, const char *formatted)
{
    int idx = s_hp.line_history_idx;
    strncpy(s_hp.line_history[idx], formatted, sizeof(s_hp.line_history[idx]));
    s_hp.line_history[idx][sizeof(s_hp.line_history[idx]) - 1] = '\0';
    s_hp.line_history_num[idx] = line_num;
    s_hp.line_history_idx = (idx + 1) % HOST_LINE_HISTORY_SIZE;
}

/** Check if a raw GCode command is long-running (G28, G29, M109, M190, M116, G4).
 *  These need standard (long) timeouts, not the short host-print timeout. */
static bool is_long_running_cmd(const char *gcode)
{
    const char *p = gcode;
    if (*p == 'G' || *p == 'g') {
        int code = atoi(p + 1);
        return (code == 4 || code == 28 || code == 29);
    }
    if (*p == 'M' || *p == 'm') {
        int code = atoi(p + 1);
        return (code == 109 || code == 190 || code == 116);
    }
    return false;
}

/** Send a numbered GCode command with checksum, handle Resend: retries.
 *  Long-running commands (G28, G29, M109, etc.) use standard timeout;
 *  normal GCode lines use the short host-print timeout with stall pings. */
static bool send_query_numbered(const char *gcode)
{
    /* Pick sender: long-running commands need standard timeout (auto-detected),
     * normal lines use short HP timeout with CDC stall ping retries. */
    bool (*sender)(const char *, query_type_t) =
        is_long_running_cmd(gcode) ? marlin_send_cmd : marlin_send_cmd_hp;

    s_hp.resend_line = -1;
    s_hp.resend_retries = 0;

    if (format_numbered_line(s_hp.numbered_line, sizeof(s_hp.numbered_line),
                             s_hp.marlin_line, gcode) < 0) {
        ESP_LOGE(TAG, "Line too long for numbering: %s", gcode);
        return sender(gcode, QUERY_CMD); /* fallback unnumbered */
    }

    history_store(s_hp.marlin_line, s_hp.numbered_line);

    if (!sender(s_hp.numbered_line, QUERY_CMD)) {
        return false;
    }

    /* Handle resend requests */
    while (s_hp.resend_line >= 0) {
        if (s_hp.resend_line == s_hp.marlin_line + 1) {
            /* Printer already executed our line and wants the next one.
             * This means our command succeeded -- just advance the counter. */
            ESP_LOGW(TAG, "Printer already has N%ld, wants N%ld -- command accepted",
                     (long)s_hp.marlin_line, (long)s_hp.resend_line);
            s_hp.resend_line = -1;
            s_hp.marlin_line++;
            return true;
        }
        if (s_hp.resend_line != s_hp.marlin_line) {
            /* Marlin wants a different line than the one we just sent.
             * Look it up in history and replay from there. */
            const char *old = history_lookup(s_hp.resend_line);
            if (old) {
                int32_t replay_from = s_hp.resend_line;
                ESP_LOGW(TAG, "Replaying from N%ld to N%ld (history hit)",
                         (long)replay_from, (long)s_hp.marlin_line);
                s_hp.resend_line = -1;
                /* Resend all lines from the requested one up to current */
                for (int32_t n = replay_from; ; n++) {
                    const char *line_data;
                    if (n == s_hp.marlin_line) {
                        line_data = s_hp.numbered_line;
                    } else {
                        line_data = history_lookup(n);
                        if (!line_data) break; /* shouldn't happen */
                    }
                    if (!sender(line_data, QUERY_CMD)) {
                        return false;
                    }
                    if (n == s_hp.marlin_line) break;
                }
                continue; /* check for further resend requests */
            }
            /* Not in history -- resync as last resort */
            ESP_LOGW(TAG, "Resend N%ld not in history -- resync + retry",
                     (long)s_hp.resend_line);
            char sync[40];
            format_numbered_line(sync, sizeof(sync), s_hp.marlin_line, "M110");
            sender(sync, QUERY_CMD);
            s_hp.resend_line = -1;
            if (!sender(s_hp.numbered_line, QUERY_CMD)) {
                return false;
            }
            break;
        }

        s_hp.resend_retries++;
        if (s_hp.resend_retries > HOST_RESEND_MAX_RETRIES) {
            ESP_LOGE(TAG, "Resend retries exhausted for N%ld", (long)s_hp.marlin_line);
            s_hp.resend_line = -1;
            return false;
        }

        ESP_LOGW(TAG, "Resending N%ld (attempt %d)", (long)s_hp.marlin_line, s_hp.resend_retries);
        s_hp.resend_line = -1;
        if (!sender(s_hp.numbered_line, QUERY_CMD)) {
            return false;
        }
    }

    s_hp.marlin_line++;
    return true;
}

/* ---- Host print lifecycle ---- */

/** Query current position and save it, then park the head */
static void host_print_park(void)
{
    /* Get fresh position */
    marlin_send_cmd("M114", QUERY_M114);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    s_hp.park_x = g_state.x;
    s_hp.park_y = g_state.y;
    s_hp.park_z = g_state.z;
    xSemaphoreGive(g_state_mutex);

    ESP_LOGI(TAG, "Parking head from (%.1f, %.1f, %.1f)", s_hp.park_x, s_hp.park_y, s_hp.park_z);

    char buf[64];
    marlin_send_cmd("M400", QUERY_CMD);           /* Wait for moves to finish */
    marlin_send_cmd("G91", QUERY_CMD);            /* Relative positioning */
    snprintf(buf, sizeof(buf), "G1 E-%.1f F2400", PARK_RETRACT);
    marlin_send_cmd(buf, QUERY_CMD);              /* Retract filament */
    snprintf(buf, sizeof(buf), "G1 Z%.1f F600", PARK_Z_LIFT);
    marlin_send_cmd(buf, QUERY_CMD);              /* Lift Z */
    marlin_send_cmd("G90", QUERY_CMD);            /* Absolute positioning */
    snprintf(buf, sizeof(buf), "G1 X%.1f Y%.1f F6000", PARK_X, PARK_Y);
    marlin_send_cmd(buf, QUERY_CMD);              /* Park at corner */
}

/** Restore head position after park */
static void host_print_unpark(void)
{
    ESP_LOGI(TAG, "Unparking head to (%.1f, %.1f, %.1f)", s_hp.park_x, s_hp.park_y, s_hp.park_z);

    char buf[64];
    marlin_send_cmd("G90", QUERY_CMD);            /* Absolute positioning */
    snprintf(buf, sizeof(buf), "G1 X%.1f Y%.1f F6000", s_hp.park_x, s_hp.park_y);
    marlin_send_cmd(buf, QUERY_CMD);              /* Move to saved XY */
    snprintf(buf, sizeof(buf), "G1 Z%.1f F600", s_hp.park_z);
    marlin_send_cmd(buf, QUERY_CMD);              /* Lower Z back */
    marlin_send_cmd("G91", QUERY_CMD);            /* Relative positioning */
    snprintf(buf, sizeof(buf), "G1 E%.1f F2400", PARK_RETRACT);
    marlin_send_cmd(buf, QUERY_CMD);              /* Prime filament */
    marlin_send_cmd("G90", QUERY_CMD);            /* Absolute positioning */
    marlin_send_cmd("M83", QUERY_CMD);            /* Extruder relative (most slicers use this) */
}

/** Finish host print (complete or cancel) */
static void host_print_finish(bool cancelled)
{
    if (s_hp.file) {
        fclose(s_hp.file);
        s_hp.file = NULL;
    }

    bool connected = usb_serial_is_connected();

    if (connected) {
        /* Reset Marlin's line counter so it stops expecting numbered lines.
         * Must be sent as a numbered command with the expected line number. */
        char reset[40];
        format_numbered_line(reset, sizeof(reset), s_hp.marlin_line, "M110 N0");
        marlin_send_cmd(reset, QUERY_CMD);

        /* Stop Marlin print timer */
        marlin_send_cmd("M77", QUERY_CMD);
        marlin_send_cmd("M117", QUERY_CMD);  /* Clear LCD message */

        /* Disable host keepalive -- restore default (disabled) to reduce chatter */
        marlin_send_cmd("M113 S0", QUERY_CMD);
    }
    s_hp.resend_line = -1;

    if (cancelled) {
        if (connected) {
            ESP_LOGI(TAG, "Host print cancelled -- parking and turning off heaters");
            marlin_send_cmd("M400", QUERY_CMD);       /* Wait for planner to drain */
            marlin_send_cmd("G91", QUERY_CMD);         /* Relative positioning */
            marlin_send_cmd("G1 Z10 F600", QUERY_CMD); /* Raise nozzle 10mm */
            marlin_send_cmd("G90", QUERY_CMD);         /* Absolute positioning */
            marlin_send_cmd("G1 X0 Y200 F3000", QUERY_CMD); /* Park head at rear-left */
            marlin_send_cmd("M104 S0", QUERY_CMD);     /* Hotend off */
            marlin_send_cmd("M140 S0", QUERY_CMD);     /* Bed off */
            marlin_send_cmd("M107", QUERY_CMD);         /* Fan off */
            marlin_send_cmd("M84", QUERY_CMD);          /* Disable steppers */
        } else {
            ESP_LOGW(TAG, "Host print cancelled -- USB disconnected, skipping park/cooldown");
        }
    } else {
        int64_t elapsed_us = s_hp.pause_elapsed_us + (esp_timer_get_time() - s_hp.start_us);
        ESP_LOGI(TAG, "Host print complete: %s (%ld lines, %lds)",
                 s_hp.filename, (long)s_hp.lines_sent,
                 (long)(elapsed_us / 1000000));
        if (connected) {
            marlin_send_cmd("M117 Print complete", QUERY_CMD);
        }
    }

    s_hp.printing = false;
    s_hp.paused = false;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = connected ? PRINTER_OPERATIONAL : PRINTER_DISCONNECTED;
    g_state.progress_pct = -1;
    g_state.print_time_s = -1;
    g_state.print_time_left_s = -1;
    g_state.host_printing = false;
    g_state.current_layer = 0;
    g_state.filename[0] = '\0';
    xSemaphoreGive(g_state_mutex);
}

/** Start host print from within the task context */
void hp_cmd_start(const char *filename)
{
    if (s_hp.printing) {
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
    gcode_scan_file(f, &scan);

    ESP_LOGI(TAG, "Host print starting: %s (%ld lines)", filename, (long)scan.total_lines);

    s_hp.file = f;
    s_hp.printing = true;
    s_hp.paused = false;
    s_hp.lines_sent = 0;
    s_hp.total_lines = scan.total_lines;
    s_hp.layer = 0;
    s_hp.has_layer_comments = false;
    s_hp.start_us = esp_timer_get_time();
    s_hp.pause_elapsed_us = 0;
    s_hp.has_pending = false;
    s_hp.line_truncated = false;
    strncpy(s_hp.filename, filename, sizeof(s_hp.filename) - 1);
    s_hp.filename[sizeof(s_hp.filename) - 1] = '\0';

    /* Reset Marlin line numbering for reliable transmission */
    s_hp.marlin_line = 0;
    s_hp.resend_line = -1;
    memset(s_hp.line_history, 0, sizeof(s_hp.line_history));
    s_hp.line_history_idx = 0;
    char sync[40];
    format_numbered_line(sync, sizeof(sync), 0, "M110 N0");
    marlin_send_cmd(sync, QUERY_CMD);
    s_hp.marlin_line = 1;

    /* Enable host keepalive -- Marlin sends "busy: processing" every 2s
     * during long-running commands (G28, G29, etc.) so we know it's alive. */
    marlin_send_cmd("M113 S2", QUERY_CMD);

    /* Tell Marlin we're printing -- starts LCD print timer & enables host mode */
    marlin_send_cmd("M75", QUERY_CMD);

    /* Show filename on LCD */
    char lcd_msg[128];
    snprintf(lcd_msg, sizeof(lcd_msg), "M117 %s", filename);
    marlin_send_cmd(lcd_msg, QUERY_CMD);

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = PRINTER_PRINTING;
    g_state.progress_pct = 0;
    g_state.print_time_s = 0;
    g_state.print_time_left_s = -1;
    g_state.host_printing = true;
    g_state.current_layer = 0;
    g_state.object_height = scan.max_z;
    g_state.total_layers = scan.total_layers;
    g_state.layer_height = scan.layer_height;
    g_state.first_layer_height = scan.first_layer_height;
    strncpy(g_state.filename, filename, sizeof(g_state.filename) - 1);
    g_state.filename[sizeof(g_state.filename) - 1] = '\0';
    xSemaphoreGive(g_state_mutex);
}

/** Pause host print: park head, stop timer */
void hp_cmd_pause(void)
{
    if (!s_hp.printing || s_hp.paused) return;
    s_hp.paused = true;
    s_hp.pause_elapsed_us += esp_timer_get_time() - s_hp.start_us;
    host_print_park();
    marlin_send_cmd("M76", QUERY_CMD);  /* Pause Marlin print timer */
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = PRINTER_PAUSED;
    xSemaphoreGive(g_state_mutex);
    ESP_LOGI(TAG, "Host print paused (head parked)");
}

/** Resume host print: restore position, restart timer */
void hp_cmd_resume(void)
{
    if (!s_hp.printing || !s_hp.paused) return;
    marlin_send_cmd("M75", QUERY_CMD);  /* Resume Marlin print timer */
    host_print_unpark();
    s_hp.paused = false;
    s_hp.start_us = esp_timer_get_time();
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    g_state.opstate = PRINTER_PRINTING;
    xSemaphoreGive(g_state_mutex);
    ESP_LOGI(TAG, "Host print resumed (head restored)");
}

/** Cancel host print */
void hp_cmd_cancel(void)
{
    if (s_hp.printing) host_print_finish(true);
}

/* ---- Host print streaming loop ---- */

void hp_tick(void)
{
    if (!s_hp.printing || s_hp.paused) return;

    /* Explicit USB disconnect handling: don't advance file position,
     * don't send commands.  Streaming resumes when USB reconnects. */
    if (!marlin_is_connected()) return;

    int lines_this_batch = 0;
    char line[256];

    while (lines_this_batch < 4 && s_hp.file) {
        if (esp_timer_get_time() < s_cmd_cooldown_until_us) break;

        /* Use pending line from previous failed send, or read new */
        if (s_hp.has_pending) {
            strncpy(line, s_hp.pending_line, sizeof(line));
            s_hp.has_pending = false;
        } else {
            if (!fgets(line, sizeof(line), s_hp.file)) {
                /* EOF -- print complete */
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

            if (s_hp.line_truncated) {
                /* Still consuming a split long line -- skip until newline */
                s_hp.line_truncated = !has_newline;
                s_hp.lines_sent++;
                continue;
            }
            if (!has_newline && ln > 0) {
                /* First chunk of a long line -- skip it and mark truncated */
                s_hp.line_truncated = true;
                /* Still count and check if it was a comment */
                if (line[0] != ';') {
                    ESP_LOGW(TAG, "Skipping long line (%d chars): %.40s...", (int)ln, line);
                }
                s_hp.lines_sent++;
                continue;
            }

            /* Parse layer comments for tracking */
            if (strncmp(line, ";LAYER:", 7) == 0) {
                s_hp.layer = atoi(line + 7);
                s_hp.has_layer_comments = true;
            } else if (strncmp(line, ";LAYER_CHANGE", 13) == 0) {
                /* First LAYER_CHANGE = layer 0 (0-based, like ;LAYER:) */
                if (s_hp.has_layer_comments)
                    s_hp.layer++;
                s_hp.has_layer_comments = true;
            }

            /* Skip empty lines and comments */
            if (ln == 0 || line[0] == ';') {
                s_hp.lines_sent++;
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
                s_hp.lines_sent++;
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
                s_hp.lines_sent++;
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
                        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                        g_state.progress_pct = (float)pct;
                        if (mins >= 0)
                            g_state.print_time_left_s = mins * 60;
                        xSemaphoreGive(g_state_mutex);
                    }
                }
                s_hp.lines_sent++;
                continue;
            }
        }

        /* Track Z from G0/G1 moves for real-time position */
        if ((line[0] == 'G') && (line[1] == '0' || line[1] == '1') && line[2] == ' ') {
            const char *zp = strstr(line, "Z");
            if (zp && (zp == line + 3 || *(zp - 1) == ' ')) {
                float z = strtof(zp + 1, NULL);
                if (z > 0.01f) {
                    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                    g_state.z = z;
                    xSemaphoreGive(g_state_mutex);
                }
            }
        }

        /* Send to printer with line numbering & checksum. */
        if (!send_query_numbered(line)) {
            if (esp_timer_get_time() < s_cmd_cooldown_until_us) {
                /* Printer said "busy: processing" -- defer */
                strncpy(s_hp.pending_line, line, sizeof(s_hp.pending_line));
                s_hp.has_pending = true;
                break;
            }
            /* send_query already retried with echo pings.  If we still
             * got no response, something is genuinely wrong -- abort to
             * avoid losing sync with Marlin's line counter. */
            ESP_LOGE(TAG, "Command failed after retries, aborting host print: %s", line);
            host_print_finish(true);
            break;
        }
        s_hp.lines_sent++;
        lines_this_batch++;

        /* Update progress */
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        if (s_hp.total_lines > 0) {
            g_state.progress_pct = (float)s_hp.lines_sent / (float)s_hp.total_lines * 100.0f;
        }
        int64_t elapsed_us = s_hp.pause_elapsed_us + (esp_timer_get_time() - s_hp.start_us);
        g_state.print_time_s = (int32_t)(elapsed_us / 1000000);
        g_state.current_layer = s_hp.layer + 1;  /* ;LAYER: is 0-based, display 1-based */
        /* If no ;LAYER: comments in file, estimate from Z */
        if (!s_hp.has_layer_comments && g_state.z > 0 && g_state.layer_height > 0) {
            float flh = g_state.first_layer_height > 0 ? g_state.first_layer_height : g_state.layer_height;
            if (g_state.z <= flh)
                g_state.current_layer = 1;
            else
                g_state.current_layer = 1 + (int32_t)((g_state.z - flh) / g_state.layer_height + 0.5f);
        }
        if (g_state.progress_pct > 0.1f) {
            float total_est = (float)g_state.print_time_s / (g_state.progress_pct / 100.0f);
            g_state.print_time_left_s = (int32_t)(total_est - g_state.print_time_s);
            if (g_state.print_time_left_s < 0) g_state.print_time_left_s = 0;
        }
        xSemaphoreGive(g_state_mutex);
    }
}

/* ---- Query functions ---- */

void hp_init(void)
{
    /* Struct is zero-initialized -- nothing to do */
}

bool hp_is_active(void)
{
    return s_hp.printing;
}

bool hp_has_pending(void)
{
    return s_hp.has_pending;
}

bool hp_is_paused(void)
{
    return s_hp.paused;
}

/* ---- Notifications from RX parsing ---- */

void hp_notify_resend(int32_t line_num)
{
    s_hp.resend_line = line_num;
}

/* ---- Fill state (no-op for now -- progress updated inline in hp_tick) ---- */

void hp_fill_state(printer_state_t *out)
{
    (void)out;
}

/* ---- Public API pass-throughs ---- */

esp_err_t hp_start(const char *filename)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_START };
    strncpy(cmd.gcode, filename, sizeof(cmd.gcode) - 1);
    cmd.gcode[sizeof(cmd.gcode) - 1] = '\0';
    return printer_comm_send_cmd(&cmd);
}

esp_err_t hp_pause(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_PAUSE };
    return printer_comm_send_cmd(&cmd);
}

esp_err_t hp_resume(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_RESUME };
    return printer_comm_send_cmd(&cmd);
}

esp_err_t hp_cancel(void)
{
    printer_cmd_t cmd = { .type = PCMD_HOST_CANCEL };
    return printer_comm_send_cmd(&cmd);
}
