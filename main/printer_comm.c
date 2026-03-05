#include "printer_comm.h"
#if CONFIG_OBICO_ENABLED
#include "printer_comm_klipper.h"
#endif

#include "usb_serial.h"

#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "nvs.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "printer_comm";

/* ---- Configuration ---- */

#define POLL_M105_INTERVAL_MS   4000
#define POLL_M27_INTERVAL_MS    10000
#define POLL_M114_INTERVAL_MS   30000
#define CMD_RESPONSE_TIMEOUT_MS 6000
#define RX_LINE_BUF_SIZE        256
#define CMD_QUEUE_SIZE          8

#define NVS_NAMESPACE           "pcomm"
#define NVS_KEY_BACKEND         "backend"
#define NVS_KEY_MR_HOST         "mr_host"
#define NVS_KEY_MR_PORT         "mr_port"
#define NVS_KEY_PAUSE_CMD       "pause_cmd"

/* ---- Backend config ---- */

typedef enum {
    PAUSE_CMD_M25  = 0,  /* Pause SD print (can resume) */
    PAUSE_CMD_M524 = 1,  /* Abort SD print (workaround for buggy firmware) */
} pause_cmd_t;

static printer_backend_t s_backend = PRINTER_BACKEND_MARLIN;
static char s_mr_host[64] = "";
static uint16_t s_mr_port = 7125;
static pause_cmd_t s_pause_cmd = PAUSE_CMD_M25;

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
    QUERY_CMD,
} query_type_t;

static query_type_t s_pending_query;

/* Timestamps for polling intervals */
static int64_t s_last_m105_us;
static int64_t s_last_m27_us;
static int64_t s_last_m114_us;

/* Print start time tracking */
static int64_t s_print_start_us;
static bool s_need_filename;
static bool s_have_m73;        /* true once we've seen M73 — prefer slicer progress over M27 bytes */

/* Cooldown after pause/resume/cancel — let firmware settle */
static int64_t s_cmd_cooldown_until_us;

/* ---- Host print state ---- */
static FILE *s_host_file;
static bool s_host_printing, s_host_paused;
static int32_t s_host_lines_sent, s_host_total_lines, s_host_layer;
static int64_t s_host_start_us, s_host_pause_elapsed_us;
static char s_host_filename[64];
static char s_host_pending_line[128]; /* line read but not yet sent */
static bool s_host_has_pending;

/* Internal command types for host print (sent via s_cmd_queue) */
#define PCMD_HOST_START  100
#define PCMD_HOST_PAUSE  101
#define PCMD_HOST_RESUME 102
#define PCMD_HOST_CANCEL 103

/* ---- Helpers ---- */

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

    /* Marlin says "busy: processing" when it can't accept commands yet */
    if (strstr(line, "busy:")) {
        s_cmd_cooldown_until_us = esp_timer_get_time() + 4000000LL; /* back off 4s */
        s_pending_query = QUERY_NONE;
        return;
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

/** Send a gcode command to the printer, wait for response lines */
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
    esp_err_t err = usb_serial_send((const uint8_t *)buf, len);
    if (err != ESP_OK) {
        s_pending_query = QUERY_NONE;
        return false;
    }

    /* Wait for "ok" or timeout, processing lines as they arrive */
    int64_t deadline = esp_timer_get_time() + CMD_RESPONSE_TIMEOUT_MS * 1000LL;
    uint8_t rx_byte;

    while (s_pending_query != QUERY_NONE) {
        int64_t remaining_us = deadline - esp_timer_get_time();
        if (remaining_us <= 0) {
            ESP_LOGW(TAG, "Timeout waiting for response to %s", gcode);
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
            }
        } else if (s_line_len < RX_LINE_BUF_SIZE - 1) {
            s_line_buf[s_line_len++] = (char)rx_byte;
        }
    }

    return true;
}

/* ---- Host print helpers ---- */

/** Count total lines in file (for progress), then rewind */
static int32_t count_lines(FILE *f)
{
    int32_t count = 0;
    char buf[128];
    rewind(f);
    while (fgets(buf, sizeof(buf), f)) count++;
    rewind(f);
    return count;
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

    int32_t total = count_lines(f);
    ESP_LOGI(TAG, "Host print starting: %s (%ld lines)", filename, (long)total);

    s_host_file = f;
    s_host_printing = true;
    s_host_paused = false;
    s_host_lines_sent = 0;
    s_host_total_lines = total;
    s_host_layer = 0;
    s_host_start_us = esp_timer_get_time();
    s_host_pause_elapsed_us = 0;
    s_host_has_pending = false;
    strncpy(s_host_filename, filename, sizeof(s_host_filename) - 1);
    s_host_filename[sizeof(s_host_filename) - 1] = '\0';

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

    /* Stop Marlin print timer */
    send_query("M77", QUERY_CMD);
    send_query("M117", QUERY_CMD);  /* Clear LCD message */

    if (cancelled) {
        ESP_LOGI(TAG, "Host print cancelled — turning off heaters");
        send_query("M400", QUERY_CMD);
        send_query("M104 S0", QUERY_CMD);
        send_query("M140 S0", QUERY_CMD);
        send_query("M84", QUERY_CMD);
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
            s_state.opstate = PRINTER_DISCONNECTED;
            xSemaphoreGive(s_state_mutex);
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
                if (s_host_printing && !s_host_paused) {
                    s_host_paused = true;
                    s_host_pause_elapsed_us += esp_timer_get_time() - s_host_start_us;
                    send_query("M76", QUERY_CMD);  /* Pause Marlin print timer */
                    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                    s_state.opstate = PRINTER_PAUSED;
                    xSemaphoreGive(s_state_mutex);
                    ESP_LOGI(TAG, "Host print paused");
                }
                continue;
            } else if (cmd.type == PCMD_HOST_RESUME) {
                if (s_host_printing && s_host_paused) {
                    s_host_paused = false;
                    s_host_start_us = esp_timer_get_time();
                    send_query("M75", QUERY_CMD);  /* Resume Marlin print timer */
                    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                    s_state.opstate = PRINTER_PRINTING;
                    xSemaphoreGive(s_state_mutex);
                    ESP_LOGI(TAG, "Host print resumed");
                }
                continue;
            } else if (cmd.type == PCMD_HOST_CANCEL) {
                if (s_host_printing) {
                    host_print_finish(true);
                }
                continue;
            }

            /* Route pause/resume/cancel to host print if active */
            if (s_host_printing) {
                if (cmd.type == PCMD_PAUSE) {
                    if (!s_host_paused) {
                        s_host_paused = true;
                        s_host_pause_elapsed_us += esp_timer_get_time() - s_host_start_us;
                        send_query("M76", QUERY_CMD);
                        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                        s_state.opstate = PRINTER_PAUSED;
                        xSemaphoreGive(s_state_mutex);
                        ESP_LOGI(TAG, "Host print paused");
                    }
                    continue;
                } else if (cmd.type == PCMD_RESUME) {
                    if (s_host_paused) {
                        s_host_paused = false;
                        s_host_start_us = esp_timer_get_time();
                        send_query("M75", QUERY_CMD);
                        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
                        s_state.opstate = PRINTER_PRINTING;
                        xSemaphoreGive(s_state_mutex);
                        ESP_LOGI(TAG, "Host print resumed");
                    }
                    continue;
                } else if (cmd.type == PCMD_CANCEL) {
                    host_print_finish(true);
                    continue;
                }
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
            char line[128];

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

                    /* Strip newline */
                    size_t ln = strlen(line);
                    while (ln > 0 && (line[ln-1] == '\n' || line[ln-1] == '\r'))
                        line[--ln] = '\0';

                    /* Parse ;LAYER:N comments for layer tracking */
                    if (strncmp(line, ";LAYER:", 7) == 0) {
                        s_host_layer = atoi(line + 7);
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
                }

                /* Send to printer — if it fails (cooldown/timeout), save for retry */
                if (!send_query(line, QUERY_CMD)) {
                    strncpy(s_host_pending_line, line, sizeof(s_host_pending_line));
                    s_host_has_pending = true;
                    break;
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
                s_state.current_layer = s_host_layer;
                if (s_state.progress_pct > 0.1f) {
                    float total_est = (float)s_state.print_time_s / (s_state.progress_pct / 100.0f);
                    s_state.print_time_left_s = (int32_t)(total_est - s_state.print_time_s);
                    if (s_state.print_time_left_s < 0) s_state.print_time_left_s = 0;
                }
                xSemaphoreGive(s_state_mutex);
            }
        }

        /* Macro: skip remaining polls if cooldown active (set by busy: handler) */
#define CHECK_COOLDOWN() \
        do { if (esp_timer_get_time() < s_cmd_cooldown_until_us) goto poll_done; } while(0)

        CHECK_COOLDOWN();

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

    nvs_close(nvs);

    /* Fall back to Marlin if Klipper selected but no host configured */
    if (s_backend == PRINTER_BACKEND_KLIPPER && s_mr_host[0] == '\0') {
        ESP_LOGW(TAG, "Klipper selected but no Moonraker host — falling back to Marlin");
        s_backend = PRINTER_BACKEND_MARLIN;
    }

    ESP_LOGI(TAG, "Config loaded: backend=%s, moonraker=%s:%u, pause=%s",
             s_backend == PRINTER_BACKEND_KLIPPER ? "Klipper" : "Marlin",
             s_mr_host, s_mr_port,
             s_pause_cmd == PAUSE_CMD_M524 ? "M524 (abort)" : "M25 (pause)");
}

/* ---- Public API ---- */

void printer_comm_rx_cb(const uint8_t *data, size_t len, void *user_ctx)
{
    if (s_backend != PRINTER_BACKEND_MARLIN) return;
    if (s_rx_stream && len > 0) {
        xStreamBufferSend(s_rx_stream, data, len, 0);
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

#if CONFIG_OBICO_ENABLED
    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        klipper_backend_get_state(out);
        return;
    }
#endif

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    memcpy(out, &s_state, sizeof(printer_state_t));
    xSemaphoreGive(s_state_mutex);
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
#if CONFIG_OBICO_ENABLED
    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        return klipper_backend_send_cmd(cmd);
    }
#endif

    if (xQueueSend(s_cmd_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t printer_comm_init(void)
{
    load_config_from_nvs();

#if CONFIG_OBICO_ENABLED
    if (s_backend == PRINTER_BACKEND_KLIPPER) {
        ESP_LOGI(TAG, "Starting Klipper/Moonraker backend → %s:%u", s_mr_host, s_mr_port);
        return klipper_backend_start(s_mr_host, s_mr_port);
    }
#endif

    /* Marlin backend */
    s_state_mutex = xSemaphoreCreateMutex();
    if (!s_state_mutex) return ESP_ERR_NO_MEM;

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
                            NULL, 8, NULL, 0);

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

/* ---- Backend getters ---- */

printer_backend_t printer_comm_get_backend(void)
{
    return s_backend;
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

static esp_err_t printer_config_get_handler(httpd_req_t *req)
{
    const char *checked_marlin  = (s_backend == PRINTER_BACKEND_MARLIN)  ? "checked" : "";
    const char *checked_klipper = (s_backend == PRINTER_BACKEND_KLIPPER) ? "checked" : "";
    const char *checked_m25  = (s_pause_cmd == PAUSE_CMD_M25)  ? "checked" : "";
    const char *checked_m524 = (s_pause_cmd == PAUSE_CMD_M524) ? "checked" : "";

    char html[2048];
    snprintf(html, sizeof(html),
        "<!DOCTYPE html><html><head><title>Printer Config</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<style>"
        "body{font-family:sans-serif;max-width:480px;margin:20px auto;padding:0 10px}"
        "label{display:block;margin:8px 0 4px}input[type=text],input[type=number]{width:100%%;box-sizing:border-box;padding:6px}"
        ".radio-group{margin:10px 0}button{margin-top:16px;padding:10px 24px;font-size:16px}"
        ".klipper-fields{margin-left:20px}"
        "</style></head><body>"
        "<h2>Printer Backend</h2>"
        "<form method='POST' action='/printer/config'>"
        "<div class='radio-group'>"
        "<label><input type='radio' name='backend' value='marlin' %s> Marlin (USB serial)</label>"
        "<label><input type='radio' name='backend' value='klipper' %s> Klipper (Moonraker HTTP)</label>"
        "</div>"
        "<div class='klipper-fields'>"
        "<label>Moonraker Host/IP:<input type='text' name='mr_host' value='%s' maxlength='63'></label>"
        "<label>Moonraker Port:<input type='number' name='mr_port' value='%u' min='1' max='65535'></label>"
        "</div>"
        "<h2>Pause Command</h2>"
        "<div class='radio-group'>"
        "<label><input type='radio' name='pause_cmd' value='m25' %s> M25 — Pause SD print (can resume)</label>"
        "<label><input type='radio' name='pause_cmd' value='m524' %s> M524 — Abort print (if M25 crashes firmware)</label>"
        "</div>"
        "<button type='submit'>Save &amp; Reboot</button>"
        "</form>"
        "<p><a href='/printer/config/test'>Test Moonraker connection</a></p>"
        "</body></html>",
        checked_marlin, checked_klipper, s_mr_host, s_mr_port,
        checked_m25, checked_m524);

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, html, strlen(html));
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

    bool pause_abort = (strstr(body, "pause_cmd=m524") != NULL);

    esp_err_t err = printer_comm_save_config(backend, mr_host, mr_port, pause_abort);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS write failed");
        return ESP_FAIL;
    }

    const char *resp =
        "<!DOCTYPE html><html><head><title>Saved</title>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "</head><body>"
        "<h2>Configuration saved!</h2>"
        "<p>Rebooting in 3 seconds...</p>"
        "<script>setTimeout(function(){location.href='/printer/config'},5000)</script>"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, resp, strlen(resp));

    /* Reboot after a short delay so the response can be sent */
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart();

    return ESP_OK;
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

    httpd_uri_t test_cfg = {
        .uri     = "/printer/config/test",
        .method  = HTTP_GET,
        .handler = printer_config_test_handler,
    };
    httpd_register_uri_handler(server, &test_cfg);

    return ESP_OK;
}
