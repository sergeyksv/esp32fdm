#include "printer_comm.h"

#include "usb_serial.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "printer_comm";

/* ---- Configuration ---- */

#define POLL_M105_INTERVAL_MS   4000
#define POLL_M27_INTERVAL_MS    10000
#define POLL_M114_INTERVAL_MS   30000
#define CMD_RESPONSE_TIMEOUT_MS 500
#define RX_LINE_BUF_SIZE        256
#define CMD_QUEUE_SIZE          8

/* ---- State ---- */

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
            s_state.progress_pct = (float)pos / (float)total * 100.0f;

            if (s_state.opstate != PRINTER_PAUSED) {
                if (s_state.opstate != PRINTER_PRINTING) {
                    s_print_start_us = esp_timer_get_time();
                }
                s_state.opstate = PRINTER_PRINTING;
            }

            /* Estimate times */
            int64_t elapsed_us = esp_timer_get_time() - s_print_start_us;
            s_state.print_time_s = (int32_t)(elapsed_us / 1000000);
            if (s_state.progress_pct > 0.1f) {
                float total_est = (float)s_state.print_time_s / (s_state.progress_pct / 100.0f);
                s_state.print_time_left_s = (int32_t)(total_est - s_state.print_time_s);
                if (s_state.print_time_left_s < 0) s_state.print_time_left_s = 0;
            }
        }
    } else if (strstr(line, "Not SD printing")) {
        if (s_state.opstate == PRINTER_PRINTING || s_state.opstate == PRINTER_PAUSED) {
            s_state.opstate = PRINTER_OPERATIONAL;
            s_state.progress_pct = -1;
            s_state.print_time_s = -1;
            s_state.print_time_left_s = -1;
            s_state.filename[0] = '\0';
        }
    }

    xSemaphoreGive(s_state_mutex);
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

    ESP_LOGD(TAG, "RX: %s", line);

    switch (s_pending_query) {
    case QUERY_M105:
        if (strstr(line, "T:")) {
            parse_m105(line);
        }
        break;
    case QUERY_M27:
        if (strstr(line, "SD printing") || strstr(line, "Not SD")) {
            parse_m27(line);
        }
        break;
    case QUERY_M114:
        if (strstr(line, "X:") && strstr(line, "Y:")) {
            parse_m114(line);
        }
        break;
    default:
        break;
    }

    /* "ok" signals end of response for current query */
    if (strncmp(line, "ok", 2) == 0) {
        s_pending_query = QUERY_NONE;
    }
}

/** Send a gcode command to the printer, wait for response lines */
static bool send_query(const char *gcode, query_type_t qtype)
{
    if (!usb_serial_is_connected()) return false;

    s_pending_query = qtype;
    s_line_len = 0;

    /* Send command with newline */
    char buf[100];
    int len = snprintf(buf, sizeof(buf), "%s\n", gcode);
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
            ESP_LOGD(TAG, "Timeout waiting for response to %s", gcode);
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
            switch (cmd.type) {
            case PCMD_PAUSE:  gcode = "M25"; break;
            case PCMD_RESUME: gcode = "M24"; break;
            case PCMD_CANCEL: gcode = "M524"; break;
            case PCMD_RAW:    gcode = cmd.gcode; break;
            }
            if (gcode) {
                ESP_LOGI(TAG, "Sending command: %s", gcode);
                send_query(gcode, QUERY_CMD);
            }
        }

        /* Poll M105 (temperatures) every 4s */
        now = esp_timer_get_time();
        if (now - s_last_m105_us >= POLL_M105_INTERVAL_MS * 1000LL) {
            send_query("M105", QUERY_M105);
            s_last_m105_us = esp_timer_get_time();
        }

        /* Poll M27 (SD progress) every 10s */
        now = esp_timer_get_time();
        if (now - s_last_m27_us >= POLL_M27_INTERVAL_MS * 1000LL) {
            send_query("M27", QUERY_M27);
            s_last_m27_us = esp_timer_get_time();
        }

        /* Poll M114 (position) every 30s */
        now = esp_timer_get_time();
        if (now - s_last_m114_us >= POLL_M114_INTERVAL_MS * 1000LL) {
            send_query("M114", QUERY_M114);
            s_last_m114_us = esp_timer_get_time();
        }

        /* Small delay to avoid busy-looping between polls */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---- Public API ---- */

void printer_comm_rx_cb(const uint8_t *data, size_t len, void *user_ctx)
{
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
    if (xQueueSend(s_cmd_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full");
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t printer_comm_init(void)
{
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

    xTaskCreatePinnedToCore(printer_comm_task, "printer_comm", 4096,
                            NULL, 8, NULL, 0);

    ESP_LOGI(TAG, "Printer comm initialized");
    return ESP_OK;
}
