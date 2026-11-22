/* Simulated Marlin printer, simulation clock, and stand-ins for the ESP-IDF
 * and firmware functions printer_comm.c and host_print.c call (see sim.h) */
#include "sim.h"

#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "nvs.h"

#include "gcode_scan.h"
#include "hub_client.h"
#include "nozzle_guard.h"
#include "layout.h"
#include "printer_comm.h"

sim_cfg_t sim_cfg;
int sim_verbose;

/* ---- Clock ---- */

static int64_t s_now;   /* us */

int64_t sim_now_us(void) { return s_now; }
int64_t esp_timer_get_time(void) { return s_now; }

/* ---- Byte buffers: printer → board (rx), and what the printer holds back ---- */

typedef struct { char *b; size_t len, cap, pos; } buf_t;

static void buf_add(buf_t *q, const char *s, size_t n)
{
    if (q->len + n + 1 > q->cap) {
        q->cap = (q->len + n + 1) * 2;
        q->b = realloc(q->b, q->cap);
    }
    memcpy(q->b + q->len, s, n);
    q->len += n;
}
static void buf_clear(buf_t *q) { q->len = q->pos = 0; }
static size_t buf_avail(const buf_t *q) { return q->len - q->pos; }

static buf_t s_rx;      /* delivered to the board, read by xStreamBufferReceive */
static buf_t s_held;    /* hold_tx: sent by the printer, not yet out of it */
static buf_t s_log;     /* log lines */
static buf_t s_txlog;   /* lines the board sent, '\n'-separated */
static int s_tx_count;

/* ---- The printer ---- */

#define FIFO_N 512
static char s_fifo[FIFO_N][160];
static int s_fifo_head, s_fifo_tail;
static char s_in[256];          /* line being received */
static size_t s_in_len;

static bool s_running;          /* a command is executing */
static char s_cur[160];
static int64_t s_cur_end;
static int64_t s_next_busy;     /* 0 = none */
static int64_t s_next_temp;     /* M109 reports */
static int64_t s_next_point;    /* G29 measured_z */
static int s_points_left;
static int64_t s_point_step;
static int s_target;            /* Hotend target from M104/M109 S, reported by M105 */

static bool s_emit_hold;        /* what the current command sends stays in the printer */

static void emit(const char *s)
{
    buf_t *q = sim_cfg.hold_tx || s_emit_hold ? &s_held : &s_rx;
    buf_add(q, s, strlen(s));
}

static bool in_list(const char *list, const char *code)
{
    if (!list) return false;
    char copy[128];
    snprintf(copy, sizeof(copy), "%s", list);
    for (char *t = strtok(copy, " "); t; t = strtok(NULL, " "))
        if (!strcmp(t, code)) return true;
    return false;
}

/* "N12 G28*35" → "G28" */
static void strip_numbering(const char *in, char *out, size_t size)
{
    const char *p = in;
    while (*p == ' ') p++;
    if ((p[0] == 'N' || p[0] == 'n') && p[1] >= '0' && p[1] <= '9') {
        while (*p && *p != ' ') p++;
        while (*p == ' ') p++;
    }
    snprintf(out, size, "%s", p);
    char *star = strchr(out, '*');
    if (star) *star = '\0';
    size_t n = strlen(out);
    while (n && (out[n - 1] == ' ' || out[n - 1] == '\r')) out[--n] = '\0';
}

/* "G28 X" → "G28" */
static void code_of(const char *line, char *code, size_t size)
{
    size_t i = 0;
    while (line[i] && line[i] != ' ' && i + 1 < size) { code[i] = line[i]; i++; }
    code[i] = '\0';
}

static bool printer_knows(const char *code)
{
    if (!strcmp(code, "M118") && !sim_cfg.knows_m118) return false;
    return !in_list(sim_cfg.unknown, code);
}

static bool is_long(const char *code)
{
    return !strcmp(code, "G28") || !strcmp(code, "G29") || !strcmp(code, "G4")
        || !strcmp(code, "M400") || !strcmp(code, "M109") || !strcmp(code, "M190");
}

static bool is_slow(const char *line)
{
    return sim_cfg.slow_line && !strncmp(line, sim_cfg.slow_line, strlen(sim_cfg.slow_line));
}

static int64_t duration_us(const char *line, const char *code)
{
    if (is_slow(line)) return sim_cfg.slow_ms * 1000LL;
    if (!strcmp(code, "G28")) return sim_cfg.g28_ms * 1000LL;
    if (!strcmp(code, "G29")) return sim_cfg.g29_ms * 1000LL;
    if (!strcmp(code, "M109") || !strcmp(code, "M190")) return sim_cfg.m109_ms * 1000LL;
    if (!strcmp(code, "G4")) {
        const char *s = strchr(line, 'S'), *p = strchr(line, 'P');
        if (s) return atoi(s + 1) * 1000000LL;
        if (p) return atoi(p + 1) * 1000LL;
        return 0;
    }
    return sim_cfg.cmd_ms * 1000LL;
}

static void start_next(void)
{
    if (s_running || s_fifo_head == s_fifo_tail) return;
    snprintf(s_cur, sizeof(s_cur), "%s", s_fifo[s_fifo_head]);
    s_fifo_head = (s_fifo_head + 1) % FIFO_N;
    char code[16];
    code_of(s_cur, code, sizeof(code));
    s_running = true;
    if (!strcmp(code, "M104") || !strcmp(code, "M109")) {
        const char *sp = strchr(s_cur, 'S');
        if (sp) s_target = atoi(sp + 1);
    }
    s_cur_end = s_now + duration_us(s_cur, code);
    s_next_busy = (sim_cfg.busy_ms > 0 && (is_slow(s_cur) || (is_long(code) && strcmp(code, "M109") && strcmp(code, "M190"))))
                  ? s_now + sim_cfg.busy_ms * 1000LL : 0;
    s_next_temp = (!strcmp(code, "M109") || !strcmp(code, "M190")) ? s_now + 1000000LL : 0;
    s_next_point = 0;
    if (!strcmp(code, "G29") && sim_cfg.g29_points > 0) {
        s_points_left = sim_cfg.g29_points;
        s_point_step = sim_cfg.g29_ms * 1000LL / (sim_cfg.g29_points + 1);
        s_next_point = s_now + s_point_step;
    }
}

static void finish_current(void)
{
    char code[16], out[256];
    code_of(s_cur, code, sizeof(code));
    s_running = false;
    s_next_busy = s_next_temp = s_next_point = 0;
    s_emit_hold = in_list(sim_cfg.hold_cmds, code);
    if (!printer_knows(code)) {
        snprintf(out, sizeof(out), "echo:Unknown command: \"%s\"\nok\n", s_cur);
        emit(out);
    } else if (!strcmp(code, "G28") || !strcmp(code, "M114")) {
        emit("X:38.80 Y:206.65 Z:2.88 E:0.00 Count X:3104 Y:16532 Z:1179\nok\n");
    } else if (!strcmp(code, "M105")) {
        snprintf(out, sizeof(out), "ok T:30.00 /%d.00 B:30.00 /0.00 @:0 B@:0\n", s_target);
        emit(out);
    } else if (!strcmp(code, "M27")) {
        emit("Not SD printing\nok\n");
    } else if (!strcmp(code, "M118")) {
        const char *msg = s_cur + 4;
        while (*msg == ' ') msg++;
        if (!strncmp(msg, "E1", 2)) { msg += 2; while (*msg == ' ') msg++; }
        snprintf(out, sizeof(out), "echo:%s\nok\n", msg);
        emit(out);
    } else {
        emit("ok\n");
    }
    s_emit_hold = false;
}

static int64_t next_event(void)
{
    if (sim_cfg.dead) return INT64_MAX;
    if (!s_running) return s_fifo_head != s_fifo_tail ? s_now : INT64_MAX;
    int64_t t = s_cur_end;
    if (s_next_busy && s_next_busy < t) t = s_next_busy;
    if (s_next_temp && s_next_temp < t) t = s_next_temp;
    if (s_next_point && s_next_point < t) t = s_next_point;
    return t;
}

static void run_due(void)
{
    for (;;) {
        int64_t t = next_event();
        if (t > s_now) return;
        if (!s_running) { start_next(); continue; }
        if (s_next_point && s_next_point <= s_now && s_next_point <= s_cur_end) {
            emit("measured_z: 0.05\n");
            s_next_point = --s_points_left > 0 ? s_next_point + s_point_step : 0;
        } else if (s_next_busy && s_next_busy <= s_now && s_next_busy < s_cur_end) {
            emit("echo:busy: processing\n");
            s_next_busy += sim_cfg.busy_ms * 1000LL;
        } else if (s_next_temp && s_next_temp <= s_now && s_next_temp < s_cur_end) {
            char rep[96];
            snprintf(rep, sizeof(rep), " T:200.00 /%d.00 B:60.00 /60.00 @:0 B@:0 W:?\n", s_target ? s_target : 200);
            emit(rep);
            s_next_temp += 1000000LL;
        } else {
            finish_current();
        }
    }
}

void sim_run_until_us(int64_t t)
{
    for (;;) {
        run_due();
        int64_t e = next_event();
        if (e > t) break;
        s_now = e;
    }
    if (t > s_now) s_now = t;
    run_due();
}

/* The board sends bytes: a USB OUT transfer */
static void printer_receive(const uint8_t *data, size_t len)
{
    run_due();
    if (sim_cfg.hold_tx && buf_avail(&s_held)) {   /* the OUT lets the held bytes go */
        buf_add(&s_rx, s_held.b + s_held.pos, buf_avail(&s_held));
        buf_clear(&s_held);
    }
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n' || c == '\r') {
            if (!s_in_len) continue;
            s_in[s_in_len] = '\0';
            s_in_len = 0;
            char line[160];
            strip_numbering(s_in, line, sizeof(line));
            buf_add(&s_txlog, line, strlen(line));
            buf_add(&s_txlog, "\n", 1);
            s_tx_count++;
            snprintf(s_fifo[s_fifo_tail], sizeof(s_fifo[0]), "%s", line);
            s_fifo_tail = (s_fifo_tail + 1) % FIFO_N;
        } else if (s_in_len < sizeof(s_in) - 1) {
            s_in[s_in_len++] = c;
        }
    }
    run_due();
}

void sim_reset(void)
{
    sim_cfg = (sim_cfg_t){
        .cmd_ms = 5, .g28_ms = 20000, .g29_ms = 60000, .g29_points = 0,
        .m109_ms = 60000, .busy_ms = 0, .knows_m118 = true,
    };
    s_now = 1000000;   /* 1 s: a zero time looks like "no cooldown" and "never" */
    buf_clear(&s_rx); buf_clear(&s_held); buf_clear(&s_log); buf_clear(&s_txlog);
    s_tx_count = 0;
    s_fifo_head = s_fifo_tail = 0;
    s_in_len = 0;
    s_running = false;
    s_next_busy = s_next_temp = s_next_point = 0;
    s_target = 0;
}

/* ---- What the board sent ---- */

int sim_tx_count(void) { return s_tx_count; }

const char *sim_tx_line(int i)
{
    static char line[160];
    const char *p = s_txlog.b;
    for (int n = 0; p && n < s_tx_count; n++) {
        const char *e = memchr(p, '\n', s_txlog.len - (size_t)(p - s_txlog.b));
        if (n == i) {
            snprintf(line, sizeof(line), "%.*s", (int)(e - p), p);
            return line;
        }
        p = e + 1;
    }
    return "";
}

int sim_tx_count_of(const char *prefix)
{
    int n = 0;
    for (int i = 0; i < s_tx_count; i++)
        if (!strncmp(sim_tx_line(i), prefix, strlen(prefix))) n++;
    return n;
}

/* ---- Log ---- */

void sim_log(char level, const char *tag, const char *fmt, ...)
{
    char line[512];
    int n = snprintf(line, sizeof(line), "%c (%lld) %s: ", level, (long long)(s_now / 1000), tag);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line + n, sizeof(line) - n, fmt, ap);
    va_end(ap);
    if (sim_verbose) printf("    %s\n", line);
    buf_add(&s_log, line, strlen(line));
    buf_add(&s_log, "\n", 1);
}

bool sim_log_has(const char *text)
{
    if (!s_log.len) return false;
    s_log.b[s_log.len] = '\0';
    return strstr(s_log.b, text) != NULL;
}

void sim_log_dump(void)
{
    if (!s_log.len) return;
    s_log.b[s_log.len] = '\0';
    printf("%s", s_log.b);
}

void sim_log_clear(void) { buf_clear(&s_log); }

/* ---- USB serial ---- */

esp_err_t usb_serial_send(const uint8_t *data, size_t len)
{
    printer_receive(data, len);
    return ESP_OK;
}

bool usb_serial_is_connected(void) { return true; }

/* ---- FreeRTOS ---- */

static jmp_buf s_task_exit;
static bool (*s_task_stop)(void);
static int64_t s_task_until;
static bool s_task_stopped;

void vTaskDelay(TickType_t ticks)
{
    sim_run_until_us(s_now + (int64_t)ticks * 1000);
    if (s_task_stop) {
        if (s_task_stop()) { s_task_stopped = true; longjmp(s_task_exit, 1); }
        if (s_now >= s_task_until) { s_task_stopped = false; longjmp(s_task_exit, 1); }
    }
}

bool sim_run_task(void (*task)(void *), bool (*stop)(void), int64_t limit_us)
{
    s_task_stop = stop;
    s_task_until = s_now + limit_us;
    s_task_stopped = false;
    if (!setjmp(s_task_exit)) task(NULL);
    s_task_stop = NULL;
    return s_task_stopped;
}
TickType_t xTaskGetTickCount(void) { return (TickType_t)(s_now / 1000); }

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t fn, const char *name, uint32_t stack,
                                   void *arg, UBaseType_t prio, TaskHandle_t *out, BaseType_t core)
{
    (void)fn; (void)name; (void)stack; (void)arg; (void)prio; (void)core;
    if (out) *out = NULL;
    return pdFAIL;     /* no background tasks in the simulation */
}
BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack,
                       void *arg, UBaseType_t prio, TaskHandle_t *out)
{
    return xTaskCreatePinnedToCore(fn, name, stack, arg, prio, out, 0);
}
void vTaskDelete(TaskHandle_t t) { (void)t; }
BaseType_t xTaskCreatePinnedToCoreWithCaps(TaskFunction_t fn, const char *name, uint32_t stack,
                                           void *arg, UBaseType_t prio, TaskHandle_t *out,
                                           BaseType_t core, UBaseType_t caps)
{
    (void)caps;
    return xTaskCreatePinnedToCore(fn, name, stack, arg, prio, out, core);
}
void vTaskDeleteWithCaps(TaskHandle_t t) { (void)t; }
BaseType_t xTaskGetStaticBuffers(TaskHandle_t t, StackType_t **stack, StaticTask_t **tcb)
{
    (void)t; (void)stack; (void)tcb;
    return pdFALSE;
}

typedef struct { size_t item, cap, head, count; uint8_t *items; } queue_t;

QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t item_size)
{
    queue_t *q = calloc(1, sizeof(*q));
    q->item = item_size; q->cap = len;
    q->items = calloc(len, item_size);
    return q;
}
BaseType_t xQueueSend(QueueHandle_t h, const void *item, TickType_t wait)
{
    (void)wait;
    queue_t *q = h;
    if (!q || q->count == q->cap) return pdFAIL;
    memcpy(q->items + ((q->head + q->count) % q->cap) * q->item, item, q->item);
    q->count++;
    return pdPASS;
}
BaseType_t xQueueReceive(QueueHandle_t h, void *item, TickType_t wait)
{
    (void)wait;
    queue_t *q = h;
    if (!q || !q->count) return pdFAIL;
    memcpy(item, q->items + q->head * q->item, q->item);
    q->head = (q->head + 1) % q->cap;
    q->count--;
    return pdPASS;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void) { return (void *)1; }
SemaphoreHandle_t xSemaphoreCreateBinary(void) { return (void *)1; }
BaseType_t xSemaphoreTake(SemaphoreHandle_t s, TickType_t wait) { (void)s; (void)wait; return pdTRUE; }
BaseType_t xSemaphoreGive(SemaphoreHandle_t s) { (void)s; return pdTRUE; }

StreamBufferHandle_t xStreamBufferCreate(size_t size, size_t trigger)
{
    (void)size; (void)trigger;
    return &s_rx;
}
StreamBufferHandle_t xStreamBufferCreateWithCaps(size_t size, size_t trigger, uint32_t caps)
{
    (void)caps;
    return xStreamBufferCreate(size, trigger);
}
size_t xStreamBufferSend(StreamBufferHandle_t s, const void *data, size_t len, TickType_t wait)
{
    (void)s; (void)wait;
    buf_add(&s_rx, data, len);
    return len;
}
BaseType_t xStreamBufferReset(StreamBufferHandle_t s) { (void)s; buf_clear(&s_rx); return pdPASS; }

/* Reads what the printer has delivered; waiting moves the clock to the
 * printer's next event, or to the end of the wait */
size_t xStreamBufferReceive(StreamBufferHandle_t s, void *out, size_t len, TickType_t ticks)
{
    (void)s;
    int64_t deadline = ticks == portMAX_DELAY ? INT64_MAX / 2 : s_now + (int64_t)ticks * 1000;
    for (;;) {
        run_due();
        size_t n = buf_avail(&s_rx);
        if (n) {
            if (n > len) n = len;
            memcpy(out, s_rx.b + s_rx.pos, n);
            s_rx.pos += n;
            if (s_rx.pos == s_rx.len) buf_clear(&s_rx);
            return n;
        }
        if (!ticks) return 0;
        int64_t e = next_event();
        if (e > deadline) {
            s_now = deadline;
            run_due();
            if (buf_avail(&s_rx)) continue;
            return 0;
        }
        s_now = e;
    }
}

/* ---- Libraries and firmware modules outside the tested code ---- */

size_t strlcpy(char *dst, const char *src, size_t size)
{
    size_t n = strlen(src);
    if (size) {
        size_t c = n < size - 1 ? n : size - 1;
        memcpy(dst, src, c);
        dst[c] = '\0';
    }
    return n;
}

const char *esp_err_to_name(esp_err_t e) { (void)e; return "ERR"; }
void esp_restart(void) { abort(); }

esp_err_t nvs_open(const char *ns, nvs_open_mode_t mode, nvs_handle_t *h) { (void)ns; (void)mode; *h = 1; return ESP_OK; }
void nvs_close(nvs_handle_t h) { (void)h; }
esp_err_t nvs_commit(nvs_handle_t h) { (void)h; return ESP_OK; }
esp_err_t nvs_get_str(nvs_handle_t h, const char *k, char *o, size_t *l) { (void)h; (void)k; (void)o; (void)l; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_set_str(nvs_handle_t h, const char *k, const char *v) { (void)h; (void)k; (void)v; return ESP_OK; }
esp_err_t nvs_get_u8(nvs_handle_t h, const char *k, uint8_t *v) { (void)h; (void)k; (void)v; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_set_u8(nvs_handle_t h, const char *k, uint8_t v) { (void)h; (void)k; (void)v; return ESP_OK; }
esp_err_t nvs_get_u16(nvs_handle_t h, const char *k, uint16_t *v) { (void)h; (void)k; (void)v; return ESP_ERR_NVS_NOT_FOUND; }
esp_err_t nvs_set_u16(nvs_handle_t h, const char *k, uint16_t v) { (void)h; (void)k; (void)v; return ESP_OK; }

esp_http_client_handle_t esp_http_client_init(const esp_http_client_config_t *c) { (void)c; return NULL; }
esp_err_t esp_http_client_perform(esp_http_client_handle_t h) { (void)h; return ESP_FAIL; }
int esp_http_client_get_status_code(esp_http_client_handle_t h) { (void)h; return 0; }
esp_err_t esp_http_client_cleanup(esp_http_client_handle_t h) { (void)h; return ESP_OK; }

int httpd_req_recv(httpd_req_t *r, char *b, size_t l) { (void)r; (void)b; (void)l; return -1; }
esp_err_t httpd_resp_send(httpd_req_t *r, const char *b, long l) { (void)r; (void)b; (void)l; return ESP_OK; }
esp_err_t httpd_resp_send_err(httpd_req_t *r, httpd_err_code_t c, const char *m) { (void)r; (void)c; (void)m; return ESP_OK; }
esp_err_t httpd_resp_set_hdr(httpd_req_t *r, const char *k, const char *v) { (void)r; (void)k; (void)v; return ESP_OK; }
esp_err_t httpd_resp_set_status(httpd_req_t *r, const char *s) { (void)r; (void)s; return ESP_OK; }
esp_err_t httpd_resp_set_type(httpd_req_t *r, const char *t) { (void)r; (void)t; return ESP_OK; }
esp_err_t httpd_register_uri_handler(httpd_handle_t h, const httpd_uri_t *u) { (void)h; (void)u; return ESP_OK; }
void httpd_note_babystep(float dz) { (void)dz; }
esp_err_t i18n_handler_wrap(httpd_req_t *req) { (void)req; return ESP_OK; }
const char *i18n_page(const char *tmpl) { return tmpl; }
void layout_html_begin(html_buf_t *p, const char *title, const char *path) { (void)p; (void)title; (void)path; }
void layout_html_end(html_buf_t *p) { (void)p; }

void terminal_feed_rx(const uint8_t *d, size_t l) { (void)d; (void)l; }
void terminal_feed_tx(const char *cmd) { (void)cmd; }

void hub_client_send_event(hub_event_type_t type, const char *message, const char *filename)
{
    sim_log('I', "hub", "event %d: %s (%s)", (int)type, message ? message : "", filename ? filename : "");
}

/* Only the layer count, which the nozzle guard needs: ";LAYER:" lines */
void gcode_scan_file(FILE *f, gcode_scan_t *out)
{
    memset(out, 0, sizeof(*out));
    char line[256];
    rewind(f);
    while (fgets(line, sizeof(line), f)) {
        if (!strncmp(line, ";LAYER:", 7)) out->total_layers++;
    }
    rewind(f);
}

esp_err_t klipper_backend_start(const char *host, uint16_t port) { (void)host; (void)port; return ESP_FAIL; }
void klipper_backend_get_state(printer_state_t *out) { (void)out; }
esp_err_t klipper_backend_send_cmd(const printer_cmd_t *cmd) { (void)cmd; return ESP_FAIL; }
esp_err_t klipper_backend_print_script(const char *name, char *script, host_script_done_t done, int arg)
{
    (void)name; free(script); (void)done; (void)arg;
    return ESP_FAIL;
}
esp_err_t klipper_backend_continue(uint32_t id) { (void)id; return ESP_FAIL; }
esp_err_t klipper_backend_zstep(float dz) { (void)dz; return ESP_FAIL; }

/* Nozzle guard (nozzle_guard.c): off unless a test turns it on */
ng_config_t sim_ng_config;
int sim_ng_plastic = -1;
void ng_config_get(ng_config_t *out) { *out = sim_ng_config; }

/* print_checks.c: the probe offsets read, for the tests */
float sim_z_seen = -99;
int sim_z_seen_n;
void print_checks_z_seen(float z, bool busy) { if (!busy) { sim_z_seen = z; sim_z_seen_n++; } }
int ng_standby_c(int target_c) { return ng_standby_for(&sim_ng_config, sim_ng_plastic, 2, target_c); }
void ng_notify_runout(int target_c, int standby_c, const char *filename)
{
    sim_log('I', "hub", "filament runout %d -> %d (%s)", target_c, standby_c, filename ? filename : "");
}
void ng_notify_standby(int from_c, int to_c, const char *filename)
{
    sim_log('I', "hub", "nozzle standby %d -> %d (%s)", from_c, to_c, filename ? filename : "");
}
