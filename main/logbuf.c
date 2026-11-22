#include "logbuf.h"
#include "task_util.h"
#include "httpd.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_attr.h"
#include "esp_cache.h"
#include "esp_system.h"
#include <inttypes.h>
#include <stdbool.h>
#include "i18n.h"
#include "layout.h"

static const char *TAG = "logbuf";

/* ---- Ring buffer ----
 *
 * The ring lives in the PSRAM .noinit segment, which nothing clears at boot
 * (the PSRAM memtest skips it), so a software reset -- panic, watchdog,
 * esp_restart -- keeps it. On boot a valid header means we carry on writing
 * after the old lines, behind a "reboot" marker with the reset reason. A
 * power loss or brownout wipes PSRAM, so then the header is garbage and the
 * ring starts empty. A new firmware may move the segment, which also fails
 * the header check.
 *
 * PSRAM sits behind a write-back cache: on a crash, lines still in the cache
 * never reach PSRAM. Each write is pushed out with esp_cache_msync, so the
 * lines right before a crash -- the ones we want -- survive.
 *
 * The segment starts at PSRAM address 0, and every boot the MSPI timing tuning
 * for 80 MHz PSRAM writes its 64-byte test pattern (0xa5ff005a) there
 * (MSPI_TIMING_PSRAM_TEST_DATA_ADDR). So the ring is one object that starts
 * with a reserved block nothing of ours is kept in. */

#define LOGBUF_SIZE (2 * 1024 * 1024)   /* 2 MB in PSRAM */
#define FMT_BUF_SIZE 512                /* stack buffer for vsnprintf */
#define LOGBUF_MAGIC 0x4C4F4742u        /* "LOGB" */
#define LOGBUF_RESERVED 4096            /* Covers the 64-byte tuning pattern, with room to spare */

typedef struct {
    uint32_t magic;
    uint32_t head;      /* next write position */
    uint32_t count;     /* valid bytes in buffer (<= LOGBUF_SIZE) */
    uint32_t boots;     /* boots this ring has seen */
    uint32_t check;     /* magic ^ head ^ count ^ boots, catches a torn header */
} logbuf_hdr_t;

/* One object, so that the linker cannot reorder the parts */
static EXT_RAM_NOINIT_ATTR struct {
    char reserved[LOGBUF_RESERVED];     /* Overwritten at boot, see above */
    logbuf_hdr_t hdr;
    char buf[LOGBUF_SIZE];
} s_ring;

#define s_hdr s_ring.hdr
#define s_buf s_ring.buf

static SemaphoreHandle_t s_mutex;
static vprintf_like_t s_orig_vprintf;

static uint32_t hdr_check(void)
{
    return s_hdr.magic ^ s_hdr.head ^ s_hdr.count ^ s_hdr.boots;
}

static void flush(const void *addr, size_t len)
{
    if (len)
        esp_cache_msync((void *)addr, len,
                        ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
}

/* ---- Ring helpers ---- */

static void ring_write(const char *data, size_t len)
{
    if (len > LOGBUF_SIZE) {
        data += len - LOGBUF_SIZE;
        len = LOGBUF_SIZE;
    }
    uint32_t head = s_hdr.head;
    size_t first = LOGBUF_SIZE - head;
    if (first > len) first = len;
    memcpy(s_buf + head, data, first);
    memcpy(s_buf, data + first, len - first);
    flush(s_buf + head, first);
    flush(s_buf, len - first);

    s_hdr.head = (head + len) % LOGBUF_SIZE;
    s_hdr.count = (s_hdr.count + len > LOGBUF_SIZE) ? LOGBUF_SIZE : s_hdr.count + len;
    s_hdr.check = hdr_check();
    flush(&s_hdr, sizeof(s_hdr));
}

static void ring_reset(void)
{
    s_hdr.head = 0;
    s_hdr.count = 0;
    s_hdr.check = hdr_check();
    flush(&s_hdr, sizeof(s_hdr));
}

/* ---- esp_log hook ---- */

static int log_vprintf(const char *fmt, va_list args)
{
    /* Format into stack buffer */
    char tmp[FMT_BUF_SIZE];
    int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
    if (len < 0) len = 0;
    if (len >= (int)sizeof(tmp)) len = (int)sizeof(tmp) - 1;

    /* Write to ring buffer */
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        ring_write(tmp, len);
        xSemaphoreGive(s_mutex);
    }

    /* Forward to original vprintf (UART) */
    /* We already have the formatted string, just write it */
    for (int i = 0; i < len; i++)
        putchar(tmp[i]);

    return len;
}

/* ---- Crash report ----
 *
 * ESP-IDF's panic handler prints the reason and the backtrace on the UART
 * only, so /logs showed a bare "reboot: reason=PANIC" and nothing said why
 * (a stack overflow looked like a power problem). The link wraps
 * esp_panic_handler (-Wl,--wrap in main/CMakeLists.txt): this copies the
 * report into the ring, then hands over to the real handler, which prints
 * and restarts as before. Interrupt watchdog timeouts come through here too.
 *
 * It runs inside the panic handler, with the other core stopped: no locks,
 * no heap, no printf. Lines are written without esp_cache_msync (it takes a
 * lock), and the whole cache is written back once at the end. */

#include "esp_private/panic_internal.h"
#include "esp_debug_helpers.h"
#include "esp_cpu_utils.h"
#include "esp32s3/rom/cache.h"
#include "xtensa_context.h"
#include "version.h"
#include "esp_timer.h"

void __real_esp_panic_handler(panic_info_t *info);

static DRAM_ATTR char s_pl[200];
static DRAM_ATTR size_t s_pl_len;
static DRAM_ATTR const char s_hex[] = "0123456789abcdef";

static void IRAM_ATTR pl_str(const char *str)
{
    while (str && *str && s_pl_len < sizeof(s_pl) - 1) s_pl[s_pl_len++] = *str++;
}

static void IRAM_ATTR pl_hex(uint32_t v)
{
    pl_str("0x");
    for (int sh = 28; sh >= 0 && s_pl_len < sizeof(s_pl) - 1; sh -= 4)
        s_pl[s_pl_len++] = s_hex[(v >> sh) & 15];
}

static void IRAM_ATTR pl_dec(uint32_t v)
{
    char d[10];
    int n = 0;
    do { d[n++] = (char)('0' + v % 10); v /= 10; } while (v && n < 10);
    while (n && s_pl_len < sizeof(s_pl) - 1) s_pl[s_pl_len++] = d[--n];
}

/* Starts a line the way ESP_LOGE does, so /logs filters and colours it */
static void IRAM_ATTR pl_begin(void)
{
    s_pl_len = 0;
    pl_str("E (");
    pl_dec((uint32_t)(esp_timer_get_time() / 1000));   /* lock-free, ISR-safe; the CPU cycle count of esp_log_early_timestamp() wraps every 18 s */
    pl_str(") panic: ");
}

static void IRAM_ATTR pl_end(void)
{
    s_pl[s_pl_len++] = '\n';
    size_t len = s_pl_len;
    uint32_t head = s_hdr.head;
    size_t first = LOGBUF_SIZE - head;
    if (first > len) first = len;
    memcpy(s_buf + head, s_pl, first);
    memcpy(s_buf, s_pl + first, len - first);
    s_hdr.head = (head + len) % LOGBUF_SIZE;
    s_hdr.count = (s_hdr.count + len > LOGBUF_SIZE) ? LOGBUF_SIZE : s_hdr.count + len;
    s_hdr.check = hdr_check();
    s_pl_len = 0;
}

static void IRAM_ATTR panic_to_ring(const panic_info_t *info)
{
    static const char *const kind[] = { "debug exception", "interrupt watchdog",
                                        "task watchdog", "abort", "fault" };

    pl_begin();
    pl_str("core ");
    pl_dec((uint32_t)info->core);
    pl_str(": ");
    if (g_panic_abort && g_panic_abort_details) {
        pl_str(g_panic_abort_details);   /* e.g. "A stack overflow in task led has been detected." */
    } else {
        if ((unsigned)info->exception < sizeof(kind) / sizeof(kind[0]))
            pl_str(kind[info->exception]);
        if (info->reason) { pl_str(", "); pl_str(info->reason); }
        if (info->description) { pl_str(", "); pl_str(info->description); }
    }
    pl_end();

    TaskHandle_t t = xTaskGetCurrentTaskHandleForCore(info->core);
    pl_begin();
    pl_str("task ");
    pl_str(t ? pcTaskGetName(t) : "?");
    pl_str(", PC ");
    pl_hex((uint32_t)info->addr);
    pl_str(", build ");
    pl_str(FW_BUILD);
    pl_end();

    /* The same walk as esp_backtrace_print_from_frame(); decode with
     * xtensa-esp32s3-elf-addr2line -pfiaC -e build/esp32fdm.elf <PCs>
     * on the ELF of that build */
    const XtExcFrame *xf = (const XtExcFrame *)info->frame;
    if (!xf) return;
    esp_backtrace_frame_t fr = { .pc = xf->pc, .sp = xf->a1, .next_pc = xf->a0, .exc_frame = (void *)xf };
    pl_begin();
    pl_str("Backtrace:");
    for (int i = 0; i < 24; i++) {
        if (s_pl_len > sizeof(s_pl) - 24) { pl_end(); pl_begin(); pl_str("Backtrace (more):"); }
        pl_str(" ");
        pl_hex(esp_cpu_process_stack_pc(fr.pc));
        pl_str(":");
        pl_hex(fr.sp);
        if (fr.next_pc == 0) break;
        if (!esp_backtrace_get_next_frame(&fr)) { pl_str(" |<-CORRUPTED"); break; }
    }
    pl_end();
}

void IRAM_ATTR __wrap_esp_panic_handler(panic_info_t *info)
{
    static DRAM_ATTR bool s_in_panic;
    if (!s_in_panic && s_hdr.magic == LOGBUF_MAGIC && s_hdr.check == hdr_check()) {
        s_in_panic = true;
        panic_to_ring(info);
        Cache_WriteBack_All();   /* PSRAM sits behind a write-back cache */
    }
    __real_esp_panic_handler(info);
}

/* ---- Init ---- */

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_USB:       return "USB";
    case ESP_RST_JTAG:      return "JTAG";
    case ESP_RST_EFUSE:     return "EFUSE";
    case ESP_RST_PWR_GLITCH: return "PWR_GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
    default:                return "UNKNOWN";
    }
}

void logbuf_init(void)
{
    bool kept = s_hdr.magic == LOGBUF_MAGIC && s_hdr.check == hdr_check() &&
                s_hdr.head < LOGBUF_SIZE && s_hdr.count <= LOGBUF_SIZE;
    if (kept) {
        s_hdr.boots++;
    } else {
        s_hdr.magic = LOGBUF_MAGIC;
        s_hdr.boots = 1;
        s_hdr.head = 0;
        s_hdr.count = 0;
    }
    s_hdr.check = hdr_check();
    flush(&s_hdr, sizeof(s_hdr));
    s_mutex = xSemaphoreCreateMutex();

    s_orig_vprintf = esp_log_set_vprintf(log_vprintf);

    esp_reset_reason_t why = esp_reset_reason();
    if (kept)
        ESP_LOGW(TAG, "==== reboot #%" PRIu32 ": reason=%s, log before it kept (%" PRIu32 " KB) ====",
                 s_hdr.boots, reset_reason_str(why), s_hdr.count / 1024);
    else
        ESP_LOGW(TAG, "==== boot: reason=%s, no previous log ====", reset_reason_str(why));
    ESP_LOGI(TAG, "Log buffer initialized (%d KB, PSRAM noinit)", LOGBUF_SIZE / 1024);
}

/* ---- TAG classification ---- */

static bool is_printer_tag(const char *line, int len)
{
    /*
     * ESP_LOG format: "X (12345) TAG: message\n"
     * TAG starts after ") " — find it and check.
     */
    const char *paren_close = memchr(line, ')', len);
    if (!paren_close || paren_close - line + 2 >= len) return false;

    const char *tag_start = paren_close + 2;  /* skip ") " */
    int remaining = len - (int)(tag_start - line);

    if (remaining >= 12 && memcmp(tag_start, "printer_comm", 12) == 0) return true;
    if (remaining >= 7  && memcmp(tag_start, "klipper", 7) == 0)      return true;
    return false;
}

static bool is_heap_line(const char *line, int len)
{
    /* Match "HEAP [" anywhere in line — covers log_heap() and heap_monitor_task */
    return memmem(line, len, "HEAP [", 6) != NULL;
}

/* ---- HTTP handler (async) ---- */

typedef enum { FILTER_ALL, FILTER_PRINTER, FILTER_SYSTEM, FILTER_HEAP } log_filter_t;

typedef struct {
    httpd_req_t *req;
    log_filter_t filter;
    int lang;           /* i18n language of the request: the task does not inherit it */
} logs_ctx_t;

static const struct {
    log_filter_t filter;
    const char *href;
    const char *label;  /* NT_(): translated with TR() */
} s_filters[] = {
    { FILTER_ALL,     "/logs",                NT_("All") },
    { FILTER_PRINTER, "/logs?filter=printer", NT_("Printer") },
    { FILTER_SYSTEM,  "/logs?filter=system",  NT_("System") },
    { FILTER_HEAP,    "/logs?filter=heap",    NT_("Heap") },
};

static void logs_task(void *arg)
{
    logs_ctx_t *ctx = (logs_ctx_t *)arg;
    httpd_req_t *req = ctx->req;
    log_filter_t filter = ctx->filter;
    i18n_set_lang(ctx->lang);
    bool send_ok = true;  /* track client disconnect */

    /* Snapshot the ring buffer to avoid holding mutex during HTTP send */
    char *snap = heap_caps_malloc(LOGBUF_SIZE, MALLOC_CAP_SPIRAM);
    int snap_len = 0;
    int snap_start = 0;

    if (snap && s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        snap_len = (int)s_hdr.count;
        snap_start = (int)((s_hdr.head - s_hdr.count + LOGBUF_SIZE) % LOGBUF_SIZE);
        /* Linearize ring into snapshot */
        if (snap_start + snap_len <= LOGBUF_SIZE) {
            memcpy(snap, s_buf + snap_start, snap_len);
        } else {
            int first = LOGBUF_SIZE - snap_start;
            memcpy(snap, s_buf + snap_start, first);
            memcpy(snap + first, s_buf, snap_len - first);
        }
        xSemaphoreGive(s_mutex);
    }

    httpd_resp_set_type(req, "text/html");

    /* HTML header via layout */
    html_buf_t page;
    html_buf_init(&page);
    layout_html_begin(&page, NT_("Logs"), "/logs");

    /* Filter buttons */
    html_buf_printf(&page, "<div style='margin:12px 0'><b>%s</b> ", _("Filter:"));
    for (size_t i = 0; i < sizeof(s_filters) / sizeof(s_filters[0]); i++) {
        if (filter == s_filters[i].filter)
            html_buf_printf(&page, "<b>[%s]</b> ", TR(s_filters[i].label));
        else
            html_buf_printf(&page, "<a href='%s'>%s</a> ", s_filters[i].href, TR(s_filters[i].label));
    }
    html_buf_printf(&page,
        " | <form method='POST' action='/logs/clear' style='display:inline'>"
        "<button type='submit' onclick=\"return confirm('%s')\">"
        "%s</button></form>",
        _("Clear all logs?"), _("Clear"));
    html_buf_printf(&page, "</div>"
        "<pre style='background:#1e1e1e;color:#d4d4d4;padding:12px;"
        "font-size:13px;border-radius:4px;white-space:pre-wrap;word-wrap:break-word;"
        "max-height:70vh;overflow-y:auto' id='log'>");

    /* Send header */
    if (httpd_resp_send_chunk(req, page.data, page.len) != ESP_OK)
        send_ok = false;
    html_buf_free(&page);

    /* Stream log lines from snapshot */
    if (send_ok && snap && snap_len > 0) {
        int pos = 0;
        while (pos < snap_len && send_ok) {
            /* Find end of line */
            int line_start = pos;
            while (pos < snap_len && snap[pos] != '\n') pos++;
            if (pos < snap_len) pos++;  /* include the \n */

            int line_len = pos - line_start;

            /* Apply filter */
            if (filter != FILTER_ALL) {
                bool printer = is_printer_tag(snap + line_start, line_len);
                bool heap = is_heap_line(snap + line_start, line_len);
                if (filter == FILTER_PRINTER && !printer) continue;
                if (filter == FILTER_SYSTEM && printer) continue;
                if (filter == FILTER_HEAP && !heap) continue;
            }

            /* HTML-escape and send: we need to escape <, >, & */
            /* Send in small chunks, escaping as needed */
            char chunk[1024];
            int ci = 0;
            for (int i = line_start; i < line_start + line_len && send_ok; i++) {
                char c = snap[i];
                if (c == '<') {
                    if (ci + 4 > (int)sizeof(chunk)) {
                        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                            send_ok = false;
                        ci = 0;
                    }
                    memcpy(chunk + ci, "&lt;", 4); ci += 4;
                } else if (c == '>') {
                    if (ci + 4 > (int)sizeof(chunk)) {
                        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                            send_ok = false;
                        ci = 0;
                    }
                    memcpy(chunk + ci, "&gt;", 4); ci += 4;
                } else if (c == '&') {
                    if (ci + 5 > (int)sizeof(chunk)) {
                        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                            send_ok = false;
                        ci = 0;
                    }
                    memcpy(chunk + ci, "&amp;", 5); ci += 5;
                } else {
                    if (ci + 1 > (int)sizeof(chunk)) {
                        if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                            send_ok = false;
                        ci = 0;
                    }
                    chunk[ci++] = c;
                }
            }
            if (ci > 0 && send_ok) {
                if (httpd_resp_send_chunk(req, chunk, ci) != ESP_OK)
                    send_ok = false;
            }
        }
    }

    if (send_ok) {
        /* HTML footer */
        html_buf_t foot;
        html_buf_init(&foot);
        html_buf_printf(&foot, "</pre>"
            "<script>var e=document.getElementById('log');e.scrollTop=e.scrollHeight;</script>");
        layout_html_end(&foot);
        httpd_resp_send_chunk(req, foot.data, foot.len);
        html_buf_free(&foot);

        /* End chunked response */
        httpd_resp_send_chunk(req, NULL, 0);
    }

    if (snap) heap_caps_free(snap);

    httpd_req_async_handler_complete(req);
    free(ctx);
    task_delete(NULL);
}

static esp_err_t logs_handler(httpd_req_t *req)
{
    /* Parse filter from query string (before async clone) */
    log_filter_t filter = FILTER_ALL;
    char qbuf[32] = {0};
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(qbuf, "filter", val, sizeof(val)) == ESP_OK) {
            if (strcmp(val, "printer") == 0) filter = FILTER_PRINTER;
            else if (strcmp(val, "system") == 0) filter = FILTER_SYSTEM;
            else if (strcmp(val, "heap") == 0) filter = FILTER_HEAP;
        }
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    logs_ctx_t *ctx = malloc(sizeof(logs_ctx_t));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    ctx->filter = filter;
    ctx->lang = i18n_lang();

    /* 6KB stack: 1KB chunk buf + html_buf + PSRAM snapshot pointer */
    if (task_create_psram(logs_task, "logs", 8192, ctx,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- Clear handler ---- */

static esp_err_t logs_clear_handler(httpd_req_t *req)
{
    if (s_mutex) {
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        ring_reset();
        xSemaphoreGive(s_mutex);
    }
    ESP_LOGI(TAG, "Log buffer cleared");
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/logs");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ---- Registration ---- */

esp_err_t logbuf_register_httpd(httpd_handle_t server)
{
    httpd_uri_t uri_get = {
        .uri = "/logs", .method = HTTP_GET, .handler = logs_handler,
    };
    HTTPD_REGISTER(server, &uri_get);

    httpd_uri_t uri_clear = {
        .uri = "/logs/clear", .method = HTTP_POST, .handler = logs_clear_handler,
    };
    HTTPD_REGISTER(server, &uri_clear);

    ESP_LOGI(TAG, "Log viewer registered at /logs");
    return ESP_OK;
}
