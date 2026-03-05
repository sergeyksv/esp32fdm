#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdcard.h"
#include "printer_backend.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Growable page buffer ---- */

typedef struct {
    char *data;
    int   len;
    int   cap;
} html_buf_t;

static inline void html_buf_init(html_buf_t *p)
{
    p->cap = 2048;
    p->data = (char *)malloc(p->cap);
    p->len = 0;
    if (p->data) p->data[0] = '\0';
}

static inline void html_buf_printf(html_buf_t *p, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

static inline void html_buf_printf(html_buf_t *p, const char *fmt, ...)
{
    if (!p->data) return;

    va_list ap;
    va_start(ap, fmt);
    int need = vsnprintf(p->data + p->len, p->cap - p->len, fmt, ap);
    va_end(ap);

    if (need >= p->cap - p->len) {
        /* Grow to fit */
        int new_cap = p->cap;
        while (new_cap - p->len <= need)
            new_cap *= 2;
        char *new_data = (char *)realloc(p->data, new_cap);
        if (!new_data) return;
        p->data = new_data;
        p->cap = new_cap;

        va_start(ap, fmt);
        need = vsnprintf(p->data + p->len, p->cap - p->len, fmt, ap);
        va_end(ap);
    }
    p->len += need;
}

static inline void html_buf_free(html_buf_t *p)
{
    free(p->data);
    p->data = NULL;
    p->len = p->cap = 0;
}

/* ---- Shared layout helpers ---- */

/**
 * Emit <!DOCTYPE html> through opening <body>, shared CSS, nav bar, and status bar placeholder.
 * current_path highlights the active nav link (e.g. "/", "/sd", "/camera/config").
 */
static inline void layout_html_begin(html_buf_t *p, const char *title, const char *current_path)
{
    #define NAV_LINK(path, label) \
        (strcmp(current_path, path) == 0 \
            ? "<b>" label "</b>" \
            : "<a href='" path "'>" label "</a>")

    const char *sd_nav = "";
    if (sdcard_is_mounted()) {
        sd_nav = strcmp(current_path, "/sd") == 0
            ? " <b>SD Card</b>"
            : " <a href='/sd'>SD Card</a>";
    }

    int is_klipper = (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER);
    const char *terminal_nav = "";
    if (!is_klipper) {
        terminal_nav = strcmp(current_path, "/terminal") == 0
            ? " <b>Terminal</b>"
            : " <a href='/terminal'>Terminal</a>";
    }

    html_buf_printf(p,
        "<!DOCTYPE html><html><head>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>%s</title>"
        "<style>"
        "body{font-family:sans-serif;max-width:600px;margin:0 auto;padding:0 1em}"
        "nav{background:#333;margin:0 -1em;padding:8px 1em;color:#fff;font-size:14px}"
        "nav a{color:#fff;text-decoration:none;margin:0 6px}"
        "nav b{color:#fff;margin:0 6px}"
        "nav .brand{font-weight:bold;margin-right:12px}"
        ".cpu-bars{display:inline-flex;align-items:center;float:right;gap:4px;font-size:11px;color:#aaa}"
        ".cpu-bar{width:40px;height:10px;background:#555;border-radius:2px;overflow:hidden;display:inline-block;vertical-align:middle}"
        ".cpu-bar>div{height:100%%;background:#4caf50;transition:width 1s}"
        "#pst{padding:8px 0;border-bottom:1px solid #ddd;font-size:14px;color:#666}"
        "#pst.printing{background:#e8f5e9;color:#2e7d32;padding:8px;border-radius:4px;margin:8px 0}"
        "#pst.paused{background:#fff3e0;color:#e65100;padding:8px;border-radius:4px;margin:8px 0}"
        "#pst.error{background:#ffebee;color:#c62828;padding:8px;border-radius:4px;margin:8px 0}"
        ".pbar{background:#ddd;border-radius:4px;height:16px;margin:4px 0;overflow:hidden}"
        ".pbar>div{background:#4caf50;height:100%%;transition:width .3s}"
        "table{width:100%%;border-collapse:collapse}"
        "th,td{text-align:left;padding:6px 8px;border-bottom:1px solid #ddd}"
        "button,input[type=submit]{padding:8px 20px;font-size:15px;cursor:pointer;border-radius:4px}"
        "</style></head><body>"
        "<nav>"
        "<span class='brand'>ESP32FDM</span>"
        "%s %s%s%s %s"
        "<span class='cpu-bars'>"
        "C0<span class='cpu-bar'><div id='cb0'></div></span>"
        "C1<span class='cpu-bar'><div id='cb1'></div></span>"
        "</span>"
        "</nav>"
        "<div id='pst'></div>",
        title,
        NAV_LINK("/", "Home"),
        NAV_LINK("/camera", "Camera"),
        sd_nav,
        terminal_nav,
        NAV_LINK("/settings", "Settings"));

    #undef NAV_LINK
}

/**
 * Emit status-bar polling JS, footer, and closing tags.
 */
static inline void layout_html_end(html_buf_t *p)
{
    html_buf_printf(p,
        "<script>"
        "!function(){var p=document.getElementById('pst');"
        "function u(){fetch('/api/status').then(function(r){return r.json()}).then(function(s){"
        "var a=s.state=='printing'||s.state=='paused';"
        "p.className=a?s.state:s.state=='error'?'error':'';"
        "if(a)p.innerHTML='<b>'+(s.state=='paused'?'PAUSED':'PRINTING')+':</b> '+(s.filename||'')"
        "+'<div class=pbar><div style=\"width:'+Math.min(100,s.progress||0)+'%%\"></div></div>'"
        "+~~s.progress+'%%';"
        "else p.textContent=s.state=='error'?s.backend+': Error':s.usb?s.backend+': Ready':s.backend+': Disconnected';"
        "}).catch(function(){})}"
        "u();setInterval(u,3000);"
        "function cs(){fetch('/api/stats').then(function(r){return r.json()}).then(function(d){"
        "var b0=document.getElementById('cb0'),b1=document.getElementById('cb1');"
        "if(b0)b0.style.width=d.cpu0+'%%';"
        "if(b1)b1.style.width=d.cpu1+'%%';"
        "var c=function(v){return v>80?'#f44336':v>50?'#ff9800':'#4caf50'};"
        "if(b0)b0.style.background=c(d.cpu0);"
        "if(b1)b1.style.background=c(d.cpu1);"
        "}).catch(function(){})}"
        "cs();setInterval(cs,3000)}()"
        "</script>"
        "<footer style='margin-top:32px;padding:12px 0;border-top:1px solid #ddd;text-align:center;font-size:12px;color:#999'>"
        "&copy; Sergey Korotkov &amp; Claude 2026</footer>"
        "</body></html>");
}

#ifdef __cplusplus
}
#endif
