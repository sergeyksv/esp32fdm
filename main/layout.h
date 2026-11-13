#pragma once

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdcard.h"
#include "printer_backend.h"
#include "version.h"

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

/* ---- Shared layout helpers (layout.c) ---- */

/**
 * Emit <!DOCTYPE html> through opening <body>, shared CSS, nav bar, and status bar placeholder.
 * title is translated: mark the literal with NT_().  current_path highlights the
 * active nav link (e.g. "/", "/sd", "/camera/config").
 */
void layout_html_begin(html_buf_t *p, const char *title, const char *current_path);

/**
 * Emit status-bar polling JS, footer, and closing tags.
 */
void layout_html_end(html_buf_t *p);

#ifdef __cplusplus
}
#endif
