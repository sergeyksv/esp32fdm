#pragma once

/*
 * Stops for the user in a file sent to Klipper.  Klipper has no M0/M1, so the
 * board rewrites each stop (M0, M1, "; @pause text") while it uploads the file:
 *
 *     M117 <text>
 *     M400
 *     M25
 *
 * M400 because Klipper reads the file ahead of the moves: without it the
 * board would see the pause, beep and notify while the head still moves.
 * M25 is virtual_sdcard's own pause: no parking, no retract, no gcode state
 * saved, so a Z offset set while waiting is kept, and M24 continues.  The
 * printer's PAUSE/RESUME macros (which park, and on the Neptune 4 purge 100 mm
 * on resume) are not involved.  The board knows a pause is one of these stops
 * because virtual_sdcard reports the file position, which right after the M25
 * is the end offset recorded here.
 *
 * A "; @zoffset" line becomes SET_GCODE_OFFSET Z=<the board's saved offset>:
 * the Neptune 4's screen keeps its Z offset to itself and sets it at power-on,
 * so the board's own (set from its pages) goes into each file it sends.
 * Without a saved offset the line is copied as it is, a comment.
 *
 * Pure string handling with no ESP-IDF dependencies, so it can be tested on a PC.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "gcode_wait.h"

#define KH_LINE_MAX 256     /* Longer lines are copied through unparsed */
#define KH_MSG_MAX  64
#define KH_ZOFFSET_TAG "@zoffset"

typedef struct {
    uint32_t end_pos;           /* File offset right after the M25 line */
    char msg[KH_MSG_MAX];
    bool before_layers;         /* Before the first layer marker of a file that has them */
} klipper_hold_t;

typedef void (*kh_emit_t)(const char *data, size_t len, void *ctx);

typedef struct {
    char line[KH_LINE_MAX];
    size_t len;
    bool long_line;             /* The current line overflowed and is being copied */
    uint32_t out_pos;           /* Bytes of output so far */
    klipper_hold_t *holds;      /* May be NULL */
    int max_holds;
    int n_holds;                /* Recorded in holds */
    int n_seen;                 /* All stops found, recorded or not */
    kh_emit_t emit;             /* NULL to count only */
    void *ctx;
    bool has_z;                 /* A Z offset for "; @zoffset" */
    float z;
    bool seen_layer;            /* A ";LAYER:" or ";LAYER_CHANGE" line so far */
} kh_xform_t;

static inline void kh_init(kh_xform_t *x, klipper_hold_t *holds, int max_holds,
                           kh_emit_t emit, void *ctx)
{
    memset(x, 0, sizeof(*x));
    x->holds = holds;
    x->max_holds = holds ? max_holds : 0;
    x->emit = emit;
    x->ctx = ctx;
}

static inline void kh_out(kh_xform_t *x, const char *data, size_t len)
{
    if (len == 0) return;
    if (x->emit) x->emit(data, len, x->ctx);
    x->out_pos += (uint32_t)len;
}

/** Returns true if the line (no newline) is "; @zoffset" */
static inline bool kh_is_zoffset(const char *line, size_t len)
{
    size_t i = 0;
    while (i < len && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= len || line[i] != ';') return false;
    i++;
    while (i < len && (line[i] == ' ' || line[i] == '\t')) i++;
    size_t tl = strlen(KH_ZOFFSET_TAG);
    if (len - i < tl || strncmp(line + i, KH_ZOFFSET_TAG, tl) != 0) return false;
    for (i += tl; i < len; i++) {
        if (line[i] != ' ' && line[i] != '\t' && line[i] != '\r') return false;
    }
    return true;
}

/** Returns true if the line (no newline) is a stop; fills msg */
/** Whether the line can be a stop at all, from its first characters: "; @..."
 *  or M0/M1 (M00, M01...).  Most lines are not, and are let go without copying. */
static inline bool kh_maybe_stop(const char *s, size_t len)
{
    size_t i = 0;
    while (i < len && (s[i] == ' ' || s[i] == '\t')) i++;
    if (i >= len) return false;
    if (s[i] == ';') {
        i++;
        while (i < len && (s[i] == ' ' || s[i] == '\t')) i++;
        return i < len && s[i] == '@';
    }
    if (s[i] != 'M' && s[i] != 'm') return false;
    i++;
    if (i >= len || s[i] < '0' || s[i] > '9') return false;
    while (i < len && s[i] == '0') i++;
    if (i < len && s[i] == '1') i++;
    return i >= len || s[i] < '0' || s[i] > '9';
}

static inline bool kh_is_stop(const char *line, size_t len, char *msg, size_t msg_size)
{
    char tmp[KH_LINE_MAX];
    if (len >= sizeof(tmp) || !kh_maybe_stop(line, len)) return false;
    memcpy(tmp, line, len);
    tmp[len] = '\0';
    while (len > 0 && (tmp[len - 1] == '\r' || tmp[len - 1] == ' ' || tmp[len - 1] == '\t'))
        tmp[--len] = '\0';
    char *p = tmp;
    while (*p == ' ' || *p == '\t') p++;

    if (gcode_parse_pause_comment(p, msg, msg_size)) return true;

    char *c = strchr(p, ';');
    if (c) {
        *c = '\0';
        size_t n = strlen(p);
        while (n > 0 && (p[n - 1] == ' ' || p[n - 1] == '\t')) p[--n] = '\0';
    }
    return p[0] && gcode_parse_user_wait(p, msg, msg_size);
}

static inline bool kh_is_layer(const char *line, size_t len)
{
    return (len >= 7 && strncmp(line, ";LAYER:", 7) == 0) ||
           (len >= 13 && strncmp(line, ";LAYER_CHANGE", 13) == 0);
}

static inline void kh_line_done(kh_xform_t *x, bool has_newline)
{
    char msg[KH_MSG_MAX];
    if (!x->seen_layer && x->len && x->line[0] == ';' && kh_is_layer(x->line, x->len))
        x->seen_layer = true;
    if (x->has_z && kh_is_zoffset(x->line, x->len)) {
        char cmd[64];
        int n = snprintf(cmd, sizeof(cmd), "SET_GCODE_OFFSET Z=%.3f MOVE=0\n", (double)x->z);
        kh_out(x, cmd, (size_t)n);
    } else if (kh_is_stop(x->line, x->len, msg, sizeof(msg))) {
        /* Klipper cuts a command at ';' */
        for (char *s = msg; *s; s++) if (*s == ';') *s = ',';
        kh_out(x, "M117 ", 5);
        kh_out(x, msg, strlen(msg));
        kh_out(x, "\nM400\nM25\n", 10);
        if (x->n_holds < x->max_holds) {
            klipper_hold_t *h = &x->holds[x->n_holds++];
            h->end_pos = x->out_pos;
            strncpy(h->msg, msg, sizeof(h->msg) - 1);
            h->msg[sizeof(h->msg) - 1] = '\0';
            h->before_layers = !x->seen_layer;  /* Settled in kh_finish() */
        }
        x->n_seen++;
    } else {
        kh_out(x, x->line, x->len);
        if (has_newline) kh_out(x, "\n", 1);
    }
    x->len = 0;
}

/** Feed the next block of the file */
static inline void kh_feed(kh_xform_t *x, const char *buf, size_t n)
{
    while (n > 0) {
        const char *nl = (const char *)memchr(buf, '\n', n);
        if (x->long_line) {
            size_t k = nl ? (size_t)(nl - buf) + 1 : n;
            kh_out(x, buf, k);
            if (nl) x->long_line = false;
            buf += k;
            n -= k;
            continue;
        }
        size_t k = nl ? (size_t)(nl - buf) : n;
        if (x->len + k >= KH_LINE_MAX) {
            /* Too long to be a stop: copy what we have, the rest follows */
            kh_out(x, x->line, x->len);
            x->len = 0;
            x->long_line = true;
            continue;
        }
        memcpy(x->line + x->len, buf, k);
        x->len += k;
        buf += k;
        n -= k;
        if (nl) {
            kh_line_done(x, true);
            buf++;
            n--;
        }
    }
}

/** The last line, if the file does not end with a newline */
static inline void kh_finish(kh_xform_t *x)
{
    if (!x->long_line && x->len > 0) kh_line_done(x, false);
    x->len = 0;
    x->long_line = false;
    /* A file without layer markers: where its stops are is not known */
    if (!x->seen_layer)
        for (int i = 0; i < x->n_holds; i++) x->holds[i].before_layers = false;
}
