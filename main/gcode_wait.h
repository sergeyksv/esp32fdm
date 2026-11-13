#pragma once

/*
 * Parsing of "wait for user" stops in a host print: the M0/M1 commands and the
 * "; @pause" comment.
 * Pure string handling with no ESP-IDF dependencies, so it can be tested on a PC.
 */

#include <ctype.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#define GCODE_WAIT_DEFAULT_MSG "Waiting for user"
#define GCODE_PAUSE_TAG        "@pause"

/**
 * Copy text into a fixed buffer so it is safe to embed in JSON and HTML:
 * quotes, backslashes, angle brackets, ampersands and control characters become
 * spaces.  UTF-8 is preserved and never cut in the middle of a character.
 */
static inline void gcode_wait_copy_msg(const char *src, char *dst, size_t dst_size)
{
    size_t n = 0;
    while (*src && n < dst_size - 1) {
        unsigned char c = (unsigned char)*src++;
        bool unsafe = c < 0x20 || c == 0x7f || c == '"' || c == '\\' ||
                      c == '<' || c == '>' || c == '&';
        dst[n++] = unsafe ? ' ' : (char)c;
    }

    /* Input was cut: drop a trailing multi-byte character if it is incomplete. */
    if (*src && n > 0) {
        size_t start = n;
        while (start > 0 && ((unsigned char)dst[start - 1] & 0xC0) == 0x80) start--;
        if (start > 0 && ((unsigned char)dst[start - 1] & 0x80)) {
            unsigned char lead = (unsigned char)dst[start - 1];
            size_t need = (lead & 0xE0) == 0xC0 ? 2 : (lead & 0xF0) == 0xE0 ? 3 : 4;
            if (n - (start - 1) < need) n = start - 1;
        }
    }

    while (n > 0 && dst[n - 1] == ' ') n--;
    dst[n] = '\0';
}

/**
 * Returns true if the line (already stripped of comments) is M0 or M1.
 * Fills msg with the text after the command, or a default if there is none.
 * The S and P time parameters are ignored: the print always waits for the user.
 */
static inline bool gcode_parse_user_wait(const char *line, char *msg, size_t msg_size)
{
    if (line[0] != 'M' && line[0] != 'm') return false;

    const char *p = line + 1;
    if (!isdigit((unsigned char)*p)) return false;
    while (*p == '0') p++;
    if (*p == '1') p++;
    if (isdigit((unsigned char)*p) || *p == '.') return false;

    /* Skip S<number> and P<number> parameters */
    while (true) {
        while (*p == ' ' || *p == '\t') p++;
        if (*p != 'S' && *p != 's' && *p != 'P' && *p != 'p') break;
        const char *q = p + 1;
        if (!isdigit((unsigned char)*q)) break;
        while (isdigit((unsigned char)*q) || *q == '.') q++;
        if (*q && *q != ' ' && *q != '\t') break;
        p = q;
    }

    gcode_wait_copy_msg(p, msg, msg_size);
    if (msg[0] == '\0') {
        strncpy(msg, GCODE_WAIT_DEFAULT_MSG, msg_size - 1);
        msg[msg_size - 1] = '\0';
    }
    return true;
}

/**
 * Returns true if the whole line is a "; @pause [text]" comment.  It is a comment
 * so that a start file shared with other printers stays harmless there: their
 * firmware ignores it, while M0 could block them waiting for a button.
 * The tag must open the comment, so settings the slicer dumps as comments at the
 * end of a file (one line per setting, starting with its name) do not match.
 * Fills msg with the text after the tag, or a default if there is none.
 */
static inline bool gcode_parse_pause_comment(const char *line, char *msg, size_t msg_size)
{
    const size_t tag_len = strlen(GCODE_PAUSE_TAG);

    if (line[0] != ';') return false;
    const char *p = line + 1;
    while (*p == ' ' || *p == '\t') p++;
    if (strncmp(p, GCODE_PAUSE_TAG, tag_len) != 0) return false;
    p += tag_len;
    if (*p && *p != ' ' && *p != '\t') return false;

    /* Skip separators, so that "; @pause - text" reads well.  "\xE2\x80\x94" is an em dash. */
    while (true) {
        if (*p == ' ' || *p == '\t' || *p == '-' || *p == ':') p++;
        else if (strncmp(p, "\xE2\x80\x94", 3) == 0) p += 3;
        else break;
    }

    gcode_wait_copy_msg(p, msg, msg_size);
    if (msg[0] == '\0') {
        strncpy(msg, GCODE_WAIT_DEFAULT_MSG, msg_size - 1);
        msg[msg_size - 1] = '\0';
    }
    return true;
}
