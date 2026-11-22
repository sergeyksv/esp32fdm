#pragma once

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

/** URL-decode a string in-place: %XX → byte, '+' → space. */
static inline void url_decode_inplace(char *s)
{
    char *d = s;
    while (*s) {
        if (*s == '%' && s[1] && s[2]) {
            char hex[3] = {s[1], s[2], '\0'};
            *d++ = (char)strtol(hex, NULL, 16);
            s += 3;
        } else if (*s == '+') {
            *d++ = ' ';
            s++;
        } else {
            *d++ = *s++;
        }
    }
    *d = '\0';
}

/** URL-decode from src into dst (max dst_size-1 chars).
 *  Stops at '&' or end of src. */
static inline void url_decode_field(const char *src, char *dst, size_t dst_size)
{
    size_t di = 0;
    while (*src && *src != '&' && di < dst_size - 1) {
        if (*src == '%' && src[1] && src[2]) {
            char hex[3] = {src[1], src[2], '\0'};
            dst[di++] = (char)strtol(hex, NULL, 16);
            src += 3;
        } else if (*src == '+') {
            dst[di++] = ' ';
            src++;
        } else {
            dst[di++] = *src++;
        }
    }
    dst[di] = '\0';
}

/** Find the integer name=value in a form body.  False if it is missing or not a number. */
static inline bool form_int(const char *body, const char *name, int *out)
{
    size_t n = strlen(name);
    for (const char *s = body; s && *s; s = strchr(s, '&'), s = s ? s + 1 : NULL) {
        if (strncmp(s, name, n) == 0 && s[n] == '=') {
            char val[12];
            char *end;
            url_decode_field(s + n + 1, val, sizeof(val));
            long v = strtol(val, &end, 10);
            if (end == val || *end != '\0') return false;
            *out = (int)v;
            return true;
        }
    }
    return false;
}

/** Find the text name=value in a form body, decoded into out.  False if it is missing. */
static inline bool form_str(const char *body, const char *name, char *out, size_t size)
{
    size_t n = strlen(name);
    for (const char *s = body; s && *s; s = strchr(s, '&'), s = s ? s + 1 : NULL) {
        if (strncmp(s, name, n) == 0 && s[n] == '=') {
            url_decode_field(s + n + 1, out, size);
            return true;
        }
    }
    return false;
}
