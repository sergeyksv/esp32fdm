#pragma once

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
