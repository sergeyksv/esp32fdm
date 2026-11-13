#include "i18n.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "i18n";

#define PAGE_CACHE_MAX  256     /* Templates (about 35) times languages actually used */
#define LANG_COOKIE     "lang"

/* Per task: each request sets it before building a page */
static __thread uint8_t s_lang;

/* ---- Languages ---- */

int i18n_lang(void)
{
    return s_lang;
}

void i18n_set_lang(int lang)
{
    s_lang = (lang > 0 && lang < i18n_lang_count()) ? lang : 0;
}

int i18n_lang_count(void)
{
    int n = 1 + i18n_table_count;
    return n < I18N_MAX_LANGS ? n : I18N_MAX_LANGS;
}

const char *i18n_lang_code(int lang)
{
    return lang > 0 && lang < i18n_lang_count() ? i18n_tables[lang - 1].code : "en";
}

const char *i18n_lang_name(int lang)
{
    return lang > 0 && lang < i18n_lang_count() ? i18n_tables[lang - 1].name : "English";
}

static int lang_by_code(const char *code, size_t len)
{
    for (int i = 0; i < i18n_lang_count(); i++) {
        const char *c = i18n_lang_code(i);
        if (strlen(c) == len && strncasecmp(c, code, len) == 0) return i;
    }
    return -1;
}

/* ---- Lookup ---- */

uint32_t i18n_hash(const char *s, size_t len)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= (uint8_t)s[i];
        h *= 16777619u;
    }
    return h;
}

/** Translation of the len bytes at s, or NULL if there is none */
static const char *lookup_n(const char *s, size_t len, int lang)
{
    if (lang <= 0 || lang >= i18n_lang_count()) return NULL;
    const i18n_table_t *t = &i18n_tables[lang - 1];
    uint32_t h = i18n_hash(s, len);

    size_t lo = 0, hi = t->count;
    while (lo < hi) {
        size_t mid = (lo + hi) / 2;
        if (t->entries[mid].hash < h) lo = mid + 1;
        else hi = mid;
    }
    for (size_t i = lo; i < t->count && t->entries[i].hash == h; i++) {
        const char *id = t->entries[i].msgid;
        if (strncmp(id, s, len) == 0 && id[len] == '\0') return t->entries[i].msgstr;
    }
    return NULL;
}

const char *i18n_lookup(const char *msgid, int lang)
{
    const char *tr = lookup_n(msgid, strlen(msgid), lang);
    return tr ? tr : msgid;
}

/* ---- Page templates ---- */

typedef struct {
    const char *tmpl;
    uint8_t lang;
    const char *out;
} page_entry_t;

static page_entry_t *s_pages;       /* PAGE_CACHE_MAX entries in PSRAM, allocated on first use */
static int s_page_count;

static SemaphoreHandle_t page_lock(void)
{
    static StaticSemaphore_t buf;
    static SemaphoreHandle_t mutex;
    static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&mux);
    if (!mutex) mutex = xSemaphoreCreateMutexStatic(&buf);
    taskEXIT_CRITICAL(&mux);
    return mutex;
}

/** Expand tmpl into out (or only measure it when out is NULL).  Returns the length. */
static size_t expand(const char *tmpl, int lang, char *out)
{
    size_t n = 0;
    const char *p = tmpl;
    while (*p) {
        const char *open = strchr(p, I18N_MARK[0]);
        const char *close = open ? strchr(open + 1, I18N_MARK[0]) : NULL;
        if (!close) {
            size_t len = strlen(p);
            if (out) memcpy(out + n, p, len);
            n += len;
            break;
        }
        size_t before = open - p;
        if (out) memcpy(out + n, p, before);
        n += before;

        const char *text = open + 1;
        size_t text_len = close - text;
        const char *tr = lookup_n(text, text_len, lang);
        size_t tr_len = tr ? strlen(tr) : text_len;
        if (out) memcpy(out + n, tr ? tr : text, tr_len);
        n += tr_len;

        p = close + 1;
    }
    if (out) out[n] = '\0';
    return n;
}

const char *i18n_page(const char *tmpl)
{
    int lang = i18n_lang();
    SemaphoreHandle_t lock = page_lock();
    xSemaphoreTake(lock, portMAX_DELAY);

    for (int i = 0; i < s_page_count; i++) {
        if (s_pages[i].tmpl == tmpl && s_pages[i].lang == lang) {
            const char *out = s_pages[i].out;
            xSemaphoreGive(lock);
            return out;
        }
    }

    if (!s_pages) {
        s_pages = heap_caps_calloc(PAGE_CACHE_MAX, sizeof(*s_pages), MALLOC_CAP_SPIRAM);
        if (!s_pages) s_pages = calloc(PAGE_CACHE_MAX, sizeof(*s_pages));
    }

    size_t len = expand(tmpl, lang, NULL);
    char *out = heap_caps_malloc(len + 1, MALLOC_CAP_SPIRAM);
    if (!out) out = malloc(len + 1);
    if (!out) {
        xSemaphoreGive(lock);
        ESP_LOGE(TAG, "No memory for a %u byte page", (unsigned)len);
        return "";
    }
    expand(tmpl, lang, out);

    if (s_pages && s_page_count < PAGE_CACHE_MAX) {
        s_pages[s_page_count++] = (page_entry_t){ .tmpl = tmpl, .lang = lang, .out = out };
    } else {
        /* Built again on every request and never freed, since the caller may
         * still be using an earlier copy: a leak, so it must not happen */
        ESP_LOGE(TAG, "Page cache full, raise PAGE_CACHE_MAX");
    }
    xSemaphoreGive(lock);
    return out;
}

/* ---- HTTP ---- */

void i18n_set_from_req(httpd_req_t *req)
{
    char val[16];
    size_t len = sizeof(val);
    if (httpd_req_get_cookie_val(req, LANG_COOKIE, val, &len) == ESP_OK) {
        int lang = lang_by_code(val, strlen(val));
        if (lang >= 0) {
            i18n_set_lang(lang);
            return;
        }
    }

    /* "ru-RU,ru;q=0.9,en-US;q=0.8": browsers list them most wanted first */
    char hdr[128];
    if (httpd_req_get_hdr_value_str(req, "Accept-Language", hdr, sizeof(hdr)) == ESP_OK) {
        for (char *save, *tok = strtok_r(hdr, ",", &save); tok; tok = strtok_r(NULL, ",", &save)) {
            while (*tok == ' ') tok++;
            size_t code_len = strcspn(tok, "-;_ ");
            int lang = lang_by_code(tok, code_len);
            if (lang >= 0) {
                i18n_set_lang(lang);
                return;
            }
        }
    }
    i18n_set_lang(0);
}

esp_err_t i18n_handler_wrap(httpd_req_t *req)
{
    esp_err_t (*handler)(httpd_req_t *) = (esp_err_t (*)(httpd_req_t *))req->user_ctx;
    i18n_set_from_req(req);
    return handler(req);
}

void i18n_render_switch(char *buf, size_t size)
{
    size_t n = 0;
    buf[0] = '\0';
    for (int i = 0; i < i18n_lang_count() && n < size; i++) {
        if (i == i18n_lang()) continue;
        int w = snprintf(buf + n, size - n,
                         "%s<a href='#' lang='%s' onclick=\"document.cookie='" LANG_COOKIE
                         "=%s;path=/;max-age=31536000';location.reload();return false\">%s</a>",
                         n ? " " : "", i18n_lang_code(i), i18n_lang_code(i), i18n_lang_name(i));
        if (w < 0) break;
        n += w;
    }
}
