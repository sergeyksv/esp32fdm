#pragma once

/*
 * Translation of the web interface, gettext style: the English text in the code
 * is the key, and i18n/<lang>.po holds the translations.  tools/i18n.py turns
 * the .po files into tables at build time (i18n_tables.c), so the board needs
 * no libintl.  A missing translation shows the English text.
 *
 *   _("text")       translated text, for a string literal (cached per call site)
 *   TR(ptr)         translated text, for a pointer to a literal marked with NT_()
 *   NT_("text")     only marks text for extraction (gettext_noop)
 *   N_("text")      marks text inside a page template; the page goes through
 *                   i18n_page(), which translates it whole and caches the result
 *
 * The language is per task: i18n_set_from_req() sets it from the "lang" cookie
 * or the Accept-Language header, and HTTPD_REGISTER does that for every handler.
 *
 * After changing marked text: python3 tools/i18n.py update, then translate
 * the empty entries in i18n/<lang>.po.
 */

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"

#define I18N_MAX_LANGS  8           /* English plus up to 7 translations */
#define I18N_MARK       "\x1F"      /* Brackets text to translate inside a template */

#define NT_(s)  s
#define N_(s)   I18N_MARK s I18N_MARK
#define TR(s)   i18n_lookup((s), i18n_lang())

#define _(s) (__extension__({                                               \
    static const char *_i18n_cache[I18N_MAX_LANGS];                         \
    int _i18n_l = i18n_lang();                                              \
    const char *_i18n_r = (s);                                              \
    if (_i18n_l > 0) {                                                      \
        if (!_i18n_cache[_i18n_l])                                          \
            _i18n_cache[_i18n_l] = i18n_lookup(_i18n_r, _i18n_l);           \
        _i18n_r = _i18n_cache[_i18n_l];                                     \
    }                                                                       \
    _i18n_r;                                                                \
}))

/* ---- Tables, generated from i18n/<lang>.po ---- */

typedef struct {
    uint32_t hash;          /* i18n_hash(msgid) */
    const char *msgid;
    const char *msgstr;
} i18n_entry_t;

typedef struct {
    const char *code;       /* "ru" */
    const char *name;       /* "Русский" */
    const i18n_entry_t *entries;    /* Sorted by hash */
    size_t count;
} i18n_table_t;

extern const i18n_table_t i18n_tables[];   /* Index 0 is language 1: English has no table */
extern const int i18n_table_count;

/* ---- Lookup ---- */

/** FNV-1a, the same as in tools/i18n.py */
uint32_t i18n_hash(const char *s, size_t len);

/** Language of the calling task: 0 is English */
int i18n_lang(void);
void i18n_set_lang(int lang);

/** Number of languages, English included, and their codes and names */
int i18n_lang_count(void);
const char *i18n_lang_code(int lang);
const char *i18n_lang_name(int lang);

/** The translation of msgid, or msgid itself */
const char *i18n_lookup(const char *msgid, int lang);

/** The template with every N_() text translated into the task's language and
 *  the marks removed.  Built once per template and language, then kept. */
const char *i18n_page(const char *tmpl);

/* ---- HTTP ---- */

/** Pick the language for this request: the "lang" cookie, else Accept-Language */
void i18n_set_from_req(httpd_req_t *req);

/** Handler wrapper used by HTTPD_REGISTER: user_ctx holds the real handler */
esp_err_t i18n_handler_wrap(httpd_req_t *req);

/** Links to switch to the other languages, for page footers and menus */
void i18n_render_switch(char *buf, size_t size);
