#include "cold_pull.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "httpd.h"
#include "i18n.h"
#include "layout.h"
#include "printer_comm.h"
#include "url_util.h"

static const char *TAG = "cold_pull";

#define PURGE_MM        30      /* Pushed through at the load temperature */
#define PURGE_FEED      120     /* mm/min: slow, so the nozzle fills completely */
#define SAFE_Z          50      /* Room under the nozzle for the purged plastic */
#define CYCLES_MAX      5
#define SCRIPT_SIZE     2048

typedef struct {
    const char *name;
    int load_c, pull_c;
} plastic_t;

/* Starting points: published values differ by up to 20 C, and the right pull
 * temperature is the one where the plug neither snaps nor stretches. */
static const plastic_t PLASTICS[] = {
    { "PLA",   210,  90 },
    { "PETG",  240, 125 },
    { "ABS",   240, 140 },
    { "Nylon", 250, 150 },
};
#define PLASTIC_COUNT (sizeof(PLASTICS) / sizeof(PLASTICS[0]))

/* ---- Script ---- */

typedef struct {
    char *buf;
    size_t size, len;
    bool overflow;
} out_t;

static void out(out_t *o, const char *fmt, ...)
{
    if (o->overflow) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(o->buf + o->len, o->size - o->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= o->size - o->len) {
        o->overflow = true;
        return;
    }
    o->len += n;
}

size_t cold_pull_build(const cold_pull_params_t *p, char *buf, size_t size)
{
    out_t o = { .buf = buf, .size = size };

    out(&o, "; Cold pull: load %d C, pull %d C, %d cycle(s)\n", p->load_c, p->pull_c, p->cycles);
    /* Homed, so that a cancel (or the hold timeout) can park the head */
    out(&o, "G28\nG90\nG1 Z%d F600\nM83\n", SAFE_Z);

    for (int i = 1; i <= p->cycles; i++) {
        out(&o, "; cycle %d of %d\n", i, p->cycles);
        out(&o, "M109 S%d\n", p->load_c);
        if (i > 1) {
            out(&o, "; @pause Cut the tip off, insert the filament, then Continue\n");
        }
        /* M400: heater and fan commands do not wait for moves, so without it
         * the nozzle starts cooling the moment the purge starts */
        out(&o, "G1 E%d F%d\nM400\n", PURGE_MM, PURGE_FEED);
        /* Cool with the part fan, then hold the pull temperature for as long as
         * the user takes to come: the window does not close on them */
        out(&o, "M104 S0\nM106 S255\nM109 R%d\nM104 S%d\nM107\n", p->pull_c, p->pull_c);
        out(&o, "; @pause Nozzle at %d C: pull the filament out now\n", p->pull_c);
    }

    out(&o, "M104 S0\nM107\nM82\n");
    return o.overflow ? 0 : o.len;
}

/* ---- Page ---- */

static esp_err_t page_handler(httpd_req_t *req)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    bool ready = st.opstate == PRINTER_OPERATIONAL && !printer_comm_is_host_printing();

    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, NT_("Cold Pull"), "/utils");
    html_buf_printf(&p, "<h2>%s</h2><p>", _("Cold Pull"));
    html_buf_printf(&p, _("Cleans the nozzle: filament is melted and pushed through, cooled to just "
                          "above solid, and pulled out by hand, taking the dirt with it. "
                          "Load the filament first. The printer homes, lifts the nozzle to Z%d and "
                          "purges %d mm. Each time it needs you it stops, beeps and sends a "
                          "notification; continue on the <a href='/print'>print page</a>."),
                    SAFE_Z, PURGE_MM);
    html_buf_printf(&p,
        "</p><form method='POST' action='/utils/coldpull'>"
        "<table>"
        "<tr><td>%s</td><td><select name='plastic' id='pl' onchange='pick()'>",
        _("Plastic"));
    for (size_t i = 0; i < PLASTIC_COUNT; i++) {
        html_buf_printf(&p, "<option value='%u'>%s</option>", (unsigned)i, PLASTICS[i].name);
    }
    html_buf_printf(&p,
        "</select></td></tr>"
        "<tr><td>%s</td><td><input name='load' id='ld' type='number' min='180' max='300' "
        "value='%d' style='width:70px'> &deg;C</td></tr>"
        "<tr><td>%s</td><td><input name='pull' id='pu' type='number' min='50' max='200' "
        "value='%d' style='width:70px'> &deg;C</td></tr>"
        "<tr><td>%s</td><td><input name='cycles' type='number' min='1' max='%d' "
        "value='2' style='width:70px'></td></tr>"
        "</table>"
        "<p style='color:#888;font-size:13px'>%s</p>"
        "<button type='submit'%s>%s</button>",
        _("Melt at"), PLASTICS[0].load_c, _("Pull at"), PLASTICS[0].pull_c,
        _("Pulls"), CYCLES_MAX,
        _("If the plug snaps, pull a few degrees hotter; if it stretches into a thread, cooler."),
        ready ? "" : " disabled", _("Start"));
    if (!ready) {
        html_buf_printf(&p, " <span style='color:#c62828'>%s</span>",
                        _("The printer must be connected and idle"));
    }
    html_buf_printf(&p, "</form><script>var T=[");
    for (size_t i = 0; i < PLASTIC_COUNT; i++) {
        html_buf_printf(&p, "%s[%d,%d]", i ? "," : "", PLASTICS[i].load_c, PLASTICS[i].pull_c);
    }
    html_buf_printf(&p,
        "];function pick(){var t=T[document.getElementById('pl').value];"
        "document.getElementById('ld').value=t[0];document.getElementById('pu').value=t[1]}"
        "</script>");
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

/** Find name=value in a form body.  Returns false if it is missing. */
static bool form_int(const char *body, const char *name, int *out)
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

static esp_err_t reply_error(httpd_req_t *req, const char *status, const char *msg)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/html");
    char html[512];     /* Cyrillic takes two bytes a letter */
    snprintf(html, sizeof(html),
             "<meta charset='utf-8'><meta name='viewport' content='width=device-width'>"
             "<p>%s</p><p><a href='/utils/coldpull'>%s</a></p>", msg, _("Back"));
    return httpd_resp_sendstr(req, html);
}

static esp_err_t start_handler(httpd_req_t *req)
{
    char body[128];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) return reply_error(req, "400 Bad Request", _("Empty form."));
    body[len] = '\0';

    int plastic;
    cold_pull_params_t prm;
    if (!form_int(body, "plastic", &plastic) || plastic < 0 || plastic >= (int)PLASTIC_COUNT ||
        !form_int(body, "load", &prm.load_c) || !form_int(body, "pull", &prm.pull_c) ||
        !form_int(body, "cycles", &prm.cycles)) {
        return reply_error(req, "400 Bad Request", _("Missing or bad field."));
    }
    if (prm.load_c < 180 || prm.load_c > 300 || prm.pull_c < 50 || prm.pull_c > 200 ||
        prm.load_c - prm.pull_c < 50 || prm.cycles < 1 || prm.cycles > CYCLES_MAX) {
        return reply_error(req, "400 Bad Request",
                           _("Melt 180-300 C, pull 50-200 C and at least 50 C below melt, 1-5 pulls."));
    }

    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate != PRINTER_OPERATIONAL || printer_comm_is_host_printing()) {
        return reply_error(req, "409 Conflict", _("The printer must be connected and idle."));
    }

    char *script = malloc(SCRIPT_SIZE);
    if (!script) return reply_error(req, "500 Internal Server Error", _("Out of memory."));
    if (cold_pull_build(&prm, script, SCRIPT_SIZE) == 0) {
        free(script);
        return reply_error(req, "500 Internal Server Error", _("Script too long."));
    }

    char name[32];
    snprintf(name, sizeof(name), "Cold pull %s", PLASTICS[plastic].name);
    ESP_LOGI(TAG, "%s: %d/%d C, %d cycle(s)", name, prm.load_c, prm.pull_c, prm.cycles);

    /* The script now belongs to the print, whatever happens */
    if (printer_comm_host_print_script(name, script) != ESP_OK) {
        return reply_error(req, "409 Conflict", _("Could not start, try again."));
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/print");
    return httpd_resp_send(req, NULL, 0);
}

esp_err_t cold_pull_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/utils/coldpull", .method = HTTP_GET,  .handler = page_handler },
        { .uri = "/utils/coldpull", .method = HTTP_POST, .handler = start_handler },
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }
    return ESP_OK;
}
