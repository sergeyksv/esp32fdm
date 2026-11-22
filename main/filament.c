#include "filament.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "httpd.h"
#include "i18n.h"
#include "layout.h"
#include "plastics.h"
#include "printer_comm.h"
#include "url_util.h"

static const char *TAG = "filament";

#define SAFE_Z          50      /* Room under the nozzle for the purged plastic */
#define PURGE_DEFAULT   100     /* mm: the cut end, the melt zone and some more */
#define PURGE_MIN       20
#define PURGE_MAX       300
#define PURGE_FEED      150     /* mm/min: what a 0.4 mm nozzle melts at any of these temperatures */
#define TEMP_MIN        180
#define TEMP_MAX        300
#define SCRIPT_SIZE     1024

/* ---- Script ---- */

size_t filament_build(const filament_params_t *p, char *buf, size_t size)
{
    int n = snprintf(buf, size,
        "; Filament change: %s -> %s, purge %d mm at %d C\n"
        /* Homed cold: the Kobra 2 homes Z on the nozzle, and molten plastic on
         * it fails the homing.  Homed also lets a cancel park the head. */
        "G28\nG90\nG1 Z%d F600\nM83\n"
        "M109 S%d\n"
        /* Not before: the user may still be pushing the new filament in */
        "; @pause Nozzle at %d C. New filament in? Continue to purge\n"
        /* M400: the heater goes off when M104 is read, not after the purge */
        "G1 E%d F%d\nM400\n"
        "M104 S0\nM82\n",
        p->from ? p->from : "unknown", p->to, p->purge_mm, p->temp_c,
        SAFE_Z, p->temp_c, p->temp_c, p->purge_mm, PURGE_FEED);
    return n > 0 && (size_t)n < size ? (size_t)n : 0;
}

/* ---- Page ---- */

static void plastic_options(html_buf_t *p, int selected, bool with_unknown)
{
    if (with_unknown) {
        html_buf_printf(p, "<option value='-1'%s>%s</option>",
                        selected < 0 ? " selected" : "", _("Unknown"));
    }
    for (int i = 0; i < PLASTIC_COUNT; i++) {
        html_buf_printf(p, "<option value='%d'%s>%s</option>", i,
                        i == selected ? " selected" : "", PLASTICS[i].name);
    }
}

static esp_err_t page_handler(httpd_req_t *req)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    bool ready = st.opstate == PRINTER_OPERATIONAL && !printer_comm_is_host_printing();
    int loaded = plastic_loaded();
    int to = loaded >= 0 ? loaded : 0;
    int hot = plastic_hotter(loaded, to);

    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, NT_("Filament Change"), "/utils");
    html_buf_printf(&p, "<h2>%s</h2><p>", _("Filament Change"));
    html_buf_printf(&p, _("No unloading: cut the old filament just above the extruder and push the "
                          "new one in until the gears take it, then Start. The printer homes, lifts "
                          "the nozzle to Z%d and heats to the hotter of the two plastics, so that "
                          "the old one comes out too. When it is hot it stops and sends a "
                          "notification; continue on the <a href='/print'>print page</a> and the "
                          "new filament pushes the old one out."),
                    SAFE_Z);
    html_buf_printf(&p,
        "</p><form method='POST' action='/utils/filament'><table>"
        "<tr><td>%s</td><td><select name='from' id='fr' onchange='upd()'>",
        _("In the nozzle now"));
    plastic_options(&p, loaded, true);
    html_buf_printf(&p,
        "</select></td></tr><tr><td>%s</td><td><select name='to' id='to' onchange='upd()'>",
        _("New filament"));
    plastic_options(&p, to, false);
    html_buf_printf(&p,
        "</select></td></tr>"
        "<tr><td>%s</td><td><input name='temp' id='tm' type='number' min='%d' max='%d' "
        "value='%d' style='width:70px'> &deg;C</td></tr>"
        "<tr><td>%s</td><td><input name='purge' type='number' min='%d' max='%d' "
        "value='%d' style='width:70px'> %s</td></tr>"
        "</table>"
        "<p class='hint'>%s</p>"
        "<button type='submit'%s>%s</button>",
        _("Purge at"), TEMP_MIN, TEMP_MAX, PLASTICS[hot].print_c,
        _("Purge"), PURGE_MIN, PURGE_MAX, PURGE_DEFAULT, _("mm"),
        _("The board remembers the plastic it loads. If the colour is not clean yet, run it again."),
        ready ? "" : " disabled", _("Start"));
    if (!ready) {
        html_buf_printf(&p, " <span style='color:#c62828'>%s</span>",
                        _("The printer must be connected and idle"));
    }
    html_buf_printf(&p, "</form><script>var T=[");
    for (int i = 0; i < PLASTIC_COUNT; i++) {
        html_buf_printf(&p, "%s%d", i ? "," : "", PLASTICS[i].print_c);
    }
    html_buf_printf(&p,
        "];function upd(){var f=+document.getElementById('fr').value,"
        "t=+document.getElementById('to').value;"
        "document.getElementById('tm').value=Math.max(f<0?0:T[f],T[t])}"
        "</script>");
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t reply_error(httpd_req_t *req, const char *status, const char *msg)
{
    httpd_resp_set_status(req, status);
    httpd_resp_set_type(req, "text/html");
    char html[512];     /* Cyrillic takes two bytes a letter */
    snprintf(html, sizeof(html),
             "<meta charset='utf-8'><meta name='viewport' content='width=device-width'>"
             "<p>%s</p><p><a href='/utils/filament'>%s</a></p>", msg, _("Back"));
    return httpd_resp_sendstr(req, html);
}

static esp_err_t start_handler(httpd_req_t *req)
{
    char body[128];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) return reply_error(req, "400 Bad Request", _("Empty form."));
    body[len] = '\0';

    int from, to;
    filament_params_t prm;
    if (!form_int(body, "from", &from) || from < -1 || from >= PLASTIC_COUNT ||
        !form_int(body, "to", &to) || to < 0 || to >= PLASTIC_COUNT ||
        !form_int(body, "temp", &prm.temp_c) || !form_int(body, "purge", &prm.purge_mm)) {
        return reply_error(req, "400 Bad Request", _("Missing or bad field."));
    }
    if (prm.temp_c < TEMP_MIN || prm.temp_c > TEMP_MAX ||
        prm.purge_mm < PURGE_MIN || prm.purge_mm > PURGE_MAX) {
        return reply_error(req, "400 Bad Request", _("Temperature 180-300 C, purge 20-300 mm."));
    }

    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate != PRINTER_OPERATIONAL || printer_comm_is_host_printing()) {
        return reply_error(req, "409 Conflict", _("The printer must be connected and idle."));
    }

    /* The user knows better than the memory: a file printed in between, a
     * change made on the printer's own screen */
    if (from != plastic_loaded()) plastic_set_loaded(from);

    prm.from = from >= 0 ? PLASTICS[from].name : NULL;
    prm.to = PLASTICS[to].name;
    char *script = malloc(SCRIPT_SIZE);
    if (!script) return reply_error(req, "500 Internal Server Error", _("Out of memory."));
    if (filament_build(&prm, script, SCRIPT_SIZE) == 0) {
        free(script);
        return reply_error(req, "500 Internal Server Error", _("Script too long."));
    }

    char name[32];
    snprintf(name, sizeof(name), "Load %s", PLASTICS[to].name);
    ESP_LOGI(TAG, "%s over %s: %d C, purge %d mm", name, prm.from ? prm.from : "unknown",
             prm.temp_c, prm.purge_mm);

    /* The script now belongs to the print, whatever happens */
    if (printer_comm_host_print_script(name, script, plastic_script_done, to) != ESP_OK) {
        return reply_error(req, "409 Conflict", _("Could not start, try again."));
    }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/print");
    return httpd_resp_send(req, NULL, 0);
}

esp_err_t filament_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/utils/filament", .method = HTTP_GET,  .handler = page_handler },
        { .uri = "/utils/filament", .method = HTTP_POST, .handler = start_handler },
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }
    return ESP_OK;
}
