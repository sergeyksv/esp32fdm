#include "settings_copy.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bedlevel.h"
#include "board_led.h"
#include "nozzle_guard_ui.h"
#include "cJSON.h"
#include "camera.h"
#include "esp_log.h"
#include "httpd.h"
#include "hub_client.h"
#include "i18n.h"
#include "maintenance.h"
#include "nvs.h"
#include "obico_client.h"
#include "printer_backend.h"
#include "printer_comm.h"
#include "url_util.h"
#include "version.h"
#include "wifi.h"
#include "wifi_page.h"

static const char *TAG = "settings_copy";

#define FORMAT          1
#define NVS_NAMESPACE   "device"
#define NVS_KEY_MODEL   "model"
#define MODEL_LEN       32
#define BODY_MAX        8192

static char s_model[MODEL_LEN];

const char *settings_copy_model(void)
{
    return s_model;
}

/** The model goes into HTML and JSON: keep what cannot break either */
static void clean_text(char *s)
{
    char *d = s;
    for (const char *p = s; *p; p++) {
        unsigned char c = (unsigned char)*p;
        if (c < 0x20 || strchr("<>&\"'\\", c)) continue;
        *d++ = (char)c;
    }
    *d = '\0';
}

static void set_model(const char *model)
{
    char m[MODEL_LEN];
    strlcpy(m, model, sizeof(m));
    clean_text(m);
    strlcpy(s_model, m, sizeof(s_model));
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    nvs_set_str(nvs, NVS_KEY_MODEL, s_model);
    nvs_commit(nvs);
    nvs_close(nvs);
}

void settings_copy_init(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    size_t len = sizeof(s_model);
    if (nvs_get_str(nvs, NVS_KEY_MODEL, s_model, &len) != ESP_OK) s_model[0] = '\0';
    nvs_close(nvs);
}

/* ---- Sections ---- */

static bool is_marlin(void)
{
    return printer_comm_get_backend() != PRINTER_BACKEND_KLIPPER;
}

static void camera_export(cJSON *o)
{
    int b, c, s, ae;
    camera_get_image_params(&b, &c, &s, &ae);
    cJSON_AddBoolToObject(o, "rotate180", camera_get_rotate180());
    cJSON_AddNumberToObject(o, "brightness", b);
    cJSON_AddNumberToObject(o, "contrast", c);
    cJSON_AddNumberToObject(o, "saturation", s);
    cJSON_AddNumberToObject(o, "ae_level", ae);
}

static int clamp_item(const cJSON *o, const char *key, int lo, int hi, int dflt)
{
    const cJSON *v = cJSON_GetObjectItem(o, key);
    if (!cJSON_IsNumber(v)) return dflt;
    return v->valueint < lo ? lo : v->valueint > hi ? hi : v->valueint;
}

static void camera_import(const cJSON *o)
{
    const cJSON *v = cJSON_GetObjectItem(o, "rotate180");
    if (cJSON_IsBool(v)) camera_set_rotate180(cJSON_IsTrue(v));
    int b, c, s, ae;
    camera_get_image_params(&b, &c, &s, &ae);
    camera_set_image_params(clamp_item(o, "brightness", -2, 2, b), clamp_item(o, "contrast", -2, 2, c),
                            clamp_item(o, "saturation", -2, 2, s), clamp_item(o, "ae_level", -2, 2, ae));
}

static void hub_export(cJSON *o)
{
    cJSON_AddStringToObject(o, "url", hub_client_get_url());
}

static void hub_import(const cJSON *o)
{
    const cJSON *v = cJSON_GetObjectItem(o, "url");
    if (cJSON_IsString(v) && hub_client_set_url(v->valuestring)) {
        ESP_LOGW(TAG, "Hub URL not taken: %s", v->valuestring);
    }
}

/* The server and the WebRTC proxy, never the link token: two boards with one
 * token would be one printer to Obico */
static void obico_export(cJSON *o)
{
    cJSON_AddStringToObject(o, "server_url", obico_get_server_url());
    cJSON_AddStringToObject(o, "janus_host", obico_get_janus_host());
    cJSON_AddNumberToObject(o, "janus_port", obico_get_janus_port());
}

static void obico_import(const cJSON *o)
{
    const cJSON *v = cJSON_GetObjectItem(o, "server_url");
    if (cJSON_IsString(v) && v->valuestring[0]) obico_set_server_url(v->valuestring);
    const cJSON *h = cJSON_GetObjectItem(o, "janus_host");
    if (cJSON_IsString(h)) {
        obico_set_janus_proxy(h->valuestring, clamp_item(o, "janus_port", 0, 65535, obico_get_janus_port()));
    }
}

static void model_export(cJSON *o)
{
    cJSON_AddStringToObject(o, "model", s_model);
}

static void model_import(const cJSON *o)
{
    const cJSON *v = cJSON_GetObjectItem(o, "model");
    if (cJSON_IsString(v)) set_model(v->valuestring);
}

typedef struct {
    const char *key;
    void (*export)(cJSON *);
    void (*import)(const cJSON *);
    bool marlin_only;
} section_t;

static const section_t SECTIONS[] = {
    { "model",       model_export,                 model_import,                 false },
    /* Also with Klipper: the beep while waiting is here, and the screw layout */
    { "marlin",      printer_comm_settings_export, printer_comm_settings_import, false },
    { "bed",         bedlevel_settings_export,     bedlevel_settings_import,     false },
    { "maintenance", maintenance_settings_export,  maintenance_settings_import,  false },
    { "hub",         hub_export,                   hub_import,                   false },
    { "camera",      camera_export,                camera_import,                false },
    /* Which light goes on for a print: the same for every printer of a model */
    { "light",       board_led_settings_export,    board_led_settings_import,    false },
    { "nozzle",      nozzle_guard_settings_export, nozzle_guard_settings_import, false },
    { "obico",       obico_export,                 obico_import,                 false },
    { "wifi_names",  wifi_page_settings_export,    wifi_page_settings_import,    false },
};
#define SECTION_COUNT (sizeof(SECTIONS) / sizeof(SECTIONS[0]))

/* ---- HTTP ---- */

static esp_err_t export_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *sections = root ? cJSON_AddObjectToObject(root, "sections") : NULL;
    if (!sections) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Out of memory."));
        return ESP_FAIL;
    }
    cJSON_AddNumberToObject(root, "format", FORMAT);
    cJSON_AddStringToObject(root, "board", wifi_get_hostname());
    cJSON_AddStringToObject(root, "model", s_model);
    cJSON_AddStringToObject(root, "fw", FW_VERSION);
    for (size_t i = 0; i < SECTION_COUNT; i++) {
        if (SECTIONS[i].marlin_only && !is_marlin()) continue;
        cJSON *o = cJSON_AddObjectToObject(sections, SECTIONS[i].key);
        if (o) SECTIONS[i].export(o);
    }
    char *json = cJSON_Print(root);
    cJSON_Delete(root);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Out of memory."));
        return ESP_FAIL;
    }

    char query[32], val[4];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "download", val, sizeof(val)) == ESP_OK) {
        char disp[80];
        snprintf(disp, sizeof(disp), "attachment; filename=\"%s-settings.json\"", wifi_get_hostname());
        httpd_resp_set_hdr(req, "Content-Disposition", disp);
    }
    httpd_resp_set_type(req, "application/json");
    /* Read by another board's settings page, from the browser */
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t ret = httpd_resp_sendstr(req, json);
    cJSON_free(json);
    return ret;
}

static esp_err_t import_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len >= BODY_MAX) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, _("Missing or bad field."));
        return ESP_FAIL;
    }
    char *body = malloc(req->content_len + 1);
    if (!body) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Out of memory."));
        return ESP_FAIL;
    }
    size_t got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, body + got, req->content_len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) {
            free(body);
            return ESP_FAIL;
        }
        got += n;
    }
    body[got] = '\0';

    cJSON *root = cJSON_Parse(body);
    free(body);
    const cJSON *sections = cJSON_GetObjectItem(root, "sections");
    if (!cJSON_IsObject(sections)) {
        cJSON_Delete(root);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, _("Not a settings file."));
        return ESP_FAIL;
    }

    char applied[160] = "";
    size_t n = 0;
    for (size_t i = 0; i < SECTION_COUNT; i++) {
        const cJSON *o = cJSON_GetObjectItem(sections, SECTIONS[i].key);
        if (!cJSON_IsObject(o)) continue;
        if (SECTIONS[i].marlin_only && !is_marlin()) continue;
        SECTIONS[i].import(o);
        int w = snprintf(applied + n, sizeof(applied) - n, "%s\"%s\"", n ? "," : "", SECTIONS[i].key);
        if (w > 0 && (size_t)w < sizeof(applied) - n) n += w;
    }
    cJSON_Delete(root);
    ESP_LOGI(TAG, "Settings applied: %s", applied);

    char json[200];
    snprintf(json, sizeof(json), "{\"applied\":[%s]}", applied);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t model_handler(httpd_req_t *req)
{
    char body[128];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, _("Empty form."));
        return ESP_FAIL;
    }
    body[len] = '\0';
    char model[MODEL_LEN] = "";
    form_str(body, "model", model, sizeof(model));
    set_model(model);
    ESP_LOGI(TAG, "Model: %s", s_model);

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/settings");
    return httpd_resp_send(req, NULL, 0);
}

esp_err_t settings_copy_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/api/settings",   .method = HTTP_GET,  .handler = export_handler },
        { .uri = "/api/settings",   .method = HTTP_POST, .handler = import_handler },
        { .uri = "/settings/model", .method = HTTP_POST, .handler = model_handler },
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }
    return ESP_OK;
}

/* ---- Settings page ---- */

/* For i18n_page(), then printf: model, this board's name, hub URL.
 * No % anywhere else in it. */
static const char COPY_TMPL[] =
    "<hr><div class='sh'><h2>" N_("Copy Settings") "</h2></div>"
    "<form method='POST' action='/settings/model' style='display:flex;gap:8px;align-items:flex-end'>"
    "<label style='flex:1'>" N_("Printer model") "<br>"
    "<input name='model' value='%s' maxlength='31' placeholder='Kobra 2' "
    "style='padding:8px;width:100%%;box-sizing:border-box;margin:4px 0'></label>"
    "<button type='submit' style='margin-bottom:4px'>" N_("Save") "</button></form>"
    "<p class='hint'>" N_("Copies everything except the name, Wi-Fi, the Obico link, the backend, the camera "
                          "clock, counters and maintenance marks. Take the bed and the maintenance list from "
                          "a printer of the same model.") "</p>"
    "<div style='display:flex;gap:8px'>"
    "<input id='cs-host' list='cs-boards' placeholder='fdm06.lan' autocapitalize='off' "
    "style='padding:8px;flex:1;min-width:0'><datalist id='cs-boards'></datalist>"
    "<button type='button' onclick='csLoad()'>" N_("Load") "</button></div>"
    "<p class='hint'><a href='/api/settings?download=1'>" N_("Download this board's settings") "</a>"
    " &middot; <label style='cursor:pointer;text-decoration:underline'>" N_("Load from a file")
    "<input type='file' id='cs-file' accept='.json,application/json' style='display:none'></label></p>"
    "<div id='cs-prev'></div>"
    "<script>"
    "!function(){var SELF='%s',HUB='%s',cs,e=document.getElementById('cs-prev');"
    "var NM={model:'" N_("Model") "',marlin:'Marlin',bed:'" N_("Bed and screws") "',"
    "maintenance:'" N_("Maintenance jobs") "',hub:'" N_("Notification hub") "',"
    "camera:'" N_("Camera picture") "',obico:'" N_("Obico server") "',"
    "wifi_names:'" N_("Wi-Fi AP names") "'};"
    "function esc(s){return String(s).replace(/[&<>\"']/g,function(c){return '&#'+c.charCodeAt(0)+';'})}"
    "function sum(k,v){if(k=='maintenance')return(v.jobs||[]).length+' " N_("jobs") "';"
    "if(k=='wifi_names')return(v.names||[]).map(function(a){return a.name}).join(', ');"
    "return Object.keys(v).map(function(x){return x+': '+v[x]}).join(', ')}"
    "function show(d){if(!d||!d.sections){e.textContent='" N_("Not a settings file.") "';return}"
    "cs=d;var h='<p><b>'+esc(d.board||'?')+'</b>'+(d.model?' &middot; '+esc(d.model):'')+' &middot; v'+esc(d.fw||'?')+'</p>';"
    "for(var k in NM){if(!d.sections[k])continue;"
    "h+='<label style=\"display:block;margin:6px 0\"><input type=checkbox class=cs-k value='+k+' checked> <b>'+NM[k]+'</b>"
    " <span class=hint>'+esc(sum(k,d.sections[k]))+'</span></label>'}"
    "e.innerHTML=h+'<p><button type=button class=btn-green id=cs-go>" N_("Apply to this board") "</button></p>';"
    "document.getElementById('cs-go').onclick=apply}"
    "function apply(){var s={},n=0;document.querySelectorAll('.cs-k').forEach(function(c){"
    "if(c.checked){s[c.value]=cs.sections[c.value];n++}});if(!n)return;"
    "fetch('/api/settings',{method:'POST',headers:{'Content-Type':'application/json'},"
    "body:JSON.stringify({sections:s})}).then(function(r){if(!r.ok)throw r;return r.json()})"
    ".then(function(r){e.textContent='" N_("Applied:") " '+r.applied.map(function(k){return NM[k]||k}).join(', ');"
    "setTimeout(function(){location.reload()},1500)})"
    ".catch(function(){e.textContent='" N_("Could not apply the settings") "'})}"
    "window.csLoad=function(){var h=document.getElementById('cs-host').value.trim().replace(/^https?:\\/\\//,'').replace(/\\/.*$/,'');"
    "if(!h)return;e.textContent='" N_("Loading...") "';"
    "fetch('http://'+h+'/api/settings').then(function(r){if(!r.ok)throw r;return r.json()}).then(show)"
    ".catch(function(){e.textContent='" N_("No answer from") " '+h})};"
    "document.getElementById('cs-file').onchange=function(){var f=this.files[0];if(!f)return;"
    "var rd=new FileReader();rd.onload=function(){try{show(JSON.parse(rd.result))}"
    "catch(x){e.textContent='" N_("Not a settings file.") "'}};rd.readAsText(f)};"
    /* The other boards, from the hub's list */
    "if(HUB)fetch(HUB+'/api/v1/printers').then(function(r){return r.json()}).then(function(d){"
    "var l=document.getElementById('cs-boards');d.printers.forEach(function(p){"
    "if(!p.link||p.name==SELF)return;var o=document.createElement('option');"
    "o.value=p.link.split('/')[2];o.label=p.name+(p.model?' · '+p.model:'');l.appendChild(o)})})"
    ".catch(function(){})}()"
    "</script>";

void settings_copy_render_settings(html_buf_t *p)
{
    html_buf_printf(p, i18n_page(COPY_TMPL), s_model, wifi_get_hostname(), hub_client_get_url());
}
