#include "print_page.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "httpd.h"
#include "i18n.h"
#include "nvs.h"
#include "printer_comm.h"
#include "sdcard.h"
#include "url_util.h"

static const char *TAG = "print_page";

/* Only small steps, so that a burst of taps cannot drive the nozzle into the bed */
#define Z_STEP_SMALL 0.01f
#define Z_STEP_LARGE 0.05f

/* Files printed from SD lately, newest first, for "Print again" */
#define RECENT_MAX       5
#define RECENT_NAME_MAX  64
#define NVS_NAMESPACE    "print"
#define NVS_KEY_RECENT   "recent"     /* Names joined with '\n' */

/* A page template for i18n_page(): N_() marks the text to translate */
static const char PAGE_HEAD[] =
"<!DOCTYPE html><html><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>" N_("Print") "</title><style>"
":root{color-scheme:light dark;--bg:#fff;--fg:#222;--dim:#666;--line:#ccc;--btn:#eee;"
"--wait:#fff3e0;--wait-fg:#e65100;--err:#c62828}"
"@media(prefers-color-scheme:dark){:root{--bg:#121212;--fg:#eee;--dim:#aaa;--line:#444;--btn:#2a2a2a;"
"--wait:#3a2a10;--wait-fg:#ffb74d;--err:#ef9a9a}}"
"body{font-family:sans-serif;background:var(--bg);color:var(--fg);max-width:480px;margin:0 auto;padding:16px}"
"h1{font-size:15px;font-weight:normal;color:var(--dim);margin:0 0 8px}"
"#st{font-size:22px;font-weight:bold}"
"#msg{background:var(--wait);color:var(--wait-fg);padding:12px;border-radius:6px;margin:12px 0;font-size:18px}"
"#err{color:var(--err);margin:8px 0}"
".row{color:var(--dim);margin:8px 0}"
"#z{margin:16px 0}"
"#zv{font-size:18px}"
"#zp{font-size:14px;color:var(--dim);min-height:20px;margin:2px 0 6px}"
".zb{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}"
"button{font-size:17px;padding:14px 4px;border:1px solid var(--line);border-radius:6px;"
"background:var(--btn);color:var(--fg);width:100%;touch-action:manipulation}"
"button:disabled{opacity:.6}"
"button{transition:transform .08s,filter .08s}"
"button:active{transform:scale(.95);filter:brightness(.75)}"
"button.ok{animation:ok .45s}"
"button.bad{animation:bad .6s}"
"@keyframes ok{0%{background:#2e7d32;color:#fff}100%{}}"
"@keyframes bad{0%{background:#c62828;color:#fff}100%{}}"
".act{margin-top:12px}"
"#go{background:#2e7d32;border-color:#2e7d32;color:#fff;font-size:21px;padding:20px 4px;font-weight:bold}"
"#stop{color:var(--err)}"
"h2{font-size:15px;font-weight:normal;color:var(--dim);margin:20px 0 8px}"
".rf{text-align:left;padding:12px;margin-bottom:8px;font-size:16px;overflow-wrap:anywhere}"
".rf small{display:block;color:var(--dim);font-size:13px}"
"[hidden]{display:none!important}"
"</style></head><body>"
"<h1 id='nm'></h1>"
"<div id='st'>" N_("Connecting...") "</div>"
"<div id='msg' hidden></div>"
"<div id='err' hidden></div>"
"<div class='row' id='file' hidden></div>"
"<div class='row' id='temp' hidden></div>"
"<div id='z' hidden>"
"<div id='zv'></div><div id='zp'></div>"
"<div class='zb'>"
"<button onclick='zs(this,-5)'>&minus;0.05</button>"
"<button onclick='zs(this,-1)'>&minus;0.01</button>"
"<button onclick='zs(this,1)'>+0.01</button>"
"<button onclick='zs(this,5)'>+0.05</button>"
"</div></div>"
"<div class='act'><button id='go' hidden></button></div>"
"<div class='act'><button id='pause' hidden></button></div>"
"<div class='act'><button id='stop' hidden></button></div>"
"<div id='rec' hidden><h2>" N_("Print again") "</h2><div id='recl'></div>"
"<p class='row'><a href='/sd'>" N_("All files") "</a></p></div>"
"<p class='row'><a id='home' href='/' hidden>" N_("Dashboard") "</a></p>"
"<p class='row'>";

/* The language links go between the two halves */
static const char PAGE_TAIL[] =
"</p>"
"<script>"
"var $=function(i){return document.getElementById(i)},S=null,tap=0,zl=0,zmsg='';"
"document.addEventListener('touchstart',function(){},{passive:true});"
"function post(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b||''})"
".then(function(r){if(!r.ok)return r.text().then(function(t){throw t});setTimeout(u0,300)})}"
"function vis(e,v){$(e).hidden=!v}"
"function flash(b,c){b.classList.remove('ok','bad');void b.offsetWidth;b.classList.add(c)}"
"function fix(c){return (c>0?'+':'')+(c/100).toFixed(2)+' mm'}"
"function drawz(){if(!S)return;var sent=Math.round(S.z_shift*100),all=sent+Math.round(S.z_pending*100)+zl;"
"$('zv').textContent='" N_("Z shift:") " '+fix(all);"
"$('zp').textContent=zmsg||(all!=sent?'" N_("Sending to printer, applied") " '+fix(sent):'')}"
"function zs(b,c){zl+=c;zmsg='';drawz();"
"post('/print/z','d='+(c/100).toFixed(2)).then(function(){zl-=c;S.z_pending=(Math.round(S.z_pending*100)+c)/100;flash(b,'ok');drawz()},"
"function(t){zl-=c;flash(b,'bad');zmsg=t=='limit'?'" N_("Limit is 1 mm per print") "':'" N_("Not sent, try again") "';drawz()})}"
"function act(b,u,body){b.disabled=true;tap=Date.now();draw(S);"
"post(u,body).catch(function(){tap=0;b.disabled=false;flash(b,'bad');draw(S)})}"
"$('go').onclick=function(){if(S)act(this,S.hold?'/print/continue':'/sd/resume',S.hold?'id='+S.hold_id:'')};"
"$('pause').onclick=function(){if(S)act(this,'/sd/pause')};"
"$('stop').onclick=function(){if(S&&confirm('" N_("Stop the print and turn off the heaters?") "'))act(this,'/sd/cancel')};"
"var rt=0,rbusy=false;"
"function recent(){rbusy=true;fetch('/print/recent').then(function(r){return r.json()}).then(function(d){"
"var l=$('recl');l.textContent='';d.files.forEach(function(f){"
"var b=document.createElement('button');b.className='rf';b.textContent=f.name;b.disabled=!f.on_sd;"
"if(!f.on_sd){var m=document.createElement('small');m.textContent='" N_("Not on the SD card") "';b.appendChild(m)}"
"b.onclick=function(){if(!confirm('" N_("Print") " '+f.name+'?\\n\\n" N_("Is the bed clear?") "'))return;"
"b.disabled=true;post('/print/start','name='+encodeURIComponent(f.name)).then(function(){rt=0},"
"function(t){b.disabled=false;flash(b,'bad');alert(typeof t=='string'&&t?t:'" N_("Not started") "')})};"
"l.appendChild(b)});vis('rec',d.files.length>0)}).catch(function(){}).then(function(){rbusy=false})}"
"function draw(s){"
"var key=s.state+':'+s.hold_id;"
"if(!S||S.key!==key||Date.now()-tap>5000){$('go').disabled=false;$('pause').disabled=false;$('stop').disabled=false}"
"s.key=key;S=s;"
"var pr=s.state=='printing',pa=s.state=='paused',on=pr||pa;"
"document.title=(s.name||'" N_("Print") "');"
"$('nm').textContent=s.name||'';"
"$('st').textContent=s.hold?'" N_("Waiting for you") "':pr?(s.preparing?'" N_("Preparing...") "':'" N_("Printing") "'):pa?'" N_("Paused") "':"
"s.state=='disconnected'?'" N_("Printer not connected") "':'" N_("No print running") "';"
"$('msg').textContent=s.hold_msg||'';vis('msg',s.hold);"
"$('err').textContent='" N_("Notification was not delivered") "';vis('err',s.hold&&s.notify=='failed');"
"$('file').textContent=(s.filename||'')+(on&&!s.hold&&!s.preparing?' \\u00b7 '+Math.floor(s.progress)+'%':'')"
"+(s.layer&&s.layer.slice(-2)!='/0'&&!s.hold?' \\u00b7 " N_("layer") " '+s.layer:'');"
"vis('file',on);"
"$('temp').textContent='" N_("Nozzle") " '+s.hotend[0].toFixed(0)+'/'+s.hotend[1].toFixed(0)"
"+'\\u00b0C \\u00b7 " N_("Bed") " '+s.bed[0].toFixed(0)+'/'+s.bed[1].toFixed(0)+'\\u00b0C';"
"vis('temp',s.usb);"
"vis('z',s.backend=='Marlin'&&(pr||s.hold));drawz();"
"$('go').textContent=$('go').disabled?'" N_("Sending...") "':'" N_("Continue") "';vis('go',pa);"
"$('pause').textContent=$('pause').disabled?'" N_("Pausing...") "':'" N_("Pause") "';vis('pause',pr);"
"$('stop').textContent=$('stop').disabled?'" N_("Stopping...") "':'" N_("Stop") "';vis('stop',on);vis('home',!on);"
"var idle=s.backend=='Marlin'&&s.usb&&s.state=='operational';"
"if(!idle){vis('rec',false);rt=0}else if(!rbusy&&Date.now()-rt>20000){rt=Date.now();recent()}}"
"function u0(){return fetch('/api/status').then(function(r){return r.json()}).then(draw)"
".catch(function(){$('st').textContent='" N_("No connection to the printer") "'})}"
"u0();setInterval(u0,1500);"
"</script></body></html>";

static esp_err_t print_page_handler(httpd_req_t *req)
{
    char langs[256];
    i18n_render_switch(langs, sizeof(langs));

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "Content-Language", i18n_lang_code(i18n_lang()));
    if (httpd_resp_sendstr_chunk(req, i18n_page(PAGE_HEAD)) != ESP_OK ||
        httpd_resp_sendstr_chunk(req, langs) != ESP_OK ||
        httpd_resp_sendstr_chunk(req, i18n_page(PAGE_TAIL)) != ESP_OK) {
        return ESP_FAIL;
    }
    return httpd_resp_sendstr_chunk(req, NULL);
}

/** Read a single field from a short form body. */
static bool read_field(httpd_req_t *req, const char *key, char *out, size_t out_size)
{
    char buf[64];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) return false;
    buf[recv_len] = '\0';

    size_t key_len = strlen(key);
    if (strncmp(buf, key, key_len) != 0 || buf[key_len] != '=') return false;
    url_decode_field(buf + key_len + 1, out, out_size);
    return true;
}

static esp_err_t print_continue_handler(httpd_req_t *req)
{
    char val[16];
    char *end = NULL;
    if (!read_field(req, "id", val, sizeof(val))) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "id required");
    }
    unsigned long id = strtoul(val, &end, 10);
    if (end == val || *end != '\0') {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad id");
    }

    printer_comm_host_print_continue((uint32_t)id);
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t print_z_handler(httpd_req_t *req)
{
    char val[16];
    char *end = NULL;
    if (!read_field(req, "d", val, sizeof(val))) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "d required");
    }
    float d = strtof(val, &end);
    if (end == val || *end != '\0' ||
        (fabsf(fabsf(d) - Z_STEP_SMALL) > 0.0001f && fabsf(fabsf(d) - Z_STEP_LARGE) > 0.0001f)) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad step");
    }
    if (printer_comm_get_backend() != PRINTER_BACKEND_MARLIN) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Marlin only");
    }

    esp_err_t err = printer_comm_host_print_zstep(d);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, err == ESP_ERR_INVALID_SIZE ? "limit" : "not now");
    }
    return httpd_resp_send(req, "OK", 2);
}

/* ---- Recent files ---- */

static char s_recent[RECENT_MAX][RECENT_NAME_MAX];
static int s_recent_count;
static portMUX_TYPE s_recent_lock = portMUX_INITIALIZER_UNLOCKED;

static void recent_load(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) != ESP_OK) return;
    char buf[RECENT_MAX * RECENT_NAME_MAX];
    size_t len = sizeof(buf);
    if (nvs_get_str(nvs, NVS_KEY_RECENT, buf, &len) == ESP_OK) {
        int n = 0;
        for (char *save, *tok = strtok_r(buf, "\n", &save); tok && n < RECENT_MAX;
             tok = strtok_r(NULL, "\n", &save)) {
            strlcpy(s_recent[n++], tok, RECENT_NAME_MAX);
        }
        s_recent_count = n;
    }
    nvs_close(nvs);
}

/** Copy the list out, so that nothing slow runs under the lock */
static int recent_copy(char out[RECENT_MAX][RECENT_NAME_MAX])
{
    taskENTER_CRITICAL(&s_recent_lock);
    int n = s_recent_count;
    memcpy(out, s_recent, sizeof(s_recent));
    taskEXIT_CRITICAL(&s_recent_lock);
    return n;
}

void print_page_note_started(const char *filename)
{
    if (!filename[0] || strlen(filename) >= RECENT_NAME_MAX || strchr(filename, '\n')) return;

    char copy[RECENT_MAX][RECENT_NAME_MAX];
    taskENTER_CRITICAL(&s_recent_lock);
    int n = 0;
    strlcpy(copy[n++], filename, RECENT_NAME_MAX);
    for (int i = 0; i < s_recent_count && n < RECENT_MAX; i++) {
        if (strcmp(s_recent[i], filename) != 0) strlcpy(copy[n++], s_recent[i], RECENT_NAME_MAX);
    }
    memcpy(s_recent, copy, sizeof(s_recent));
    s_recent_count = n;
    taskEXIT_CRITICAL(&s_recent_lock);

    char buf[RECENT_MAX * RECENT_NAME_MAX];
    size_t len = 0;
    for (int i = 0; i < n; i++) {
        len += snprintf(buf + len, sizeof(buf) - len, "%s%s", i ? "\n" : "", copy[i]);
    }
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        if (nvs_set_str(nvs, NVS_KEY_RECENT, buf) == ESP_OK) nvs_commit(nvs);
        nvs_close(nvs);
    }
}

static bool on_sd(const char *name)
{
    char path[RECENT_NAME_MAX + 16];
    struct stat st;
    snprintf(path, sizeof(path), "/sdcard/%s", name);
    return sdcard_is_mounted() && stat(path, &st) == 0 && S_ISREG(st.st_mode);
}

static esp_err_t print_recent_handler(httpd_req_t *req)
{
    char names[RECENT_MAX][RECENT_NAME_MAX];
    int n = recent_copy(names);

    cJSON *root = cJSON_CreateObject();
    cJSON *files = cJSON_AddArrayToObject(root, "files");
    for (int i = 0; i < n && files; i++) {
        cJSON *f = cJSON_CreateObject();
        if (!f) break;
        cJSON_AddStringToObject(f, "name", names[i]);
        cJSON_AddBoolToObject(f, "on_sd", on_sd(names[i]));
        cJSON_AddItemToArray(files, f);
    }
    char *json = root ? cJSON_PrintUnformatted(root) : NULL;
    cJSON_Delete(root);
    if (!json) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = httpd_resp_sendstr(req, json);
    cJSON_free(json);
    return ret;
}

static esp_err_t conflict(httpd_req_t *req, const char *msg)
{
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_sendstr(req, msg);
}

/* Starts only a file from the recent list: this page is opened from the hub's
 * dashboard and Telegram, so it should not start just any file it is given */
static esp_err_t print_start_handler(httpd_req_t *req)
{
    char body[RECENT_NAME_MAX * 3 + 8];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "name required");
    body[len] = '\0';
    if (strncmp(body, "name=", 5) != 0) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "name required");
    }
    char name[RECENT_NAME_MAX];
    url_decode_field(body + 5, name, sizeof(name));

    if (printer_comm_get_backend() != PRINTER_BACKEND_MARLIN) return conflict(req, _("Marlin only"));

    char names[RECENT_MAX][RECENT_NAME_MAX];
    int n = recent_copy(names);
    bool known = false;
    for (int i = 0; i < n && !known; i++) known = strcmp(names[i], name) == 0;
    if (!known) return conflict(req, _("Not in the recent files"));
    if (!on_sd(name)) return conflict(req, _("The file is not on the SD card"));

    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate != PRINTER_OPERATIONAL || printer_comm_is_host_printing()) {
        return conflict(req, _("The printer is not idle"));
    }

    if (printer_comm_host_print_start(name) != ESP_OK) return conflict(req, _("Not started, try again"));
    ESP_LOGI(TAG, "Print again: %s", name);
    print_page_note_started(name);
    return httpd_resp_send(req, "OK", 2);
}

esp_err_t print_page_register_httpd(httpd_handle_t server)
{
    recent_load();

    const httpd_uri_t uris[] = {
        { .uri = "/print",          .method = HTTP_GET,  .handler = print_page_handler },
        { .uri = "/print/continue", .method = HTTP_POST, .handler = print_continue_handler },
        { .uri = "/print/z",        .method = HTTP_POST, .handler = print_z_handler },
        { .uri = "/print/recent",   .method = HTTP_GET,  .handler = print_recent_handler },
        { .uri = "/print/start",    .method = HTTP_POST, .handler = print_start_handler },
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }
    return ESP_OK;
}
