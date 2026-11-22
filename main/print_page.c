#include "print_page.h"
#include "task_util.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "bedlevel.h"
#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "httpd.h"
#include "layout.h"
#include "i18n.h"
#include "nvs.h"
#include "print_checks.h"
#include "printer_comm.h"
#include "printer_comm_klipper.h"
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

/* The print screen: what a printer is doing, at a glance and on a phone,
 * inside the common layout.  The dashboard (/) keeps the manual controls.
 * A template for i18n_page(): not a printf format, % is written once. */
static const char PAGE[] =
"<style>"
"#mt{display:block;background:#ffebee;color:#c62828;padding:10px 12px;border-radius:6px;margin:10px 0;"
"text-decoration:none;font-weight:bold}"
"#ck{background:#ffebee;color:#c62828;padding:10px 12px;border-radius:6px;margin:10px 0}"
"#ckt{font-weight:bold;white-space:pre-line}#ck .tools{margin:8px 0 0}#ck a{color:#c62828}"
".card{border-radius:8px;padding:12px;margin:10px 0;background:#f5f5f5}"
".card.printing{background:#e8f5e9}.card.paused{background:#fff3e0}.card.error{background:#ffebee}"
"#st{font-size:22px;font-weight:bold}"
".card.printing #st{color:#2e7d32}.card.paused #st{color:#e65100}.card.error #st{color:#c62828}"
"#msg{background:#fff3e0;color:#e65100;padding:12px;border-radius:6px;margin:10px 0;font-size:18px;border:1px solid #ffb74d}"
"#err{color:#c62828;margin:6px 0}"
".row{color:#555;margin:6px 0;font-size:14px}"
"#file{display:flex;gap:10px;align-items:center;margin:8px 0}"
"#thumb{width:64px;height:64px;object-fit:contain;border-radius:4px;background:#fff;flex:none}"
"#fname{overflow-wrap:anywhere;font-size:15px}"
"#cam{width:100%;border-radius:6px;background:#000;display:block;margin:10px 0;cursor:pointer;min-height:60px}"
"#z{margin:12px 0}#zv{font-size:17px}#zp{font-size:13px;color:#666;min-height:18px;margin:2px 0 6px}"
".zb{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}"
".big button,.zb button{font-size:17px;padding:13px 4px;width:100%;touch-action:manipulation}"
".big{margin-top:10px}"
".zb button{transition:transform .08s,filter .08s}.zb button:active{transform:scale(.95);filter:brightness(.75)}"
"button.ok{animation:ok .45s}button.bad{animation:bad .6s}"
"@keyframes ok{0%{background:#2e7d32;color:#fff}100%{}}@keyframes bad{0%{background:#c62828;color:#fff}100%{}}"
"#go{background:#2e7d32;border-color:#2e7d32;color:#fff;font-size:21px;padding:18px 4px;font-weight:bold}"
"#stop{color:#c62828}"
".again{display:flex;gap:8px;margin:10px 0}"
"#again{flex:1;min-width:0;text-align:left;padding:12px;font-size:16px;overflow-wrap:anywhere}"
"#again small{display:block;color:#666;font-size:13px}"
"#more{flex:none;min-width:56px;font-size:22px;padding:8px}#more.wide{flex:1;font-size:16px}"
".tools{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:10px 0}"
".tools button{font-size:16px;padding:13px 4px;width:100%;touch-action:manipulation}"
"[hidden]{display:none!important}"
"</style>"
"<a id='mt' href='/maintenance' hidden></a>"
/* The checks before a print: an offset a calibration changed, a stale mesh profile */
"<div id='ck' hidden><div id='ckt'></div>"
"<div class='tools' id='ckz' hidden><button id='zr'></button><button id='zk'></button>"
"<button id='zpp'>" N_("Check with paper") "</button></div>"
"<div id='ckm' hidden><a href='/bedlevel'>" N_("Bed Leveling") "</a></div></div>"
"<div class='card' id='card'>"
"<div id='st'>" N_("Connecting...") "</div>"
"<div id='msg' hidden></div>"
"<div id='nz' class='hint' hidden></div>"
"<div id='err' hidden></div>"
"<div id='prog' hidden><div class='pbar'><div id='pb'></div></div><div class='row' id='pt'></div></div>"
"<div id='file' hidden><img id='thumb' alt='' hidden><div id='fname'></div></div>"
"<div class='row' id='times' hidden></div>"
"<div class='row' id='temp' hidden></div>"
"<div class='big'><button id='go' hidden></button></div>"
"</div>"
"<div id='z' hidden>"
"<div id='zv'></div><div id='zp'></div>"
"<div class='zb'>"
"<button onclick='zs(this,-5)'>&minus;0.05</button>"
"<button onclick='zs(this,-1)'>&minus;0.01</button>"
"<button onclick='zs(this,1)'>+0.01</button>"
"<button onclick='zs(this,5)'>+0.05</button>"
"</div></div>"
"<div class='big'><button id='pause' hidden></button></div>"
"<div class='big'><button id='stop' hidden></button></div>"
"<div id='idle' hidden>"
"<div class='row' id='plastic' hidden></div>"
/* Usually it is the last file again: one wide button for it, the rest behind the dots */
"<div class='again'><button id='again' hidden></button>"
"<button id='more' class='wide' onclick=\"location.href='/sd'\" title='" N_("All files") "'>" N_("All files") "</button></div>"
"<div class='tools'><button id='fil' onclick=\"location.href='/utils/filament'\">" N_("Change filament") "</button>"
"<button id='zo'>" N_("Set Z offset") "</button>"
"<button id='lz'>" N_("Raise Z 100 mm") "</button>"
"<button id='mo'>" N_("Motors off") "</button></div>"
"</div>"
"<img id='cam' alt='" N_("Camera") "' title='" N_("Camera") "'>"
"<script>"
"var $=function(i){return document.getElementById(i)},S=null,tap=0,zl=0,zmsg='',thumbOf=null;"
"document.addEventListener('touchstart',function(){},{passive:true});"
"function post(u,b){return fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b||''})"
".then(function(r){if(!r.ok)return r.text().then(function(t){throw t});setTimeout(u0,300)})}"
"function vis(e,v){$(e).hidden=!v}"
"function flash(b,c){b.classList.remove('ok','bad');void b.offsetWidth;b.classList.add(c)}"
"function fix(c){return (c>0?'+':'')+(c/100).toFixed(2)+' mm'}"
"function dur(s){var h=Math.floor(s/3600),m=Math.floor(s%3600/60);"
"return (h?h+' " N_("h") " ':'')+m+' " N_("min") "'}"
"function drawz(){if(!S)return;var sent=Math.round(S.z_shift*100),all=sent+Math.round(S.z_pending*100)+zl;"
"$('zv').textContent='" N_("Z shift:") " '+fix(all);"
"$('zp').textContent=zmsg||(all!=sent?'" N_("Sending to printer, applied") " '+fix(sent):'')}"
"function zs(b,c){zl+=c;zmsg='';drawz();"
"post('/print/z','d='+(c/100).toFixed(2)).then(function(){zl-=c;S.z_pending=(Math.round(S.z_pending*100)+c)/100;flash(b,'ok');drawz()},"
"function(t){zl-=c;flash(b,'bad');zmsg=t=='limit 1'?'" N_("Limit is 1 mm per print") "':t=='limit 3'?'" N_("Limit is 3 mm here") "':'" N_("Not sent, try again") "';drawz()})}"
"function act(b,u,body){b.disabled=true;tap=Date.now();draw(S);"
"post(u,body).catch(function(){tap=0;b.disabled=false;flash(b,'bad');draw(S)})}"
"$('go').onclick=function(){if(S)act(this,S.hold?'/print/continue':'/sd/resume',S.hold?'id='+S.hold_id:'')};"
"$('pause').onclick=function(){if(S)act(this,'/sd/pause')};"
"$('zo').onclick=function(){var b=this;if(!confirm('" N_("The printer homes and lowers the nozzle to Z0 in the middle of the bed, then waits. Put a sheet of paper under the nozzle and move Z until the paper drags slightly; Continue saves the offset.") "\\n\\n" N_("Is the bed clear?") "'))return;"
"b.disabled=true;post('/print/zoffset').catch(function(t){flash(b,'bad');alert(typeof t=='string'&&t?t:'" N_("Not started") "')}).then(function(){b.disabled=false})};"
"function tool(b,u){b.disabled=true;post(u).then(function(){flash(b,'ok')},function(t){flash(b,'bad');alert(typeof t=='string'&&t?t:'" N_("Not sent, try again") "')})"
".then(function(){b.disabled=false})}"
"$('lz').onclick=function(){tool(this,'/print/lift')};$('mo').onclick=function(){tool(this,'/print/motors')};"
"$('stop').onclick=function(){if(S&&confirm('" N_("Stop the print and turn off the heaters?") "'))act(this,'/sd/cancel')};"
/* Overdue maintenance on top: the first thing seen */
"function mt(){fetch('/maintenance/due').then(function(r){return r.json()}).then(function(d){"
"$('mt').textContent='\\ud83d\\udd27 " N_("Maintenance is overdue:") " '+d.text;vis('mt',d.n>0)}).catch(function(){})}"
/* The checks, shown while idle: the Z offset buttons on Marlin, the bed page for a mesh */
"var CK=null;function ck(){if(S&&(S.state=='printing'||S.state=='paused'))return;fetch('/print/checks').then(function(r){return r.json()}).then(function(d){CK=d;"
"$('ckt').textContent='\\u26a0 '+d.warn;vis('ckz',!!d.z);vis('ckm',!d.z||d.warn.indexOf('\\n')>0);"
"if(d.z){$('zr').textContent='" N_("Put back") " '+d.z.good.toFixed(2);$('zk').textContent='" N_("Keep") " '+d.z.now.toFixed(2)}"
"if(S)draw(S)}).catch(function(){})}"
"function zfix(b,a){b.disabled=true;post('/print/zfix','a='+a).then(function(){flash(b,'ok');setTimeout(ck,6000)},"
"function(t){flash(b,'bad');alert(typeof t=='string'&&t?t:'" N_("Not sent, try again") "')}).then(function(){b.disabled=false})}"
"$('zr').onclick=function(){zfix(this,'restore')};$('zk').onclick=function(){zfix(this,'keep')};"
"$('zpp').onclick=function(){$('zo').click()};"
/* The model's picture, from the board's file info, once per file */
"function thumb(f){if(f===thumbOf)return;thumbOf=f;vis('thumb',false);if(!f)return;"
"fetch('/sd/info?filename='+encodeURIComponent(f)).then(function(r){return r.ok?r.json():null}).then(function(d){"
"if(d&&d.thumbnail&&thumbOf===f){$('thumb').src=d.thumbnail;vis('thumb',true)}}).catch(function(){})}"
"var rt=0,rbusy=false;"
"function small(b,t){var m=document.createElement('small');m.textContent=t;b.appendChild(m)}"
"function again(f){var b=$('again'),m=$('more');b.textContent='';vis('again',!!f);"
"m.textContent=f?'\\u22ef':'" N_("All files") "';m.className=f?'':'wide';if(!f)return;"
"small(b,'" N_("Print again") "');b.appendChild(document.createTextNode(f.name));"
"if(!f.on_sd)small(b,'" N_("Not on the SD card") "');b.disabled=!f.on_sd;"
"b.onclick=function(){fetch('/print/checks').then(function(r){return r.json()}).catch(function(){return {}}).then(function(d){"
"if(!confirm('" N_("Print") " '+f.name+'?\\n\\n" N_("Is the bed clear?") "'"
"+(d.warn?'\\n\\n\\u26a0 '+d.warn+'\\n" N_("Print anyway?") "':'')"
"+(d.n?'\\n\\n" N_("Maintenance is overdue:") " '+d.text+'\\n" N_("Print anyway? It is recorded in the maintenance log.") "':'')))return;"
"b.disabled=true;post('/print/start','name='+encodeURIComponent(f.name)).then(function(){rt=0},"
"function(t){b.disabled=false;flash(b,'bad');alert(typeof t=='string'&&t?t:'" N_("Not started") "')})})}}"
"function recent(){rbusy=true;fetch('/print/recent').then(function(r){return r.json()}).then(function(d){again(d.files[0])})"
".catch(function(){}).then(function(){rbusy=false})}"
"function draw(s){"
/* A stop is a pause: with the printer unreachable the board still has the flag,
 * but nothing on this page can act on it */
"s.hold=s.hold&&s.state=='paused';"
"var key=s.state+':'+s.hold_id;"
"if(!S||S.key!==key||Date.now()-tap>5000){$('go').disabled=false;$('pause').disabled=false;$('stop').disabled=false}"
"s.key=key;S=s;"
"var pr=s.state=='printing',pa=s.state=='paused',on=pr||pa,up=s.upload>=0,con=s.usb||s.backend=='Klipper'&&s.state!='disconnected';"
"document.title=(s.name||'" N_("Print") "')+(on?' '+Math.floor(s.progress)+'%':'');"
"$('card').className='card '+(s.hold||pa?'paused':pr||up?'printing':s.state=='error'?'error':'');"
"$('st').textContent=up?'" N_("Sending to the printer") " '+s.upload+'%':s.hold?'" N_("Waiting for you") "':"
"pr?(s.preparing?'" N_("Preparing...") "':'" N_("Printing") "'):pa?'" N_("Paused") "':"
"s.state=='error'?'" N_("Error") "':s.state=='disconnected'?'" N_("Printer not connected") "':'" N_("No print running") "';"
"$('msg').textContent=s.hold_msg||'';vis('msg',s.hold);"
"$('nz').textContent=s.nozzle=='heating'?'" N_("Heating the nozzle back, then the print goes on") "':"
"'" N_("The nozzle is down at its standby temperature so the plastic does not char: Continue heats it first") "';"
"vis('nz',pa&&(s.nozzle=='standby'||s.nozzle=='heating'));"
"$('err').textContent='" N_("Notification was not delivered") "';vis('err',s.hold&&s.notify=='failed');"
"var p=up?s.upload:Math.floor(s.progress);$('pb').style.width=Math.min(100,p)+'%';"
"$('pt').textContent=up?'':p+'%'+(s.layer&&s.layer.slice(-2)!='/0'?' \\u00b7 " N_("layer") " '+s.layer:'');"
"vis('prog',up||on&&!s.preparing);"
"$('fname').textContent=s.filename||'';vis('file',on&&!!s.filename);thumb(on?s.filename:'');"
"var t='';if(on&&!s.preparing&&s.elapsed>0){t='" N_("elapsed") " '+dur(s.elapsed);"
"if(s.remaining>0){var e=new Date(Date.now()+s.remaining*1000);"
"t+=' \\u00b7 " N_("left") " '+dur(s.remaining)+' \\u00b7 " N_("done at") " '+e.getHours()+':'+('0'+e.getMinutes()).slice(-2)}}"
"$('times').textContent=t;vis('times',!!t);"
"$('temp').textContent='" N_("Nozzle") " '+s.hotend[0].toFixed(0)+'/'+s.hotend[1].toFixed(0)"
"+'\\u00b0C \\u00b7 " N_("Bed") " '+s.bed[0].toFixed(0)+'/'+s.bed[1].toFixed(0)+'\\u00b0C'"
"+(s.fan>=0&&on?' \\u00b7 " N_("Fan") " '+s.fan+'%':'');"
"vis('temp',con);"
"vis('z',pr||s.hold);drawz();"
"$('go').textContent=$('go').disabled?'" N_("Sending...") "':'" N_("Continue") "';vis('go',pa);"
"$('pause').textContent=$('pause').disabled?'" N_("Pausing...") "':'" N_("Pause") "';vis('pause',pr);"
"$('stop').textContent=$('stop').disabled?'" N_("Stopping...") "':'" N_("Stop") "';vis('stop',on);"
"vis('idle',!on&&!up);vis('ck',!on&&!up&&!!CK&&!!CK.warn);"
"$('plastic').textContent=s.plastic?'" N_("In the nozzle:") " '+s.plastic:'';vis('plastic',!!s.plastic);"
"var idle=s.state=='operational'&&con&&!up;$('fil').disabled=$('zo').disabled=$('lz').disabled=$('mo').disabled=!idle;vis('mo',s.motors!==0);"
"if(!idle){again(null);rt=0}else if(!rbusy&&Date.now()-rt>20000){rt=Date.now();recent()}}"
"function u0(){return fetch('/api/status').then(function(r){return r.json()}).then(draw)"
".catch(function(){$('st').textContent='" N_("No connection to the printer") "'})}"
/* The camera, a fresh picture every few seconds, as on the dashboard */
"var cam=$('cam');function shot(){cam.src='/capture?'+Date.now()}"
"cam.onclick=function(){location.href='/camera'};cam.onerror=function(){vis('cam',false)};cam.onload=function(){vis('cam',true)};"
"shot();setInterval(shot,5000);"
"u0();setInterval(u0,1500);mt();setInterval(mt,60000);ck();setInterval(ck,20000);"
"</script>";

static esp_err_t print_page_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, NT_("Print"), "/print");
    html_buf_printf(&p, "%s", i18n_page(PAGE));
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
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
    esp_err_t err = printer_comm_host_print_zstep(d);
    if (err != ESP_OK) {
        httpd_resp_set_status(req, "409 Conflict");
        if (err != ESP_ERR_INVALID_SIZE) return httpd_resp_sendstr(req, "not now");
        printer_state_t st;
        printer_comm_get_state(&st);
        return httpd_resp_sendstr(req, st.builtin_script ? "limit 3" : "limit 1");
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

/* Klipper: the upload takes a while, so it runs in its own task and the page
 * sees the print start in the status */
static volatile bool s_klipper_starting;

static void klipper_start_task(void *arg)
{
    char *name = arg;
    if (klipper_backend_print_file(name) == ESP_OK) {
        ESP_LOGI(TAG, "Print again: %s", name);
        print_page_note_started(name);
    } else {
        ESP_LOGW(TAG, "Print again failed: %s", name);
    }
    free(name);
    s_klipper_starting = false;
    task_delete(NULL);
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

    char names[RECENT_MAX][RECENT_NAME_MAX];
    int n = recent_copy(names);
    bool known = false;
    for (int i = 0; i < n && !known; i++) known = strcmp(names[i], name) == 0;
    if (!known) return conflict(req, _("Not in the recent files"));
    if (!on_sd(name)) return conflict(req, _("The file is not on the SD card"));

    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate != PRINTER_OPERATIONAL || printer_comm_is_host_printing() || s_klipper_starting ||
        (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER && klipper_backend_sending())) {
        return conflict(req, _("The printer is not idle"));
    }

    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        char *copy = strdup(name);
        s_klipper_starting = copy != NULL;
        if (!copy || xTaskCreatePinnedToCore(klipper_start_task, "print_again", 8192, copy,
                                             5, NULL, 0) != pdPASS) {
            free(copy);
            s_klipper_starting = false;
            return conflict(req, _("Not started, try again"));
        }
        return httpd_resp_send(req, "OK", 2);
    }

    if (printer_comm_host_print_start(name) != ESP_OK) return conflict(req, _("Not started, try again"));
    ESP_LOGI(TAG, "Print again: %s", name);
    print_page_note_started(name);
    return httpd_resp_send(req, "OK", 2);
}

/* ---- Z offset ---- */

/* Paper test: homed, the nozzle at Z0 in the middle of the bed, a stop for
 * the user to move Z with the page's buttons.  Marlin's M290 moves the probe
 * offset (the dashboard's Save is M500 too); Klipper's Z_ADJUST moves the
 * gcode offset, which the done callback keeps on the board for its prints. */
#define ZOFF_SCRIPT_SIZE 512

static void zoffset_done(bool completed, int backend)
{
    if (!completed) return;
    /* Marlin: the script's M500 saved what the paper showed */
    if (backend != PRINTER_BACKEND_KLIPPER) {
        print_checks_z_confirmed();
        return;
    }
    printer_state_t st;
    printer_comm_get_state(&st);
    esp_err_t err = klipper_backend_set_zoffset(true, st.probe_z_offset);
    ESP_LOGI(TAG, "Z offset %.3f saved for prints: %s", st.probe_z_offset, esp_err_to_name(err));
}

static esp_err_t print_zoffset_handler(httpd_req_t *req)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    bool klipper = printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER;
    if (st.opstate != PRINTER_OPERATIONAL || printer_comm_is_host_printing() || s_klipper_starting ||
        (klipper && klipper_backend_sending())) {
        return conflict(req, _("The printer is not idle"));
    }

    float cx, cy;
    bedlevel_bed_center(&cx, &cy);
    char *script = malloc(ZOFF_SCRIPT_SIZE);
    if (!script) return conflict(req, _("Not started, try again"));
    int n = snprintf(script, ZOFF_SCRIPT_SIZE,
        "; Z offset: paper test in the middle of the bed\n"
        "G28\nG90\n"
        "%s"                            /* Klipper: start from the board's saved offset */
        "G1 Z5 F600\nG1 X%.0f Y%.0f F6000\nG1 Z0 F300\n"
        "; @pause Paper under the nozzle: move Z until it drags slightly, then Continue\n"
        "G1 Z10 F600\n"
        "%s",
        klipper ? "; @zoffset\n" : "", cx, cy, klipper ? "" : "M500\n");
    if (n <= 0 || n >= ZOFF_SCRIPT_SIZE) {
        free(script);
        return conflict(req, _("Not started, try again"));
    }

    /* The script now belongs to the print, whatever happens */
    if (printer_comm_host_print_script("Z offset", script, zoffset_done,
                                       printer_comm_get_backend()) != ESP_OK) {
        return conflict(req, _("Not started, try again"));
    }
    ESP_LOGI(TAG, "Z offset script started, nozzle to X%.0f Y%.0f", cx, cy);
    return httpd_resp_send(req, "OK", 2);
}

/* ---- Raise Z, motors off ---- */

#define LIFT_MM 100

static bool idle_now(void)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    return st.opstate == PRINTER_OPERATIONAL && !printer_comm_is_host_printing() && !s_klipper_starting &&
           !(printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER && klipper_backend_sending());
}

static esp_err_t send_raw(const char *gcode)
{
    printer_cmd_t cmd = { .type = PCMD_RAW };
    strlcpy(cmd.gcode, gcode, sizeof(cmd.gcode));
    return printer_comm_send_cmd(&cmd);
}

/* Moonraker answers a script when its moves are over: not in the httpd task */
static void klipper_lift_task(void *arg)
{
    if (send_raw(arg) != ESP_OK) ESP_LOGW(TAG, "Raise Z failed");
    free(arg);
    task_delete(NULL);
}

static esp_err_t print_lift_handler(httpd_req_t *req)
{
    if (!idle_now()) return conflict(req, _("The printer is not idle"));

    if (printer_comm_get_backend() != PRINTER_BACKEND_KLIPPER) {
        /* Marlin keeps the position after M84 and allows moves before homing;
         * its soft endstops stop the move at the top */
        char g1[32];
        snprintf(g1, sizeof(g1), "G1 Z%d F1200", LIFT_MM);
        if (send_raw("G91") != ESP_OK || send_raw(g1) != ESP_OK || send_raw("G90") != ESP_OK) {
            return conflict(req, _("Not sent, try again"));
        }
        return httpd_resp_send(req, "OK", 2);
    }

    char *script = malloc(96);
    if (!script) return conflict(req, _("Not sent, try again"));
    esp_err_t err = klipper_backend_lift_script(LIFT_MM, script, 96);
    if (err != ESP_OK) {
        free(script);
        return conflict(req, err == ESP_ERR_INVALID_SIZE ? _("Z is already at the top")
                             : _("The printer does not know where Z is since Klipper restarted: home it first"));
    }
    ESP_LOGI(TAG, "Raise Z: %s", script);
    /* 6 KB: a request to Moonraker takes ~4.5 KB (see klipper_poll) */
    if (task_create_psram(klipper_lift_task, "lift", 12288, script, 5, NULL, 0) != pdPASS) {
        free(script);
        return conflict(req, _("Not sent, try again"));
    }
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t print_motors_handler(httpd_req_t *req)
{
    if (!idle_now()) return conflict(req, _("The printer is not idle"));
    if (send_raw("M84") != ESP_OK) return conflict(req, _("Not sent, try again"));
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
        { .uri = "/print/zoffset",  .method = HTTP_POST, .handler = print_zoffset_handler },
        { .uri = "/print/lift",     .method = HTTP_POST, .handler = print_lift_handler },
        { .uri = "/print/motors",   .method = HTTP_POST, .handler = print_motors_handler },
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }
    return ESP_OK;
}
