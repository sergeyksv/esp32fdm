#include "layout.h"

#include "i18n.h"
#include "printer_backend.h"
#include "sdcard.h"
#include "wifi.h"

static void nav_link(html_buf_t *p, const char *path, const char *label, bool active)
{
    if (active) html_buf_printf(p, " <b>%s</b>", TR(label));
    else html_buf_printf(p, " <a href='%s'>%s</a>", path, TR(label));
}

void layout_html_begin(html_buf_t *p, const char *title, const char *current_path)
{
    bool is_klipper = printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER;
#define IS(path) (strcmp(current_path, path) == 0)

    html_buf_printf(p,
        "<!DOCTYPE html><html lang='%s'><head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<link rel='icon' type='image/svg+xml' href='/favicon.svg'>"
        "<title>%s</title>"
        "<style>"
        "body{font-family:sans-serif;max-width:600px;margin:0 auto;padding:0 1em}"
        "nav{background:#333;margin:0 -1em;padding:8px 1em;color:#fff;font-size:14px;"
        "display:flex;flex-wrap:wrap;align-items:center}"
        "nav a{color:#fff;text-decoration:none}"
        "nav .items a,nav .items b{color:#fff;margin:0 6px;white-space:nowrap}"
        "nav .brand{font-weight:bold;margin-right:12px}"
        "nav .pn{font-size:21px;font-weight:800;color:#ffca28;line-height:1}"
        "nav .cur,nav .mb,nav .items .home{display:none}"
        /* Phone: one line with the page name and a menu button, the links
         * open as a list, so the length of the labels does not matter */
        "@media(max-width:560px){"
        "nav .cur{display:block;flex:1;color:#ccc;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}"
        "nav .mb{display:block;background:none;border:0;color:#fff;font-size:22px;line-height:1;padding:2px 4px;margin:0}"
        "nav .items{display:none;flex-basis:100%%;flex-direction:column;padding-top:6px}"
        "nav.open .items{display:flex}"
        "nav .items a,nav .items b{margin:0;padding:10px 4px;font-size:17px;border-top:1px solid #444}"
        "nav .items .home{display:block}}"
        "footer .lng a{color:#888}"
        ".cpu-bars{display:inline-flex;align-items:center;gap:4px;font-size:11px;color:#aaa;vertical-align:middle}"
        ".cpu-bar{width:40px;height:10px;background:#555;border-radius:2px;overflow:hidden;display:inline-block;vertical-align:middle}"
        ".cpu-bar>div{height:100%%;background:#4caf50;transition:width 1s}"
        "#pst{padding:8px 0;border-bottom:1px solid #ddd;font-size:14px;color:#666}"
        "#pst.printing{background:#e8f5e9;color:#2e7d32;padding:8px;border-radius:4px;margin:8px 0}"
        "#pst.paused{background:#fff3e0;color:#e65100;padding:8px;border-radius:4px;margin:8px 0}"
        "#pst.error{background:#ffebee;color:#c62828;padding:8px;border-radius:4px;margin:8px 0}"
        ".pbar{background:#ddd;border-radius:4px;height:16px;margin:4px 0;overflow:hidden}"
        ".pbar>div{background:#4caf50;height:100%%;transition:width .3s}"
        "#pst .pctl{float:right;margin-top:-2px}"
        "#pst .pctl button{padding:2px 10px;font-size:12px;margin-left:4px;vertical-align:middle}"
        "table{width:100%%;border-collapse:collapse}"
        "th,td{text-align:left;padding:6px 8px;border-bottom:1px solid #ddd}"
        "button,input[type=submit]{padding:6px 16px;font-size:14px;cursor:pointer;border-radius:4px;border:1px solid #ccc;background:#f5f5f5}"
        "button:hover{background:#e8e8e8}"
        ".btn-red{background:#f44336;color:#fff;border-color:#d32f2f}"
        ".btn-red:hover{background:#d32f2f}"
        ".btn-green{background:#4CAF50;color:#fff;border-color:#388E3C}"
        ".btn-green:hover{background:#388E3C}"
        ".sh{display:flex;align-items:center;justify-content:space-between}"
        ".sh h2{margin:0}"
        ".hint{font-size:13px;color:#666}"
        "</style></head><body>"
        "<nav id='nav'>"
        "<a class='brand%s' href='/'>%s</a>"
        "<span class='cur'>%s</span>"
        "<button class='mb' aria-label='%s' "
        "onclick=\"document.getElementById('nav').classList.toggle('open')\">&#9776;</button>"
        "<span class='items'>",
        i18n_lang_code(i18n_lang()), TR(title),
        /* The print screen is the one for a printer at a glance: its name */
        IS("/print") ? " pn" : "", IS("/print") ? wifi_get_hostname() : "ESP32FDM",
        IS("/") ? "" : TR(title), _("Menu"));

    /* The brand links home, so on wide screens the list skips it */
    if (IS("/")) html_buf_printf(p, " <b class='home'>%s</b>", _("Home"));
    else html_buf_printf(p, " <a class='home' href='/'>%s</a>", _("Home"));
    nav_link(p, "/print", NT_("Print"), IS("/print"));
    nav_link(p, "/camera", NT_("Cam"), IS("/camera"));
    if (sdcard_is_mounted()) nav_link(p, "/sd", NT_("SD"), IS("/sd"));
    if (!is_klipper) nav_link(p, "/terminal", NT_("Term"), IS("/terminal"));
    nav_link(p, "/utils", NT_("Utils"),
             IS("/utils") || strncmp(current_path, "/bedlevel", 9) == 0);
    nav_link(p, "/logs", NT_("Logs"), IS("/logs"));
    nav_link(p, "/settings", NT_("Settings"), IS("/settings"));
#undef IS

    /* The print screen shows all of it itself */
    html_buf_printf(p, "</span></nav><div id='pst'%s></div>",
                    strcmp(current_path, "/print") == 0 ? " hidden" : "");
}

/* Built once per language by i18n_page(): a printf format, so % is doubled */
static const char END_TMPL[] =
    "<script>"
    "!function(){var p=document.getElementById('pst');"
    "function tp(s){return '" N_("Nozzle") " '+s.hotend[0].toFixed(0)+'/'+s.hotend[1].toFixed(0)+'\\u00b0C'"
    "+' &middot; " N_("Bed") " '+s.bed[0].toFixed(0)+'/'+s.bed[1].toFixed(0)+'\\u00b0C'}"
    "function u(){fetch('/api/status').then(function(r){return r.json()}).then(function(s){"
    "var a=s.state=='printing'||s.state=='paused';"
    "p.className=a?s.state:s.state=='error'?'error':'';"
    "if(a){var bt='<span class=pctl>';"
    "if(s.state=='printing')bt+='<button onclick=\"fetch(\\'/sd/pause\\',{method:\\'POST\\'})\">" N_("Pause") "</button>';"
    "else bt+='<button class=btn-green onclick=\"'+(s.hold?'fetch(\\'/print/continue\\',{method:\\'POST\\',body:\\'id='+s.hold_id+'\\'})':'fetch(\\'/sd/resume\\',{method:\\'POST\\'})')+'\">'+(s.hold?'" N_("Continue") "':'" N_("Resume") "')+'</button>';"
    "bt+='<button class=btn-red onclick=\"if(confirm(\\'" N_("Stop print?") "\\'))fetch(\\'/sd/cancel\\',{method:\\'POST\\'})\">" N_("Stop") "</button></span>';"
    "p.innerHTML=bt+'<b>'+(s.hold?'" N_("WAITING") "':s.state=='paused'?'" N_("PAUSED") "':s.preparing?'" N_("PREPARING") "':'" N_("PRINTING") "')+':</b> '+(s.hold?s.hold_msg+' &middot; ':'')+(s.filename||'')"
    "+' <a href=/print>" N_("Control") "</a>'"
    "+'<div class=pbar><div style=\"width:'+Math.min(100,s.progress||0)+'%%\"></div></div>'"
    "+(s.preparing?'':~~s.progress+'%%')+(s.layer&&s.layer.slice(-2)!='/0'?' &middot; " N_("Layer") " '+s.layer:'')"
    "+'<div style=\"font-size:12px;color:#555;margin-top:2px\">'+tp(s)+'</div>';}"
    "else if(s.upload>=0){p.className='printing';p.innerHTML='<b>" N_("Sending to the printer") ":</b> '+s.upload+'%%'"
    "+'<div class=pbar><div style=\"width:'+s.upload+'%%\"></div></div>'}"
    "else{p.innerHTML=s.backend+': '+(s.state=='error'?'" N_("Error") "':(s.backend=='Klipper'?s.state!='disconnected':s.usb)?'" N_("Ready") "':'" N_("Disconnected") "')"
    "+'<div style=\"font-size:12px;color:#888\">'+tp(s)+'</div>';}"
    "}).catch(function(){})}"
    "u();setInterval(u,3000);"
    "function cs(){fetch('/api/stats').then(function(r){return r.json()}).then(function(d){"
    "var b0=document.getElementById('cb0'),b1=document.getElementById('cb1');"
    "if(b0)b0.style.width=d.cpu0+'%%';"
    "if(b1)b1.style.width=d.cpu1+'%%';"
    "var c=function(v){return v>80?'#f44336':v>50?'#ff9800':'#4caf50'};"
    "if(b0)b0.style.background=c(d.cpu0);"
    "if(b1)b1.style.background=c(d.cpu1);"
    "}).catch(function(){})}"
    "cs();setInterval(cs,3000)}()"
    "</script>"
    "<footer style='margin-top:32px;padding:12px 0;border-top:1px solid #ddd;text-align:center;font-size:12px;color:#999'>"
    "&copy; Sergey Korotkov &amp; Claude 2026 &middot; ";

void layout_html_end(html_buf_t *p)
{
    char langs[256];
    i18n_render_switch(langs, sizeof(langs));

    html_buf_printf(p, i18n_page(END_TMPL));
    html_buf_printf(p,
        "<span title='%s'>v%s &middot; %s</span>"
        " &middot; <span class='lng'>%s</span> &middot; "
        "<span class='cpu-bars'>"
        "C0<span class='cpu-bar'><div id='cb0'></div></span>"
        "C1<span class='cpu-bar'><div id='cb1'></div></span>"
        "</span></footer></body></html>",
        _("Build time and commit, + if there were uncommitted changes"), FW_VERSION, FW_BUILD, langs);
}
