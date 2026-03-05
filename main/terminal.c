#include "terminal.h"

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "printer_comm.h"
#include "layout.h"

static const char *TAG = "terminal";

/* ---- Ring buffer ---- */

#define TERM_RING_SIZE 4096

static char s_ring[TERM_RING_SIZE];
static int  s_ring_head = 0;       /* write position in ring */
static uint32_t s_seq = 0;         /* monotonic total bytes written */
static SemaphoreHandle_t s_mutex;

void terminal_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Terminal initialized (ring %d bytes)", TERM_RING_SIZE);
}

static void ring_write(const char *data, size_t len)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (size_t i = 0; i < len; i++) {
        s_ring[s_ring_head] = data[i];
        s_ring_head = (s_ring_head + 1) % TERM_RING_SIZE;
    }
    s_seq += len;
    xSemaphoreGive(s_mutex);
}

void terminal_feed_rx(const uint8_t *data, size_t len)
{
    if (len == 0 || !s_mutex) return;
    ring_write((const char *)data, len);
}

/* ---- Send command ---- */

static esp_err_t terminal_send_cmd(const char *cmd)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED)
        return ESP_ERR_INVALID_STATE;

    /* Echo to ring buffer */
    ring_write("> ", 2);
    ring_write(cmd, strlen(cmd));
    ring_write("\n", 1);

    /* Queue via printer_comm */
    printer_cmd_t pcmd = { .type = PCMD_RAW };
    strncpy(pcmd.gcode, cmd, sizeof(pcmd.gcode) - 1);
    pcmd.gcode[sizeof(pcmd.gcode) - 1] = '\0';
    return printer_comm_send_cmd(&pcmd);
}

/* ---- HTTP handlers ---- */

static esp_err_t terminal_page_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);

    layout_html_begin(&p, "Terminal", "/terminal");
    html_buf_printf(&p,
        "<h2>Terminal</h2>"
        "<pre id='out' style='background:#1e1e1e;color:#d4d4d4;padding:12px;"
        "height:400px;overflow-y:auto;font-size:13px;border-radius:4px;"
        "white-space:pre-wrap;word-wrap:break-word'></pre>"
        "<div id='lock' style='display:none;padding:8px;background:#fff3e0;"
        "color:#e65100;border-radius:4px;margin:8px 0;text-align:center'>"
        "Terminal locked during print</div>"
        "<form id='frm' style='display:flex;gap:8px;margin-top:8px' onsubmit='return send()'>"
        "<input id='inp' type='text' placeholder='GCode command (e.g. M105)'"
        " style='flex:1;padding:8px;font-family:monospace;font-size:14px' autocomplete='off'>"
        "<button type='submit' style='padding:8px 20px'>Send</button>"
        "</form>");

    html_buf_printf(&p,
        "<script>"
        "var out=document.getElementById('out'),"
        "inp=document.getElementById('inp'),"
        "frm=document.getElementById('frm'),"
        "lck=document.getElementById('lock'),"
        "seq=0,locked=false;"

        "function poll(){"
        "fetch('/terminal/poll?seq='+seq).then(function(r){return r.json()}).then(function(j){"
        "if(j.data){out.textContent+=j.data;out.scrollTop=out.scrollHeight}"
        "seq=j.seq"
        "}).catch(function(){})}"

        "function chk(){"
        "fetch('/api/status').then(function(r){return r.json()}).then(function(s){"
        "locked=s.state=='printing'||s.state=='paused';"
        "inp.disabled=locked;lck.style.display=locked?'':'none'"
        "}).catch(function(){})}"

        "function send(){"
        "if(locked)return false;"
        "var c=inp.value.trim();if(!c)return false;"
        "fetch('/terminal/send',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},"
        "body:'cmd='+encodeURIComponent(c)}).then(function(r){"
        "if(!r.ok)r.text().then(function(t){out.textContent+='[Error: '+t+']\\n';out.scrollTop=out.scrollHeight})"
        "});inp.value='';return false}"

        "poll();setInterval(poll,500);"
        "chk();setInterval(chk,3000);"
        "inp.focus();"
        "</script>");

    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t terminal_send_handler(httpd_req_t *req)
{
    char buf[128];
    int recv_len = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (recv_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No data");
        return ESP_FAIL;
    }
    buf[recv_len] = '\0';

    /* Parse cmd= field */
    const char *cmd_start = strstr(buf, "cmd=");
    if (!cmd_start) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing cmd");
        return ESP_FAIL;
    }
    cmd_start += 4;

    /* URL-decode in place (simple: just handle + and %XX) */
    char cmd[96];
    size_t ci = 0;
    const char *s = cmd_start;
    while (*s && *s != '&' && ci < sizeof(cmd) - 1) {
        if (*s == '+') {
            cmd[ci++] = ' ';
            s++;
        } else if (*s == '%' && s[1] && s[2]) {
            char hex[3] = {s[1], s[2], '\0'};
            cmd[ci++] = (char)strtol(hex, NULL, 16);
            s += 3;
        } else {
            cmd[ci++] = *s++;
        }
    }
    cmd[ci] = '\0';

    if (cmd[0] == '\0') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty command");
        return ESP_FAIL;
    }

    esp_err_t err = terminal_send_cmd(cmd);
    if (err == ESP_ERR_INVALID_STATE) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, "Printing in progress");
        return ESP_OK;
    }
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Send failed");
        return ESP_FAIL;
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;
}

static esp_err_t terminal_poll_handler(httpd_req_t *req)
{
    /* Parse seq from query string */
    char qbuf[32] = {0};
    uint32_t client_seq = 0;
    if (httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(qbuf, "seq", val, sizeof(val)) == ESP_OK) {
            client_seq = (uint32_t)strtoul(val, NULL, 10);
        }
    }

    char *json = NULL;
    xSemaphoreTake(s_mutex, portMAX_DELAY);

    uint32_t avail;
    if (client_seq >= s_seq) {
        /* No new data */
        avail = 0;
    } else if (s_seq - client_seq > TERM_RING_SIZE) {
        /* Client fell behind — send everything in buffer */
        avail = TERM_RING_SIZE;
    } else {
        avail = s_seq - client_seq;
    }

    if (avail == 0) {
        xSemaphoreGive(s_mutex);
        /* Empty response */
        char resp[32];
        snprintf(resp, sizeof(resp), "{\"seq\":%lu,\"data\":\"\"}", (unsigned long)s_seq);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, resp, strlen(resp));
    }

    /* Allocate buffer for JSON: {"seq":N,"data":"..."} with escaping headroom */
    size_t json_cap = 32 + avail * 2;  /* worst case: every byte needs escaping */
    json = malloc(json_cap);
    if (!json) {
        xSemaphoreGive(s_mutex);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
        return ESP_FAIL;
    }

    int pos = snprintf(json, json_cap, "{\"seq\":%lu,\"data\":\"", (unsigned long)s_seq);

    /* Read from ring buffer, JSON-escape as we go */
    int start = (s_ring_head - (int)avail + TERM_RING_SIZE) % TERM_RING_SIZE;
    for (uint32_t i = 0; i < avail && pos < (int)json_cap - 8; i++) {
        char c = s_ring[(start + i) % TERM_RING_SIZE];
        if (c == '"') { json[pos++] = '\\'; json[pos++] = '"'; }
        else if (c == '\\') { json[pos++] = '\\'; json[pos++] = '\\'; }
        else if (c == '\n') { json[pos++] = '\\'; json[pos++] = 'n'; }
        else if (c == '\r') { json[pos++] = '\\'; json[pos++] = 'r'; }
        else if (c == '\t') { json[pos++] = '\\'; json[pos++] = 't'; }
        else if ((unsigned char)c < 0x20) { /* skip other control chars */ }
        else { json[pos++] = c; }
    }
    json[pos++] = '"';
    json[pos++] = '}';
    json[pos] = '\0';

    xSemaphoreGive(s_mutex);

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, json, pos);
    free(json);
    return ret;
}

/* ---- Registration ---- */

esp_err_t terminal_register_httpd(httpd_handle_t server)
{
    httpd_uri_t page = {
        .uri = "/terminal", .method = HTTP_GET, .handler = terminal_page_handler,
    };
    httpd_register_uri_handler(server, &page);

    httpd_uri_t send_uri = {
        .uri = "/terminal/send", .method = HTTP_POST, .handler = terminal_send_handler,
    };
    httpd_register_uri_handler(server, &send_uri);

    httpd_uri_t poll_uri = {
        .uri = "/terminal/poll", .method = HTTP_GET, .handler = terminal_poll_handler,
    };
    httpd_register_uri_handler(server, &poll_uri);

    ESP_LOGI(TAG, "Terminal endpoints registered");
    return ESP_OK;
}
