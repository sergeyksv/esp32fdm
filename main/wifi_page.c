#include "wifi_page.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cJSON.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "httpd.h"
#include "i18n.h"
#include "layout.h"
#include "url_util.h"
#include "wifi.h"

static const char *TAG = "wifi_page";

#define ROWS_MAX (WIFI_SEEN_MAX + WIFI_AP_NAMES_MAX)

static void fmt_bssid(char *out, const uint8_t *b)
{
    sprintf(out, MACSTR, MAC2STR(b));
}

static bool parse_bssid(const char *s, uint8_t *out)
{
    unsigned v[6];
    if (sscanf(s, "%2x:%2x:%2x:%2x:%2x:%2x", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) != 6) return false;
    for (int i = 0; i < 6; i++) out[i] = v[i];
    return true;
}

/** A name goes into HTML attributes and JSON: keep what cannot break them */
static void clean_name(char *s)
{
    char *d = s;
    for (const char *p = s; *p; p++) {
        unsigned char c = (unsigned char)*p;
        if (c < 0x20 || strchr("<>&\"'\\", c)) continue;
        *d++ = (char)c;
    }
    *d = '\0';
    /* No half of a UTF-8 letter that the length limit cut */
    size_t len = strlen(s), start = len;
    while (start > 0 && ((unsigned char)s[start - 1] & 0xC0) == 0x80) start--;
    if (start > 0 && ((unsigned char)s[start - 1] & 0x80)) {
        unsigned char lead = (unsigned char)s[start - 1];
        size_t need = lead >= 0xF0 ? 4 : lead >= 0xE0 ? 3 : 2;
        if (len - (start - 1) < need) s[start - 1] = '\0';
    }
}

static const char *signal_word(int rssi)
{
    if (rssi >= -55) return _("excellent");
    if (rssi >= -67) return _("good");
    if (rssi >= -75) return _("weak");
    return _("poor");
}

static const char *reason_text(unsigned r)
{
    switch (r) {
    case WIFI_REASON_AUTH_EXPIRE:          return _("authentication expired");
    case WIFI_REASON_AUTH_LEAVE:           return _("the AP disconnected the board");
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY: return _("inactivity");
    case WIFI_REASON_ASSOC_LEAVE:          return _("the board left (switching APs)");
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
    case WIFI_REASON_HANDSHAKE_TIMEOUT:    return _("handshake timeout (wrong password?)");
    case WIFI_REASON_BEACON_TIMEOUT:       return _("the AP stopped answering");
    case WIFI_REASON_NO_AP_FOUND:          return _("no AP found");
    case WIFI_REASON_AUTH_FAIL:            return _("authentication failed");
    case WIFI_REASON_ASSOC_FAIL:           return _("association failed");
    case WIFI_REASON_CONNECTION_FAIL:      return _("connection failed");
    case WIFI_REASON_ROAMING:              return _("moved by the network (802.11v)");
    default:                               return "";
    }
}

static void fmt_ago(char *out, size_t size, int64_t us)
{
    long s = (long)(us / 1000000);
    if (s < 120) snprintf(out, size, "%ld %s", s, _("s"));
    else if (s < 7200) snprintf(out, size, "%ld %s", s / 60, _("min"));
    else if (s < 172800) snprintf(out, size, "%ld %s %ld %s", s / 3600, _("h"), s / 60 % 60, _("min"));
    else snprintf(out, size, "%ld %s", s / 86400, _("d"));
}

static void row(html_buf_t *p, const char *label, const char *value)
{
    html_buf_printf(p, "<tr><td>%s</td><td>%s</td></tr>", label, value);
}

typedef struct {
    uint8_t bssid[6];
    int channel;            /* 0 = not seen in the last scan */
    int rssi;
    char name[26];
} ap_row_t;

static esp_err_t page_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, NT_("Wi-Fi"), "/settings");
    html_buf_printf(&p, "<h2>Wi-Fi</h2>");

    wifi_stats_t st;
    wifi_get_stats(&st);
    wifi_ap_record_t ap;
    bool connected = esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    int64_t now = esp_timer_get_time();
    char buf[96], name[26], ago[32];

    html_buf_printf(&p, "<table>");
    if (connected) {
        snprintf(buf, sizeof(buf), "<b>%s</b>", (char *)ap.ssid);
        row(&p, _("Network"), buf);
        char bssid[18];
        fmt_bssid(bssid, ap.bssid);
        wifi_ap_name(ap.bssid, name, sizeof(name));
        snprintf(buf, sizeof(buf), "%s%s<code>%s</code>", name, name[0] ? " &middot; " : "", bssid);
        row(&p, _("Access point"), buf);
        snprintf(buf, sizeof(buf), "<b>%d dBm</b> &middot; %s", ap.rssi, signal_word(ap.rssi));
        row(&p, _("Signal"), buf);
        snprintf(buf, sizeof(buf), "%d &middot; 802.11%s%s%s%s", ap.primary,
                 ap.phy_11b ? "b" : "", ap.phy_11g ? "g" : "", ap.phy_11n ? "n" : "", ap.phy_11ax ? "ax" : "");
        row(&p, _("Channel, mode"), buf);
        if (st.connected_us) {
            fmt_ago(ago, sizeof(ago), now - st.connected_us);
            row(&p, _("Connected for"), ago);
        }
    } else {
        row(&p, _("Network"), _("not connected"));
    }

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;
    if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        snprintf(buf, sizeof(buf), IPSTR " / " IPSTR, IP2STR(&ip.ip), IP2STR(&ip.netmask));
        row(&p, _("IP / mask"), buf);
        snprintf(buf, sizeof(buf), IPSTR, IP2STR(&ip.gw));
        row(&p, _("Gateway"), buf);
        esp_netif_dns_info_t dns;
        if (esp_netif_get_dns_info(netif, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK) {
            snprintf(buf, sizeof(buf), IPSTR, IP2STR(&dns.ip.u_addr.ip4));
            row(&p, "DNS", buf);
        }
    }
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(buf, sizeof(buf), "<code>" MACSTR "</code>", MAC2STR(mac));
    row(&p, _("Board MAC"), buf);

    snprintf(buf, sizeof(buf), "%lu / %lu", (unsigned long)st.disconnects, (unsigned long)st.reconnects);
    row(&p, _("Disconnects / reconnects"), buf);
    snprintf(buf, sizeof(buf), "%lu / %lu", (unsigned long)st.roams, (unsigned long)st.btm_roams);
    row(&p, _("Moves: by the board / by the network"), buf);
    if (st.disconnected_us) {
        fmt_ago(ago, sizeof(ago), now - st.disconnected_us);
        const char *why = reason_text(st.last_reason);
        snprintf(buf, sizeof(buf), "%s %s &middot; %s%s%u%s", ago, _("ago"),
                 why, why[0] ? " (" : "", st.last_reason, why[0] ? ")" : "");
        row(&p, _("Last disconnect"), buf);
    }
    html_buf_printf(&p, "</table><p class='hint'>%s</p>", _("Counted since the board started."));

    /* The APs: seen in the last scan, then named ones not seen */
    ap_row_t *rows = calloc(ROWS_MAX, sizeof(ap_row_t));
    wifi_seen_ap_t *seen = calloc(WIFI_SEEN_MAX, sizeof(wifi_seen_ap_t));
    wifi_ap_name_t *names = calloc(WIFI_AP_NAMES_MAX, sizeof(wifi_ap_name_t));
    if (!rows || !seen || !names) {
        free(rows);
        free(seen);
        free(names);
        html_buf_free(&p);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Out of memory."));
        return ESP_FAIL;
    }
    int64_t scan_us;
    int n_seen = wifi_get_seen(seen, WIFI_SEEN_MAX, &scan_us);
    int n_names = wifi_get_ap_names(names, WIFI_AP_NAMES_MAX);
    int n = 0;
    for (int i = 0; i < n_seen; i++) {
        memcpy(rows[n].bssid, seen[i].bssid, 6);
        rows[n].channel = seen[i].channel;
        rows[n].rssi = seen[i].rssi;
        wifi_ap_name(seen[i].bssid, rows[n].name, sizeof(rows[n].name));
        n++;
    }
    for (int i = 0; i < n_names && n < ROWS_MAX; i++) {
        bool dup = false;
        for (int k = 0; k < n; k++) dup |= memcmp(rows[k].bssid, names[i].bssid, 6) == 0;
        if (dup) continue;
        memcpy(rows[n].bssid, names[i].bssid, 6);
        strlcpy(rows[n].name, names[i].name, sizeof(rows[n].name));
        n++;
    }
    uint8_t pref[6];
    bool has_pref = wifi_get_preferred(pref);

    html_buf_printf(&p, "<div class='sh' style='margin-top:16px'><h3 style='margin:0'>%s</h3>"
                        "<button type='button' id='scan' onclick=\"this.disabled=true;"
                        "fetch('/wifi/scan',{method:'POST'}).then(function(){setTimeout(function(){location.reload()},4000)})\">"
                        "%s</button></div>",
                    _("Access points"), _("Scan"));
    if (scan_us) {
        fmt_ago(ago, sizeof(ago), now - scan_us);
        html_buf_printf(&p, "<p class='hint'>%s %s %s. %s</p>", _("Last scan"), ago, _("ago"),
                        _("A scan pauses Wi-Fi for about 2 s: the camera skips a few frames, a print over USB does not notice."));
    }
    html_buf_printf(&p,
        "<form method='POST' action='/wifi/aps'><table>"
        "<tr><th>%s</th><th>%s</th><th>BSSID</th><th>%s</th><th>dBm</th></tr>",
        _("Preferred"), _("Name"), _("Ch"));
    for (int i = 0; i < n; i++) {
        char bssid[18];
        fmt_bssid(bssid, rows[i].bssid);
        bool cur = connected && memcmp(rows[i].bssid, ap.bssid, 6) == 0;
        bool is_pref = has_pref && memcmp(rows[i].bssid, pref, 6) == 0;
        html_buf_printf(&p,
            "<tr%s><td><input type='radio' name='pref' value='%s'%s></td>"
            "<td><input type='hidden' name='b%d' value='%s'>"
            "<input name='n%d' value='%s' maxlength='24' style='width:100%%;min-width:70px;box-sizing:border-box'></td>"
            "<td><code style='font-size:12px'>%s</code>%s</td><td>%s</td><td>%s</td></tr>",
            cur ? " style='background:#e8f5e9'" : "",
            bssid, is_pref ? " checked" : "",
            i, bssid, i, rows[i].name,
            bssid, cur ? " &#10003;" : "",
            rows[i].channel ? (snprintf(buf, sizeof(buf), "%d", rows[i].channel), buf) : "&mdash;",
            rows[i].channel ? (snprintf(name, sizeof(name), "%d", rows[i].rssi), name) : _("not seen"));
    }
    html_buf_printf(&p,
        "<tr><td><input type='radio' name='pref' value=''%s></td><td colspan='4'>%s</td></tr>"
        "</table><input type='hidden' name='rows' value='%d'>"
        "<p class='hint'>%s</p><p><button type='submit'>%s</button></p></form>",
        has_pref ? "" : " checked", _("No preferred AP: always the strongest"), n,
        _("The board takes the preferred AP while its signal is -75 dBm or better, else the strongest, "
          "and goes back to it within minutes when it is in range again (not for 30 min after the "
          "network moved the board). The network (DAWN) can still move it with 802.11v. "
          "Names are the same on every board: they are copied with the settings."),
        _("Save"));

    free(rows);
    free(seen);
    free(names);
    layout_html_end(&p);
    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t scan_handler(httpd_req_t *req)
{
    wifi_request_scan();
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t aps_handler(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > 8192) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, _("Missing or bad field."));
        return ESP_FAIL;
    }
    char *body = malloc(req->content_len + 1);
    if (!body) return ESP_FAIL;
    size_t got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, body + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (r <= 0) {
            free(body);
            return ESP_FAIL;
        }
        got += r;
    }
    body[got] = '\0';

    int rows = 0;
    form_int(body, "rows", &rows);
    if (rows > ROWS_MAX) rows = ROWS_MAX;
    wifi_ap_name_t names[WIFI_AP_NAMES_MAX];
    int n = 0;
    for (int i = 0; i < rows && n < WIFI_AP_NAMES_MAX; i++) {
        char key[8], bssid_s[24], nm[26];
        snprintf(key, sizeof(key), "b%d", i);
        if (!form_str(body, key, bssid_s, sizeof(bssid_s))) continue;
        snprintf(key, sizeof(key), "n%d", i);
        if (!form_str(body, key, nm, sizeof(nm))) continue;
        clean_name(nm);
        if (!nm[0] || !parse_bssid(bssid_s, names[n].bssid)) continue;
        strlcpy(names[n].name, nm, sizeof(names[n].name));
        n++;
    }
    wifi_set_ap_names(names, n);

    char pref_s[24] = "";
    uint8_t pref[6];
    form_str(body, "pref", pref_s, sizeof(pref_s));
    wifi_set_preferred(parse_bssid(pref_s, pref) ? pref : NULL);
    free(body);
    ESP_LOGI(TAG, "%d AP names saved, preferred %s", n, pref_s[0] ? pref_s : "none");

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/wifi");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- Copy between boards: the names, not the preferred AP, which depends
 * on where the printer stands ---- */

void wifi_page_settings_export(cJSON *obj)
{
    wifi_ap_name_t names[WIFI_AP_NAMES_MAX];
    int n = wifi_get_ap_names(names, WIFI_AP_NAMES_MAX);
    cJSON *list = cJSON_AddArrayToObject(obj, "names");
    for (int i = 0; list && i < n; i++) {
        cJSON *o = cJSON_CreateObject();
        if (!o) break;
        char b[18];
        fmt_bssid(b, names[i].bssid);
        cJSON_AddStringToObject(o, "bssid", b);
        cJSON_AddStringToObject(o, "name", names[i].name);
        cJSON_AddItemToArray(list, o);
    }
}

void wifi_page_settings_import(const cJSON *obj)
{
    const cJSON *list = cJSON_GetObjectItem(obj, "names");
    if (!cJSON_IsArray(list)) return;
    wifi_ap_name_t names[WIFI_AP_NAMES_MAX];
    int n = 0;
    const cJSON *o;
    cJSON_ArrayForEach(o, list) {
        if (n >= WIFI_AP_NAMES_MAX) break;
        const cJSON *b = cJSON_GetObjectItem(o, "bssid");
        const cJSON *nm = cJSON_GetObjectItem(o, "name");
        if (!cJSON_IsString(b) || !cJSON_IsString(nm)) continue;
        if (!parse_bssid(b->valuestring, names[n].bssid)) continue;
        strlcpy(names[n].name, nm->valuestring, sizeof(names[n].name));
        clean_name(names[n].name);
        if (names[n].name[0]) n++;
    }
    wifi_set_ap_names(names, n);
}

esp_err_t wifi_page_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/wifi",      .method = HTTP_GET,  .handler = page_handler },
        { .uri = "/wifi/scan", .method = HTTP_POST, .handler = scan_handler },
        { .uri = "/wifi/aps",  .method = HTTP_POST, .handler = aps_handler },
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }
    return ESP_OK;
}
