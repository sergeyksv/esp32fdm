#include "sdcard_httpd.h"
#include "sdcard.h"
#include "printer_comm.h"

#include "esp_log.h"
#include "esp_http_server.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

static const char *TAG = "sdcard_httpd";

#define MOUNT_POINT "/sdcard"
#define UPLOAD_BUF_SIZE 4096

/* ---- HTML page ---- */

static const char SD_HTML[] =
    "<!DOCTYPE html><html><head><title>SD Card</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>"
    "body{font-family:sans-serif;max-width:600px;margin:20px auto;padding:0 10px}"
    "table{width:100%;border-collapse:collapse}th,td{text-align:left;padding:6px 8px;border-bottom:1px solid #ddd}"
    ".sz{color:#666;font-size:.9em}"
    "button{padding:4px 12px;margin:2px;cursor:pointer}"
    ".del{background:#f44;color:#fff;border:none;border-radius:3px}"
    ".prt{background:#4a4;color:#fff;border:none;border-radius:3px}"
    ".ctrl{background:#48f;color:#fff;border:none;border-radius:3px;padding:8px 20px;font-size:15px;margin:4px}"
    "#prog{width:100%;height:20px;margin:8px 0}"
    "#upload-prog{width:100%;display:none;margin:4px 0}"
    "#status{padding:8px;margin:8px 0;background:#eef;border-radius:4px;display:none}"
    "</style></head><body>"
    "<h1>SD Card</h1>"
    "<div id='status'></div>"
    "<div id='controls' style='display:none'>"
    "<button class='ctrl' onclick='doCtrl(\"pause\")'>Pause</button>"
    "<button class='ctrl' onclick='doCtrl(\"resume\")'>Resume</button>"
    "<button class='del ctrl' onclick='doCtrl(\"cancel\")'>Cancel</button>"
    "</div>"
    "<h2>Upload</h2>"
    "<input type='file' id='file' accept='.gcode,.g,.gc'>"
    "<button onclick='upload()'>Upload</button>"
    "<progress id='upload-prog' value='0' max='100'></progress>"
    "<span id='upload-status'></span>"
    "<h2>Files</h2>"
    "<table><thead><tr><th>Name</th><th>Size</th><th></th></tr></thead>"
    "<tbody id='files'></tbody></table>"
    "<p><a href='/'>Home</a></p>"
    "<script>"
    "function fmt(b){if(b<1024)return b+'B';if(b<1048576)return (b/1024).toFixed(1)+'KB';return (b/1048576).toFixed(1)+'MB'}"
    "function load(){"
    "fetch('/sd/files').then(r=>r.json()).then(f=>{"
    "let h='';f.forEach(x=>{"
    "h+='<tr><td>'+x.name+'</td><td class=sz>'+fmt(x.size)+'</td><td>'"
    "+'<form method=POST action=/sd/print style=display:inline><input type=hidden name=filename value=\"'+x.name+'\"><button class=prt type=submit>Print</button></form> '"
    "+'<form method=POST action=/sd/delete style=display:inline><input type=hidden name=filename value=\"'+x.name+'\"><button class=del type=submit>Del</button></form>'"
    "+'</td></tr>'});"
    "document.getElementById('files').innerHTML=h||'<tr><td colspan=3>No files</td></tr>';"
    "});}"
    "function upload(){"
    "let f=document.getElementById('file').files[0];if(!f)return;"
    "let x=new XMLHttpRequest();"
    "x.upload.onprogress=function(e){if(e.lengthComputable){"
    "let p=document.getElementById('upload-prog');p.style.display='block';p.value=Math.round(e.loaded/e.total*100);"
    "document.getElementById('upload-status').textContent=fmt(e.loaded)+'/'+fmt(e.total);}};"
    "x.onload=function(){document.getElementById('upload-prog').style.display='none';"
    "document.getElementById('upload-status').textContent=x.status==200?'Done':'Error: '+x.responseText;load();};"
    "x.open('POST','/sd/upload');x.setRequestHeader('X-Filename',f.name);x.send(f);}"
    "function doCtrl(act){fetch('/sd/'+act,{method:'POST'}).then(()=>pollStatus());}"
    "function pollStatus(){"
    "fetch('/sd/status').then(r=>r.json()).then(s=>{"
    "let el=document.getElementById('status');"
    "let ctrl=document.getElementById('controls');"
    "if(s.printing){"
    "el.style.display='block';ctrl.style.display='block';"
    "el.innerHTML='<b>'+(s.paused?'PAUSED':'PRINTING')+':</b> '+s.filename"
    "+'<br>Progress: '+s.progress.toFixed(1)+'% | Layer: '+s.layer"
    "+'<br>Elapsed: '+Math.floor(s.elapsed/60)+'m'+s.elapsed%60+'s';"
    "}else{el.style.display='none';ctrl.style.display='none';}});}"
    "load();pollStatus();setInterval(pollStatus,3000);"
    "</script></body></html>";

/* ---- GET /sd — HTML file manager ---- */

static esp_err_t sd_page_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, SD_HTML, sizeof(SD_HTML) - 1);
}

/* ---- GET /sd/files — JSON file list ---- */

static esp_err_t sd_files_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    if (!sdcard_is_mounted()) {
        return httpd_resp_send(req, "[]", 2);
    }

    DIR *dir = opendir(MOUNT_POINT);
    if (!dir) {
        return httpd_resp_send(req, "[]", 2);
    }

    httpd_resp_sendstr_chunk(req, "[");
    bool first = true;
    struct dirent *ent;
    char path[300];
    struct stat st;

    while ((ent = readdir(dir)) != NULL) {
        if (ent->d_name[0] == '.') continue;

        snprintf(path, sizeof(path), MOUNT_POINT "/%s", ent->d_name);
        if (stat(path, &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) continue;

        char entry[320];
        snprintf(entry, sizeof(entry), "%s{\"name\":\"%s\",\"size\":%ld}",
                 first ? "" : ",", ent->d_name, (long)st.st_size);
        httpd_resp_sendstr_chunk(req, entry);
        first = false;
    }
    closedir(dir);

    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* ---- POST /sd/upload — stream file to SD ---- */

static esp_err_t sd_upload_handler(httpd_req_t *req)
{
    if (!sdcard_is_mounted()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD card not mounted");
        return ESP_FAIL;
    }

    /* Get filename from header */
    char filename[64];
    if (httpd_req_get_hdr_value_str(req, "X-Filename", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing X-Filename header");
        return ESP_FAIL;
    }

    /* Sanitize: strip path separators */
    char *slash = strrchr(filename, '/');
    if (slash) memmove(filename, slash + 1, strlen(slash + 1) + 1);
    slash = strrchr(filename, '\\');
    if (slash) memmove(filename, slash + 1, strlen(slash + 1) + 1);

    if (filename[0] == '\0' || filename[0] == '.') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }

    char path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/%s", filename);

    FILE *f = fopen(path, "w");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
        return ESP_FAIL;
    }

    char *buf = malloc(UPLOAD_BUF_SIZE);
    if (!buf) {
        fclose(f);
        unlink(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int total = remaining;
    esp_err_t ret = ESP_OK;

    while (remaining > 0) {
        int to_read = remaining < UPLOAD_BUF_SIZE ? remaining : UPLOAD_BUF_SIZE;
        int received = httpd_req_recv(req, buf, to_read);
        if (received <= 0) {
            ESP_LOGE(TAG, "Upload recv error: %d", received);
            ret = ESP_FAIL;
            break;
        }
        if (fwrite(buf, 1, received, f) != (size_t)received) {
            ESP_LOGE(TAG, "SD write error");
            ret = ESP_FAIL;
            break;
        }
        remaining -= received;
    }

    free(buf);
    fclose(f);

    if (ret != ESP_OK) {
        unlink(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Upload failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Uploaded %s (%d bytes)", filename, total);
    return httpd_resp_send(req, "OK", 2);
}

/* ---- POST /sd/delete ---- */

static esp_err_t sd_delete_handler(httpd_req_t *req)
{
    char body[128];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    char *fn = strstr(body, "filename=");
    if (!fn) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }
    fn += 9;
    char *amp = strchr(fn, '&');
    if (amp) *amp = '\0';

    char path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/%s", fn);

    if (unlink(path) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Deleted %s", fn);

    /* Redirect back to /sd */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/sd");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- POST /sd/print ---- */

static esp_err_t sd_print_handler(httpd_req_t *req)
{
    char body[128];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Empty body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    char *fn = strstr(body, "filename=");
    if (!fn) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }
    fn += 9;
    char *amp = strchr(fn, '&');
    if (amp) *amp = '\0';

    esp_err_t err = printer_comm_host_print_start(fn);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to start print");
        return ESP_FAIL;
    }

    /* Redirect back to /sd */
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/sd");
    return httpd_resp_send(req, NULL, 0);
}

/* ---- POST /sd/pause, /sd/resume, /sd/cancel ---- */

static esp_err_t sd_pause_handler(httpd_req_t *req)
{
    printer_comm_host_print_pause();
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t sd_resume_handler(httpd_req_t *req)
{
    printer_comm_host_print_resume();
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t sd_cancel_handler(httpd_req_t *req)
{
    printer_comm_host_print_cancel();
    return httpd_resp_send(req, "OK", 2);
}

/* ---- GET /sd/status — JSON print status ---- */

static esp_err_t sd_status_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    printer_state_t st;
    printer_comm_get_state(&st);

    bool host = printer_comm_is_host_printing();
    bool paused = (st.opstate == PRINTER_PAUSED);

    char json[256];
    if (host) {
        snprintf(json, sizeof(json),
                 "{\"printing\":true,\"paused\":%s,\"filename\":\"%s\","
                 "\"progress\":%.1f,\"layer\":%ld,\"elapsed\":%ld}",
                 paused ? "true" : "false",
                 st.filename,
                 st.progress_pct >= 0 ? st.progress_pct : 0.0f,
                 (long)st.current_layer,
                 (long)(st.print_time_s >= 0 ? st.print_time_s : 0));
    } else {
        snprintf(json, sizeof(json), "{\"printing\":false}");
    }

    return httpd_resp_send(req, json, strlen(json));
}

/* ---- Registration ---- */

esp_err_t sdcard_httpd_register(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/sd",        .method = HTTP_GET,  .handler = sd_page_handler },
        { .uri = "/sd/files",  .method = HTTP_GET,  .handler = sd_files_handler },
        { .uri = "/sd/upload", .method = HTTP_POST, .handler = sd_upload_handler },
        { .uri = "/sd/delete", .method = HTTP_POST, .handler = sd_delete_handler },
        { .uri = "/sd/print",  .method = HTTP_POST, .handler = sd_print_handler },
        { .uri = "/sd/pause",  .method = HTTP_POST, .handler = sd_pause_handler },
        { .uri = "/sd/resume", .method = HTTP_POST, .handler = sd_resume_handler },
        { .uri = "/sd/cancel", .method = HTTP_POST, .handler = sd_cancel_handler },
        { .uri = "/sd/status", .method = HTTP_GET,  .handler = sd_status_handler },
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        httpd_register_uri_handler(server, &uris[i]);
    }

    ESP_LOGI(TAG, "SD card HTTP endpoints registered");
    return ESP_OK;
}
