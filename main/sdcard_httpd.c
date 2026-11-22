#include "sdcard_httpd.h"
#include "task_util.h"
#include "httpd.h"
#include "sdcard.h"
#include "printer_comm.h"
#include "printer_comm_klipper.h"
#include "layout.h"
#include "i18n.h"
#include "print_page.h"
#include "url_util.h"

#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"

#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/unistd.h>

static const char *TAG = "sdcard_httpd";

#define MOUNT_POINT "/sdcard"
#define UPLOAD_BUF_SIZE 8192

static volatile int s_uploads;          /* uploads to the SD card running */
static volatile bool s_format_running;  /* set in the handler (httpd task), cleared by the format task */


/* ---- GET /sd — HTML file manager ---- */

/* A template for i18n_page(): not a printf format, % is written once */
static const char SD_PAGE[] =
        "<style>"
        "#modal-bg{display:none;position:fixed;top:0;left:0;width:100%;height:100%;background:rgba(0,0,0,0.5);z-index:100;align-items:center;justify-content:center}"
        "#modal-bg.show{display:flex}"
        "#modal{background:#fff;border-radius:8px;padding:20px;max-width:420px;width:90%;max-height:80vh;overflow-y:auto;position:relative}"
        "#modal h3{margin:0 0 12px}"
        "#modal .close{position:absolute;top:8px;right:12px;font-size:1.4em;cursor:pointer;background:none;border:none;color:#666}"
        "#modal .thumb{text-align:center;margin:8px 0}"
        "#modal .thumb img{max-width:100%;max-height:200px;border-radius:4px}"
        "#modal table{width:100%}"
        "#modal td{padding:4px 8px}"
        "#modal td:first-child{color:#666;white-space:nowrap}"
        "</style>"
        "<div id='modal-bg' onclick='if(event.target===this)closeInfo()'>"
        "<div id='modal'>"
        "<button class='close' onclick='closeInfo()'>&times;</button>"
        "<h3 id='info-name'></h3>"
        "<div class='thumb' id='info-thumb'></div>"
        "<table>"
        "<tr><td>" N_("Size") "</td><td id='info-size'></td></tr>"
        "<tr><td>" N_("Layers") "</td><td id='info-layers'></td></tr>"
        "<tr><td>" N_("Layer height") "</td><td id='info-lh'></td></tr>"
        "<tr><td>" N_("Object height") "</td><td id='info-oh'></td></tr>"
        "<tr><td>" N_("Estimated time") "</td><td id='info-time'></td></tr>"
        "<tr><td>" N_("Filament") "</td><td id='info-fil'></td></tr>"
        "</table>"
        "</div></div>"
        "<p id='card' class='hint'></p>"
        "<h3>" N_("Upload") "</h3>"
        "<input type='file' id='file' accept='.gcode,.g,.gc'>"
        "<button onclick='upload()'>" N_("Upload") "</button>"
        "<progress id='upload-prog' value='0' max='100' style='width:100%;display:none;margin:4px 0'></progress>"
        "<span id='upload-status'></span>"
        "<h3>" N_("Files") "</h3>"
        "<table><thead><tr><th>" N_("File") "</th><th></th></tr></thead>"
        "<tbody id='files'></tbody></table>"
        "<script>"
        /* Before a print: the checks and what maintenance is overdue, asked once, then the form goes */
        "function sdPrint(f){var b=f.querySelector('button');"
        "function go(){b.disabled=true;b.textContent='" N_("Starting...") "';f.submit()}"
        "fetch('/print/checks').then(function(r){return r.json()}).then(function(d){"
        "var q=(d.warn?'\\u26a0 '+d.warn+'\\n\\n':'')+(d.n?'" N_("Maintenance is overdue:") " '+d.text+'\\n\\n"
        N_("Print anyway? It is recorded in the maintenance log.") "':'" N_("Print anyway?") "');"
        "if(!d.n&&!d.warn||confirm(q))go()})"
        ".catch(go);return false}"
        "function fmt(b){if(b<1024)return b+'B';if(b<1048576)return(b/1024).toFixed(1)+'KB';return(b/1048576).toFixed(1)+'MB'}"
        "function fmtDate(t){if(!t)return'-';var d=new Date(t*1000),m=d.getMonth()+1,dy=d.getDate(),y=d.getFullYear(),h=d.getHours(),mi=d.getMinutes();return y+'-'+(m<10?'0':'')+m+'-'+(dy<10?'0':'')+dy+' '+(h<10?'0':'')+h+':'+(mi<10?'0':'')+mi}"
        "function fmtTime(s){if(s<0)return'" N_("Unknown") "';var h=Math.floor(s/3600),m=Math.floor((s%3600)/60);return h>0?h+'" N_("h") " '+m+'" N_("min") "':m+'" N_("min") "'}"
        "function info(name){"
        "document.getElementById('info-name').textContent=name;"
        "document.getElementById('info-thumb').innerHTML='" N_("Loading...") "';"
        "document.getElementById('info-size').textContent='...';"
        "document.getElementById('info-layers').textContent='...';"
        "document.getElementById('info-lh').textContent='...';"
        "document.getElementById('info-oh').textContent='...';"
        "document.getElementById('info-time').textContent='...';"
        "document.getElementById('info-fil').textContent='...';"
        "document.getElementById('modal-bg').classList.add('show');"
        "fetch('/sd/info?filename='+encodeURIComponent(name)).then(function(r){return r.json()}).then(function(d){"
        "document.getElementById('info-size').textContent=fmt(d.size);"
        "document.getElementById('info-layers').textContent=d.layers>0?d.layers:'" N_("Unknown") "';"
        "document.getElementById('info-lh').textContent=d.layerHeight>0?d.layerHeight.toFixed(2)+' " N_("mm") "':'" N_("Unknown") "';"
        "document.getElementById('info-oh').textContent=d.objectHeight>0?d.objectHeight.toFixed(1)+' " N_("mm") "':'" N_("Unknown") "';"
        "document.getElementById('info-time').textContent=fmtTime(d.estTime);"
        "var fil='';"
        "if(d.filamentMm>0){fil=(d.filamentMm/1000).toFixed(2)+' " N_("m") "';if(d.filamentG>0)fil+=' / '+d.filamentG.toFixed(1)+' " N_("g") "'}"
        "else fil='" N_("Unknown") "';"
        "document.getElementById('info-fil').textContent=fil;"
        "if(d.thumbnail)document.getElementById('info-thumb').innerHTML='<img src=\"'+d.thumbnail+'\">';"
        "else document.getElementById('info-thumb').innerHTML='<em>" N_("No preview") "</em>';"
        "}).catch(function(){document.getElementById('info-thumb').innerHTML='<em>" N_("Could not load the file info") "</em>'})}"
        "function closeInfo(){document.getElementById('modal-bg').classList.remove('show')}"
        "function load(){"
        "fetch('/sd/files').then(function(r){return r.json()}).then(function(f){"
        "f.sort(function(a,b){return(b.mtime||0)-(a.mtime||0)});"
        "var h='';f.forEach(function(x){"
        "h+='<tr><td>'+x.name+'<br><span style=\"color:#888;font-size:.75em\">'+fmt(x.size)+' &mdash; '+fmtDate(x.mtime)+'</span></td><td>'"
        "+'<button onclick=\"info(\\''+x.name.replace(/'/g,\"\\\\'\")+'\\')\" style=\"background:#48f;color:#fff;border:none;border-radius:3px;padding:4px 12px;margin:2px;cursor:pointer\">" N_("Info") "</button> '"
        "+'<form method=POST action=/sd/print style=display:inline onsubmit=\"return sdPrint(this)\"><input type=hidden name=filename value=\"'+x.name+'\"><button type=submit style=\"background:#4a4;color:#fff;border:none;border-radius:3px;padding:4px 12px;margin:2px;cursor:pointer\">" N_("Print") "</button></form> '"
        "+'<form method=POST action=/sd/delete style=display:inline><input type=hidden name=filename value=\"'+x.name+'\"><button type=submit style=\"background:#f44;color:#fff;border:none;border-radius:3px;padding:4px 12px;margin:2px;cursor:pointer\">" N_("Delete") "</button></form>'"
        "+'</td></tr>'});"
        "document.getElementById('files').innerHTML=h||'<tr><td colspan=2>" N_("No files") "</td></tr>';});}"
        "window.addEventListener('pageshow',function(e){if(e.persisted)load()});"
        "function upload(){"
        "var f=document.getElementById('file').files[0];if(!f)return;"
        "var x=new XMLHttpRequest();"
        "x.upload.onprogress=function(e){if(e.lengthComputable){"
        "var p=document.getElementById('upload-prog');p.style.display='block';p.value=Math.round(e.loaded/e.total*100);"
        "document.getElementById('upload-status').textContent=fmt(e.loaded)+'/'+fmt(e.total);}};"
        "x.onload=function(){document.getElementById('upload-prog').style.display='none';"
        "document.getElementById('upload-status').textContent=x.status==200?'" N_("Done") "':'" N_("Error:") " '+x.responseText;load();};"
        "x.open('POST','/sd/upload');x.setRequestHeader('X-Filename',f.name);x.send(f);}"
        "function gb(k){return k<1048576?(k/1024).toFixed(0)+' " N_("MB") "':(k/1048576).toFixed(1)+' " N_("GB") "'}"
        "var fmb='<button id=fmt onclick=\"fmtCard()\" style=\"margin-left:8px\">" N_("Format") "</button>',fdone=0;"
        "function card(){fetch('/sd/status').then(function(r){return r.json()}).then(function(d){var c=document.getElementById('card');"
        "if(d.card=='formatting'){c.textContent='" N_("Formatting the card...") "';setTimeout(card,2000);return}"
        "c.innerHTML=d.card=='mounted'?'" N_("Card:") " '+gb(d.total_kb)+', " N_("free") " '+gb(d.free_kb)+fmb"
        ":d.card=='unformatted'?'" N_("The card is not formatted, or the board cannot read its file system") "'+fmb:'" N_("No SD card") "';"
        "if(fdone){fdone=0;load()}}).catch(function(){})}"
        "function fmtCard(){if(!confirm('" N_("Format the SD card? Every file on it is erased.") "'))return;"
        "var b=document.getElementById('fmt');b.disabled=true;"
        "fetch('/sd/format',{method:'POST'}).then(function(r){return r.text().then(function(t){if(!r.ok)throw t})})"
        ".then(function(){fdone=1;card()},function(t){b.disabled=false;alert(typeof t=='string'&&t?t:'" N_("Not started, try again") "')})}"
        "load();card();"
        "</script>";

static esp_err_t sd_page_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);

    layout_html_begin(&p, NT_("SD Card"), "/sd");
    html_buf_printf(&p, "%s", i18n_page(SD_PAGE));
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
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

        char entry[360];
        snprintf(entry, sizeof(entry), "%s{\"name\":\"%s\",\"size\":%ld,\"mtime\":%ld}",
                 first ? "" : ",", ent->d_name, (long)st.st_size, (long)st.st_mtime);
        httpd_resp_sendstr_chunk(req, entry);
        first = false;
    }
    closedir(dir);

    httpd_resp_sendstr_chunk(req, "]");
    return httpd_resp_send_chunk(req, NULL, 0);
}

/* ---- POST /sd/upload — stream file to SD (async, dual-core) ---- */

typedef struct {
    httpd_req_t *req;
    char filename[64];
} sd_upload_ctx_t;

/* SD writer context — shared between recv task (Core 0) and writer task (Core 1) */
typedef struct {
    StreamBufferHandle_t stream;
    volatile bool        recv_done;   /* recv task finished (success or error) */
    volatile bool        recv_error;  /* recv had an error */
    volatile bool        write_error; /* writer had an error */
    FILE                *file;
    int                  total;
    char                 path[300];
} sd_writer_ctx_t;

#define SD_STREAM_SIZE  1048576
#define SD_WRITE_BUF    32768

static void sd_writer_task(void *arg)
{
    sd_writer_ctx_t *wctx = (sd_writer_ctx_t *)arg;
    /* SDMMC DMA on ESP32-S3 cannot access PSRAM — use internal DMA-capable RAM */
    int wbuf_size = SD_WRITE_BUF;
    char *wbuf = NULL;
    while (wbuf_size >= 512) {
        wbuf = heap_caps_malloc(wbuf_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
        if (wbuf) break;
        wbuf_size /= 2;
    }
    if (!wbuf) {
        ESP_LOGE(TAG, "SD writer: no internal DMA memory");
        wctx->write_error = true;
        task_delete(NULL);
        return;
    }
    ESP_LOGI(TAG, "SD writer buffer: %d bytes (internal DMA)", wbuf_size);

    int fd = fileno(wctx->file);
    if (fd < 0) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        wctx->write_error = true;
        free(wbuf);
        task_delete(NULL);
        return;
    }

    while (true) {
        size_t got = xStreamBufferReceive(wctx->stream, wbuf, wbuf_size,
                                           pdMS_TO_TICKS(100));
        if (got > 0) {
            if (write(fd, wbuf, got) != (ssize_t)got) {
                ESP_LOGE(TAG, "SD write error");
                wctx->write_error = true;
                break;
            }
        }

        /* Exit when recv is done and stream is drained */
        if (wctx->recv_done && xStreamBufferIsEmpty(wctx->stream)) break;
        if (wctx->recv_error) break;
    }

    fclose(wctx->file);
    wctx->file = NULL;  /* signal main task: we closed it */
    free(wbuf);
    task_delete(NULL);
}

static void sd_upload_task(void *arg)
{
    sd_upload_ctx_t *ctx = (sd_upload_ctx_t *)arg;
    httpd_req_t *req = ctx->req;

    char path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/%s", ctx->filename);

    FILE *f = fopen(path, "w");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to create file");
        goto done;
    }

    /* Allocate stream buffer storage in PSRAM to save internal RAM */
    uint8_t *stream_store = heap_caps_malloc(SD_STREAM_SIZE + 1, MALLOC_CAP_SPIRAM);
    StaticStreamBuffer_t *stream_static = heap_caps_malloc(sizeof(StaticStreamBuffer_t), MALLOC_CAP_SPIRAM);
    StreamBufferHandle_t stream = NULL;
    if (stream_store && stream_static) {
        stream = xStreamBufferCreateStatic(SD_STREAM_SIZE, 1, stream_store, stream_static);
    }
    if (!stream) {
        free(stream_store);
        free(stream_static);
        fclose(f);
        unlink(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        goto done;
    }

    sd_writer_ctx_t wctx = {
        .stream      = stream,
        .recv_done   = false,
        .recv_error  = false,
        .write_error = false,
        .file        = f,
        .total       = req->content_len,
    };
    strlcpy(wctx.path, path, sizeof(wctx.path));

    /* Writer task on Core 1 — handles SD writes in parallel with recv */
    TaskHandle_t writer = NULL;
    if (xTaskCreatePinnedToCore(sd_writer_task, "sd_wr", 6144, &wctx,
                                 5, &writer, 1) != pdPASS) {
        vStreamBufferDelete(stream);
        free(stream_store);
        free(stream_static);
        fclose(f);
        unlink(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Task create failed");
        goto done;
    }

    /* Recv loop on Core 0 — feeds data into StreamBuffer */
    char *rbuf = malloc(UPLOAD_BUF_SIZE);
    int remaining = req->content_len;
    int total = remaining;
    esp_err_t ret = ESP_OK;

    if (!rbuf) {
        wctx.recv_error = true;
        wctx.recv_done = true;
        ret = ESP_FAIL;
    }

    while (remaining > 0 && ret == ESP_OK) {
        if (wctx.write_error) {
            ret = ESP_FAIL;
            break;
        }

        int to_read = remaining < UPLOAD_BUF_SIZE ? remaining : UPLOAD_BUF_SIZE;
        int received = httpd_req_recv(req, rbuf, to_read);
        if (received <= 0) {
            ESP_LOGE(TAG, "Upload recv error: %d", received);
            ret = ESP_FAIL;
            break;
        }

        /* Push into stream buffer — may block if writer is slow */
        size_t sent = 0;
        while (sent < (size_t)received) {
            if (wctx.write_error) { ret = ESP_FAIL; break; }
            size_t n = xStreamBufferSend(stream, rbuf + sent, received - sent,
                                          pdMS_TO_TICKS(100));
            sent += n;
        }

        remaining -= received;
    }

    if (ret != ESP_OK) wctx.recv_error = true;
    wctx.recv_done = true;

    /* Wait for writer to finish draining */
    while (eTaskGetState(writer) != eDeleted) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    free(rbuf);
    vStreamBufferDelete(stream);
    free(stream_store);
    free(stream_static);
    if (wctx.file) fclose(wctx.file);  /* writer task may have closed it already */

    if (ret != ESP_OK || wctx.write_error) {
        unlink(path);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Upload failed");
    } else {
        ESP_LOGI(TAG, "Uploaded %s (%d bytes)", ctx->filename, total);
        httpd_resp_send(req, "OK", 2);
    }

done:
    httpd_req_async_handler_complete(req);
    free(ctx);
    s_uploads--;
    task_delete(NULL);
}

static esp_err_t sd_upload_handler(httpd_req_t *req)
{
    if (!sdcard_is_mounted()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD card not mounted");
        return ESP_FAIL;
    }

    /* Get filename from header (must read before async clone) */
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

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }

    sd_upload_ctx_t *ctx = malloc(sizeof(sd_upload_ctx_t));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    strlcpy(ctx->filename, filename, sizeof(ctx->filename));

    s_uploads++;
    if (xTaskCreatePinnedToCore(sd_upload_task, "sd_upload", 6144, ctx,
                                 5, NULL, 0) != pdPASS) {
        s_uploads--;
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- POST /sd/format ---- */

static void sd_format_task(void *arg)
{
    sdcard_format();
    s_format_running = false;
    task_delete(NULL);
}

/* Refused while the card may be in use: a print (from it, or a Klipper print
 * whose stops the board may scan it for again), a file being sent to Klipper,
 * an upload. The page polls /sd/status for the result */
static esp_err_t sd_format_handler(httpd_req_t *req)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    const char *why = NULL;
    sdcard_state_t card = sdcard_state();
    if (s_format_running || card == SDCARD_FORMATTING) why = _("Already formatting");
    else if (card == SDCARD_NONE) why = _("No SD card");
    else if (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED ||
             printer_comm_is_host_printing()) why = _("Not while printing");
    else if (s_uploads > 0 ||
             (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER && klipper_backend_sending())) {
        why = _("A file is being sent, try again when it is done");
    }
    if (why) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, why);
    }

    s_format_running = true;
    if (xTaskCreatePinnedToCore(sd_format_task, "sd_format", 6144, NULL, 5, NULL, 1) != pdPASS) {
        s_format_running = false;
        httpd_resp_set_status(req, "500 Internal Server Error");
        return httpd_resp_sendstr(req, _("Not started, try again"));
    }
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
    url_decode_inplace(fn);

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

/* ---- POST /sd/print (async) ---- */

typedef struct {
    httpd_req_t *req;
    char filename[64];
} sd_print_ctx_t;

static void sd_print_task(void *arg)
{
    sd_print_ctx_t *ctx = (sd_print_ctx_t *)arg;
    httpd_req_t *req = ctx->req;

    esp_err_t err;
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        err = klipper_backend_print_file(ctx->filename);
        if (err == ESP_OK) print_page_note_started(ctx->filename);
    } else {
        err = printer_comm_host_print_start(ctx->filename);
        if (err == ESP_OK) print_page_note_started(ctx->filename);
    }

    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to start print");
    } else {
        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Location", "/sd");
        httpd_resp_send(req, NULL, 0);
    }

    httpd_req_async_handler_complete(req);
    free(ctx);
    task_delete(NULL);
}

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
    url_decode_inplace(fn);

    /* A file is still on its way to Moonraker: a second start would wait for
     * it, then fail, or print the wrong one */
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER && klipper_backend_sending()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, _("A file is still being sent to the printer"));
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }

    sd_print_ctx_t *ctx = malloc(sizeof(sd_print_ctx_t));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    strlcpy(ctx->filename, fn, sizeof(ctx->filename));

    if (xTaskCreatePinnedToCore(sd_print_task, "sd_print", 8192, ctx,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- POST /sd/pause, /sd/resume, /sd/cancel ---- */

static esp_err_t sd_pause_handler(httpd_req_t *req)
{
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        printer_cmd_t cmd = { .type = PCMD_PAUSE };
        printer_comm_send_cmd(&cmd);
    } else {
        printer_comm_host_print_pause();
    }
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t sd_resume_handler(httpd_req_t *req)
{
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        printer_cmd_t cmd = { .type = PCMD_RESUME };
        printer_comm_send_cmd(&cmd);
    } else {
        printer_comm_host_print_resume();
    }
    return httpd_resp_send(req, "OK", 2);
}

static esp_err_t sd_cancel_handler(httpd_req_t *req)
{
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        printer_cmd_t cmd = { .type = PCMD_CANCEL };
        printer_comm_send_cmd(&cmd);
    } else {
        printer_comm_host_print_cancel();
    }
    return httpd_resp_send(req, "OK", 2);
}

/* ---- GET /sd/status — JSON print status ---- */

static esp_err_t sd_status_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");

    printer_state_t st;
    printer_comm_get_state(&st);

    bool is_printing;
    if (printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER) {
        /* Klipper: print status comes from Moonraker polling */
        is_printing = (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED);
    } else {
        /* Marlin: check host print state */
        is_printing = printer_comm_is_host_printing();
    }

    bool paused = (st.opstate == PRINTER_PAUSED);

    char card[96];
    sdcard_state_t cs = s_format_running ? SDCARD_FORMATTING : sdcard_state();
    static const char *const card_names[] = { "none", "unformatted", "mounted", "formatting" };
    snprintf(card, sizeof(card), "\"card\":\"%s\",\"total_kb\":%ld,\"free_kb\":%ld,",
             card_names[cs], (long)sdcard_get_total_kb(), (long)sdcard_get_free_kb());

    char json[352];
    if (is_printing) {
        snprintf(json, sizeof(json),
                 "{%s\"printing\":true,\"paused\":%s,\"filename\":\"%s\","
                 "\"progress\":%.1f,\"layer\":%ld,\"elapsed\":%ld}",
                 card, paused ? "true" : "false",
                 st.filename,
                 st.progress_pct >= 0 ? st.progress_pct : 0.0f,
                 (long)st.current_layer,
                 (long)(st.print_time_s >= 0 ? st.print_time_s : 0));
    } else {
        snprintf(json, sizeof(json), "{%s\"printing\":false}", card);
    }

    return httpd_resp_send(req, json, strlen(json));
}

/* ---- GET /sd/info — async G-code file info ---- */

typedef struct {
    httpd_req_t *req;
    char filename[64];
} sd_info_ctx_t;

static void sd_info_task(void *arg)
{
    sd_info_ctx_t *ctx = (sd_info_ctx_t *)arg;
    httpd_req_t *req = ctx->req;

    char path[300];
    snprintf(path, sizeof(path), MOUNT_POINT "/%s", ctx->filename);

    struct stat st;
    if (stat(path, &st) != 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        goto done;
    }

    gcode_file_info_t info;
    gcode_scan_file_info(path, &info);

    httpd_resp_set_type(req, "application/json");

    /* Send JSON in chunks to avoid huge stack buffer */
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"name\":\"%s\",\"size\":%ld,\"layers\":%ld,"
             "\"layerHeight\":%.2f,\"objectHeight\":%.2f,"
             "\"estTime\":%ld,\"filamentMm\":%.1f,\"filamentG\":%.1f",
             ctx->filename, (long)st.st_size, (long)info.total_layers,
             info.layer_height, info.max_z,
             (long)info.est_time_s, info.filament_used_mm, info.filament_used_g);
    httpd_resp_sendstr_chunk(req, buf);

    if (info.thumbnail_base64) {
        httpd_resp_sendstr_chunk(req, ",\"thumbnail\":\"data:image/png;base64,");
        httpd_resp_sendstr_chunk(req, info.thumbnail_base64);
        httpd_resp_sendstr_chunk(req, "\"");
        free(info.thumbnail_base64);
    } else {
        httpd_resp_sendstr_chunk(req, ",\"thumbnail\":null");
    }

    httpd_resp_sendstr_chunk(req, "}");
    httpd_resp_send_chunk(req, NULL, 0);

done:
    httpd_req_async_handler_complete(req);
    free(ctx);
    task_delete(NULL);
}

static esp_err_t sd_info_handler(httpd_req_t *req)
{
    if (!sdcard_is_mounted()) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "SD card not mounted");
        return ESP_FAIL;
    }

    char query[128];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing query");
        return ESP_FAIL;
    }

    char filename[64];
    if (httpd_query_key_value(query, "filename", filename, sizeof(filename)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing filename");
        return ESP_FAIL;
    }
    url_decode_inplace(filename);

    /* Sanitize */
    char *slash = strrchr(filename, '/');
    if (slash) memmove(filename, slash + 1, strlen(slash + 1) + 1);
    if (filename[0] == '\0' || filename[0] == '.') {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid filename");
        return ESP_FAIL;
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async init failed");
        return ESP_FAIL;
    }

    sd_info_ctx_t *ctx = malloc(sizeof(sd_info_ctx_t));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    strlcpy(ctx->filename, filename, sizeof(ctx->filename));

    if (task_create_psram(sd_info_task, "sd_info", 8192, ctx,
                                 5, NULL, 0) != pdPASS) {
        httpd_req_async_handler_complete(async_req);
        free(ctx);
        return ESP_FAIL;
    }

    return ESP_OK;
}

/* ---- Registration ---- */

esp_err_t sdcard_httpd_register(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/sd",        .method = HTTP_GET,  .handler = sd_page_handler },
        { .uri = "/sd/files",  .method = HTTP_GET,  .handler = sd_files_handler },
        { .uri = "/sd/upload", .method = HTTP_POST, .handler = sd_upload_handler },
        { .uri = "/sd/delete", .method = HTTP_POST, .handler = sd_delete_handler },
        { .uri = "/sd/format", .method = HTTP_POST, .handler = sd_format_handler },
        { .uri = "/sd/print",  .method = HTTP_POST, .handler = sd_print_handler },
        { .uri = "/sd/pause",  .method = HTTP_POST, .handler = sd_pause_handler },
        { .uri = "/sd/resume", .method = HTTP_POST, .handler = sd_resume_handler },
        { .uri = "/sd/cancel", .method = HTTP_POST, .handler = sd_cancel_handler },
        { .uri = "/sd/info",   .method = HTTP_GET,  .handler = sd_info_handler },
        { .uri = "/sd/status", .method = HTTP_GET,  .handler = sd_status_handler },
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }

    ESP_LOGI(TAG, "SD card HTTP endpoints registered");
    return ESP_OK;
}
