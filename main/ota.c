#include "ota.h"
#include "httpd.h"

#include <string.h>
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_image_format.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "hub_client.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "printer_comm.h"
#include "printer_comm_klipper.h"
#include "rfc2217.h"
#include "version.h"

static const char *TAG = "ota";

#define OTA_BUF_SIZE 4096

/* ---- Deferred update ----
 *
 * An image uploaded while the printer is busy waits in PSRAM and is written
 * to flash once the printer has been idle for DEFER_IDLE_S.  Not to flash
 * right away: erasing and writing 1.5 MB stalls the caches, and with them
 * both cores, for half a minute, and a host print would starve the printer.
 * A reboot before the install loses it: the status shows it, upload again.
 *
 * Klipper prints on its own host, so a print there does not wait: only a file
 * being sent (the upload would break) and a built-in script (the board would
 * not learn how it ended, so not which plastic is in the nozzle).
 */

#define DEFER_IDLE_S        60
#define DEFER_CHECK_MS      2000
#define NVS_NAMESPACE       "ota"
#define NVS_KEY_FROM        "deferred_from"

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t *s_pend_buf;             /* Whole image, in PSRAM */
static size_t s_pend_len;
static char s_pend_ver[48];             /* "" = nothing pending */
static bool s_flashing;                 /* An upload or install owns the OTA partition */
static bool s_installing;               /* The pending image is being written, keep it */
static bool s_defer_task_running;

static bool claim_flash(void)
{
    taskENTER_CRITICAL(&s_mux);
    bool ok = !s_flashing;
    s_flashing = true;
    taskEXIT_CRITICAL(&s_mux);
    return ok;
}

static void release_flash(void)
{
    taskENTER_CRITICAL(&s_mux);
    s_flashing = false;
    taskEXIT_CRITICAL(&s_mux);
}

void ota_get_pending(char *out, size_t size)
{
    taskENTER_CRITICAL(&s_mux);
    strlcpy(out, s_pend_ver, size);
    taskEXIT_CRITICAL(&s_mux);
}

/** Drop the pending image, unless an install is writing it.  False if it is. */
static bool drop_pending(void)
{
    taskENTER_CRITICAL(&s_mux);
    if (s_installing) {
        taskEXIT_CRITICAL(&s_mux);
        return false;
    }
    uint8_t *buf = s_pend_buf;
    s_pend_buf = NULL;
    s_pend_len = 0;
    s_pend_ver[0] = '\0';
    taskEXIT_CRITICAL(&s_mux);
    free(buf);
    return true;
}

/** Why an update must wait for later, or NULL if it may go now.  A reboot now
 * would break a print, or something the user is doing on the printer. */
static bool is_klipper(void)
{
    return printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER;
}

static const char *wait_reason(void)
{
    printer_state_t st;
    printer_comm_get_state(&st);
    if (is_klipper()) {
        if (klipper_backend_sending()) return NT_("sending a file to the printer");
        if (st.builtin_script) return NT_("filament change or cold pull");
        return NULL;
    }
    if (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED || st.host_printing)
        return NT_("printing");
    if (st.opstate == PRINTER_ERROR) return NT_("printer error");
    if (rfc2217_client_connected()) return NT_("OctoPrint connected");
    if (printer_comm_user_cmd_running()) return NT_("command running");
    return NULL;
}

/** As wait_reason(), plus what says someone is still at the printer */
static const char *install_wait_reason(void)
{
    const char *why = wait_reason();
    if (why || is_klipper()) return why;

    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate != PRINTER_DISCONNECTED && (st.hotend_target > 0 || st.bed_target > 0))
        return NT_("heater on");
    int64_t cmd_us = printer_comm_last_user_cmd_us();
    if (cmd_us && esp_timer_get_time() - cmd_us < DEFER_IDLE_S * 1000000LL)
        return NT_("command running");
    return NULL;
}

/** The app version from an image header, or an error to show */
static const char *check_header(const uint8_t *buf, size_t len, char *version, size_t size)
{
    size_t desc_off = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t);
    if (len < desc_off + sizeof(esp_app_desc_t) || buf[0] != ESP_IMAGE_HEADER_MAGIC)
        return NT_("Not a valid ESP32 application image");

    /* esp_app_desc_t starts at offset 32 (image header + segment header) */
    esp_app_desc_t desc;
    memcpy(&desc, buf + desc_off, sizeof(desc));
    if (desc.magic_word != ESP_APP_DESC_MAGIC_WORD) {
        ESP_LOGE(TAG, "Bad app descriptor magic: 0x%08x", (unsigned)desc.magic_word);
        return NT_("Not a valid ESP32 application image");
    }

    const esp_app_desc_t *cur = esp_app_get_description();
    if (strncmp(desc.project_name, cur->project_name, sizeof(desc.project_name)) != 0) {
        ESP_LOGE(TAG, "Project mismatch: running \"%s\", uploaded \"%.32s\"",
                 cur->project_name, desc.project_name);
        return NT_("Wrong firmware — project name does not match");
    }

    snprintf(version, size, "%.*s", (int)sizeof(desc.version), desc.version);
    ESP_LOGI(TAG, "Firmware verified: project \"%s\", version \"%s\"", cur->project_name, version);
    return NULL;
}

/** Append the image's build stamp (FW_BUILD, after its "FWBUILD=" tag) to version */
static void add_image_build(const uint8_t *buf, size_t len, char *version, size_t size)
{
    /* Built here, so that the search does not find its own needle in our image */
    char tag[9];
    memcpy(tag, "FWBUILD", 7);
    tag[7] = '=';
    tag[8] = '\0';
    const uint8_t *p = memmem(buf, len, tag, 8);
    if (!p) return;
    p += 8;
    size_t n = 0;
    while (p + n < buf + len && n < 32 && p[n] >= ' ' && p[n] < 0x7f) n++;
    size_t at = strlen(version);
    if (n && at + 1 < size) snprintf(version + at, size - at, " %.*s", (int)n, p);
}

/** Whole-image check for an image that has to wait: a truncated or damaged
 * upload must fail now, while the user is there, not after the print */
static bool image_hash_ok(const uint8_t *buf, size_t len)
{
    const esp_image_header_t *hdr = (const esp_image_header_t *)buf;
    if (!hdr->hash_appended) return true;   /* esp_ota_end() still checks the checksum */
    if (len < sizeof(*hdr) + 32) return false;

    uint8_t sum[32];
    if (mbedtls_sha256(buf, len - 32, sum, 0) != 0) return false;
    return memcmp(sum, buf + len - 32, 32) == 0;
}

/** Write an image from memory to the next OTA partition and boot from it.
 * Returns NULL on success, or what failed. */
static const char *flash_image(const uint8_t *image, size_t len)
{
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) return "no OTA partition";

    esp_ota_handle_t handle = 0;
    esp_err_t err = esp_ota_begin(update, len, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        return "OTA begin failed";
    }

    /* Through internal RAM: flash writes run with the cache, and so PSRAM, off */
    uint8_t *chunk = heap_caps_malloc(OTA_BUF_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!chunk) {
        esp_ota_abort(handle);
        return "out of memory";
    }
    for (size_t off = 0; off < len; off += OTA_BUF_SIZE) {
        size_t n = len - off < OTA_BUF_SIZE ? len - off : OTA_BUF_SIZE;
        memcpy(chunk, image + off, n);
        err = esp_ota_write(handle, chunk, n);
        if (err != ESP_OK) break;
    }
    free(chunk);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
        esp_ota_abort(handle);
        return "flash write failed";
    }

    err = esp_ota_end(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        return "image validation failed";
    }
    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        return "set boot partition failed";
    }
    ESP_LOGI(TAG, "Firmware written to %s", update->label);
    return NULL;
}

static void remember_deferred(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    char from[64];
    snprintf(from, sizeof(from), "%s %s", FW_VERSION, FW_BUILD);
    nvs_set_str(nvs, NVS_KEY_FROM, from);
    nvs_commit(nvs);
    nvs_close(nvs);
}

static void defer_task(void *arg)
{
    int64_t idle_since = 0;
    const char *last_why = "";

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(DEFER_CHECK_MS));

        char ver[sizeof(s_pend_ver)];
        ota_get_pending(ver, sizeof(ver));
        if (!ver[0]) break;     /* Cancelled */

        const char *why = install_wait_reason();
        if (why) {
            if (why != last_why) ESP_LOGI(TAG, "Update %s waits: %s", ver, why);
            last_why = why;
            idle_since = 0;
            continue;
        }
        int64_t now = esp_timer_get_time();
        if (!idle_since) {
            ESP_LOGI(TAG, "Printer idle, installing %s in %d s", ver, DEFER_IDLE_S);
            idle_since = now;
            last_why = "";
        }
        if (!is_klipper() && now - idle_since < DEFER_IDLE_S * 1000000LL) continue;
        if (!claim_flash()) continue;   /* An upload is running: it decides */

        taskENTER_CRITICAL(&s_mux);
        const uint8_t *image = s_pend_buf;
        size_t len = s_pend_len;
        s_installing = true;
        taskEXIT_CRITICAL(&s_mux);

        ESP_LOGI(TAG, "Installing deferred update %s (%u bytes)", ver, (unsigned)len);
        const char *err = flash_image(image, len);
        if (err) {
            ESP_LOGE(TAG, "Deferred update %s failed: %s", ver, err);
            char msg[96];
            snprintf(msg, sizeof(msg), "%s: %s", ver, err);
            hub_client_send_event(HUB_EVENT_UPDATE_FAILED, msg, "");
            taskENTER_CRITICAL(&s_mux);
            s_installing = false;
            taskEXIT_CRITICAL(&s_mux);
            drop_pending();
            release_flash();
            break;
        }

        /* A print may have started during the write: the new firmware boots
         * on the next reboot either way, so this one can wait for it */
        for (const char *w; (w = wait_reason()) != NULL; vTaskDelay(pdMS_TO_TICKS(DEFER_CHECK_MS))) {
            if (w != last_why) ESP_LOGI(TAG, "Update %s written, reboot waits: %s", ver, w);
            last_why = w;
        }
        remember_deferred();
        ESP_LOGI(TAG, "Rebooting into %s", ver);
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    taskENTER_CRITICAL(&s_mux);
    s_defer_task_running = false;
    taskEXIT_CRITICAL(&s_mux);
    vTaskDelete(NULL);
}

/** Keep the image for later; takes ownership of buf */
static bool set_pending(uint8_t *buf, size_t len, const char *version)
{
    taskENTER_CRITICAL(&s_mux);
    uint8_t *old = s_pend_buf;
    s_pend_buf = buf;
    s_pend_len = len;
    strlcpy(s_pend_ver, version, sizeof(s_pend_ver));
    bool start = !s_defer_task_running;
    s_defer_task_running = true;
    taskEXIT_CRITICAL(&s_mux);
    free(old);

    if (start && xTaskCreatePinnedToCore(defer_task, "ota_defer", 6144, NULL, 2, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start the deferred update task");
        taskENTER_CRITICAL(&s_mux);
        s_defer_task_running = false;
        taskEXIT_CRITICAL(&s_mux);
        drop_pending();
        return false;
    }
    ESP_LOGI(TAG, "Update %s deferred until the printer is idle", version);
    return true;
}

void ota_report_update(void)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) != ESP_OK) return;
    char from[64];
    size_t len = sizeof(from);
    if (nvs_get_str(nvs, NVS_KEY_FROM, from, &len) == ESP_OK) {
        nvs_erase_key(nvs, NVS_KEY_FROM);
        nvs_commit(nvs);
        char msg[160];
        snprintf(msg, sizeof(msg), "%s → %s %s", from, FW_VERSION, FW_BUILD);
        ESP_LOGI(TAG, "Deferred update installed: %s", msg);
        hub_client_send_event(HUB_EVENT_UPDATED, msg, "");
    }
    nvs_close(nvs);
}

/* ---- Boot confirmation ---- */

void ota_confirm_boot(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) == ESP_OK) {
        if (state == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "First boot after OTA — marking firmware valid");
            esp_ota_mark_app_valid_cancel_rollback();
        }
    }
}

/* ---- OTA upload (async) ---- */

typedef struct {
    httpd_req_t *req;
    int lang;               /* Of the request: the errors go back to the page */
} ota_upload_ctx_t;

/** Receive exactly len bytes into buf.  False if the client went away. */
static bool recv_all(httpd_req_t *req, uint8_t *buf, size_t len)
{
    size_t got = 0;
    while (got < len) {
        int n = httpd_req_recv(req, (char *)buf + got, len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) {
            ESP_LOGE(TAG, "OTA recv error at %u/%u", (unsigned)got, (unsigned)len);
            return false;
        }
        got += n;
    }
    return true;
}

/** The printer is busy: keep the whole image in PSRAM for later */
static void receive_deferred(httpd_req_t *req, const char *why)
{
    size_t total = req->content_len;
    uint8_t *image = heap_caps_malloc(total, MALLOC_CAP_SPIRAM);
    if (!image) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Out of memory."));
        return;
    }
    ESP_LOGI(TAG, "OTA upload while busy (%s): %u bytes to PSRAM", why, (unsigned)total);
    if (!recv_all(req, image, total)) {
        free(image);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Upload failed"));
        return;
    }

    char version[sizeof(s_pend_ver)];
    const char *bad = check_header(image, total, version, sizeof(version));
    if (!bad && !image_hash_ok(image, total)) bad = NT_("Firmware validation failed — bad image?");
    if (!bad) add_image_build(image, total, version, sizeof(version));
    if (bad) {
        free(image);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, TR(bad));
        return;
    }
    if (!set_pending(image, total, version)) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Out of memory."));
        return;
    }

    char json[256];
    snprintf(json, sizeof(json),
             "{\"status\":\"deferred\",\"version\":\"%s\",\"waiting\":\"%s\",\"message\":\"%s\"}",
             version, why, _("Will be installed when the printer is idle"));
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
}

/** The printer is idle: stream straight into flash and reboot.  Returns only on error. */
static void receive_now(httpd_req_t *req)
{
    const esp_partition_t *update = esp_ota_get_next_update_partition(NULL);
    if (!update) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("No OTA partition"));
        return;
    }

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update, OTA_SIZE_UNKNOWN, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("OTA begin failed"));
        return;
    }

    uint8_t *buf = malloc(OTA_BUF_SIZE);
    if (!buf) {
        esp_ota_abort(ota_handle);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Out of memory."));
        return;
    }

    int remaining = req->content_len;
    int total = remaining;
    bool first = true;

    ESP_LOGI(TAG, "OTA update starting: %d bytes → %s", total, update->label);

    while (remaining > 0) {
        /* The first chunk is read whole: it holds the header to check */
        int to_read = remaining < OTA_BUF_SIZE ? remaining : OTA_BUF_SIZE;
        if (first) {
            if (!recv_all(req, buf, to_read)) goto write_error;
            char version[32];
            const char *bad = check_header(buf, to_read, version, sizeof(version));
            if (bad) {
                free(buf);
                esp_ota_abort(ota_handle);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, TR(bad));
                return;
            }
            first = false;
        } else {
            int received = httpd_req_recv(req, (char *)buf, to_read);
            if (received == HTTPD_SOCK_ERR_TIMEOUT) continue;
            if (received <= 0) {
                ESP_LOGE(TAG, "OTA recv error at %d/%d", total - remaining, total);
                goto write_error;
            }
            to_read = received;
        }

        err = esp_ota_write(ota_handle, buf, to_read);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            goto write_error;
        }
        remaining -= to_read;
    }
    free(buf);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s (image validation error?)", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            _("Firmware validation failed — bad image?"));
        return;
    }

    err = esp_ota_set_boot_partition(update);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Set boot partition failed"));
        return;
    }

    ESP_LOGI(TAG, "OTA update successful! Rebooting to %s", update->label);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\":\"ok\",\"message\":\"Firmware updated. Rebooting...\"}");

    httpd_req_async_handler_complete(req);

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return;  /* unreachable */

write_error:
    free(buf);
    esp_ota_abort(ota_handle);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("OTA write failed"));
}

static void ota_upload_task(void *arg)
{
    ota_upload_ctx_t *ctx = (ota_upload_ctx_t *)arg;
    httpd_req_t *req = ctx->req;
    i18n_set_lang(ctx->lang);
    free(ctx);

    if (!claim_flash()) {
        httpd_resp_set_status(req, "409 Conflict");
        httpd_resp_sendstr(req, _("Another update is being written, try again in a minute"));
    } else {
        const char *why = wait_reason();
        if (why) {
            receive_deferred(req, why);
        } else {
            /* Replaces anything pending, which would otherwise install over it */
            drop_pending();
            receive_now(req);
        }
        release_flash();
    }

    httpd_req_async_handler_complete(req);
    vTaskDelete(NULL);
}

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    if (req->content_len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, _("No firmware data"));
        return ESP_FAIL;
    }

    /* Sanity check: firmware should be at least 256 bytes and under 2MB */
    if (req->content_len < 256 || req->content_len > 0x200000) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, _("Invalid firmware size"));
        return ESP_FAIL;
    }

    httpd_req_t *async_req = NULL;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Async begin failed");
        return ESP_FAIL;
    }

    ota_upload_ctx_t *ctx = malloc(sizeof(*ctx));
    if (!ctx) {
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }
    ctx->req = async_req;
    ctx->lang = i18n_lang();

    if (xTaskCreatePinnedToCore(ota_upload_task, "ota_up", 8192, ctx,
                                 5, NULL, 0) != pdPASS) {
        free(ctx);
        httpd_req_async_handler_complete(async_req);
        return ESP_FAIL;
    }

    return ESP_OK;
}

static esp_err_t ota_cancel_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    if (!drop_pending()) {
        httpd_resp_set_status(req, "409 Conflict");
        return httpd_resp_sendstr(req, "{\"status\":\"installing\"}");
    }
    ESP_LOGI(TAG, "Deferred update cancelled");
    return httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
}

/* ---- HTTP registration ---- */

void ota_register_httpd(httpd_handle_t server)
{
    httpd_uri_t upload_uri = {
        .uri     = "/ota/upload",
        .method  = HTTP_POST,
        .handler = ota_upload_handler,
    };
    HTTPD_REGISTER(server, &upload_uri);

    httpd_uri_t cancel_uri = {
        .uri     = "/ota/cancel",
        .method  = HTTP_POST,
        .handler = ota_cancel_handler,
    };
    HTTPD_REGISTER(server, &cancel_uri);
}

/* ---- Settings page section ---- */

void ota_render_settings(html_buf_t *p)
{
    const esp_partition_t *running = esp_ota_get_running_partition();

    char pending[sizeof(s_pend_ver)];
    ota_get_pending(pending, sizeof(pending));
    bool installing;
    taskENTER_CRITICAL(&s_mux);
    installing = s_installing;
    taskEXIT_CRITICAL(&s_mux);

    html_buf_printf(p,
        i18n_page(
        "<hr><div class='sh'><h2>" N_("Firmware Update") "</h2></div>"
        "<p class='hint'>" N_("Running:") " <b>v%s</b> " N_("on partition") " <b>%s</b>"
        " &middot; " N_("Built") " %s</p>"),
        FW_VERSION, running ? running->label : "?", FW_BUILD);

    if (pending[0]) {
        const char *why = installing ? NULL : install_wait_reason();
        html_buf_printf(p,
            "<p style='background:#fff3e0;padding:8px;border-radius:4px'>%s <b>v%s</b>: %s ",
            _("Pending update"), pending,
            installing ? _("installing now") :
            why ? TR(why) : _("printer idle, installing shortly"));
        if (!installing) {
            html_buf_printf(p,
                "<button onclick=\"fetch('/ota/cancel',{method:'POST'}).then(function(){location.reload()})\">%s</button>",
                _("Cancel"));
        }
        html_buf_printf(p, "</p><p class='hint'>%s</p>",
            _("An update uploaded during a print waits in memory and installs after the printer "
              "has been idle for a minute. Uploading again replaces it, and a reboot loses it."));
    }

    html_buf_printf(p,
        i18n_page(
        "<form id='ota-form'>"
        "<input type='file' id='ota-file' accept='.bin' "
        "style='margin:8px 0;display:block'>"
        "<button type='submit' id='ota-btn'>" N_("Upload &amp; Flash") "</button>"
        "</form>"
        "<progress id='ota-prog' max='100' value='0' "
        "style='width:100%%;display:none;margin:8px 0'></progress>"
        "<div id='ota-status' style='font-size:13px;color:#666;margin:4px 0'></div>"
        "<script>"
        "document.getElementById('ota-form').onsubmit=function(e){"
        "e.preventDefault();"
        "var f=document.getElementById('ota-file').files[0];"
        "if(!f){alert('" N_("Select a firmware .bin file") "');return;}"
        "if(!confirm('" N_("Flash firmware") " '+f.name+' ('+Math.round(f.size/1024)+' KB)?'))return;"
        "var xhr=new XMLHttpRequest();"
        "var prog=document.getElementById('ota-prog');"
        "var stat=document.getElementById('ota-status');"
        "var btn=document.getElementById('ota-btn');"
        "btn.disabled=true;prog.style.display='block';"
        "xhr.upload.onprogress=function(ev){"
        "if(ev.lengthComputable){prog.value=Math.round(ev.loaded/ev.total*100);"
        "stat.textContent='" N_("Uploading:") " '+Math.round(ev.loaded/1024)+'/'+Math.round(ev.total/1024)+' KB';}};"
        "xhr.onload=function(){"
        "if(xhr.status==200){var r={};try{r=JSON.parse(xhr.responseText)}catch(e){}"
        "if(r.status=='deferred'){stat.textContent='v'+r.version+': '+r.message;"
        "stat.style.color='#e65100';setTimeout(function(){location.reload()},3000);return;}"
        "stat.textContent='" N_("Firmware flashed! Rebooting...") "';"
        "stat.style.color='#2e7d32';"
        "setTimeout(function(){location.reload()},8000);}"
        "else{stat.textContent='" N_("Error:") " '+xhr.responseText;stat.style.color='#c62828';btn.disabled=false;}};"
        "xhr.onerror=function(){stat.textContent='" N_("Upload failed") "';stat.style.color='#c62828';btn.disabled=false;};"
        "xhr.open('POST','/ota/upload');"
        "xhr.setRequestHeader('Content-Type','application/octet-stream');"
        "xhr.send(f);};"
        "</script>"));
}
