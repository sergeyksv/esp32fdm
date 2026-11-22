#include "bedlevel.h"
#include "task_util.h"
#include "httpd.h"
#include "printer_comm.h"
#include "printer_comm_klipper.h"
#include "cache.h"
#include "layout.h"
#include "i18n.h"

#include "cJSON.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <time.h>
#include <sys/stat.h>

static const char *TAG = "bedlevel";

/* ---- Constants ---- */
#define BL_MESH_MAX    15         /* Klipper allows up to 15 x 15 for the mesh we read */
#define BL_CACHE_DIR   "/cache/beds"
#define BL_CONFIG_FILE BL_CACHE_DIR "/config.json"
#define BL_CACHE_MAX   100
#define PROBE_TIMEOUT_MS 1200000  /* 20 min — covers up to 10x10 mesh */
#define BL_SCREWS_MAX  9
#define BL_FIXED_MAX   4
#define BL_CONFIG_MAX  2048

/* ---- Data types ---- */

typedef struct {
    int64_t timestamp;
    int grid_x, grid_y;
    float data[BL_MESH_MAX][BL_MESH_MAX];   /* [y][x], row 0 at the front with Klipper */
    float min_z, max_z;
    bool has_xy;                /* Klipper: where the points are, so screws can be placed */
    char profile[24];           /* Klipper: the profile it was, "default" for an unsaved probe */
    float x0, y0, x1, y1;       /* mesh_min, mesh_max */
} bl_mesh_t;

typedef struct {
    float bed_w, bed_h;
    float screw_spacing;
    float screw_pitch;
    int origin;  /* 0=FL, 1=FR, 2=BL, 3=BR */
    bool cw_raises;             /* Turning a screw clockwise raises the bed there */
    int turn_div;               /* Turns are given in 1/turn_div: 8, 6, 4 or 3 (a 3-lobe knob) */
    int bed_c;                  /* Bed temperature to probe at (Klipper), 0 = cold */
    char print_mesh[24];        /* Klipper profile the mesh for printing is saved to, "" = the printer's own */
    int n_screws, n_fixed;      /* Screws at nozzle coordinates; fixed supports cannot be turned */
    float screws[BL_SCREWS_MAX][2];
    float fixed[BL_FIXED_MAX][2];
} bl_config_t;

typedef enum {
    BL_IDLE,
    BL_HOMING,
    BL_PROBING,
    BL_DONE,
    BL_ERROR,
} bl_state_t;

/* ---- Static state ---- */
static bl_state_t s_state = BL_IDLE;
static bl_mesh_t  s_current_mesh;
static bl_config_t s_config = { .bed_w = 220, .bed_h = 220, .screw_spacing = 180, .screw_pitch = 1.0f,
                                 .bed_c = 60, .turn_div = 8 };
/* Why the last probe failed.  The text is made in the status reply, in the
 * language of that request: the probe task has no request language. */
typedef enum { BL_ERR_NONE, BL_ERR_G28, BL_ERR_G29, BL_ERR_NO_MESH, BL_ERR_CANCELLED,
               BL_ERR_START, BL_ERR_SAVE } bl_err_t;
static volatile bool s_fetch_mesh;   /* Klipper: the probe script ended, read its mesh */
static int64_t s_probe_start_us;     /* Klipper: when the probe script was asked for */
static bool s_probe_seen;            /* ...and whether it has been seen running */
#define PROBE_START_WAIT_US (3 * 60 * 1000000LL)
static bl_err_t s_error = BL_ERR_NONE;
static esp_err_t s_error_code = ESP_OK;

/* ---- Config persistence ---- */

static void apply_config_json(const cJSON *root);

static void load_config(void)
{
    FILE *f = fopen(BL_CONFIG_FILE, "r");
    if (!f) return;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > BL_CONFIG_MAX) { fclose(f); return; }
    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return; }
    fread(buf, 1, sz, f);
    buf[sz] = '\0';
    fclose(f);

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) return;
    apply_config_json(root);
    cJSON_Delete(root);
    ESP_LOGI(TAG, "Config loaded: bed %.0fx%.0f screw_sp=%.0f pitch=%.2f origin=%d, %d screws, %d fixed",
             s_config.bed_w, s_config.bed_h, s_config.screw_spacing, s_config.screw_pitch, s_config.origin,
             s_config.n_screws, s_config.n_fixed);
}

/** The config as JSON, for the file, the page and the settings copy */
static void config_to_json(cJSON *root)
{
    cJSON_AddNumberToObject(root, "bed_w", s_config.bed_w);
    cJSON_AddNumberToObject(root, "bed_h", s_config.bed_h);
    cJSON_AddNumberToObject(root, "screw_spacing", s_config.screw_spacing);
    /* Floats print as 0.699999988 otherwise */
    cJSON_AddNumberToObject(root, "screw_pitch", round(s_config.screw_pitch * 1000.0) / 1000.0);
    cJSON_AddNumberToObject(root, "origin", s_config.origin);
    cJSON_AddBoolToObject(root, "cw_raises", s_config.cw_raises);
    cJSON_AddNumberToObject(root, "turn_div", s_config.turn_div);
    cJSON_AddNumberToObject(root, "bed_c", s_config.bed_c);
    cJSON_AddStringToObject(root, "print_mesh", s_config.print_mesh);
    cJSON *a = cJSON_AddArrayToObject(root, "screws");
    for (int i = 0; a && i < s_config.n_screws; i++)
        cJSON_AddItemToArray(a, cJSON_CreateFloatArray(s_config.screws[i], 2));
    a = cJSON_AddArrayToObject(root, "fixed");
    for (int i = 0; a && i < s_config.n_fixed; i++)
        cJSON_AddItemToArray(a, cJSON_CreateFloatArray(s_config.fixed[i], 2));
}

static void save_config(void)
{
    mkdir(BL_CACHE_DIR, 0755);
    cJSON *root = cJSON_CreateObject();
    if (!root) return;
    config_to_json(root);
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;

    FILE *f = fopen(BL_CONFIG_FILE, "w");
    if (f) { fputs(json, f); fclose(f); }
    free(json);
}

/* ---- Mesh cache I/O ---- */

static void save_mesh(const bl_mesh_t *mesh)
{
    mkdir(BL_CACHE_DIR, 0755);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "ts", (double)mesh->timestamp);
    cJSON_AddNumberToObject(root, "gx", mesh->grid_x);
    cJSON_AddNumberToObject(root, "gy", mesh->grid_y);
    cJSON_AddNumberToObject(root, "min", mesh->min_z);
    cJSON_AddNumberToObject(root, "max", mesh->max_z);
    if (mesh->profile[0]) cJSON_AddStringToObject(root, "pr", mesh->profile);
    if (mesh->has_xy) {
        cJSON_AddNumberToObject(root, "x0", mesh->x0);
        cJSON_AddNumberToObject(root, "y0", mesh->y0);
        cJSON_AddNumberToObject(root, "x1", mesh->x1);
        cJSON_AddNumberToObject(root, "y1", mesh->y1);
    }

    cJSON *rows = cJSON_AddArrayToObject(root, "d");
    for (int y = 0; y < mesh->grid_y; y++) {
        cJSON *row = cJSON_CreateArray();
        for (int x = 0; x < mesh->grid_x; x++) {
            cJSON_AddItemToArray(row, cJSON_CreateNumber(mesh->data[y][x]));
        }
        cJSON_AddItemToArray(rows, row);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;

    char path[80];
    snprintf(path, sizeof(path), BL_CACHE_DIR "/mesh_%lld.json", (long long)mesh->timestamp);

    FILE *f = fopen(path, "w");
    if (f) { fputs(json, f); fclose(f); }
    free(json);

    cache_touch(path);
    cache_evict_lru(BL_CACHE_DIR, BL_CACHE_MAX + 1);  /* +1 for config.json */
    ESP_LOGI(TAG, "Mesh saved: %s (%dx%d)", path, mesh->grid_x, mesh->grid_y);
}

static bool load_mesh_file(const char *path, bl_mesh_t *mesh)
{
    FILE *f = fopen(path, "r");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0 || sz > 16384) { fclose(f); return false; }
    char *buf = malloc(sz + 1);
    if (!buf) { fclose(f); return false; }
    fread(buf, 1, sz, f);
    buf[sz] = '\0';
    fclose(f);

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) return false;

    memset(mesh, 0, sizeof(*mesh));
    mesh->timestamp = (int64_t)cJSON_GetObjectItem(root, "ts")->valuedouble;
    mesh->grid_x = cJSON_GetObjectItem(root, "gx")->valueint;
    mesh->grid_y = cJSON_GetObjectItem(root, "gy")->valueint;
    mesh->min_z = cJSON_GetObjectItem(root, "min")->valuedouble;
    mesh->max_z = cJSON_GetObjectItem(root, "max")->valuedouble;
    cJSON *pr = cJSON_GetObjectItem(root, "pr");
    if (cJSON_IsString(pr)) strlcpy(mesh->profile, pr->valuestring, sizeof(mesh->profile));
    cJSON *x0 = cJSON_GetObjectItem(root, "x0"), *y0 = cJSON_GetObjectItem(root, "y0");
    cJSON *x1 = cJSON_GetObjectItem(root, "x1"), *y1 = cJSON_GetObjectItem(root, "y1");
    if (cJSON_IsNumber(x0) && cJSON_IsNumber(y0) && cJSON_IsNumber(x1) && cJSON_IsNumber(y1)) {
        mesh->has_xy = true;
        mesh->x0 = x0->valuedouble;
        mesh->y0 = y0->valuedouble;
        mesh->x1 = x1->valuedouble;
        mesh->y1 = y1->valuedouble;
    }

    cJSON *rows = cJSON_GetObjectItem(root, "d");
    if (rows) {
        int y = 0;
        cJSON *row;
        cJSON_ArrayForEach(row, rows) {
            if (y >= BL_MESH_MAX) break;
            int x = 0;
            cJSON *val;
            cJSON_ArrayForEach(val, row) {
                if (x >= BL_MESH_MAX) break;
                mesh->data[y][x] = val->valuedouble;
                x++;
            }
            y++;
        }
    }

    cJSON_Delete(root);
    return true;
}

/* ---- Mesh list (sorted by timestamp descending) ---- */

typedef struct { int64_t ts; char path[48]; } mesh_entry_t;

static int mesh_list_cmp(const void *a, const void *b)
{
    int64_t ta = ((const mesh_entry_t *)a)->ts;
    int64_t tb = ((const mesh_entry_t *)b)->ts;
    return (tb > ta) - (tb < ta);  /* descending */
}

static int get_mesh_list(mesh_entry_t *out, int max)
{
    DIR *d = opendir(BL_CACHE_DIR);
    if (!d) return 0;
    int n = 0;
    struct dirent *e;
    while ((e = readdir(d)) && n < max) {
        if (strncmp(e->d_name, "mesh_", 5) != 0) continue;
        if (strlen(e->d_name) > 30) continue;  /* sanity */
        int64_t ts = 0;
        if (sscanf(e->d_name, "mesh_%lld.json", (long long *)&ts) != 1) continue;
        out[n].ts = ts;
        snprintf(out[n].path, sizeof(out[n].path), BL_CACHE_DIR "/%s", e->d_name);
        n++;
    }
    closedir(d);
    qsort(out, n, sizeof(mesh_entry_t), mesh_list_cmp);
    return n;
}

/* ---- Line-by-line mesh parsing state machine ---- */

typedef enum {
    MESH_PARSE_IDLE,
    MESH_PARSE_SKIP_HEADER,   /* saw marker, next line is column headers — skip it */
    MESH_PARSE_DATA,          /* reading data rows */
} mesh_parse_state_t;

static mesh_parse_state_t s_parse_state = MESH_PARSE_IDLE;
static bl_mesh_t s_parse_mesh;

/**
 * Called from process_line() for every RX line — always active.
 * Detects mesh output from G29 / M420 V regardless of how it was triggered
 * (probe button, terminal, printer UI, OctoPrint, etc.).
 */
static void mesh_parse_line(const char *line)
{
    switch (s_parse_state) {
    case MESH_PARSE_IDLE:
        if (strstr(line, "Leveling Grid:") || strstr(line, "Bed Topography")) {
            memset(&s_parse_mesh, 0, sizeof(s_parse_mesh));
            s_parse_mesh.min_z = 999.0f;
            s_parse_mesh.max_z = -999.0f;
            s_parse_state = MESH_PARSE_SKIP_HEADER;
            ESP_LOGI(TAG, "Mesh marker detected");
        }
        break;

    case MESH_PARSE_SKIP_HEADER:
        /* Column header line (e.g. "      0      1      2 ...") — skip */
        s_parse_state = MESH_PARSE_DATA;
        break;

    case MESH_PARSE_DATA: {
        const char *p = line;
        while (*p == ' ') p++;
        if (*p < '0' || *p > '9') {
            /* Not a data row — mesh is complete */
            s_parse_state = MESH_PARSE_IDLE;
            if (s_parse_mesh.grid_y > 0 && s_parse_mesh.grid_x > 0) {
                s_parse_mesh.timestamp = (int64_t)time(NULL);
                save_mesh(&s_parse_mesh);
                s_current_mesh = s_parse_mesh;
                /* A calibration may have measured the Z offset again (the Kobra does) */
                printer_comm_query_probe_offset();
                ESP_LOGI(TAG, "Mesh captured: %dx%d, range %.3f..%.3f",
                         s_parse_mesh.grid_x, s_parse_mesh.grid_y,
                         s_parse_mesh.min_z, s_parse_mesh.max_z);
            }
            break;
        }
        if (s_parse_mesh.grid_y >= BL_MESH_MAX) {
            s_parse_state = MESH_PARSE_IDLE;
            break;
        }
        /* Skip row index */
        while (*p >= '0' && *p <= '9') p++;

        int col = 0;
        while (*p && col < BL_MESH_MAX) {
            char *end;
            float val = strtof(p, &end);
            if (end == p) break;
            s_parse_mesh.data[s_parse_mesh.grid_y][col] = val;
            if (val < s_parse_mesh.min_z) s_parse_mesh.min_z = val;
            if (val > s_parse_mesh.max_z) s_parse_mesh.max_z = val;
            col++;
            p = end;
        }
        if (col > 0) {
            if (s_parse_mesh.grid_y == 0) s_parse_mesh.grid_x = col;
            s_parse_mesh.grid_y++;
        }
        break;
    }
    }
}

/* ---- Probe task ---- */

static void probe_task(void *arg)
{
    s_state = BL_HOMING;
    printer_comm_set_polling_suppressed(true);

    ESP_LOGI(TAG, "Probe: homing (G28)...");
    esp_err_t err = printer_comm_send_gcode_sync("G28", PROBE_TIMEOUT_MS);
    if (err != ESP_OK) {
        s_error = BL_ERR_G28;
        s_error_code = err;
        ESP_LOGW(TAG, "G28 failed: %s", esp_err_to_name(err));
        s_state = BL_ERROR;
        goto cleanup;
    }

    s_state = BL_PROBING;
    ESP_LOGI(TAG, "Probe: leveling (G29)...");
    int64_t mesh_ts_before = s_current_mesh.timestamp;
    err = printer_comm_send_gcode_sync("G29", PROBE_TIMEOUT_MS);

    if (err != ESP_OK) {
        s_error = BL_ERR_G29;
        s_error_code = err;
        ESP_LOGW(TAG, "G29 failed: %s", esp_err_to_name(err));
        s_state = BL_ERROR;
        goto cleanup;
    }

    printer_comm_set_polling_suppressed(false);

    /* Mesh is auto-captured by the always-active line callback.
     * The parser may still be processing trailing lines — poll briefly. */
    bool found = false;
    for (int i = 0; i < 10; i++) {
        if (s_current_mesh.timestamp != mesh_ts_before &&
            s_current_mesh.grid_y > 0 && s_current_mesh.grid_x > 0) {
            found = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    if (!found) {
        s_error = BL_ERR_NO_MESH;
        ESP_LOGW(TAG, "No mesh found in G29 output");
        s_state = BL_ERROR;
    } else {
        s_state = BL_DONE;
    }
    task_delete(NULL);
    return;

cleanup:
    printer_comm_set_polling_suppressed(false);
    task_delete(NULL);
}

/* ---- Klipper ---- */

static bool is_klipper(void)
{
    return printer_comm_get_backend() == PRINTER_BACKEND_KLIPPER;
}

/** Read Klipper's last mesh; keep it in the history if it is new.
 *  Returns false if Klipper has none or cannot be reached. */
static bool klipper_sync_mesh(bool probed_now)
{
    bl_mesh_t *m = calloc(1, sizeof(*m));
    if (!m) return false;
    float bounds[4];
    klipper_mesh_info_t info;
    bool ok = klipper_backend_get_mesh(&m->data[0][0], BL_MESH_MAX, &m->grid_x, &m->grid_y,
                                       bounds, &info) == ESP_OK;
    /* A saved profile loaded at start (the Neptune 4's screen loads one) is
     * not a new mesh: it may be from before the screws were turned */
    if (ok && !probed_now && info.from_saved && strcmp(info.profile, "default") != 0) {
        free(m);
        return true;
    }
    if (ok) {
        strlcpy(m->profile, info.profile, sizeof(m->profile));
        m->has_xy = true;
        m->x0 = bounds[0];
        m->y0 = bounds[1];
        m->x1 = bounds[2];
        m->y1 = bounds[3];
        m->min_z = 999.0f;
        m->max_z = -999.0f;
        for (int y = 0; y < m->grid_y; y++) {
            for (int x = 0; x < m->grid_x; x++) {
                if (m->data[y][x] < m->min_z) m->min_z = m->data[y][x];
                if (m->data[y][x] > m->max_z) m->max_z = m->data[y][x];
            }
        }
        /* The same mesh read again is not a new one: compare to the last kept */
        bool same = s_current_mesh.has_xy && s_current_mesh.grid_x == m->grid_x &&
                    s_current_mesh.grid_y == m->grid_y;
        for (int y = 0; same && y < m->grid_y; y++) {
            for (int x = 0; same && x < m->grid_x; x++) {
                same = fabsf(s_current_mesh.data[y][x] - m->data[y][x]) < 0.0005f;
            }
        }
        if (!same) {
            m->timestamp = (int64_t)time(NULL);
            save_mesh(m);
            s_current_mesh = *m;
            ESP_LOGI(TAG, "Klipper mesh %dx%d, range %.3f..%.3f", m->grid_x, m->grid_y,
                     m->min_z, m->max_z);
        }
    }
    free(m);
    return ok;
}

/* Saving for printing: the profile, then SAVE_CONFIG once the script is over */
static char s_save_name[24];
static volatile bool s_save_config;

/** The probe script ended (on the Klipper poll task); arg 1 = it saved a profile */
static void klipper_probe_done(bool completed, int arg)
{
    if (completed) {
        s_save_config = arg == 1;
        s_fetch_mesh = true;
    } else {
        s_error = BL_ERR_CANCELLED;
        s_state = BL_ERROR;
    }
}

/** The profile the mesh for printing goes to: the one chosen, else the one
 *  the printer loads when it starts, else the one loaded now.  Never
 *  "default": Klipper keeps that name for the mesh it has just probed, and
 *  refuses to save to it.  "" if none is known. */
static void print_mesh_name(char *out, size_t size)
{
    if (s_config.print_mesh[0]) {
        strlcpy(out, s_config.print_mesh, size);
        return;
    }
    klipper_backend_boot_mesh(out, size);
    if (out[0]) return;
    float *d = malloc(BL_MESH_MAX * BL_MESH_MAX * sizeof(float));
    klipper_mesh_info_t info;
    float b[4];
    int nx, ny;
    if (d) klipper_backend_get_mesh(d, BL_MESH_MAX, &nx, &ny, b, &info);
    free(d);
    if (d && strcmp(info.profile, "default") != 0) strlcpy(out, info.profile, size);
}

/** Height of a mesh at (x, y), bilinear, clamped to its grid */
static float mesh_at(const float *d, int max_n, int nx, int ny, const float b[4], float x, float y)
{
    float fx = b[2] > b[0] ? (x - b[0]) / (b[2] - b[0]) * (nx - 1) : 0;
    float fy = b[3] > b[1] ? (y - b[1]) / (b[3] - b[1]) * (ny - 1) : 0;
    if (fx < 0) fx = 0;
    if (fx > nx - 1) fx = nx - 1;
    if (fy < 0) fy = 0;
    if (fy > ny - 1) fy = ny - 1;
    int ix = (int)fx, iy = (int)fy;
    if (ix > nx - 2) ix = nx - 2;
    if (iy > ny - 2) iy = ny - 2;
    float tx = fx - ix, ty = fy - iy;
    const float *r0 = d + iy * max_n, *r1 = r0 + max_n;
    float a = r0[ix] + (r0[ix + 1] - r0[ix]) * tx;
    float c = r1[ix] + (r1[ix + 1] - r1[ix]) * tx;
    return a + (c - a) * ty;
}

#define PRINT_MESH_CACHE_US (30 * 1000000LL)
static SemaphoreHandle_t s_check_lock;

bool bedlevel_print_mesh_check(bl_print_mesh_t *out)
{
    static bool cached_ok;
    static bl_print_mesh_t cached;
    static int64_t cached_us;
    SemaphoreHandle_t lock = s_check_lock;
    if (!is_klipper() || !lock) return false;
    xSemaphoreTake(lock, portMAX_DELAY);
    int64_t now = esp_timer_get_time();
    if (cached_us && now - cached_us < PRINT_MESH_CACHE_US) {
        *out = cached;
        bool ok = cached_ok;
        xSemaphoreGive(lock);
        return ok;
    }

    bool ok = false;
    memset(out, 0, sizeof(*out));
    bl_mesh_t *probe = malloc(sizeof(*probe));
    float *prof = malloc(BL_MESH_MAX * BL_MESH_MAX * sizeof(float));
    int pnx = 0, pny = 0;
    float pb[4];
    if (probe && prof && klipper_sync_mesh(false)) {
        *probe = s_current_mesh;
        print_mesh_name(out->profile, sizeof(out->profile));
        if (probe->has_xy && probe->grid_x >= 2 && probe->grid_y >= 2 && out->profile[0] &&
            klipper_backend_get_profile(out->profile, prof, BL_MESH_MAX, &pnx, &pny, pb) == ESP_OK) {
            /* Only where both were probed: past the profile's edge it would be extrapolated */
            float lo = 1e9f, hi = -1e9f;
            int n = 0;
            for (int y = 0; y < probe->grid_y; y++) {
                float py = probe->y0 + (probe->y1 - probe->y0) * y / (probe->grid_y - 1);
                if (py < pb[1] - 1 || py > pb[3] + 1) continue;
                for (int x = 0; x < probe->grid_x; x++) {
                    float px = probe->x0 + (probe->x1 - probe->x0) * x / (probe->grid_x - 1);
                    if (px < pb[0] - 1 || px > pb[2] + 1) continue;
                    float d = probe->data[y][x] - mesh_at(prof, BL_MESH_MAX, pnx, pny, pb, px, py);
                    if (d < lo) lo = d;
                    if (d > hi) hi = d;
                    n++;
                }
            }
            if (n >= 4) {
                ok = true;
                out->off_mm = hi - lo;
                out->nx = probe->grid_x;
                out->ny = probe->grid_y;
                out->ts = probe->timestamp;
            }
        }
    }
    free(probe);
    free(prof);
    /* Klipper unreachable: try again on the next call */
    cached_ok = ok;
    cached = *out;
    cached_us = ok ? now : 0;
    xSemaphoreGive(lock);
    return ok;
}

/** Heat the bed, home, probe, lift: sent as a built-in script, so it shows on
 *  /print and can be stopped there.  Not the printer's G29: on the Neptune 4
 *  it ends with the nozzle lowered to Z0 in the middle of the bed.
 *  save: probe as densely as the profile's name says (Elegoo names them 6 and
 *  11 for 6x6 and 11x11) and save it; SAVE_CONFIG follows the script. */
static esp_err_t klipper_probe_start(bool save)
{
    char *script = malloc(384);
    if (!script) return ESP_ERR_NO_MEM;
    int n = 0;
    n += snprintf(script + n, 384 - n, "; Bed mesh for %s\n", save ? "printing" : "leveling");
    if (s_config.bed_c > 0) n += snprintf(script + n, 384 - n, "M190 S%d\n", s_config.bed_c);
    n += snprintf(script + n, 384 - n, "BED_MESH_CLEAR\nG28\nBED_MESH_CALIBRATE");
    if (save) {
        print_mesh_name(s_save_name, sizeof(s_save_name));
        if (!s_save_name[0]) {
            free(script);
            return ESP_ERR_NOT_FOUND;
        }
        char *end;
        long count = strtol(s_save_name, &end, 10);
        if (*end == '\0' && count >= 3 && count <= BL_MESH_MAX)
            n += snprintf(script + n, 384 - n, " PROBE_COUNT=%ld,%ld", count, count);
        n += snprintf(script + n, 384 - n, "\nBED_MESH_PROFILE SAVE=%s", s_save_name);
    }
    snprintf(script + n, 384 - n, "\nG90\nG1 Z10 F600\n");
    s_fetch_mesh = false;
    s_save_config = false;
    return printer_comm_host_print_script(save ? "Bed mesh for printing" : "Bed mesh", script,
                                          klipper_probe_done, save ? 1 : 0);
}

/* ---- Mesh JSON builder ---- */

static void mesh_to_json(const bl_mesh_t *mesh, html_buf_t *p)
{
    html_buf_printf(p, "{\"ts\":%lld,\"gx\":%d,\"gy\":%d,\"min\":%.3f,\"max\":%.3f,",
                    (long long)mesh->timestamp, mesh->grid_x, mesh->grid_y, mesh->min_z, mesh->max_z);
    if (mesh->has_xy) {
        html_buf_printf(p, "\"x0\":%.1f,\"y0\":%.1f,\"x1\":%.1f,\"y1\":%.1f,",
                        mesh->x0, mesh->y0, mesh->x1, mesh->y1);
    }
    if (mesh->profile[0]) html_buf_printf(p, "\"pr\":\"%s\",", mesh->profile);
    html_buf_printf(p, "\"d\":[");
    for (int y = 0; y < mesh->grid_y; y++) {
        html_buf_printf(p, "%s[", y ? "," : "");
        for (int x = 0; x < mesh->grid_x; x++) {
            html_buf_printf(p, "%s%.3f", x ? "," : "", mesh->data[y][x]);
        }
        html_buf_printf(p, "]");
    }
    html_buf_printf(p, "]}");
}

/* ---- HTTP handlers ---- */

/* {"ok":false,"err":msg}: msg is shown on the page as is */
static esp_err_t probe_refuse(httpd_req_t *req, const char *msg)
{
    char buf[192];
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"err\":\"%s\"}", msg);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t bedlevel_probe_handler(httpd_req_t *req)
{
    if (s_state == BL_HOMING || s_state == BL_PROBING) {
        return probe_refuse(req, _("Probing already in progress"));
    }

    printer_state_t st;
    printer_comm_get_state(&st);
    if (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED) {
        return probe_refuse(req, _("Cannot probe while printing"));
    }

    s_state = BL_IDLE;
    s_error = BL_ERR_NONE;

    if (is_klipper()) {
        char body[16] = "";
        int len = httpd_req_recv(req, body, sizeof(body) - 1);
        body[len > 0 ? len : 0] = '\0';
        bool save = strstr(body, "save=1") != NULL;
        if (st.opstate != PRINTER_OPERATIONAL) return probe_refuse(req, _("The printer must be connected and idle."));
        esp_err_t err = klipper_probe_start(save);
        if (err == ESP_ERR_NOT_FOUND)
            return probe_refuse(req, _("Choose the mesh for printing in the bed configuration."));
        if (err != ESP_OK) return probe_refuse(req, _("Could not start, try again."));
        s_state = BL_PROBING;
        s_probe_start_us = esp_timer_get_time();
        s_probe_seen = false;
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_sendstr(req, "{\"ok\":true}");
    }

    if (task_create_psram(probe_task, "bl_probe", 8192, NULL, 5, NULL, 0) != pdPASS) {
        return probe_refuse(req, _("Failed to start probe task"));
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t bedlevel_status_handler(httpd_req_t *req)
{
    if (is_klipper()) {
        if (s_fetch_mesh) {
            s_fetch_mesh = false;
            int64_t before = s_current_mesh.timestamp;
            if (klipper_sync_mesh(true) && s_current_mesh.timestamp != before) {
                s_state = BL_DONE;
                /* Only if Klipper took the profile: it answers a refused SAVE
                 * with an error the script cannot see */
                if (s_save_config && !klipper_backend_profile_has_mesh(s_save_name)) {
                    s_save_config = false;
                    s_error = BL_ERR_SAVE;
                    s_state = BL_ERROR;
                    ESP_LOGW(TAG, "Klipper did not save the mesh to profile %s", s_save_name);
                }
                if (s_save_config) {
                    /* Writes the profile into printer.cfg; Klipper restarts */
                    s_save_config = false;
                    printer_cmd_t cmd = { .type = PCMD_RAW };
                    strlcpy(cmd.gcode, "SAVE_CONFIG", sizeof(cmd.gcode));
                    printer_comm_send_cmd(&cmd);
                    ESP_LOGI(TAG, "Mesh saved to profile %s, SAVE_CONFIG", s_save_name);
                } else {
                    s_save_name[0] = '\0';
                }
            } else {
                s_error = BL_ERR_NO_MESH;
                s_state = BL_ERROR;
            }
        } else if (s_state != BL_PROBING) {
            klipper_sync_mesh(false);   /* A mesh probed from the printer's screen or Fluidd */
        } else {
            printer_state_t st;
            printer_comm_get_state(&st);
            if (st.builtin_script) s_probe_seen = true;
            if (!s_probe_seen && !klipper_backend_sending() &&
                esp_timer_get_time() - s_probe_start_us > PROBE_START_WAIT_US) {
                s_error = BL_ERR_START;
                s_state = BL_ERROR;
            }
        }
    }

    const char *state_str;
    switch (s_state) {
    case BL_HOMING:  state_str = "homing"; break;
    case BL_PROBING: state_str = "probing"; break;
    case BL_DONE:    state_str = "done"; break;
    case BL_ERROR:   state_str = "error"; break;
    default:         state_str = "idle"; break;
    }

    html_buf_t p;
    html_buf_init(&p);
    html_buf_printf(&p, "{\"state\":\"%s\"", state_str);
    if (s_state == BL_ERROR && s_error != BL_ERR_NONE) {
        html_buf_printf(&p, ",\"err\":\"");
        switch (s_error) {
        case BL_ERR_G28: html_buf_printf(&p, _("G28 failed: %s"), esp_err_to_name(s_error_code)); break;
        case BL_ERR_G29: html_buf_printf(&p, _("G29 failed: %s"), esp_err_to_name(s_error_code)); break;
        case BL_ERR_CANCELLED: html_buf_printf(&p, "%s", _("Probing was stopped")); break;
        case BL_ERR_START: html_buf_printf(&p, "%s", _("Probing did not start")); break;
        case BL_ERR_SAVE: html_buf_printf(&p, _("Klipper did not save the mesh to profile %s"), s_save_name); break;
        default:
            html_buf_printf(&p, "%s", is_klipper() ? _("Klipper has no new mesh") : _("No mesh found in G29 output"));
            break;
        }
        html_buf_printf(&p, "\"");
    }
    if (s_state == BL_DONE && s_save_name[0]) html_buf_printf(&p, ",\"saved\":\"%s\"", s_save_name);
    if (s_state == BL_DONE && s_current_mesh.grid_x > 0) {
        html_buf_printf(&p, ",\"mesh\":");
        mesh_to_json(&s_current_mesh, &p);
    }
    html_buf_printf(&p, "}");

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t bedlevel_meshlist_handler(httpd_req_t *req)
{
    mesh_entry_t *list = malloc(BL_CACHE_MAX * sizeof(mesh_entry_t));
    if (!list) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM"); return ESP_FAIL; }
    int n = get_mesh_list(list, BL_CACHE_MAX);

    html_buf_t p;
    html_buf_init(&p);
    html_buf_printf(&p, "[");
    for (int i = 0; i < n; i++) {
        html_buf_printf(&p, "%s%lld", i ? "," : "", (long long)list[i].ts);
    }
    html_buf_printf(&p, "]");

    free(list);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t bedlevel_mesh_handler(httpd_req_t *req)
{
    char ts_str[24] = {0};
    httpd_req_get_url_query_str(req, ts_str, sizeof(ts_str));
    int64_t ts = 0;
    char *eq = strchr(ts_str, '=');
    if (eq) ts = atoll(eq + 1);

    char path[48];
    snprintf(path, sizeof(path), BL_CACHE_DIR "/mesh_%lld.json", (long long)ts);

    bl_mesh_t *mesh = malloc(sizeof(*mesh));
    if (!mesh || !load_mesh_file(path, mesh)) {
        free(mesh);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Mesh not found");
        return ESP_FAIL;
    }

    html_buf_t p;
    html_buf_init(&p);
    mesh_to_json(mesh, &p);
    free(mesh);

    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

static esp_err_t bedlevel_config_get_handler(httpd_req_t *req)
{
    cJSON *root = cJSON_CreateObject();
    if (root) {
        config_to_json(root);
        cJSON_AddBoolToObject(root, "klipper", is_klipper());
        if (is_klipper()) {
            char boot[24], use[24];
            klipper_backend_boot_mesh(boot, sizeof(boot));
            print_mesh_name(use, sizeof(use));
            cJSON_AddStringToObject(root, "boot_mesh", boot);
            cJSON_AddStringToObject(root, "use_mesh", use);
            bl_mesh_t *m = malloc(sizeof(*m));
            klipper_mesh_info_t info;
            float b[4];
            if (m && klipper_backend_get_mesh(&m->data[0][0], BL_MESH_MAX, &m->grid_x, &m->grid_y,
                                              b, &info) != ESP_ERR_INVALID_STATE) {
                cJSON *a = cJSON_AddArrayToObject(root, "profiles");
                for (int i = 0; a && i < info.n_names; i++)
                    cJSON_AddItemToArray(a, cJSON_CreateString(info.names[i]));
                cJSON_AddStringToObject(root, "active_mesh", info.profile);
            }
            free(m);
        }
    }
    char *json = root ? cJSON_PrintUnformatted(root) : NULL;
    cJSON_Delete(root);
    if (!json) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OOM");
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_sendstr(req, json);
    free(json);
    return ret;
}

/** Points [[x, y], ...] within the bed into out; returns how many */
static int read_points(const cJSON *arr, float out[][2], int max)
{
    int n = 0;
    const cJSON *pt;
    cJSON_ArrayForEach(pt, arr) {
        if (n >= max) break;
        const cJSON *x = cJSON_GetArrayItem(pt, 0), *y = cJSON_GetArrayItem(pt, 1);
        if (!cJSON_IsNumber(x) || !cJSON_IsNumber(y)) continue;
        if (x->valuedouble < -50 || x->valuedouble > 1000 || y->valuedouble < -50 || y->valuedouble > 1000)
            continue;
        out[n][0] = x->valuedouble;
        out[n][1] = y->valuedouble;
        n++;
    }
    return n;
}

static int read_points(const cJSON *arr, float out[][2], int max);

static void apply_config_json(const cJSON *root)
{
    const cJSON *v;
    if ((v = cJSON_GetObjectItem(root, "bed_w")) && cJSON_IsNumber(v)) s_config.bed_w = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "bed_h")) && cJSON_IsNumber(v)) s_config.bed_h = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "screw_spacing")) && cJSON_IsNumber(v)) s_config.screw_spacing = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "screw_pitch")) && cJSON_IsNumber(v)) s_config.screw_pitch = v->valuedouble;
    if ((v = cJSON_GetObjectItem(root, "origin")) && cJSON_IsNumber(v)) s_config.origin = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "cw_raises")) && cJSON_IsBool(v)) s_config.cw_raises = cJSON_IsTrue(v);
    if ((v = cJSON_GetObjectItem(root, "turn_div")) && cJSON_IsNumber(v) &&
        (v->valueint == 3 || v->valueint == 4 || v->valueint == 6 || v->valueint == 8))
        s_config.turn_div = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "bed_c")) && cJSON_IsNumber(v) && v->valueint >= 0 && v->valueint <= 110)
        s_config.bed_c = v->valueint;
    if ((v = cJSON_GetObjectItem(root, "print_mesh")) && cJSON_IsString(v)) {
        /* A Klipper profile name: goes into a G-code command, so letters, digits, _ and - only */
        char name[sizeof(s_config.print_mesh)];
        size_t n = 0;
        for (const char *c = v->valuestring; *c && n < sizeof(name) - 1; c++) {
            if (isalnum((unsigned char)*c) || *c == '_' || *c == '-') name[n++] = *c;
        }
        name[n] = '\0';
        strcpy(s_config.print_mesh, name);
    }
    if ((v = cJSON_GetObjectItem(root, "screws")) && cJSON_IsArray(v))
        s_config.n_screws = read_points(v, s_config.screws, BL_SCREWS_MAX);
    if ((v = cJSON_GetObjectItem(root, "fixed")) && cJSON_IsArray(v))
        s_config.n_fixed = read_points(v, s_config.fixed, BL_FIXED_MAX);
    if (s_config.screw_pitch < 0.1f || s_config.screw_pitch > 5.0f) s_config.screw_pitch = 0.7f;
}

void bedlevel_bed_center(float *x, float *y)
{
    *x = s_config.bed_w / 2;
    *y = s_config.bed_h / 2;
}

void bedlevel_settings_export(cJSON *obj)
{
    config_to_json(obj);
}

void bedlevel_settings_import(const cJSON *obj)
{
    apply_config_json(obj);
    save_config();
}

static esp_err_t bedlevel_config_post_handler(httpd_req_t *req)
{
    char buf[1024];
    int len = 0;
    while (len < (int)req->content_len && len < (int)sizeof(buf) - 1) {
        int r = httpd_req_recv(req, buf + len, sizeof(buf) - 1 - len);
        if (r <= 0) break;
        len += r;
    }
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");
        return ESP_FAIL;
    }
    buf[len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    if (!root) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    apply_config_json(root);
    cJSON_Delete(root);

    save_config();

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

/* ---- Page handler ---- */

/* A template for i18n_page(): not a printf format, % is written once */
static const char BL_PAGE[] =
        "<style>"
        ".mesh-grid{display:inline-grid;gap:2px;margin:12px 0}"
        ".mesh-cell{width:48px;height:48px;display:flex;align-items:center;justify-content:center;"
        "font-size:11px;color:#fff;border-radius:3px;text-shadow:0 0 2px rgba(0,0,0,0.7)}"
        ".stats{display:flex;gap:16px;flex-wrap:wrap;margin:8px 0;font-size:14px}"
        ".stats span{padding:2px 8px;border-radius:4px}"
        ".badge-good{background:#4caf50;color:#fff}"
        ".badge-fair{background:#ff9800;color:#fff}"
        ".badge-poor{background:#f44336;color:#fff}"
        ".verdict{font-size:17px;font-weight:bold;margin:10px 0 4px}"
        ".screw-tbl{margin:12px 0;border-collapse:collapse}"
        ".screw-tbl td,.screw-tbl th{padding:4px 10px;text-align:left}"
        ".turn{font-weight:bold}"
        ".hist-nav{display:flex;align-items:center;gap:12px;margin:8px 0}"
        "#probe-btn:disabled{opacity:0.5}"
        "#bedmap svg{max-width:100%;height:auto}"
        "details{margin:12px 0}"
        "summary{cursor:pointer;font-weight:bold;font-size:15px}"
        "input[type=number]{width:80px;padding:4px}"
        ".cfg-row{margin:6px 0}"
        ".cfg-row label{display:inline-block;width:180px}"
        ".cfg-row input[type=text]{width:100%;max-width:420px;padding:4px;font-family:monospace}"
        "</style>"

        "<h2>" N_("Bed Leveling") "</h2>"

        "<button id='probe-btn' onclick='probe(0)'></button>"
        " <button id='save-btn' onclick='probe(1)' style='display:none'></button>"
        " <span id='probe-status'></span>"

        "<div id='verdict' class='verdict'></div>"
        "<div id='mesh-stats'></div>"
        "<div id='bedmap'></div>"
        "<div id='mesh-box'></div>"

        "<div class='hist-nav'>"
        "<button onclick='histPrev()'>&#9664; " N_("Prev") "</button>"
        "<span id='hist-label'>-</span>"
        "<button onclick='histNext()'>" N_("Next") " &#9654;</button>"
        "</div>"

        "<div id='screws-xy'></div>"

        "<details id='quad'>"
        "<summary>" N_("Screw Adjustment") "</summary>"
        "<div style='margin:8px 0'>"
        "<button onclick='calcScrews(\"tighten\")'>" N_("Tighten (CW)") "</button>"
        " <button onclick='calcScrews(\"loosen\")'>" N_("Loosen (CCW)") "</button>"
        "</div>"
        "<div id='screw-result'></div>"
        "</details>"

        "<details>"
        "<summary>" N_("Bed Configuration") "</summary>"
        "<div class='cfg-row'><label>" N_("Bed width (mm):") "</label>"
        "<input type='number' id='cfg-bw' step='1'></div>"
        "<div class='cfg-row'><label>" N_("Bed height (mm):") "</label>"
        "<input type='number' id='cfg-bh' step='1'></div>"
        "<div class='cfg-row'><label>" N_("Screw spacing (mm):") "</label>"
        "<input type='number' id='cfg-ss' step='1'></div>"
        "<div class='cfg-row'><label>" N_("Screw pitch (mm/rot):") "</label>"
        "<input type='number' id='cfg-sp' step='0.01' value='1.0'> "
        "<span class='hint'>M3 0.5, M4 0.7, M5 0.8</span></div>"
        "<div class='cfg-row'><label>" N_("Clockwise:") "</label>"
        "<select id='cfg-cw'>"
        "<option value='0'>" N_("lowers the bed") "</option>"
        "<option value='1'>" N_("raises the bed") "</option>"
        "</select></div>"
        "<div class='cfg-row'><label>" N_("Turn in steps of:") "</label>"
        "<select id='cfg-td'>"
        "<option value='8'>1/8</option><option value='6'>1/6</option>"
        "<option value='4'>1/4</option><option value='3'>1/3</option>"
        "</select> <span class='hint'>" N_("1/3 for a knob with three lobes") "</span></div>"
        "<div class='cfg-row'><label>" N_("Origin (0,0) corner:") "</label>"
        "<select id='cfg-or'>"
        "<option value='0'>" N_("Front-Left") "</option>"
        "<option value='1'>" N_("Front-Right") "</option>"
        "<option value='2'>" N_("Back-Left") "</option>"
        "<option value='3'>" N_("Back-Right") "</option>"
        "</select></div>"
        "<div id='cfg-k'>"
        "<div class='cfg-row'><label>" N_("Probe at bed temperature:") "</label>"
        "<input type='number' id='cfg-bt' min='0' max='110' step='5'> &deg;C "
        "<span class='hint'>" N_("0 = cold") "</span></div>"
        "<div class='cfg-row'><label>" N_("Mesh for printing:") "</label>"
        "<select id='cfg-pm'></select> <span class='hint' id='cfg-pm-hint'></span></div>"
        "<div class='cfg-row'><label>" N_("Screws (X,Y; X,Y; ...):") "</label><br>"
        "<input type='text' id='cfg-scr' placeholder='37.5,392.5; 37.5,37.5; 392.5,392.5; 392.5,37.5'></div>"
        "<div class='cfg-row'><label>" N_("Fixed supports (X,Y; ...):") "</label><br>"
        "<input type='text' id='cfg-fix' placeholder='215,155; 215,275'></div>"
        "<p class='hint'>" N_("Nozzle coordinates over each screw. Fixed supports cannot be turned: "
        "the screws are set to the line or plane through them. Without fixed supports the screws "
        "are set to their middle height.") "</p>"
        "</div>"
        "<button onclick='saveCfg()' style='margin-top:8px'>" N_("Save Config") "</button>"
        " <span id='cfg-status'></span>"
        "</details>"

        "<script>"
        "var meshList=[],meshIdx=-1,curMesh=null,cfg={};"
        "function $(i){return document.getElementById(i)}"

        /* Color mapping: blue(low) -> green(0) -> red(high) */
        "function zColor(z,mn,mx){"
        "var am=Math.max(Math.abs(mn),Math.abs(mx),0.01);"
        "var n=z/am;"
        "var h=n<=0?120+(-n)*120:120-n*120;"
        "var s=Math.min(100,Math.abs(n)*150+30);"
        "return 'hsl('+h+','+s+'%,42%)'}"

        /* Height of the bed at (x,y): bilinear, straight on past the edge points */
        "function zAt(m,x,y){var nx=m.gx,ny=m.gy,d=m.d;"
        "var fx=(x-m.x0)/(m.x1-m.x0)*(nx-1),fy=(y-m.y0)/(m.y1-m.y0)*(ny-1);"
        "var i=Math.min(Math.max(Math.floor(fx),0),nx-2),j=Math.min(Math.max(Math.floor(fy),0),ny-2),tx=fx-i,ty=fy-j;"
        "return (d[j][i]*(1-tx)+d[j][i+1]*tx)*(1-ty)+(d[j+1][i]*(1-tx)+d[j+1][i+1]*tx)*ty}"

        /* The height each screw is set to: the line or plane through the fixed
           supports, or the middle of the screw heights if there are none */
        "function target(m,zs){var f=(cfg.fixed||[]).map(function(p){return [p[0],p[1],zAt(m,p[0],p[1])]});"
        "if(f.length==1)return function(){return f[0][2]};"
        "if(f.length==2){var a=f[0],b=f[1],dx=b[0]-a[0],dy=b[1]-a[1],L=dx*dx+dy*dy||1;"
        "return function(x,y){return a[2]+(b[2]-a[2])*((x-a[0])*dx+(y-a[1])*dy)/L}}"
        "if(f.length>=3){var S=[[0,0,0],[0,0,0],[0,0,0]],R=[0,0,0];"
        "f.forEach(function(p){var v=[1,p[0],p[1]];for(var r=0;r<3;r++){R[r]+=v[r]*p[2];for(var c=0;c<3;c++)S[r][c]+=v[r]*v[c]}});"
        "var k=solve3(S,R);return function(x,y){return k[0]+k[1]*x+k[2]*y}}"
        "var z=zs.slice().sort(function(a,b){return a-b}),mid=(z[0]+z[z.length-1])/2;"
        "return function(){return mid}}"
        "function solve3(A,b){var M=A.map(function(r,i){return r.concat([b[i]])});"
        "for(var c=0;c<3;c++){var p=c;for(var r=c+1;r<3;r++)if(Math.abs(M[r][c])>Math.abs(M[p][c]))p=r;"
        "var t=M[c];M[c]=M[p];M[p]=t;for(var r=0;r<3;r++)if(r!=c){var q=M[r][c]/(M[c][c]||1e-9);"
        "for(var k=c;k<4;k++)M[r][k]-=q*M[c][k]}}"
        "return [M[0][3]/M[0][0],M[1][3]/M[1][1],M[2][3]/M[2][2]]}"

        /* Steps of 1/div turn as text: 3 eighths -> 3/8, 10 eighths -> 1 1/4, 4 sixths -> 2/3 */
        "function gcd(a,b){return b?gcd(b,a%b):a}"
        "function frac(n){var q=cfg.turn_div||8,w=Math.floor(n/q),r=n%q,s=w?''+w:'';"
        "if(r){var g=gcd(r,q);s+=(s?' ':'')+(r/g)+'/'+(q/g)}return s}"

        /* Where on the bed a point is, in words */
        "function where(x,y){var bw=cfg.bed_w||220,bh=cfg.bed_h||220,v='',h='';"
        "if(y>bh*0.62)v='" N_("back") "';else if(y<bh*0.38)v='" N_("front") "';"
        "if(x<bw*0.38)h='" N_("left") "';else if(x>bw*0.62)h='" N_("right") "';"
        "var s=v&&h?v+' '+h:v||h||'" N_("middle") "';return s.charAt(0).toUpperCase()+s.slice(1)}"

        /* What to do with each screw */
        "function screwPlan(m){var sc=cfg.screws||[];if(m.x0===undefined||!sc.length)return null;"
        "var zs=sc.map(function(p){return zAt(m,p[0],p[1])}),t=target(m,zs);"
        "var e=(cfg.screw_pitch||0.7)/(cfg.turn_div||8),bw=cfg.bed_w||220,bh=cfg.bed_h||220;"
        "return sc.map(function(p,i){var diff=zs[i]-t(p[0],p[1]),n=Math.round(Math.abs(diff)/e),up=diff<0;"
        "var corner=Math.min(Math.abs(p[0]-bw/2)/bw,Math.abs(p[1]-bh/2)/bh);"
        "return {i:i+1,x:p[0],y:p[1],z:zs[i],diff:diff,n:n,up:up,cw:up==!!cfg.cw_raises,corner:corner}})}"

        /* Where each point is, in mm: a mesh without coordinates (Marlin) spans the bed */
        "function ptXY(m,i,j){var bw=cfg.bed_w||220,bh=cfg.bed_h||220;"
        "return m.x0!==undefined?[m.x0+(m.x1-m.x0)*i/(m.gx-1),m.y0+(m.y1-m.y0)*j/(m.gy-1)]:[bw*i/(m.gx-1),bh*j/(m.gy-1)]}"

        /* The plane that fits the mesh best: its tilt the mesh makes up for,
           what is left over (bumps, sag) is what matters for the first layer */
        "function fitPlane(m){var S=[[0,0,0],[0,0,0],[0,0,0]],R=[0,0,0];"
        "for(var j=0;j<m.gy;j++)for(var i=0;i<m.gx;i++){var p=ptXY(m,i,j),v=[1,p[0],p[1]],z=m.d[j][i];"
        "for(var r=0;r<3;r++){R[r]+=v[r]*z;for(var c=0;c<3;c++)S[r][c]+=v[r]*v[c]}}"
        "var k=solve3(S,R),lo=1e9,hi=-1e9;"
        "for(var j=0;j<m.gy;j++)for(var i=0;i<m.gx;i++){var p=ptXY(m,i,j),e=m.d[j][i]-(k[0]+k[1]*p[0]+k[2]*p[1]);"
        "lo=Math.min(lo,e);hi=Math.max(hi,e)}"
        "var a=ptXY(m,0,0),b=ptXY(m,m.gx-1,m.gy-1);"
        "return {flat:hi-lo,tx:Math.abs(k[1]*(b[0]-a[0])),ty:Math.abs(k[2]*(b[1]-a[1]))}}"

        /* The tilt no screw can take out: along the line of two fixed supports,
           or of the plane through three or more; mm over the screws' span */
        "function fixedTilt(m){var f=cfg.fixed||[],sc=cfg.screws||[];if(f.length<2||m.x0===undefined)return 0;"
        "var z=f.map(function(p){return zAt(m,p[0],p[1])}),span=0,dz=0;"
        "if(f.length==2){var dx=f[1][0]-f[0][0],dy=f[1][1]-f[0][1],L=Math.hypot(dx,dy)||1;"
        "sc.forEach(function(p){sc.forEach(function(q){span=Math.max(span,Math.abs((p[0]-q[0])*dx+(p[1]-q[1])*dy)/L)})});"
        "return Math.abs(z[1]-z[0])/L*span}"
        "var t=target(m,[]),a=t(0,0),bx=t(1,0)-a,by=t(0,1)-a;"
        "sc.forEach(function(p){sc.forEach(function(q){span=Math.max(span,Math.hypot(p[0]-q[0],p[1]-q[1]))})});"
        "return Math.hypot(bx,by)*span}"
        "var MECH_TILT=2.0;"

        /* Verdict: screws to turn first, then how uneven the bed is without its
           tilt; the tilt on a line of its own */
        "function verdict(m,plan){var f=fitPlane(m),r=f.flat,b=r<=0.15?'good':r<=0.30?'fair':'poor';"
        "var t={good:'" N_("Excellent, nothing to do") "',fair:'" N_("Will do: the mesh makes up for it") "',"
        "poor:'" N_("Needs leveling") "'}[b];"
        "if(plan&&plan.some(function(s){return s.n>0})){b=b=='poor'?'poor':'fair';t='" N_("Turn the screws") "'}"
        "if(fixedTilt(m)>MECH_TILT){b='poor';t='" N_("Fix the mechanics first") "'}"
        "var tilt=Math.max(f.tx,f.ty),big=tilt>1.0;"
        "var h='<span class=\"badge-'+b+'\" style=\"padding:4px 10px;border-radius:4px\">'+t+'</span>'"
        "+'<div class=\"stats\" style=\"font-weight:normal\"><span>" N_("Unevenness without the tilt:") " '+r.toFixed(2)+' " N_("mm") "</span></div>'"
        "+'<div style=\"font-weight:normal;font-size:14px'+(big?';color:#c62828':'')+'\">" N_("Tilt:") " '"
        "+f.ty.toFixed(2)+' " N_("mm front to back") ", '+f.tx.toFixed(2)+' " N_("mm left to right") " &mdash; '"
        "+(big?'" N_("too much: check the mechanics (wheels, carriage, supports)") "':'" N_("the mesh makes up for it") "')"
        "+((cfg.fixed||[]).length>=2&&m.x0!==undefined?'. " N_("The screws cannot change a tilt along the fixed supports.") "':'')+'</div>';"
        "$('verdict').innerHTML=h}"

        /* The bed seen from above, back at the top: probed points, screws, supports */
        "function bedMap(m,plan){var bw=cfg.bed_w||220,bh=cfg.bed_h||220,W=Math.min(420,window.innerWidth-40),k=W/bw,H=bh*k;"
        "var X=function(x){return x*k},Y=function(y){return H-y*k};"
        "var dx=(m.x1-m.x0)/(m.gx-1)*k,dy=(m.y1-m.y0)/(m.gy-1)*k,fs=Math.min(10,Math.min(dx,dy)/3.4);"
        "var h='<svg viewBox=\"-2 -2 '+(W+4)+' '+(H+4)+'\" width=\"'+(W+4)+'\" font-family=\"sans-serif\">'"
        "+'<rect x=\"0\" y=\"0\" width=\"'+W+'\" height=\"'+H+'\" fill=\"#ddd\" stroke=\"#888\"/>';"
        /* Each cell reaches halfway to the next point, the outer ones to the bed's
           edge: centred cells would hang off the bed where the first point is
           nearer the edge than half a step (x 20 with 75 mm steps) */
        "var sx=(m.x1-m.x0)/(m.gx-1),sy=(m.y1-m.y0)/(m.gy-1);"
        "for(var j=0;j<m.gy;j++)for(var i=0;i<m.gx;i++){var x=m.x0+sx*i,y=m.y0+sy*j,v=m.d[j][i];"
        "var xl=i?x-sx/2:Math.min(0,x),xr=i<m.gx-1?x+sx/2:Math.max(bw,x),yb=j?y-sy/2:Math.min(0,y),yt=j<m.gy-1?y+sy/2:Math.max(bh,y);"
        "h+='<rect x=\"'+X(xl)+'\" y=\"'+Y(yt)+'\" width=\"'+(X(xr)-X(xl))+'\" height=\"'+(Y(yb)-Y(yt))+'\" fill=\"'+zColor(v,m.min,m.max)+'\"/>';"
        "if(fs>=6)h+='<text x=\"'+X(x)+'\" y=\"'+(Y(y)+fs/3)+'\" font-size=\"'+fs+'\" fill=\"#fff\" text-anchor=\"middle\">'+(v*100).toFixed(0)+'</text>'}"
        "(cfg.fixed||[]).forEach(function(p){h+='<rect x=\"'+(X(p[0])-6)+'\" y=\"'+(Y(p[1])-6)+'\" width=\"12\" height=\"12\" fill=\"#555\" stroke=\"#fff\"/>'});"
        "(plan||[]).forEach(function(s){var c=s.n==0?'#2e7d32':s.n<=2?'#ef6c00':'#c62828';"
        "h+='<circle cx=\"'+X(s.x)+'\" cy=\"'+Y(s.y)+'\" r=\"11\" fill=\"'+c+'\" stroke=\"#fff\" stroke-width=\"2\"/>'"
        "+'<text x=\"'+X(s.x)+'\" y=\"'+(Y(s.y)+4)+'\" font-size=\"12\" fill=\"#fff\" text-anchor=\"middle\" font-weight=\"bold\">'+s.i+'</text>';"
        "if(s.n){var ty=Y(s.y)>H-24?Y(s.y)-16:Y(s.y)+25;"
        "h+='<text x=\"'+X(s.x)+'\" y=\"'+ty+'\" font-size=\"12\" text-anchor=\"middle\" font-weight=\"bold\" fill=\"#000\" stroke=\"#fff\" stroke-width=\"3\" paint-order=\"stroke\">'"
        "+(s.up?'\\u2191':'\\u2193')+frac(s.n)+'</text>'}});"
        "h+='</svg><div class=\"hint\">" N_("Back at the top. Numbers are hundredths of a mm; circles are screws, squares fixed supports.") "</div>';"
        "$('bedmap').innerHTML=h}"

        /* The table of screws, corners first */
        "function screwTable(m,plan){var el=$('screws-xy');if(!plan){el.innerHTML=m.x0!==undefined?'<p class=\"hint\">"
        N_("Enter the screw positions in the bed configuration to get what to turn.") "</p>':'';return}"
        "var cw='" N_("clockwise") "',ccw='" N_("counterclockwise") "';"
        "var rows=plan.slice().sort(function(a,b){return b.corner-a.corner||a.i-b.i});"
        "var h='<h3>" N_("Screws") "</h3><table class=\"screw-tbl\"><tr><th>#</th><th>" N_("Where") "</th><th>" N_("Off by") "</th><th>"
        N_("Turn") "</th></tr>';"
        "rows.forEach(function(s){h+='<tr><td>'+s.i+'</td><td>'+where(s.x,s.y)+'</td><td>'+(s.diff>=0?'+':'')+s.diff.toFixed(2)+' " N_("mm") "</td><td class=\"turn\">';"
        "h+=s.n?(s.up?'\\u2191 ':'\\u2193 ')+frac(s.n)+' '+(s.cw?cw:ccw):'" N_("leave it") "';h+='</td></tr>'});"
        "h+='</table><p class=\"hint\">" N_("Corners first, then the screws between them. Probe again after turning: "
        "the screws pull on each other.") "</p>';"
        "el.innerHTML=h}"

        /* Render mesh grid + stats */
        "function renderMesh(m){"
        "$('bedmap').innerHTML='';$('screws-xy').innerHTML='';$('verdict').innerHTML='';"
        "if(!m||!m.gx){$('mesh-box').innerHTML='<p class=\"hint\">" N_("No mesh data") "</p>';$('mesh-stats').innerHTML='';return}"
        "var g=m.gx,r=m.gy,d=m.d;"

        /* Stats */
        "var range=m.max-m.min;"
        "var maxDev=Math.max(Math.abs(m.min),Math.abs(m.max));"
        "var sum=0,sum2=0,cnt=0;"
        "for(var y=0;y<r;y++)for(var x=0;x<g;x++){sum+=d[y][x];cnt++}"
        "var avg=sum/cnt;sum2=0;"
        "for(var y=0;y<r;y++)for(var x=0;x<g;x++){var v=d[y][x]-avg;sum2+=v*v}"
        "var sd=Math.sqrt(sum2/cnt);"
        "var mech=fixedTilt(m)>MECH_TILT,plan=m.x0!==undefined&&!mech?screwPlan(m):null;verdict(m,plan);"
        "$('mesh-stats').innerHTML="
        "'<div class=\"stats\">"
        "<span>" N_("Range:") " '+range.toFixed(3)+' " N_("mm") "</span>"
        "<span>" N_("Max dev:") " '+maxDev.toFixed(3)+' " N_("mm") "</span>"
        "<span>" N_("Std dev:") " '+sd.toFixed(3)+' " N_("mm") "</span>"
        "<span>'+g+'&times;'+r+(m.pr?' &middot; '+(m.pr=='default'?'" N_("not saved") "':'" N_("profile") " '+m.pr):'')+'</span>"
        "</div>';"

        /* A Klipper mesh knows where its points are: draw the bed */
        "if(m.x0!==undefined){$('mesh-box').innerHTML='';$('quad').style.display='none';"
        "bedMap(m,plan);if(mech)$('screws-xy').innerHTML='<p>" N_("The bed is tilted along the fixed supports by") " '"
        "+fixedTilt(m).toFixed(1)+' " N_("mm. No screw changes that, so there is nothing to turn yet: check what carries the bed "
        "(wheels and their eccentric nuts, the carriage) and probe again.") "</p>';else screwTable(m,plan);return}"
        "$('quad').style.display='';"

        /* Grid */
        /* Render grid oriented so back=top, front=bottom, left=left, right=right.
           Origin config tells us where mesh[0][0] is physically. */
        "var o=cfg.origin||0;"
        "var flipY=(o<2);var flipX=(o===1||o===3);"
        "var h='<div class=\"mesh-grid\" style=\"grid-template-columns:repeat('+g+',48px)\">';"
        "for(var yi=0;yi<r;yi++){var y=flipY?r-1-yi:yi;"
        "for(var xi=0;xi<g;xi++){var x=flipX?g-1-xi:xi;"
        "h+='<div class=\"mesh-cell\" style=\"background:'+zColor(d[y][x],m.min,m.max)+'\">'+d[y][x].toFixed(3)+'</div>'}}"
        "h+='</div>';"
        "$('mesh-box').innerHTML=h}"

        /* Probe */
        "var pollId=0;"
        "function busy(b){$('probe-btn').disabled=b;$('save-btn').disabled=b}"
        "function probe(save){"
        "if(cfg.klipper&&!confirm(save?'" N_("Probe the bed and save the mesh for printing to profile") " '+cfg.use_mesh+'? "
        N_("Klipper restarts afterwards.") "':'" N_("Heat the bed, home and probe it?") "'))return;"
        "busy(true);"
        "$('probe-status').textContent='" N_("Starting...") "';"
        "fetch('/bedlevel/probe',{method:'POST',body:save?'save=1':''}).then(r=>r.json()).then(function(j){"
        "if(!j.ok){$('probe-status').textContent=j.err;busy(false);return}"
        "pollId=setInterval(pollStatus,3000)})}"

        "function pollStatus(){"
        "fetch('/bedlevel/status').then(r=>r.json()).then(function(j){"
        "var st=j.state;"
        "$('probe-status').textContent=st==='homing'?'" N_("Homing...") "':st==='probing'?'" N_("Probing...") "':st;"
        "if(st==='done'||st==='error'){"
        "clearInterval(pollId);"
        "busy(false);"
        "if(st==='done'&&j.mesh){curMesh=j.mesh;renderMesh(curMesh);loadMeshList()}"
        "if(st==='done')$('probe-status').textContent=j.saved?'" N_("Saved to profile") " '+j.saved+'. "
        N_("Klipper restarts.") "':'" N_("Done") "';"
        "if(st==='error')$('probe-status').textContent='" N_("Error:") " '+(j.err||'" N_("Unknown") "')}})}"

        /* History */
        "function loadMeshList(){"
        "fetch('/bedlevel/meshlist').then(r=>r.json()).then(function(l){"
        "meshList=l;meshIdx=0;updateHistLabel()})}"

        "function updateHistLabel(){"
        "var lbl=$('hist-label');"
        "if(meshList.length===0){lbl.textContent='-';return}"
        "var ts=meshList[meshIdx];"
        "var d=new Date(ts*1000);"
        "lbl.textContent=(meshIdx+1)+'/'+meshList.length+' — '+d.toLocaleString()}"

        "function histPrev(){if(meshIdx<meshList.length-1){meshIdx++;loadMeshAt(meshIdx)}}"
        "function histNext(){if(meshIdx>0){meshIdx--;loadMeshAt(meshIdx)}}"

        "function loadMeshAt(i){"
        "fetch('/bedlevel/mesh?ts='+meshList[i]).then(r=>r.json()).then(function(m){"
        "curMesh=m;renderMesh(m);updateHistLabel()})}"

        /* Screw adjustment for a mesh without coordinates (Marlin): quadrants */
        "function calcScrews(mode){"
        "if(!curMesh||!curMesh.gx){$('screw-result').innerHTML='<p>" N_("No mesh data") "</p>';return}"
        "var m=curMesh,gx=m.gx,gy=m.gy,d=m.d;"
        "var mx=Math.floor(gx/2),my=Math.floor(gy/2);"
        "var o=cfg.origin||0;var sFlipY=(o>=2);var sFlipX=(o===1||o===3);"
        "var avg=[0,0,0,0],cnt=[0,0,0,0];"
        "for(var y=0;y<gy;y++)for(var x=0;x<gx;x++){"
        "var py=sFlipY?gy-1-y:y;var px=sFlipX?gx-1-x:x;"
        "var qi=(py>=my?2:0)+(px>=mx?1:0);"
        "avg[qi]+=d[y][x];cnt[qi]++}"
        "for(var i=0;i<4;i++)if(cnt[i])avg[i]/=cnt[i];"
        "var labels=['" N_("Front-Left") "','" N_("Front-Right") "','" N_("Back-Left") "','" N_("Back-Right") "'];"
        "var ref;"
        "if(mode==='tighten'){"
        "ref=Math.min.apply(null,avg);"
        "}else{"
        "ref=Math.max.apply(null,avg)}"
        "var h='<table class=\"screw-tbl\"><tr><th>" N_("Screw") "</th><th>" N_("Avg Z") "</th><th>" N_("Correction") "</th></tr>';"
        "var pitch=cfg.screw_pitch||1;"
        "for(var i=0;i<4;i++){"
        "var delta=mode==='tighten'?(avg[i]-ref):(ref-avg[i]);"
        "var turns=Math.round(delta/pitch*8)/8;"
        "var dir=mode==='tighten'?'" N_("CW") "':'" N_("CCW") "';"
        "h+='<tr><td>'+labels[i]+'</td><td>'+avg[i].toFixed(3)+'</td><td>';"
        "if(Math.abs(turns)<0.06)h+='" N_("OK (reference)") "';"
        "else{var q=Math.floor(turns*4)/4;var e=turns-q;"
        "var s='';if(q>=0.25)s+=Math.round(q*4)+'/4';"
        "if(e>=0.1){if(s)s+=' + ';s+='1/8'}"
        "if(!s)s='1/8';"
        "h+=s+' '+(turns>0.25?'" N_("turns") "':'" N_("turn") "')+' '+dir}"
        "h+='</td></tr>'}"
        "h+='</table>';"
        "$('screw-result').innerHTML=h}"

        /* Config */
        "function pts(a){return (a||[]).map(function(p){return p[0]+','+p[1]}).join('; ')}"
        "function parsePts(s){return s.split(';').map(function(t){return t.split(',').map(Number)})"
        ".filter(function(p){return p.length==2&&isFinite(p[0])&&isFinite(p[1])&&(p[0]||p[1])})}"
        "function loadCfg(){"
        "return fetch('/bedlevel/config').then(r=>r.json()).then(function(c){"
        "cfg=c;"
        "$('cfg-bw').value=c.bed_w;"
        "$('cfg-bh').value=c.bed_h;"
        "$('cfg-ss').value=c.screw_spacing;"
        "$('cfg-sp').value=c.screw_pitch;"
        "$('cfg-cw').value=c.cw_raises?1:0;"
        "$('cfg-td').value=c.turn_div||8;"
        "$('cfg-or').value=c.origin||0;"
        "$('cfg-bt').value=c.bed_c;"
        "$('cfg-scr').value=pts(c.screws);"
        "$('cfg-fix').value=pts(c.fixed);"
        "$('cfg-k').style.display=c.klipper?'':'none';"
        "if(c.klipper){var sel=$('cfg-pm'),names=(c.profiles||[]).slice();"
        "if(c.print_mesh&&names.indexOf(c.print_mesh)<0)names.push(c.print_mesh);"
        "sel.innerHTML='<option value=\"\">" N_("the one the printer loads") "'+(c.boot_mesh?' ('+c.boot_mesh+')':'')+'</option>'"
        "+names.map(function(n){return '<option value=\"'+n+'\">'+n+'</option>'}).join('');"
        "sel.value=c.print_mesh||'';"
        "$('cfg-pm-hint').textContent=c.boot_mesh?'" N_("The printer loads") " '+c.boot_mesh+' " N_("when it starts") "':'" N_("Not seen yet which profile the printer loads when it starts") "';"
        "var n=parseInt(c.use_mesh,10);"
        "$('save-btn').textContent='" N_("Save mesh for printing") "'+' ('+c.use_mesh+(n>=3?', '+n+'\u00d7'+n:'')+')';"
        "$('save-btn').style.display=''}"
        "$('probe-btn').textContent=c.klipper?'" N_("Probe Bed") "':'" N_("Probe Bed") " (G28 + G29)'})}"

        "function saveCfg(){"
        "var c={bed_w:+$('cfg-bw').value,"
        "bed_h:+$('cfg-bh').value,"
        "screw_spacing:+$('cfg-ss').value,"
        "screw_pitch:+$('cfg-sp').value,"
        "cw_raises:$('cfg-cw').value=='1',"
        "turn_div:+$('cfg-td').value,"
        "origin:+$('cfg-or').value,"
        "bed_c:+$('cfg-bt').value,"
        "screws:parsePts($('cfg-scr').value),"
        "print_mesh:$('cfg-pm').value,"
        "fixed:parsePts($('cfg-fix').value)};"
        "fetch('/bedlevel/config',{method:'POST',body:JSON.stringify(c)}).then(r=>r.json()).then(function(j){"
        "if(j.ok){loadCfg().then(function(){if(curMesh)renderMesh(curMesh)});$('cfg-status').textContent='" N_("Saved!") "';"
        "setTimeout(function(){$('cfg-status').textContent=''},2000)}})}"

        /* Init: config, then the status (which reads Klipper's mesh), then the history */
        "loadCfg().then(function(){return fetch('/bedlevel/status')}).then(r=>r.json()).then(function(j){"
        "if(j.state==='homing'||j.state==='probing'){"
        "$('probe-status').textContent=j.state==='homing'?'" N_("Homing...") "':'" N_("Probing...") "';"
        "busy(true);pollId=setInterval(pollStatus,3000)}"
        "else if(j.state==='done'&&j.mesh){curMesh=j.mesh;renderMesh(curMesh)}"
        "return fetch('/bedlevel/meshlist')}).then(r=>r.json()).then(function(l){"
        "meshList=l;meshIdx=0;updateHistLabel();"
        "if(l.length>0&&!curMesh)loadMeshAt(0);"
        "else if(!curMesh)renderMesh(null)});"
        "</script>";

static esp_err_t bedlevel_page_handler(httpd_req_t *req)
{
    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, NT_("Bed Leveling"), "/bedlevel");
    html_buf_printf(&p, "%s", i18n_page(BL_PAGE));
    /* The checks before a print (print_checks.c): a stale mesh profile, a Z
     * offset the calibration changed; again after each probe */
    html_buf_printf(&p, "%s",
        "<script>(function(){var e=document.createElement('p');e.hidden=true;"
        "e.style.cssText='background:#ffebee;color:#c62828;padding:10px 12px;border-radius:6px;font-weight:bold;white-space:pre-line';"
        "var h=document.querySelector('h2');if(h)h.parentNode.insertBefore(e,h.nextSibling);else document.body.appendChild(e);"
        "function ck(){fetch('/print/checks').then(function(r){return r.json()}).then(function(d){"
        "e.textContent='\u26a0 '+d.warn;e.hidden=!d.warn}).catch(function(){})}"
        "ck();setInterval(ck,30000)})()</script>");
    layout_html_end(&p);

    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

/* ---- Registration ---- */

esp_err_t bedlevel_register_httpd(httpd_handle_t server)
{
    load_config();
    s_check_lock = xSemaphoreCreateMutex();

    /* Always listen for mesh output from the printer */
    printer_comm_set_line_callback(mesh_parse_line);

    /* Load latest mesh into s_current_mesh */
    mesh_entry_t *list = malloc(BL_CACHE_MAX * sizeof(mesh_entry_t));
    if (list) {
        int n = get_mesh_list(list, BL_CACHE_MAX);
        if (n > 0) {
            load_mesh_file(list[0].path, &s_current_mesh);
            ESP_LOGI(TAG, "Loaded latest mesh: %dx%d", s_current_mesh.grid_x, s_current_mesh.grid_y);
        }
        free(list);
    }

    httpd_uri_t uris[] = {
        { .uri = "/bedlevel",        .method = HTTP_GET,  .handler = bedlevel_page_handler },
        { .uri = "/bedlevel/probe",  .method = HTTP_POST, .handler = bedlevel_probe_handler },
        { .uri = "/bedlevel/status", .method = HTTP_GET,  .handler = bedlevel_status_handler },
        { .uri = "/bedlevel/meshlist", .method = HTTP_GET, .handler = bedlevel_meshlist_handler },
        { .uri = "/bedlevel/mesh",   .method = HTTP_GET,  .handler = bedlevel_mesh_handler },
        { .uri = "/bedlevel/config", .method = HTTP_GET,  .handler = bedlevel_config_get_handler },
        { .uri = "/bedlevel/config", .method = HTTP_POST, .handler = bedlevel_config_post_handler },
    };

    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }

    return ESP_OK;
}
