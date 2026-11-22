#include "maintenance.h"
#include "task_util.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include "cJSON.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "httpd.h"
#include "hub_client.h"
#include "i18n.h"
#include "layout.h"
#include "nvs.h"
#include "print_stats.h"
#include "printer_comm.h"
#include "url_util.h"

static const char *TAG = "maint";

/* On LittleFS, not NVS: with descriptions the list is a few KB, and NVS
 * (24 KB, shared with Wi-Fi) needs room for two copies while it rewrites.
 * The cache housekeeping only evicts in its own folders. */
#define MAINT_DIR       "/cache/maint"
#define JOBS_FILE       MAINT_DIR "/jobs.json"
#define JOBS_TMP        MAINT_DIR "/jobs.tmp"
#define LOG_FILE        MAINT_DIR "/log.txt"
#define LOG_MAX         300         /* Lines: trimmed to LOG_KEEP when it passes this */
#define LOG_KEEP        200
#define LOG_SHOWN       20
#define LOG_LINE        768

#define OLD_NVS_NS      "maint"     /* Before 2.5 shipped: the list was an NVS blob */
#define MAX_JOBS        12
#define NAME_LEN        64          /* Bytes */
#define NAME_CHARS      30          /* Letters the editor takes: Cyrillic is 2 bytes each */
#define DESC_LEN        320
#define DESC_CHARS      150
#define NOTE_LEN        320
#define NOTE_CHARS      150
#define HOURS_MAX       10000
#define DAYS_MAX        3650
#define PRINTS_MAX      10000
#define LIST_VERSION    2           /* 2: the glue job of the template was added once */
#define CHECK_MS        60000
#define VALID_TIME      1700000000  /* Before this, SNTP has not set the clock yet */
#define SOON_FRACTION   10          /* Shown as soon in the last 1/10 of the interval */
#define EMPTY_ROWS      2           /* Blank rows in the editor, for new jobs */

typedef struct {
    char name[NAME_LEN];
    char desc[DESC_LEN];    /* What to do: the checklist */
    uint16_t hours;         /* Print hours between, 0 = not by hours */
    uint16_t days;          /* Days between, 0 = not by days */
    uint16_t prints;        /* Completed prints between, 0 = not by prints */
    uint32_t done_prints;   /* Completed prints when last done */
    uint32_t done_print_s;  /* print_s when last done */
    int64_t done_time;      /* Unix time when last done, 0 = not known yet */
    bool notified;          /* Reported as due since it was last done */
} job_t;

/* Three levels, like a car's: the hours are multiples, so the levels fall due
 * together and are done at once.  The Z screw interval is near Anycubic's
 * (3 months) and Prusa's (500 h). */
static const struct { const char *name, *desc; uint16_t hours, days, prints; } TEMPLATE[] = {
    { NT_("Cleaning"),
      NT_("Vacuum the printer, wipe the rails and the Z screw, clean the magnetic base under the sheet"),
      50, 30, 0 },
    { NT_("Service 1"),
      NT_("Cleaning, plus: belt tension, wheels and eccentric nuts, blow out the fans, "
          "check the head wires and connectors"),
      200, 90, 0 },
    { NT_("Service 2"),
      NT_("Service 1, plus: clean and grease the Z lead screw"),
      600, 180, 0 },
    /* Glue is sprayed on the sheet for every print and builds up; by count,
     * not by eye, so that it is done whether or not someone notices */
    { NT_("Bed: wash off the glue"),
      NT_("Take the sheet off. Wash the glue off with warm water and a little dish soap, "
          "then wipe with isopropyl alcohol and let it dry. No acetone: it eats PEI."),
      0, 0, 10 },
};
#define GLUE_JOB 3              /* Added once to lists made before it */
#define TEMPLATE_COUNT (sizeof(TEMPLATE) / sizeof(TEMPLATE[0]))

static job_t *s_jobs;       /* MAX_JOBS, in PSRAM */
static int s_count;
static uint8_t s_lang;      /* Of whoever edited the list: for the reminders */
static SemaphoreHandle_t s_lock;
static int s_log_lines;

static int64_t now_time(void)
{
    time_t t = time(NULL);
    return t > VALID_TIME ? (int64_t)t : 0;
}

static uint32_t print_s_now(void)
{
    print_stats_t st;
    print_stats_get(&st);
    return st.print_s;
}

/** Completed prints: a job due "every N prints" counts these */
static uint32_t prints_now(void)
{
    print_stats_t st;
    print_stats_get(&st);
    return st.completed;
}

/* ---- Text ---- */

/** Typed text kept as typed, except control characters and half of a UTF-8
 *  letter that a length limit cut; trimmed.  HTML escapes it on output. */
static void clean_text(char *s)
{
    char *d = s;
    for (const char *p = s; *p; p++) {
        unsigned char c = (unsigned char)*p;
        *d++ = c < 0x20 ? ' ' : (char)c;
    }
    *d = '\0';

    size_t len = strlen(s);
    size_t start = len;
    while (start > 0 && ((unsigned char)s[start - 1] & 0xC0) == 0x80) start--;
    if (start > 0 && ((unsigned char)s[start - 1] & 0x80)) {
        unsigned char lead = (unsigned char)s[start - 1];
        size_t need = lead >= 0xF0 ? 4 : lead >= 0xE0 ? 3 : 2;
        if (len - (start - 1) < need) s[start - 1] = '\0';
    }

    len = strlen(s);
    while (len && s[len - 1] == ' ') s[--len] = '\0';
    char *b = s;
    while (*b == ' ') b++;
    if (b != s) memmove(s, b, strlen(b) + 1);
}

/** Append s to the page with <>&"' escaped */
static void put_esc(html_buf_t *p, const char *s)
{
    const char *run = s;
    for (; *s; s++) {
        const char *ent = NULL;
        switch (*s) {
        case '<':  ent = "&lt;"; break;
        case '>':  ent = "&gt;"; break;
        case '&':  ent = "&amp;"; break;
        case '"':  ent = "&quot;"; break;
        case '\'': ent = "&#39;"; break;
        default: continue;
        }
        if (s > run) html_buf_printf(p, "%.*s", (int)(s - run), run);
        html_buf_printf(p, "%s", ent);
        run = s + 1;
    }
    if (*run) html_buf_printf(p, "%s", run);
}

/** A template text's msgid if text is it in any language, so that it shows
 *  translated whatever language saved it; else text itself */
static const char *canonical(const char *text, bool desc)
{
    for (size_t t = 0; t < TEMPLATE_COUNT; t++) {
        const char *id = desc ? TEMPLATE[t].desc : TEMPLATE[t].name;
        for (int lang = 0; lang < i18n_lang_count(); lang++) {
            if (strcmp(text, i18n_lookup(id, lang)) == 0) return id;
        }
    }
    return text;
}

/* ---- Storage ---- */

static void save_locked(void)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *list = root ? cJSON_AddArrayToObject(root, "jobs") : NULL;
    if (!list) {
        cJSON_Delete(root);
        return;
    }
    cJSON_AddNumberToObject(root, "lang", s_lang);
    cJSON_AddNumberToObject(root, "v", LIST_VERSION);
    for (int i = 0; i < s_count; i++) {
        const job_t *j = &s_jobs[i];
        cJSON *o = cJSON_CreateObject();
        if (!o) break;
        cJSON_AddStringToObject(o, "name", j->name);
        cJSON_AddStringToObject(o, "desc", j->desc);
        cJSON_AddNumberToObject(o, "hours", j->hours);
        cJSON_AddNumberToObject(o, "days", j->days);
        cJSON_AddNumberToObject(o, "prints", j->prints);
        cJSON_AddNumberToObject(o, "done_p", j->done_prints);
        cJSON_AddNumberToObject(o, "done_s", j->done_print_s);
        cJSON_AddNumberToObject(o, "done_t", (double)j->done_time);
        cJSON_AddBoolToObject(o, "notified", j->notified);
        cJSON_AddItemToArray(list, o);
    }
    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!json) return;

    /* Written aside and renamed, so a reset mid-write keeps the old list */
    FILE *f = fopen(JOBS_TMP, "w");
    bool ok = f && fputs(json, f) >= 0;
    if (f) ok = (fclose(f) == 0) && ok;
    cJSON_free(json);
    if (ok) {
        unlink(JOBS_FILE);
        ok = rename(JOBS_TMP, JOBS_FILE) == 0;
    }
    if (!ok) ESP_LOGE(TAG, "Could not save " JOBS_FILE);
}

static int s_loaded_version;

static bool load(void)
{
    FILE *f = fopen(JOBS_FILE, "r");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buf = size > 0 && size < 32768 ? malloc(size + 1) : NULL;
    size_t got = buf ? fread(buf, 1, size, f) : 0;
    fclose(f);
    if (!buf) return false;
    buf[got] = '\0';
    cJSON *root = cJSON_Parse(buf);
    free(buf);
    const cJSON *list = cJSON_GetObjectItem(root, "jobs");
    if (!cJSON_IsArray(list)) {
        cJSON_Delete(root);
        return false;
    }

    const cJSON *v = cJSON_GetObjectItem(root, "lang");
    s_lang = cJSON_IsNumber(v) ? v->valueint : 0;
    v = cJSON_GetObjectItem(root, "v");
    s_loaded_version = cJSON_IsNumber(v) ? v->valueint : 1;
    s_count = 0;
    const cJSON *o;
    cJSON_ArrayForEach(o, list) {
        if (s_count >= MAX_JOBS) break;
        job_t *j = &s_jobs[s_count];
        memset(j, 0, sizeof(*j));
        if (!cJSON_IsString(v = cJSON_GetObjectItem(o, "name"))) continue;
        strlcpy(j->name, v->valuestring, NAME_LEN);
        if (cJSON_IsString(v = cJSON_GetObjectItem(o, "desc"))) strlcpy(j->desc, v->valuestring, DESC_LEN);
        if (cJSON_IsNumber(v = cJSON_GetObjectItem(o, "hours"))) j->hours = v->valueint;
        if (cJSON_IsNumber(v = cJSON_GetObjectItem(o, "days"))) j->days = v->valueint;
        if (cJSON_IsNumber(v = cJSON_GetObjectItem(o, "prints"))) j->prints = v->valueint;
        if (cJSON_IsNumber(v = cJSON_GetObjectItem(o, "done_p"))) j->done_prints = (uint32_t)v->valuedouble;
        else j->done_prints = prints_now();     /* A list from before prints counted */
        if (cJSON_IsNumber(v = cJSON_GetObjectItem(o, "done_s"))) j->done_print_s = (uint32_t)v->valuedouble;
        if (cJSON_IsNumber(v = cJSON_GetObjectItem(o, "done_t"))) j->done_time = (int64_t)v->valuedouble;
        j->notified = cJSON_IsTrue(cJSON_GetObjectItem(o, "notified"));
        s_count++;
    }
    cJSON_Delete(root);
    return true;
}

/* ---- Log ---- */

static int count_log_lines(void)
{
    FILE *f = fopen(LOG_FILE, "r");
    if (!f) return 0;
    int n = 0, c;
    while ((c = fgetc(f)) != EOF) {
        if (c == '\n') n++;
    }
    fclose(f);
    return n;
}

/** Keep the last LOG_KEEP lines */
static void trim_log(void)
{
    FILE *in = fopen(LOG_FILE, "r");
    FILE *out = in ? fopen(LOG_FILE ".tmp", "w") : NULL;
    char *line = malloc(LOG_LINE);
    if (!in || !out || !line) {
        if (in) fclose(in);
        if (out) fclose(out);
        free(line);
        return;
    }
    int skip = s_log_lines - LOG_KEEP, n = 0;
    while (fgets(line, LOG_LINE, in)) {
        if (n++ >= skip) fputs(line, out);
    }
    fclose(in);
    fclose(out);
    free(line);
    unlink(LOG_FILE);
    rename(LOG_FILE ".tmp", LOG_FILE);
    s_log_lines = n - skip;
}

/** Add an entry: job is a job's name (msgid for a template job) or "" */
static void log_entry(const char *job, const char *note)
{
    cJSON *o = cJSON_CreateObject();
    if (!o) return;
    cJSON_AddNumberToObject(o, "t", (double)now_time());
    cJSON_AddNumberToObject(o, "h", print_s_now());
    cJSON_AddStringToObject(o, "j", job);
    cJSON_AddStringToObject(o, "n", note);
    char *json = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    if (!json) return;
    FILE *f = fopen(LOG_FILE, "a");
    if (f) {
        fprintf(f, "%s\n", json);
        fclose(f);
        if (++s_log_lines > LOG_MAX) trim_log();
    }
    cJSON_free(json);
    ESP_LOGI(TAG, "Logged: %s %s", job[0] ? job : "(extra)", note);
}

/* ---- Due ---- */

typedef struct {
    uint32_t hours_since;
    uint32_t days_since;    /* Only valid if days_known */
    uint32_t prints_since;
    bool days_known;
    bool due, soon;
} job_state_t;

static job_state_t job_state(const job_t *j, uint32_t print_s, int64_t now)
{
    job_state_t s = { 0 };
    uint32_t prints = prints_now();
    s.prints_since = prints > j->done_prints ? prints - j->done_prints : 0;
    if (j->prints) {
        if (s.prints_since >= j->prints) s.due = true;
        else if ((j->prints - s.prints_since) * SOON_FRACTION <= j->prints) s.soon = true;
    }
    s.hours_since = print_s > j->done_print_s ? (print_s - j->done_print_s) / 3600 : 0;
    s.days_known = now && j->done_time;
    if (s.days_known) s.days_since = now > j->done_time ? (now - j->done_time) / 86400 : 0;

    if (j->hours) {
        if (s.hours_since >= j->hours) s.due = true;
        else if ((j->hours - s.hours_since) * SOON_FRACTION <= j->hours) s.soon = true;
    }
    if (j->days && s.days_known) {
        if (s.days_since >= j->days) s.due = true;
        else if ((j->days - s.days_since) * SOON_FRACTION <= j->days) s.soon = true;
    }
    if (s.due) s.soon = false;
    return s;
}

int maintenance_due_count(void)
{
    if (!s_lock) return 0;
    uint32_t print_s = print_s_now();
    int64_t now = now_time();
    int n = 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_count; i++) {
        if (job_state(&s_jobs[i], print_s, now).due) n++;
    }
    xSemaphoreGive(s_lock);
    return n;
}

void maintenance_shift_print_s(int64_t delta_s)
{
    if (!s_lock || delta_s == 0) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_count; i++) {
        int64_t v = (int64_t)s_jobs[i].done_print_s + delta_s;
        s_jobs[i].done_print_s = v < 0 ? 0 : (uint32_t)v;
    }
    save_locked();
    xSemaphoreGive(s_lock);
}

/** Mark job i done now, with an entry in the log.  Lock held. */
static void mark_done_locked(int i, const char *note, uint32_t print_s, int64_t now)
{
    job_t *j = &s_jobs[i];
    j->done_print_s = print_s;
    j->done_time = now;
    j->done_prints = prints_now();
    j->notified = false;
    log_entry(j->name, note);
}

/* ---- Overdue ---- */

/** The overdue jobs as "Name (+3 prints, +12 h); ..." in language lang.
 *  Returns how many.  Lock held. */
static int overdue_text_locked(char *buf, size_t size, int lang)
{
    uint32_t print_s = print_s_now();
    int64_t now = now_time();
    size_t n = 0;
    int count = 0;
    buf[0] = '\0';
    for (int i = 0; i < s_count; i++) {
        const job_t *j = &s_jobs[i];
        job_state_t st = job_state(j, print_s, now);
        if (!st.due) continue;
        count++;
        n += snprintf(buf + n, n < size ? size - n : 0, "%s%s (", n ? "; " : "", i18n_lookup(j->name, lang));
        const char *sep = "";
        if (j->prints && st.prints_since >= j->prints && n < size) {
            n += snprintf(buf + n, size - n, "+%lu %s", (unsigned long)(st.prints_since - j->prints),
                          i18n_lookup("prints", lang));
            sep = ", ";
        }
        if (j->hours && st.hours_since >= j->hours && n < size) {
            n += snprintf(buf + n, size - n, "%s+%lu %s", sep, (unsigned long)(st.hours_since - j->hours),
                          i18n_lookup("h", lang));
            sep = ", ";
        }
        if (j->days && st.days_known && st.days_since >= j->days && n < size) {
            n += snprintf(buf + n, size - n, "%s+%lu %s", sep, (unsigned long)(st.days_since - j->days),
                          i18n_lookup("d", lang));
        }
        if (n < size) n += snprintf(buf + n, size - n, ")");
    }
    if (n >= size && size > 4) strcpy(buf + size - 4, "...");
    return count;
}

int maintenance_overdue_text(char *buf, size_t size, int lang)
{
    if (!s_lock) { if (size) buf[0] = '\0'; return 0; }
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = overdue_text_locked(buf, size, lang);
    xSemaphoreGive(s_lock);
    return n;
}

void maintenance_note_print_start(const char *filename)
{
    if (!s_lock) return;
    char text[160];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = overdue_text_locked(text, sizeof(text), s_lang);
    if (n) {
        /* In the log for whoever looks at the printer's page, as a warning line */
        cJSON *o = cJSON_CreateObject();
        if (o) {
            cJSON_AddNumberToObject(o, "t", (double)now_time());
            cJSON_AddNumberToObject(o, "h", print_s_now());
            cJSON_AddStringToObject(o, "j", "");
            cJSON_AddStringToObject(o, "n", text);
            cJSON_AddStringToObject(o, "f", filename);
            cJSON_AddBoolToObject(o, "w", true);
            char *json = cJSON_PrintUnformatted(o);
            cJSON_Delete(o);
            FILE *f = json ? fopen(LOG_FILE, "a") : NULL;
            if (f) {
                fprintf(f, "%s\n", json);
                fclose(f);
                if (++s_log_lines > LOG_MAX) trim_log();
            }
            cJSON_free(json);
        }
    }
    xSemaphoreGive(s_lock);
    if (n) {
        ESP_LOGW(TAG, "Print %s started with maintenance overdue: %s", filename, text);
        hub_client_send_event(HUB_EVENT_MAINTENANCE_SKIPPED, text, filename);
    }
}

/* ---- Reminders ---- */

static void maint_task(void *arg)
{
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(CHECK_MS));

        /* Not during a print: the user sees it when the printer is free */
        printer_state_t st;
        printer_comm_get_state(&st);
        if (st.opstate == PRINTER_PRINTING || st.opstate == PRINTER_PAUSED) continue;

        uint32_t print_s = print_s_now();
        int64_t now = now_time();
        char msg[192] = "";
        size_t len = 0;
        bool changed = false;

        xSemaphoreTake(s_lock, portMAX_DELAY);
        for (int i = 0; i < s_count; i++) {
            job_t *j = &s_jobs[i];
            /* Done before the clock was set: start the days from now */
            if (!j->done_time && now) {
                j->done_time = now;
                changed = true;
            }
            if (j->notified || !job_state(j, print_s, now).due) continue;
            j->notified = true;
            changed = true;
            const char *name = i18n_lookup(j->name, s_lang);
            int n = snprintf(msg + len, sizeof(msg) - len, "%s%s", len ? "; " : "", name);
            if (n > 0 && (size_t)n < sizeof(msg) - len) len += n;
        }
        if (changed) save_locked();
        xSemaphoreGive(s_lock);

        if (len) {
            ESP_LOGI(TAG, "Due: %s", msg);
            hub_client_send_event(HUB_EVENT_MAINTENANCE, msg, "");
        }
    }
}

static void create_template(void)
{
    uint32_t print_s = print_s_now();
    int64_t now = now_time();
    s_count = TEMPLATE_COUNT;
    for (int i = 0; i < s_count; i++) {
        s_jobs[i] = (job_t){ .hours = TEMPLATE[i].hours, .days = TEMPLATE[i].days,
                             .prints = TEMPLATE[i].prints, .done_prints = prints_now(),
                             .done_print_s = print_s, .done_time = now };
        strlcpy(s_jobs[i].name, TEMPLATE[i].name, NAME_LEN);
        strlcpy(s_jobs[i].desc, TEMPLATE[i].desc, DESC_LEN);
    }
    save_locked();
    ESP_LOGI(TAG, "Created the template: cleaning, service 1, service 2, glue");
}

void maintenance_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    s_jobs = heap_caps_calloc(MAX_JOBS, sizeof(job_t), MALLOC_CAP_SPIRAM);
    if (!s_jobs) s_jobs = calloc(MAX_JOBS, sizeof(job_t));
    if (!s_jobs) {
        ESP_LOGE(TAG, "No memory");
        return;
    }
    mkdir(MAINT_DIR, 0755);

    /* The first list lived in NVS, before the levels: drop it */
    nvs_handle_t nvs;
    if (nvs_open(OLD_NVS_NS, NVS_READWRITE, &nvs) == ESP_OK) {
        if (nvs_erase_all(nvs) == ESP_OK) nvs_commit(nvs);
        nvs_close(nvs);
    }

    if (!load()) {
        create_template();
    } else if (s_loaded_version < LIST_VERSION) {
        /* A list from before the glue job: add it once, unless it is there */
        bool have = false;
        for (int i = 0; i < s_count; i++) have |= strcmp(s_jobs[i].name, TEMPLATE[GLUE_JOB].name) == 0;
        if (!have && s_count < MAX_JOBS) {
            s_jobs[s_count++] = (job_t){ .prints = TEMPLATE[GLUE_JOB].prints, .done_prints = prints_now(),
                                         .done_print_s = print_s_now(), .done_time = now_time() };
            strlcpy(s_jobs[s_count - 1].name, TEMPLATE[GLUE_JOB].name, NAME_LEN);
            strlcpy(s_jobs[s_count - 1].desc, TEMPLATE[GLUE_JOB].desc, DESC_LEN);
            ESP_LOGI(TAG, "Added the glue job");
        }
        save_locked();
    }
    s_log_lines = count_log_lines();
    ESP_LOGI(TAG, "%d jobs, %d due, %d log entries", s_count, maintenance_due_count(), s_log_lines);

    if (task_create_psram(maint_task, "maint", 8192, NULL, 2, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start");
    }
}

/* ---- Page ---- */

static void fmt_hours_days(char *out, size_t size, unsigned hours, bool by_hours,
                           unsigned days, bool by_days, bool days_known,
                           unsigned prints, bool by_prints)
{
    size_t n = 0;
    out[0] = '\0';
    if (by_hours) n += snprintf(out + n, size - n, "%u %s", hours, _("h"));
    if (by_days && n < size) {
        if (days_known) n += snprintf(out + n, size - n, "%s%u %s", n ? " · " : "", days, _("d"));
        else n += snprintf(out + n, size - n, "%s? %s", n ? " · " : "", _("d"));
    }
    if (by_prints && n < size) snprintf(out + n, size - n, "%s%u %s", n ? " · " : "", prints, _("prints"));
}

/** The last LOG_SHOWN entries, newest first */
static void render_log(html_buf_t *p)
{
    FILE *f = fopen(LOG_FILE, "r");
    char *lines = f ? heap_caps_malloc(LOG_SHOWN * LOG_LINE, MALLOC_CAP_SPIRAM) : NULL;
    if (!lines) {
        if (f) fclose(f);
        html_buf_printf(p, "<p class='hint'>%s</p>", _("Nothing yet."));
        return;
    }
    int n = 0;
    while (fgets(lines + (n % LOG_SHOWN) * LOG_LINE, LOG_LINE, f)) n++;
    fclose(f);
    if (!n) html_buf_printf(p, "<p class='hint'>%s</p>", _("Nothing yet."));

    int shown = n < LOG_SHOWN ? n : LOG_SHOWN;
    for (int k = 0; k < shown; k++) {
        cJSON *o = cJSON_Parse(lines + ((n - 1 - k) % LOG_SHOWN) * LOG_LINE);
        if (!o) continue;
        const cJSON *t = cJSON_GetObjectItem(o, "t");
        const cJSON *h = cJSON_GetObjectItem(o, "h");
        const cJSON *j = cJSON_GetObjectItem(o, "j");
        const cJSON *note = cJSON_GetObjectItem(o, "n");

        char date[16] = "?";
        if (cJSON_IsNumber(t) && t->valuedouble > VALID_TIME) {
            time_t tt = (time_t)t->valuedouble;
            struct tm tm;
            localtime_r(&tt, &tm);
            strftime(date, sizeof(date), "%d.%m.%Y", &tm);
        }
        html_buf_printf(p, "<div style='padding:6px 0;border-bottom:1px solid #eee'>"
                           "<span class='hint'>%s &middot; %.0f %s</span> ",
                        date, cJSON_IsNumber(h) ? h->valuedouble / 3600.0 : 0.0, _("h"));
        const cJSON *w = cJSON_GetObjectItem(o, "w");
        const cJSON *file = cJSON_GetObjectItem(o, "f");
        if (cJSON_IsTrue(w)) {
            html_buf_printf(p, "<b style='color:#c62828'>&#9888; %s</b>", _("Print started with maintenance overdue"));
            if (cJSON_IsString(file) && file->valuestring[0]) {
                html_buf_printf(p, " &middot; ");
                put_esc(p, file->valuestring);
            }
        } else if (cJSON_IsString(j) && j->valuestring[0]) {
            html_buf_printf(p, "<b>");
            put_esc(p, TR(j->valuestring));
            html_buf_printf(p, "</b>");
        } else {
            html_buf_printf(p, "<b>%s</b>", _("Extra work"));
        }
        if (cJSON_IsString(note) && note->valuestring[0]) {
            html_buf_printf(p, " &mdash; ");
            put_esc(p, note->valuestring);
        }
        html_buf_printf(p, "</div>");
        cJSON_Delete(o);
    }
    free(lines);
}

static const char NOTE_INPUT[] =
    "<input name='note' maxlength='%d' placeholder='%s' "
    "style='flex:1;min-width:0;padding:6px'>";

static esp_err_t page_handler(httpd_req_t *req)
{
    print_stats_t ps;
    print_stats_get(&ps);
    int64_t now = now_time();

    html_buf_t p;
    html_buf_init(&p);
    layout_html_begin(&p, NT_("Maintenance"), "/utils");
    html_buf_printf(&p, "<h2>%s</h2><p class='hint'>", _("Maintenance"));
    html_buf_printf(&p, _("Printed %.1f h in %lu prints, %lu completed."), ps.print_s / 3600.0,
                    (unsigned long)ps.started, (unsigned long)ps.completed);
    html_buf_printf(&p, " %s</p>",
        _("A job is due after its print hours, its days or its completed prints, whichever comes first. "
          "When one falls due, the hub sends a notification once the printer is free. "
          "A print started while a job is overdue is written to the log and reported."));

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int due = 0;
    for (int i = 0; i < s_count; i++) {
        if (job_state(&s_jobs[i], ps.print_s, now).due) due++;
    }
    if (due > 1) {
        html_buf_printf(&p,
            "<form method='POST' action='/maintenance/done' style='display:flex;gap:6px;margin:8px 0' "
            "onsubmit=\"return confirm(this.dataset.q)\" data-q='%s'>"
            "<input type='hidden' name='i' value='all'>",
            _("Mark every job that is due as done?"));
        html_buf_printf(&p, NOTE_INPUT, NOTE_CHARS, _("Comment"));
        html_buf_printf(&p, "<button class='btn-green'>%s (%d)</button></form>", _("All due done"), due);
    }

    for (int i = 0; i < s_count; i++) {
        const job_t *j = &s_jobs[i];
        job_state_t s = job_state(j, ps.print_s, now);
        char since[64], every[64];
        fmt_hours_days(since, sizeof(since), s.hours_since, j->hours, s.days_since, j->days, s.days_known,
                       s.prints_since, j->prints);
        fmt_hours_days(every, sizeof(every), j->hours, j->hours, j->days, j->days, true, j->prints, j->prints);

        html_buf_printf(&p,
            "<div style='border:1px solid #ddd;border-radius:6px;padding:8px 10px;margin:8px 0%s'>"
            "<div class='sh'><b>%s",
            s.due ? ";background:#ffebee" : s.soon ? ";background:#fff3e0" : "",
            s.due ? "&#128295; " : "");
        put_esc(&p, TR(j->name));
        html_buf_printf(&p, "</b><span class='hint' style='text-align:right'>%s %s<br>%s %s</span></div>",
                        _("since:"), since, _("every:"), every);
        if (j->desc[0]) {
            html_buf_printf(&p, "<div class='hint' style='margin:4px 0'>");
            put_esc(&p, TR(j->desc));
            html_buf_printf(&p, "</div>");
        }
        html_buf_printf(&p,
            "<form method='POST' action='/maintenance/done' style='display:flex;gap:6px;margin-top:6px' "
            "onsubmit=\"return confirm(this.dataset.q)\" data-q='%s'>"
            "<input type='hidden' name='i' value='%d'>",
            _("Mark as done?"), i);
        html_buf_printf(&p, NOTE_INPUT, NOTE_CHARS, _("Comment"));
        html_buf_printf(&p, "<button>%s</button></form></div>", _("Done"));
    }
    if (!s_count) html_buf_printf(&p, "<p class='hint'>%s</p>", _("No jobs yet."));

    /* Work done outside the list: a part replaced, a belt tightened */
    html_buf_printf(&p,
        "<h3>%s</h3><form method='POST' action='/maintenance/log' style='display:flex;gap:6px'>",
        _("Record other work"));
    html_buf_printf(&p, NOTE_INPUT, NOTE_CHARS, _("What was done, e.g. replaced a Y wheel"));
    html_buf_printf(&p, "<button>%s</button></form>", _("Record"));

    html_buf_printf(&p, "<h3>%s</h3>", _("Log"));
    render_log(&p);

    /* Editor: one row per job, then blank rows for new ones */
    html_buf_printf(&p,
        "<details style='margin-top:16px'><summary>%s</summary>"
        "<form method='POST' action='/maintenance/save'>"
        "<p class='hint'>%s</p>",
        _("Edit the list"),
        _("Empty the name to delete a job, 0 to not count hours, days or prints. "
          "Editing a job keeps when it was last done."));
    int rows = s_count + EMPTY_ROWS <= MAX_JOBS ? s_count + EMPTY_ROWS : MAX_JOBS;
    for (int i = 0; i < rows; i++) {
        const job_t *j = i < s_count ? &s_jobs[i] : NULL;
        html_buf_printf(&p,
            "<div style='border-top:1px solid #ddd;padding:8px 0'>"
            "<div style='display:flex;gap:6px;align-items:center'>"
            "<input name='n%d' maxlength='%d' placeholder='%s' style='flex:1;min-width:0;padding:6px' value='",
            i, NAME_CHARS, _("Job"));
        if (j) put_esc(&p, TR(j->name));
        html_buf_printf(&p,
            "'><input name='h%d' type='number' min='0' max='%d' value='%u' style='width:64px'> %s"
            "<input name='d%d' type='number' min='0' max='%d' value='%u' style='width:56px'> %s"
            "<input name='p%d' type='number' min='0' max='%d' value='%u' style='width:56px'> %s</div>"
            "<textarea name='s%d' maxlength='%d' rows='2' placeholder='%s' "
            "style='width:100%%;box-sizing:border-box;margin-top:4px;padding:6px'>",
            i, HOURS_MAX, j ? j->hours : 0, _("h"),
            i, DAYS_MAX, j ? j->days : 0, _("d"),
            i, PRINTS_MAX, j ? j->prints : 0, _("prints"),
            i, DESC_CHARS, _("What to do"));
        if (j) put_esc(&p, TR(j->desc));
        html_buf_printf(&p, "</textarea></div>");
    }
    xSemaphoreGive(s_lock);
    html_buf_printf(&p,
        "<input type='hidden' name='rows' value='%d'>"
        "<p><button type='submit'>%s</button></p></form></details>",
        rows, _("Save"));

    layout_html_end(&p);
    httpd_resp_set_type(req, "text/html");
    esp_err_t ret = httpd_resp_send(req, p.data, p.len);
    html_buf_free(&p);
    return ret;
}

/** Read the whole form body into a malloc'd string, or NULL */
static char *recv_body(httpd_req_t *req, size_t max)
{
    if (req->content_len <= 0 || req->content_len >= max) return NULL;
    char *buf = malloc(req->content_len + 1);
    if (!buf) return NULL;
    size_t got = 0;
    while (got < req->content_len) {
        int n = httpd_req_recv(req, buf + got, req->content_len - got);
        if (n == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (n <= 0) {
            free(buf);
            return NULL;
        }
        got += n;
    }
    buf[got] = '\0';
    return buf;
}

static esp_err_t back_to_page(httpd_req_t *req)
{
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/maintenance");
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t bad_form(httpd_req_t *req)
{
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, _("Missing or bad field."));
    return ESP_FAIL;
}

static esp_err_t done_handler(httpd_req_t *req)
{
    char *body = recv_body(req, 2048);
    char which[8] = "", note[NOTE_LEN] = "";
    if (!body || !form_str(body, "i", which, sizeof(which))) {
        free(body);
        return bad_form(req);
    }
    form_str(body, "note", note, sizeof(note));
    free(body);
    clean_text(note);

    uint32_t print_s = print_s_now();
    int64_t now = now_time();
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (strcmp(which, "all") == 0) {
        for (int i = 0; i < s_count; i++) {
            if (job_state(&s_jobs[i], print_s, now).due) mark_done_locked(i, note, print_s, now);
        }
    } else {
        int i = atoi(which);
        if (i >= 0 && i < s_count) mark_done_locked(i, note, print_s, now);
    }
    save_locked();
    xSemaphoreGive(s_lock);
    return back_to_page(req);
}

static esp_err_t log_handler(httpd_req_t *req)
{
    char *body = recv_body(req, 2048);
    char note[NOTE_LEN] = "";
    if (body) form_str(body, "note", note, sizeof(note));
    free(body);
    clean_text(note);
    if (!note[0]) return bad_form(req);

    xSemaphoreTake(s_lock, portMAX_DELAY);
    log_entry("", note);
    xSemaphoreGive(s_lock);
    return back_to_page(req);
}

/** Fill job j from a name, a description and intervals, as typed or copied */
static void set_job(job_t *j, char *name, char *desc, int hours, int days, int prints)
{
    clean_text(name);
    clean_text(desc);
    strlcpy(j->name, canonical(name, false), NAME_LEN);
    strlcpy(j->desc, canonical(desc, true), DESC_LEN);
    j->hours = hours < 0 ? 0 : hours > HOURS_MAX ? HOURS_MAX : hours;
    j->days = days < 0 ? 0 : days > DAYS_MAX ? DAYS_MAX : days;
    j->prints = prints < 0 ? 0 : prints > PRINTS_MAX ? PRINTS_MAX : prints;
}

static esp_err_t save_handler(httpd_req_t *req)
{
    /* 14 rows of a name and a description, Cyrillic sent as %D0%BF */
    char *body = recv_body(req, 24576);
    int rows;
    if (!body || !form_int(body, "rows", &rows) || rows < 0 || rows > MAX_JOBS) {
        free(body);
        return bad_form(req);
    }
    job_t *jobs = heap_caps_calloc(MAX_JOBS, sizeof(job_t), MALLOC_CAP_SPIRAM);
    if (!jobs) {
        free(body);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, _("Out of memory."));
        return ESP_FAIL;
    }
    uint32_t print_s = print_s_now();
    int64_t now = now_time();
    int lang = i18n_lang();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int count = 0;
    for (int i = 0; i < rows && count < MAX_JOBS; i++) {
        char key[8], name[NAME_LEN], desc[DESC_LEN] = "";
        int hours = 0, days = 0, prints = 0;
        snprintf(key, sizeof(key), "n%d", i);
        if (!form_str(body, key, name, sizeof(name))) continue;
        clean_text(name);
        if (!name[0]) continue;     /* Deleted, or a blank row */
        snprintf(key, sizeof(key), "s%d", i);
        form_str(body, key, desc, sizeof(desc));
        snprintf(key, sizeof(key), "h%d", i);
        form_int(body, key, &hours);
        snprintf(key, sizeof(key), "d%d", i);
        form_int(body, key, &days);
        snprintf(key, sizeof(key), "p%d", i);
        form_int(body, key, &prints);

        job_t *j = &jobs[count++];
        /* Keeps when it was last done; a new one counts from now */
        *j = i < s_count ? s_jobs[i]
                         : (job_t){ .done_print_s = print_s, .done_time = now, .done_prints = prints_now() };
        set_job(j, name, desc, hours, days, prints);
        if (!job_state(j, print_s, now).due) j->notified = false;
    }
    memcpy(s_jobs, jobs, sizeof(job_t) * MAX_JOBS);
    s_count = count;
    s_lang = lang;
    save_locked();
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "List saved: %d jobs", count);

    free(jobs);
    free(body);
    return back_to_page(req);
}

/* ---- Copy between boards ---- */

void maintenance_settings_export(cJSON *obj)
{
    cJSON *list = cJSON_AddArrayToObject(obj, "jobs");
    if (!list) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < s_count; i++) {
        cJSON *j = cJSON_CreateObject();
        if (!j) break;
        cJSON_AddStringToObject(j, "name", s_jobs[i].name);   /* The msgid, for a template job */
        cJSON_AddStringToObject(j, "desc", s_jobs[i].desc);
        cJSON_AddNumberToObject(j, "hours", s_jobs[i].hours);
        cJSON_AddNumberToObject(j, "days", s_jobs[i].days);
        cJSON_AddNumberToObject(j, "prints", s_jobs[i].prints);
        cJSON_AddItemToArray(list, j);
    }
    xSemaphoreGive(s_lock);
}

void maintenance_settings_import(const cJSON *obj)
{
    const cJSON *list = cJSON_GetObjectItem(obj, "jobs");
    if (!cJSON_IsArray(list)) return;

    job_t *jobs = heap_caps_calloc(MAX_JOBS, sizeof(job_t), MALLOC_CAP_SPIRAM);
    if (!jobs) return;
    uint32_t print_s = print_s_now();
    int64_t now = now_time();

    xSemaphoreTake(s_lock, portMAX_DELAY);
    int count = 0;
    const cJSON *item;
    cJSON_ArrayForEach(item, list) {
        if (count >= MAX_JOBS) break;
        const cJSON *n = cJSON_GetObjectItem(item, "name");
        const cJSON *s = cJSON_GetObjectItem(item, "desc");
        const cJSON *h = cJSON_GetObjectItem(item, "hours");
        const cJSON *d = cJSON_GetObjectItem(item, "days");
        const cJSON *pr = cJSON_GetObjectItem(item, "prints");
        if (!cJSON_IsString(n)) continue;
        char name[NAME_LEN], desc[DESC_LEN] = "";
        strlcpy(name, n->valuestring, sizeof(name));
        clean_text(name);
        if (!name[0]) continue;
        if (cJSON_IsString(s)) strlcpy(desc, s->valuestring, sizeof(desc));

        /* A job this printer already has keeps when it was last done */
        job_t *j = &jobs[count++];
        *j = (job_t){ .done_print_s = print_s, .done_time = now, .done_prints = prints_now() };
        const char *canon = canonical(name, false);
        for (int k = 0; k < s_count; k++) {
            if (strcmp(s_jobs[k].name, canon) == 0) {
                *j = s_jobs[k];
                break;
            }
        }
        set_job(j, name, desc, cJSON_IsNumber(h) ? h->valueint : 0, cJSON_IsNumber(d) ? d->valueint : 0,
                cJSON_IsNumber(pr) ? pr->valueint : 0);
        if (!job_state(j, print_s, now).due) j->notified = false;
    }
    memcpy(s_jobs, jobs, sizeof(job_t) * MAX_JOBS);
    s_count = count;
    save_locked();
    xSemaphoreGive(s_lock);
    free(jobs);
    ESP_LOGI(TAG, "List copied: %d jobs", count);
}

/** GET /maintenance/due: what is overdue, for the question before a print */
static esp_err_t due_handler(httpd_req_t *req)
{
    char text[200];
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int n = overdue_text_locked(text, sizeof(text), i18n_lang());
    xSemaphoreGive(s_lock);
    cJSON *o = cJSON_CreateObject();
    if (o) {
        cJSON_AddNumberToObject(o, "n", n);
        cJSON_AddStringToObject(o, "text", text);
    }
    char *json = o ? cJSON_PrintUnformatted(o) : NULL;
    cJSON_Delete(o);
    if (!json) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no memory");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t ret = httpd_resp_sendstr(req, json);
    cJSON_free(json);
    return ret;
}

esp_err_t maintenance_register_httpd(httpd_handle_t server)
{
    const httpd_uri_t uris[] = {
        { .uri = "/maintenance",      .method = HTTP_GET,  .handler = page_handler },
        { .uri = "/maintenance/done", .method = HTTP_POST, .handler = done_handler },
        { .uri = "/maintenance/log",  .method = HTTP_POST, .handler = log_handler },
        { .uri = "/maintenance/save", .method = HTTP_POST, .handler = save_handler },
        { .uri = "/maintenance/due",  .method = HTTP_GET,  .handler = due_handler },
    };
    for (int i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        HTTPD_REGISTER(server, &uris[i]);
    }
    return ESP_OK;
}
