#pragma once

/*
 * Maintenance: a list of jobs per printer, each due after so many print hours,
 * so many days or so many completed prints, whichever comes first.  "Done" restarts both.  A job that
 * falls due is reported to the hub once, when the printer is not printing.
 *
 * The list starts as a template for the Kobra 2, whose names are English
 * msgids, so they show translated; names the user types show as typed.
 */

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"

/** Load the list (or create the template) and start the reminder task */
void maintenance_init(void);

/** Jobs due now */
int maintenance_due_count(void);

/**
 * A print (not a built-in script) started: if a job is overdue, the log gets a
 * warning line and the hub a maintenance_skipped event.  Staff who ignore the
 * reminders show up there.
 */
/** What is overdue, as the question before a print shows it, in language
 *  lang; returns how many jobs */
int maintenance_overdue_text(char *buf, size_t size, int lang);

void maintenance_note_print_start(const char *filename);

/**
 * The print hours were set by hand: move every job's "done at" by the same
 * amount, so that entering a printer's past does not make every job due.
 */
void maintenance_shift_print_s(int64_t delta_s);

esp_err_t maintenance_register_httpd(httpd_handle_t server);

/* Settings copied between boards: the job list.  Jobs this board already has
 * (by name) keep when they were last done; new ones count from now. */
struct cJSON;
void maintenance_settings_export(struct cJSON *obj);
void maintenance_settings_import(const struct cJSON *obj);
