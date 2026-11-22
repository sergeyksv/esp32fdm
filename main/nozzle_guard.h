#pragma once

/*
 * Nozzle guard: a plastic left hot in a waiting nozzle (ABS above all) turns
 * to char, and the nozzle with it.  During a wait for the user the board keeps
 * the plastic moving or takes the nozzle down to a standby temperature:
 *
 *   - a stop before the first layer (in the start G-code): the head is not
 *     over a part, so for the first purge_for_min minutes it pushes purge_mm
 *     out every purge_every_s seconds and the print can go on at once; after
 *     that a 2 mm retract and the standby temperature
 *   - a stop during the print: no purge (the nozzle is over the part); after
 *     mid_after_min minutes the retract and the standby temperature
 *   - a pause from Obico: the standby temperature at once
 *   - a pause because the filament ran out (the board's own pause in a
 *     Marlin host print; Klipper's PAUSE with its sensor showing no
 *     filament): as a stop during the print, standby after mid_after_min
 *
 * Continue heats back to the temperature it had and waits for it first.
 * Pauses made on the printer itself are left alone: they are resumed from its
 * screen, which would print with a cold nozzle.
 *
 * The decisions are pure (no ESP-IDF), so they are tested on a PC; the
 * settings and the notification live in nozzle_guard.c.
 */

#include <stdbool.h>
#include <stdint.h>

#define NG_MAX_PLASTICS   8
#define NG_HOT_C          230   /* An unknown plastic printed this hot is taken as ABS */
#define NG_MIN_DROP_C     20    /* Standby must be this much below the print temperature */
#define NG_RETRACT_MM     2.0f

typedef struct {
    bool enabled;
    uint16_t purge_every_s;     /* Before the first layer: push plastic out this often */
    uint8_t purge_mm;           /* ...this much */
    uint16_t purge_for_min;     /* ...for this long, then standby */
    uint16_t mid_after_min;     /* During the print: standby after this long */
    int16_t standby_c[NG_MAX_PLASTICS]; /* By plastic (plastics.c order); 0 = not guarded */
} ng_config_t;

/* One wait, as seen by a backend */
typedef struct {
    bool active;
    bool before_layers;         /* The stop is in the start G-code */
    int64_t since_us;
    int64_t last_purge_us;
    bool standby;               /* Taken down to standby */
    bool retracted;             /* ...after a retract that may have happened */
    int restore_c;              /* The temperature to go back to */
} ng_hold_t;

typedef enum { NG_ACT_NONE, NG_ACT_PURGE, NG_ACT_STANDBY } ng_action_t;

/** The standby temperature for a print at target_c, or 0 to leave it alone.
 *  idx is the plastic in the nozzle (-1 unknown), abs_idx the ABS entry.  A
 *  plastic without a standby but printed at NG_HOT_C or more gets ABS's: the
 *  plastic on record may be an old one. */
static inline int ng_standby_for(const ng_config_t *c, int idx, int abs_idx, int target_c)
{
    if (!c->enabled || target_c <= 0) return 0;
    int s = (idx >= 0 && idx < NG_MAX_PLASTICS) ? c->standby_c[idx] : 0;
    if (s <= 0 && target_c >= NG_HOT_C && abs_idx >= 0 && abs_idx < NG_MAX_PLASTICS)
        s = c->standby_c[abs_idx];
    if (s <= 0 || target_c - s < NG_MIN_DROP_C) return 0;
    return s;
}

static inline void ng_hold_begin(ng_hold_t *h, int64_t now_us, bool before_layers)
{
    h->active = true;
    h->before_layers = before_layers;
    h->since_us = now_us;
    h->last_purge_us = now_us;
    h->standby = false;
    h->retracted = false;
    h->restore_c = 0;
}

static inline void ng_hold_end(ng_hold_t *h)
{
    h->active = false;
    h->standby = false;
    h->retracted = false;
    h->restore_c = 0;
}

/** What to do now in a wait.  standby_c is ng_standby_for() for the current
 *  target: 0 means no guard for this plastic, so nothing is done.  The caller
 *  records what it did: last_purge_us after a purge, standby after standby. */
static inline ng_action_t ng_hold_step(const ng_config_t *c, const ng_hold_t *h,
                                       int64_t now_us, int standby_c)
{
    if (!c->enabled || !h->active || h->standby || standby_c <= 0) return NG_ACT_NONE;
    int64_t waited = now_us - h->since_us;
    if (h->before_layers) {
        if (waited >= (int64_t)c->purge_for_min * 60 * 1000000) return NG_ACT_STANDBY;
        if (c->purge_mm > 0 && c->purge_every_s > 0 &&
            now_us - h->last_purge_us >= (int64_t)c->purge_every_s * 1000000)
            return NG_ACT_PURGE;
        return NG_ACT_NONE;
    }
    return waited >= (int64_t)c->mid_after_min * 60 * 1000000 ? NG_ACT_STANDBY : NG_ACT_NONE;
}

/* ---- nozzle_guard.c (ESP-IDF) ---- */

/** Load the settings.  Call once at boot, before the backends start. */
void nozzle_guard_init(void);

/** The current settings */
void ng_config_get(ng_config_t *out);

/** ng_standby_for() with the settings and the plastic on record */
int ng_standby_c(int target_c);

/** Tell the hub the nozzle was taken down (from_c to to_c) in a wait */
void ng_notify_standby(int from_c, int to_c, const char *filename);

/** Tell the hub the filament ran out and the print is paused; standby_c is
 *  where the nozzle goes after mid_after_min (0: it stays hot) */
void ng_notify_runout(int target_c, int standby_c, const char *filename);
