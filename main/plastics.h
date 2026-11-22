#pragma once

/*
 * The plastics the nozzle tools know, and which one is in the nozzle now.
 * Filament change and cold pull both choose from this table, and both record
 * what they leave in the nozzle, so the next change knows how hot to purge.
 */

#include <stdbool.h>

typedef struct {
    const char *name;
    int print_c;    /* Melt and purge */
    int pull_c;     /* Cold pull: just above solid */
} plastic_t;

extern const plastic_t PLASTICS[];
extern const int PLASTIC_COUNT;

/** Index of the plastic in the nozzle, or -1 if unknown */
int plastic_loaded(void);

/** Remember what is in the nozzle; -1 forgets it */
void plastic_set_loaded(int idx);

/** The hotter of two plastics, either may be -1 */
int plastic_hotter(int a, int b);

/**
 * host_script_done_t for a script that leaves plastic idx in the nozzle.
 * Cancelled, either may be there, so the hotter of it and the old one stays.
 */
void plastic_script_done(bool completed, int idx);
