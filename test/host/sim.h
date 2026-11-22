/* Simulated Marlin printer and the fake environment around printer_comm.c.
 *
 * Time is a simulation clock in microseconds: esp_timer_get_time() reads it,
 * and a wait (xStreamBufferReceive with a timeout, vTaskDelay) moves it on to
 * the printer's next event or to the end of the wait, so a 240 s timeout runs
 * in microseconds of real time.
 *
 * The printer runs the commands it receives one after another, in the order
 * they came, and answers like Marlin. Its quirks are switches in sim_cfg. */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    /* STM32 CDC on some boards (MKS Robin Nano; the Kobra 2 Neo, it seems):
     * what the printer sends stays in it until the board's next USB OUT
     * transfer, i.e. until the board sends something */
    bool hold_tx;
    /* Only the answers to these commands (space-separated codes) stay in the
     * printer until the next OUT: fdm05 (Kobra 2 Neo) held G28's on 17.11.26
     * while it answered everything else, and a G4 S3, on time */
    const char *hold_cmds;
    int cmd_ms;          /* an ordinary command: 5 ms */
    int g28_ms;          /* homing: 20 s, as fdm05 took before it got stuck */
    int g29_ms;          /* bed probe after its homing */
    int g29_points;      /* "measured_z:" lines spread over g29_ms (0 = none) */
    int m109_ms;         /* heating, with a temperature report each second */
    int busy_ms;         /* M113 host keepalive: "busy: processing" this often
                          * while a long command runs (0 = the printer sends none) */
    bool knows_m118;     /* else "Unknown command" */
    const char *unknown; /* space-separated codes the printer does not know, e.g. "M73" */
    const char *slow_line; /* a command starting with this takes slow_ms, with the busy
                            * keepalive: a move that waits for room in a full planner */
    int slow_ms;
    bool dead;           /* receives, never answers */
} sim_cfg_t;

extern sim_cfg_t sim_cfg;

/* Back to t = 0, an idle printer with the defaults above, empty buffers and log */
void sim_reset(void);

/* The clock, in microseconds and in milliseconds */
int64_t sim_now_us(void);
static inline int64_t sim_now_ms(void) { return sim_now_us() / 1000; }

/* Lets the simulation run on to t, as if the board were busy elsewhere */
void sim_run_until_us(int64_t t);

/* What the board sent, one line per command ("G28", "M118 E1 ping", ...),
 * with line numbers and checksums removed; and how many lines it sent */
const char *sim_tx_line(int i);
int sim_tx_count(void);
int sim_tx_count_of(const char *prefix);

/* The log lines printer_comm.c and host_print.c wrote */
bool sim_log_has(const char *text);
void sim_log_dump(void);
void sim_log_clear(void);

/* Runs an endless task loop (printer_comm_task) until stop() returns true or
 * limit_us of simulated time pass: vTaskDelay() leaves the loop with longjmp.
 * Returns true if stop() did */
bool sim_run_task(void (*task)(void *), bool (*stop)(void), int64_t limit_us);

/* Nonzero to echo the log to stdout as it is written */
extern int sim_verbose;

/* Nozzle guard settings the host print sees (off by default), and the plastic in the nozzle */
#include "nozzle_guard.h"
extern ng_config_t sim_ng_config;
extern int sim_ng_plastic;
