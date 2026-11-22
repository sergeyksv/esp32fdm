/* PC tests for the Marlin side of printer_comm.c (sending a command and
 * waiting for its "ok") and for host_print.c on top of it, against the
 * simulated printer in sim.c.
 *
 * printer_comm.c is included as it is, so the tests see its static functions
 * and state; nothing in it is changed for the tests.
 *
 * Two kinds of test:
 *   TEST  — what works now and must keep working.
 *   KNOWN — a problem seen on a real printer that the code does not handle
 *           yet. The test states the behaviour we want, so it fails today and
 *           the run reports it as a known problem, not a failure. The day it
 *           passes the run fails with "now passes": make it a TEST.
 *
 * Run: test/host/run.sh   (-v shows the log of each test) */
#include "../../main/printer_comm.c"

#include "sim.h"

/* ---- A minimal test runner ---- */

static bool s_failed;          /* the current test */
static bool s_known;           /* the current test is a KNOWN one */
static int s_pass, s_fail, s_known_open, s_known_fixed;

#define CHECK(cond, ...) do { \
    if (!(cond)) { \
        s_failed = true; \
        printf("      %s:%d: %s — ", __FILE__, __LINE__, #cond); \
        printf(__VA_ARGS__); \
        printf("\n"); \
    } \
} while (0)

/* ---- Fresh state for each test ---- */

static void setup(void)
{
    sim_reset();
    static bool once;
    if (!once) {
        s_rx_stream = xStreamBufferCreate(4096, 1);
        g_state_mutex = xSemaphoreCreateMutex();
        s_cmd_queue = xQueueCreate(CMD_QUEUE_SIZE, sizeof(printer_cmd_t));
        s_temp_history = calloc(TEMP_HISTORY_MAX, sizeof(temp_sample_t));
        hp_init();
        once = true;
    }
    printer_cmd_t junk;
    while (xQueueReceive(s_cmd_queue, &junk, 0) == pdTRUE) {}
    s_pending_query = QUERY_NONE;
    s_line_len = 0;
    s_cmd_cooldown_until_us = 0;
    s_echo_ping_cmds[0] = '\0';
    s_unsupported_count = 0;
    s_line_callback = NULL;
    s_polling_suppressed = false;
    memset(&g_state, 0, sizeof(g_state));
}

/* Sends one command the way the printer task does, returns its result and
 * how long it took in simulated milliseconds */
static bool send(const char *gcode, int64_t *ms)
{
    int64_t t0 = sim_now_us();
    bool ok = marlin_send_cmd(gcode, QUERY_CMD);
    if (ms) *ms = (sim_now_us() - t0) / 1000;
    return ok;
}

/* The same through the host print's sender: 3 s timeout, 5 stall pings */
static bool send_hp(const char *gcode, int64_t *ms)
{
    int64_t t0 = sim_now_us();
    bool ok = marlin_send_cmd_hp(gcode, QUERY_CMD);
    if (ms) *ms = (sim_now_us() - t0) / 1000;
    return ok;
}

/* A built-in script run as a host print by the real printer task loop */
static int s_script_done;      /* -1 running, 0 cancelled, 1 completed */

static void script_done(bool completed, int arg)
{
    (void)arg;
    s_script_done = completed ? 1 : 0;
}

static bool script_over(void) { return s_script_done >= 0; }

static int run_script(const char *text, int64_t limit_s)
{
    s_script_done = -1;
    CHECK(hp_start_script("test", strdup(text), script_done, 0) == ESP_OK, "script not queued");
    sim_run_task(printer_comm_task, script_over, limit_s * 1000000LL);
    return s_script_done;
}

/* ================= What works now ================= */

static void t_instant_command(void)
{
    int64_t ms;
    CHECK(send("G90", &ms), "no ok");
    CHECK(ms < 100, "took %lld ms", (long long)ms);
}

static void t_m105_temperatures(void)
{
    CHECK(marlin_send_cmd("M105", QUERY_M105), "no ok");
    CHECK(g_state.hotend_actual > 29 && g_state.hotend_actual < 31, "hotend %.1f", g_state.hotend_actual);
}

static void t_homing_20s(void)
{
    int64_t ms;
    CHECK(send("G28", &ms), "no ok");
    CHECK(ms >= 19900 && ms <= 20500, "took %lld ms", (long long)ms);
    CHECK(g_state.z > 2.8f && g_state.z < 3.0f, "Z %.2f from the position after homing", g_state.z);
}

static void t_homing_long_with_keepalive(void)
{
    /* Marlin with M113: "busy: processing" every 2 s keeps the wait going */
    sim_cfg.g28_ms = 300000;
    sim_cfg.busy_ms = 2000;
    int64_t ms;
    CHECK(send("G28", &ms), "no ok");
    CHECK(ms >= 299000 && ms <= 301000, "took %lld ms", (long long)ms);
}

static void t_silent_240s_gives_up(void)
{
    /* No keepalive and no answer: G28 is given up after 240 s */
    sim_cfg.g28_ms = 400000;
    int64_t ms;
    CHECK(!send("G28", &ms), "ok from a printer that did not answer");
    CHECK(ms >= 239000 && ms <= 241000, "gave up after %lld ms", (long long)ms);
    CHECK(sim_log_has("giving up"), "no 'giving up' in the log");
}

static void t_heating_with_reports(void)
{
    /* M109 for 60 s, past its 30 s timeout: each temperature report extends it */
    int64_t ms;
    CHECK(send("M109 S200", &ms), "no ok");
    CHECK(ms >= 59000 && ms <= 61000, "took %lld ms", (long long)ms);
}

static void t_dwell(void)
{
    int64_t ms;
    CHECK(send("G4 S3", &ms), "no ok");
    CHECK(ms >= 2900 && ms <= 3200, "took %lld ms", (long long)ms);
}

static void t_dead_printer_6s(void)
{
    sim_cfg.dead = true;
    int64_t ms;
    CHECK(!send("G90", &ms), "ok from a dead printer");
    CHECK(ms >= 5900 && ms <= 6200, "gave up after %lld ms", (long long)ms);
}

static void t_bed_probe_with_points(void)
{
    /* G29 for 150 s: each "measured_z:" restarts the wait */
    sim_cfg.g29_ms = 150000;
    sim_cfg.g29_points = 25;
    int64_t ms;
    CHECK(send("G29", &ms), "no ok");
    CHECK(ms >= 149000 && ms <= 151000, "took %lld ms", (long long)ms);
}

static void t_unknown_command_remembered(void)
{
    sim_cfg.unknown = "M73";
    CHECK(send("M73 P0 R10", NULL), "no ok");
    bool found = false;
    for (int i = 0; i < s_unsupported_count; i++) found |= s_unsupported_cmds[i] == 73;
    CHECK(found, "M73 not marked unsupported");
}

static void t_hp_held_ok_flushed_by_stall_ping(void)
{
    /* MKS Robin Nano: the ok stays in the printer until the next OUT.
     * The host print sender pings after 3 s and the ok comes out */
    sim_cfg.hold_tx = true;
    int64_t ms;
    CHECK(send_hp("G1 X10 F3000", &ms), "no ok");
    CHECK(ms >= 2900 && ms <= 3200, "took %lld ms", (long long)ms);
    CHECK(sim_tx_count_of("M118") == 1, "%d pings", sim_tx_count_of("M118"));
}

static void t_hp_dead_printer_5_pings(void)
{
    sim_cfg.dead = true;
    int64_t ms;
    CHECK(!send_hp("G1 X10", &ms), "ok from a dead printer");
    CHECK(ms >= 17900 && ms <= 18300, "gave up after %lld ms", (long long)ms);
    CHECK(sim_tx_count_of("M118") == 5, "%d pings", sim_tx_count_of("M118"));
}

static void t_echo_ping_list_flushes(void)
{
    /* Settings > Marlin > CDC Echo Ping "G92": M118 right after G92 brings
     * out its held ok at once */
    sim_cfg.hold_tx = true;
    set_echo_ping("G92");
    int64_t ms;
    CHECK(send("G92 E0", &ms), "no ok");
    CHECK(ms < 100, "took %lld ms", (long long)ms);
}

static void t_script_completes(void)
{
    int r = run_script("G28\nG1 X10 F3000\nG1 X20\nG4 S2\n", 600);
    CHECK(r == 1, "script %s", r < 0 ? "still running" : "cancelled");
    CHECK(sim_tx_count_of("G28") == 1, "G28 sent %d times", sim_tx_count_of("G28"));
    CHECK(sim_tx_count_of("G1 X20") == 1, "last move sent %d times", sim_tx_count_of("G1 X20"));
}

/* ---- Nozzle guard (nozzle_guard.h) ---- */

static int64_t s_stop_at;
static bool stop_time(void) { return sim_now_us() >= s_stop_at; }
static bool stop_holding(void) { return g_state.holding; }

static void run_for_s(int64_t s)
{
    s_stop_at = sim_now_us() + s * 1000000LL;
    sim_run_task(printer_comm_task, stop_time, s * 1000000LL + 1000000LL);
}

/* ABS in the nozzle, the defaults of nozzle_guard.c */
static void guard_on(void)
{
    sim_ng_config = (ng_config_t){ .enabled = true, .purge_every_s = 120, .purge_mm = 5,
                                   .purge_for_min = 15, .mid_after_min = 10,
                                   .standby_c = { 0, 160, 160, 170 } };
    sim_ng_plastic = 2;
    sim_cfg.m109_ms = 3000;
}

static void guard_off(void)
{
    memset(&sim_ng_config, 0, sizeof(sim_ng_config));
    sim_ng_plastic = -1;
}

/* Index of the n-th (0-based) line sent that starts with prefix, or -1 */
static int tx_index(const char *prefix, int n)
{
    for (int i = 0; i < sim_tx_count(); i++) {
        if (!strncmp(sim_tx_line(i), prefix, strlen(prefix)) && n-- == 0) return i;
    }
    return -1;
}

static int start_and_hold(const char *text)
{
    s_script_done = -1;
    CHECK(hp_start_script("test", strdup(text), script_done, 0) == ESP_OK, "script not queued");
    sim_run_task(printer_comm_task, stop_holding, 600 * 1000000LL);
    CHECK(g_state.holding, "the print did not stop for the user");
    return (int)g_state.hold_id;
}

static void finish_after_continue(int hold_id)
{
    CHECK(hp_continue((uint32_t)hold_id) == ESP_OK, "continue not queued");
    sim_run_task(printer_comm_task, script_over, 600 * 1000000LL);
    CHECK(s_script_done == 1, "script %s", s_script_done < 0 ? "still running" : "cancelled");
}

static void t_guard_before_first_layer(void)
{
    guard_on();
    int id = start_and_hold("M109 S245\nM82\nG92 E0\nG1 X5 E2.5 F1200\n; @pause Check Z\n"
                            ";LAYER:0\nG1 X10 E3 F1200\n;LAYER:1\nG1 X20 E4\n");
    run_for_s(16 * 60);
    int purges = sim_tx_count_of("G1 E5.00 F180");
    CHECK(purges >= 6 && purges <= 7, "%d purges in 15 min, every 2 min", purges);
    CHECK(sim_tx_count_of("M104 S160") == 1, "standby sent %d times", sim_tx_count_of("M104 S160"));
    CHECK(sim_tx_count_of("G1 E-2.00 F1800") == 1, "no retract before standby");
    /* Absolute E: every own move ends with the file's position again */
    CHECK(sim_tx_count_of("G92 E2.50000") == purges + 1, "G92 E2.5 sent %d times", sim_tx_count_of("G92 E2.50000"));
    CHECK(g_state.nozzle == NOZZLE_STANDBY, "nozzle state %d", g_state.nozzle);
    finish_after_continue(id);
    int heat = tx_index("M109 S245", 1), prime = tx_index("G1 E2.00 F1800", 0), next = tx_index("G1 X10 E3", 0);
    CHECK(heat >= 0 && prime > heat && next > prime, "order: M109 %d, prime %d, next line %d", heat, prime, next);
    CHECK(g_state.nozzle == NOZZLE_HOT, "nozzle state %d after", g_state.nozzle);
    guard_off();
}

static void t_guard_during_print(void)
{
    guard_on();
    int id = start_and_hold("M109 S245\nM83\n;LAYER:0\nG1 X5 E1 F1200\nM0 Insert magnets\n"
                            ";LAYER:1\nG1 X10 E1\n");
    run_for_s(9 * 60);
    CHECK(sim_tx_count_of("M104 S160") == 0, "standby before 10 min");
    run_for_s(2 * 60);
    CHECK(sim_tx_count_of("G1 E5.00") == 0, "purged over the part");
    CHECK(sim_tx_count_of("M104 S160") == 1, "standby sent %d times", sim_tx_count_of("M104 S160"));
    CHECK(sim_tx_count_of("G92 E") == 0, "G92 in a relative-E file");
    finish_after_continue(id);
    int heat = tx_index("M109 S245", 1), next = tx_index("G1 X10 E1", 0);
    CHECK(heat >= 0 && next > heat, "order: M109 %d, next line %d", heat, next);
    guard_off();
}

static void t_guard_leaves_pla(void)
{
    guard_on();
    sim_ng_plastic = 0;                 /* PLA: standby 0 */
    int id = start_and_hold("M109 S210\nM83\n; @pause Check Z\n;LAYER:0\nG1 X10 E1 F1200\n");
    run_for_s(20 * 60);
    CHECK(sim_tx_count_of("G1 E5.00") == 0 && sim_tx_count_of("M104") == 0, "PLA was touched");
    finish_after_continue(id);
    CHECK(sim_tx_count_of("M109 S210") == 1, "heated again for PLA");
    guard_off();
}

static void t_guard_obico_pause(void)
{
    guard_on();
    s_script_done = -1;
    CHECK(hp_start_script("test", strdup("M109 S245\nM83\n;LAYER:0\nG1 X5 E1 F1200\nG4 S20\nG1 X6 E1\n"),
                          script_done, 0) == ESP_OK, "script not queued");
    run_for_s(10);
    printer_cmd_t pause = { .type = PCMD_PAUSE, .standby = true };
    CHECK(printer_comm_send_cmd(&pause) == ESP_OK, "pause not queued");
    run_for_s(30);
    CHECK(sim_tx_count_of("M104 S160") == 1, "standby sent %d times", sim_tx_count_of("M104 S160"));
    CHECK(g_state.nozzle == NOZZLE_STANDBY, "nozzle state %d", g_state.nozzle);
    printer_cmd_t resume = { .type = PCMD_RESUME };
    CHECK(printer_comm_send_cmd(&resume) == ESP_OK, "resume not queued");
    sim_run_task(printer_comm_task, script_over, 600 * 1000000LL);
    CHECK(s_script_done == 1, "script %s", s_script_done < 0 ? "still running" : "cancelled");
    int heat = tx_index("M109 S245", 1), back = tx_index("G1 E2.0 F2400", 0), next = tx_index("G1 X6 E1", 0);
    CHECK(heat >= 0 && back > heat && next > back, "order: M109 %d, prime %d, next line %d", heat, back, next);
    guard_off();
}

static void t_guard_runout(void)
{
    guard_on();
    s_script_done = -1;
    CHECK(hp_start_script("test", strdup("M109 S245\nM83\n;LAYER:0\nG1 X5 E1 F1200\nG4 S20\nG1 X6 E1\n"),
                          script_done, 0) == ESP_OK, "script not queued");
    run_for_s(10);
    printer_cmd_t pause = { .type = PCMD_PAUSE, .runout = true };
    CHECK(printer_comm_send_cmd(&pause) == ESP_OK, "pause not queued");
    run_for_s(30);
    CHECK(g_state.opstate == PRINTER_PAUSED, "not paused");
    CHECK(sim_log_has("filament runout 245 -> 160"), "no hub event");
    run_for_s(9 * 60);
    CHECK(sim_tx_count_of("M104 S160") == 0, "standby before 10 min");
    run_for_s(2 * 60);
    CHECK(sim_tx_count_of("M104 S160") == 1, "standby sent %d times", sim_tx_count_of("M104 S160"));
    CHECK(sim_tx_count_of("G1 E-2.00") == 0, "retracted again after the park");
    CHECK(g_state.nozzle == NOZZLE_STANDBY, "nozzle state %d", g_state.nozzle);
    printer_cmd_t resume = { .type = PCMD_RESUME };
    CHECK(printer_comm_send_cmd(&resume) == ESP_OK, "resume not queued");
    sim_run_task(printer_comm_task, script_over, 600 * 1000000LL);
    CHECK(s_script_done == 1, "script %s", s_script_done < 0 ? "still running" : "cancelled");
    int heat = tx_index("M109 S245", 1), back = tx_index("G1 E2.0 F2400", 0), next = tx_index("G1 X6 E1", 0);
    CHECK(heat >= 0 && back > heat && next > back, "order: M109 %d, prime %d, next line %d", heat, back, next);
    CHECK(g_state.nozzle == NOZZLE_HOT, "nozzle state %d after", g_state.nozzle);
    guard_off();
}

/* ================= Known problems (the behaviour we want) ================= */

static void k_homing_on_holding_printer(void)
{
    /* fdm05 (Kobra 2 Neo), 17.11.26: G28's answer stayed in the printer until
     * the board sent its next command, so every G28 waited the full 240 s and
     * failed. Wanted: the board gets the answer without waiting 240 s */
    sim_cfg.hold_cmds = "G28";
    int64_t ms;
    CHECK(send("G28", &ms), "no ok (gave up after %lld ms)", (long long)ms);
    CHECK(ms <= 60000, "took %lld ms", (long long)ms);
}

static void k_late_ok_not_taken_by_next_command(void)
{
    /* A G28 given up on still answers later. fdm05: the bed probe sent G29
     * after the failed G28 and took G28's late "ok" as G29's, so it went on
     * while the printer was probing. Wanted: the next command waits for its own ok */
    sim_cfg.g28_ms = 300000;            /* silent, longer than the 240 s wait */
    CHECK(!send("G28", NULL), "G28 should time out");
    int64_t ms;
    send("G4 S100", &ms);               /* runs after G28 ends at 300 s: done at 400 s */
    CHECK(ms >= 159000, "G4 S100 returned after %lld ms, on G28's late ok", (long long)ms);
}

static void k_echo_ping_ok_not_taken_by_next_command(void)
{
    /* The echo ping (M118) answers "echo:ping" and an "ok" of its own, which
     * the next command takes as its answer */
    set_echo_ping("G92");
    CHECK(send("G92 E0", NULL), "no ok");
    int64_t ms;
    CHECK(send("G4 S5", &ms), "no ok for G4 S5");
    CHECK(ms >= 4900 && ms <= 60000, "G4 S5 returned after %lld ms", (long long)ms);
}

static void k_stall_ping_ok_not_taken_by_next_command(void)
{
    /* The same for the host print's stall ping */
    sim_cfg.hold_tx = true;
    CHECK(send_hp("G1 X10", NULL), "no ok");
    sim_run_until_us(sim_now_us() + 100000);   /* the next command a moment later */
    int64_t ms;
    CHECK(send("G4 S5", &ms), "no ok for G4 S5");
    CHECK(ms >= 4900 && ms <= 60000, "G4 S5 returned after %lld ms", (long long)ms);
}

static void k_script_homes_on_holding_printer(void)
{
    /* fdm05, 17.11.26: "Command failed after retries, aborting host print: G28".
     * Wanted: the print gets past homing */
    sim_cfg.hold_cmds = "G28";
    int r = run_script("G28\nG1 X10 F3000\nG1 X20\n", 1200);
    CHECK(r == 1, "script %s", r < 0 ? "still running" : "cancelled");
    CHECK(!sim_log_has("aborting host print"), "the host print was aborted");
}

static void t_busy_move_does_not_stall_the_print(void)
{
    /* fdm20 (Neptune X2, MKS Robin Nano), 20.11.26: the brim stopped in the same
     * corner every loop.  A long move with the planner full answers "busy:
     * processing" (the M113 S2 keepalive the host print turns on), its ok comes
     * a second later, and the board sent nothing for the 4 s busy cooldown, so
     * the planner ran dry.  Now the next line goes out at the ok. */
    sim_cfg.busy_ms = 2000;
    sim_cfg.slow_line = "G1 X200";
    sim_cfg.slow_ms = 3000;
    int64_t t0 = sim_now_ms();
    int r = run_script("G1 X5 F3000\nG1 X200\nG1 X6\n", 600);
    int64_t ms = sim_now_ms() - t0;
    CHECK(r == 1, "script %s", r < 0 ? "still running" : "cancelled");
    CHECK(sim_tx_count_of("G1 X200") == 1 && sim_tx_count_of("G1 X6") == 1, "a move sent twice or never");
    CHECK(ms < 3500, "the 3 s move took the script %lld ms", (long long)ms);
}

/* ---- Runner ---- */

typedef struct { const char *name; void (*fn)(void); bool known; } test_t;

#define TEST(f, name)  { name, f, false }
#define KNOWN(f, name) { name, f, true }

static const test_t TESTS[] = {
    TEST(t_instant_command, "an ordinary command is answered at once"),
    TEST(t_m105_temperatures, "M105 fills in the temperatures"),
    TEST(t_homing_20s, "homing that answers in 20 s"),
    TEST(t_homing_long_with_keepalive, "homing over 240 s with busy keepalive"),
    TEST(t_silent_240s_gives_up, "a G28 silent for 240 s is given up on"),
    TEST(t_heating_with_reports, "M109 for 60 s with temperature reports"),
    TEST(t_dwell, "G4 S3 waits 3 s"),
    TEST(t_dead_printer_6s, "a command to a dead printer gives up after 6 s"),
    TEST(t_bed_probe_with_points, "G29 for 150 s with measured_z lines"),
    TEST(t_unknown_command_remembered, "an unknown command is remembered"),
    TEST(t_hp_held_ok_flushed_by_stall_ping, "host print: a held ok comes out with the stall ping"),
    TEST(t_hp_dead_printer_5_pings, "host print: a dead printer gets 5 pings, then gives up"),
    TEST(t_echo_ping_list_flushes, "echo ping list: G92's held ok comes out at once"),
    TEST(t_script_completes, "a built-in script prints to the end"),
    TEST(t_busy_move_does_not_stall_the_print, "a move answered with busy: the next line goes at its ok (fdm20)"),
    TEST(t_guard_before_first_layer, "guard: a wait before the first layer purges, then standby, heats back"),
    TEST(t_guard_during_print, "guard: a wait in the print goes to standby after 10 min, no purge"),
    TEST(t_guard_leaves_pla, "guard: PLA (standby 0) is left alone"),
    TEST(t_guard_runout, "guard: a runout pause goes to standby after 10 min, resume heats first"),
    TEST(t_guard_obico_pause, "guard: a pause from Obico goes to standby, resume heats first"),
    KNOWN(k_homing_on_holding_printer, "G28 on a printer that holds its answers (fdm05)"),
    KNOWN(k_late_ok_not_taken_by_next_command, "a timed-out G28's late ok is not taken by the next command"),
    KNOWN(k_echo_ping_ok_not_taken_by_next_command, "the echo ping's own ok is not taken by the next command"),
    KNOWN(k_stall_ping_ok_not_taken_by_next_command, "the stall ping's own ok is not taken by the next command"),
    KNOWN(k_script_homes_on_holding_printer, "a host print gets past G28 on a holding printer (fdm05)"),
};

int main(int argc, char **argv)
{
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "-v")) sim_verbose = 1;

    for (size_t i = 0; i < sizeof(TESTS) / sizeof(TESTS[0]); i++) {
        const test_t *t = &TESTS[i];
        printf("%s %s\n", t->known ? "KNOWN" : "TEST ", t->name);
        setup();
        s_failed = false;
        s_known = t->known;
        t->fn();
        if (!t->known) {
            if (s_failed) { s_fail++; printf("  FAIL\n"); }
            else s_pass++;
        } else if (s_failed) {
            s_known_open++;
            printf("  still a known problem\n");
        } else {
            s_known_fixed++;
            printf("  NOW PASSES: make it a TEST\n");
        }
    }
    printf("\n%d passed, %d failed; known problems: %d open, %d now passing\n",
           s_pass, s_fail, s_known_open, s_known_fixed);
    return s_fail || s_known_fixed ? 1 : 0;
}
