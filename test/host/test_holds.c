/* PC test for klipper_holds.h: where each stop is, before or after the first
 * layer marker, for the nozzle guard. Run: test/host/run.sh */
#include <stdio.h>
#include <string.h>

#include "klipper_holds.h"

static int s_fail;

#define CHECK(cond, ...) do { if (!(cond)) { s_fail++; printf("      %s:%d: %s — ", __FILE__, __LINE__, #cond); \
    printf(__VA_ARGS__); printf("\n"); } } while (0)

static int run(const char *text, klipper_hold_t *holds, int max)
{
    kh_xform_t x;
    kh_init(&x, holds, max, NULL, NULL);
    /* In pieces, as it comes from the SD card */
    size_t n = strlen(text);
    for (size_t i = 0; i < n; i += 7) kh_feed(&x, text + i, n - i < 7 ? n - i : 7);
    kh_finish(&x);
    return x.n_holds;
}

int main(void)
{
    klipper_hold_t h[4];
    printf("TEST  stops before and after the first layer (Cura)\n");
    int n = run("G28\n; @pause Check Z\nG1 Z0.3\n;LAYER:0\nG1 X1 E1\nM0 Magnets\n;LAYER:1\n", h, 4);
    CHECK(n == 2, "%d stops", n);
    CHECK(h[0].before_layers, "the start stop is not before the layers");
    CHECK(!h[1].before_layers, "the stop in the print counts as before the layers");

    printf("TEST  OrcaSlicer's ;LAYER_CHANGE\n");
    n = run("M0 Start\n;LAYER_CHANGE\n;Z:0.2\nG1 X1\n; @pause Later\n", h, 4);
    CHECK(n == 2 && h[0].before_layers && !h[1].before_layers, "%d stops, %d %d", n, h[0].before_layers, h[1].before_layers);

    printf("TEST  a file without layer markers: no stop is before the layers\n");
    n = run("G28\nM0 Start\nG1 X1 E1\nM1 Later\n", h, 4);
    CHECK(n == 2 && !h[0].before_layers && !h[1].before_layers, "%d stops, %d %d", n, h[0].before_layers, h[1].before_layers);

    printf("%s\n", s_fail ? "holds: FAILED" : "holds: all passed");
    return s_fail ? 1 : 0;
}
