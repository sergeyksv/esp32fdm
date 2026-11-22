/* The whole-file G-code scan (gcode_scan_file) reads an SD file with read()
 * into a buffer; a built-in script in memory (fmemopen) still goes through
 * fgets(). Both must give the same result, and total_lines must count lines as
 * the host print sender does (fgets(line, 256): a line over 255 characters
 * counts once per piece), or progress drifts. This runs both paths over
 * generated files with awkward line lengths, CRLF, no final newline and lines
 * across read boundaries, and over ../../real.gcode if it is there. */
#include "../../main/gcode_scan.c"

#include <stdarg.h>

/* What gcode_scan.c needs from elsewhere */
void cache_touch(const char *path) { (void)path; }
void cache_evict_lru(const char *dir_path, int max_entries) { (void)dir_path; (void)max_entries; }
void vTaskDelay(TickType_t ticks) { (void)ticks; }
void sim_log(char level, const char *tag, const char *fmt, ...) { (void)level; (void)tag; (void)fmt; }

static int s_failed, s_passed;

/* The sender's own count: fgets(256) calls until EOF */
static int32_t sender_lines(const char *path)
{
    FILE *f = fopen(path, "r");
    char line[256];
    int32_t n = 0;
    while (fgets(line, sizeof(line), f)) n++;
    fclose(f);
    return n;
}

static void check(const char *name, const char *path)
{
    FILE *f = fopen(path, "r");
    gcode_scan_t fd_scan;
    gcode_scan_file(f, &fd_scan);                /* a real file: the read() path */
    fclose(f);

    /* The same bytes through fmemopen: no descriptor, the fgets() path */
    f = fopen(path, "rb");
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    rewind(f);
    char *data = malloc(size + 1);
    size_t got = fread(data, 1, size, f);
    fclose(f);
    gcode_scan_t mem_scan;
    FILE *m = fmemopen(data, got ? got : 1, "r");
    if (got) gcode_scan_file(m, &mem_scan);
    else memset(&mem_scan, 0, sizeof(mem_scan)), mem_scan.total_layers = -1;
    fclose(m);
    free(data);

    int32_t want = sender_lines(path);
    bool ok = fd_scan.total_lines == want && mem_scan.total_lines == want &&
              fd_scan.total_layers == mem_scan.total_layers &&
              fd_scan.max_z == mem_scan.max_z &&
              fd_scan.layer_height == mem_scan.layer_height &&
              fd_scan.first_layer_height == mem_scan.first_layer_height;
    if (ok) {
        s_passed++;
    } else {
        s_failed++;
        printf("FAIL %s: lines read()=%ld fgets()=%ld sender=%ld, layers %ld/%ld, maxZ %.3f/%.3f, "
               "lh %.3f/%.3f, flh %.3f/%.3f\n", name,
               (long)fd_scan.total_lines, (long)mem_scan.total_lines, (long)want,
               (long)fd_scan.total_layers, (long)mem_scan.total_layers,
               fd_scan.max_z, mem_scan.max_z, fd_scan.layer_height, mem_scan.layer_height,
               fd_scan.first_layer_height, mem_scan.first_layer_height);
    }
}

static void put_len(FILE *f, int len, char fill)
{
    for (int i = 0; i < len; i++) fputc(fill, f);
}

int main(void)
{
    const char *dir = getenv("TMPDIR") ? getenv("TMPDIR") : "/tmp";
    char path[512];
    snprintf(path, sizeof(path), "%s/esp32fdm-test-scan.gcode", dir);

    /* Line lengths around the 255-character fgets piece */
    int lens[] = { 0, 1, 253, 254, 255, 256, 257, 509, 510, 511, 1000 };
    for (int nl = 0; nl < 3; nl++) {             /* "\n", "\r\n", no final newline */
        for (size_t i = 0; i < sizeof(lens) / sizeof(lens[0]); i++) {
            FILE *f = fopen(path, "w");
            fprintf(f, "; layer_height = 0.2\n;LAYER:0\nG1 X1 Z0.3\n");
            put_len(f, lens[i], ';');
            fputs(nl == 1 ? "\r\n" : "\n", f);
            fprintf(f, ";LAYER:1\nG1 Z0.5 E1\n");
            put_len(f, lens[i], 'X');
            if (nl != 2) fputs(nl == 1 ? "\r\n" : "\n", f);
            fclose(f);
            char name[64];
            snprintf(name, sizeof(name), "length %d, %s", lens[i],
                     nl == 0 ? "LF" : nl == 1 ? "CRLF" : "no final newline");
            check(name, path);
        }
    }

    /* Random files of 20-300 KB: lines across the 16 KB read boundaries */
    srand(1);
    for (int t = 0; t < 200; t++) {
        FILE *f = fopen(path, "w");
        long target = 20000 + rand() % 280000;
        int layer = 0;
        float z = 0.2f;
        bool crlf = rand() % 4 == 0;
        for (long n = 0; n < target;) {
            int kind = rand() % 10;
            int w;
            if (kind == 0) {
                w = fprintf(f, ";LAYER:%d", layer++);
                z += 0.2f;
            } else if (kind == 1) {
                w = fprintf(f, "G1 Z%.2f F600", z);
            } else if (kind == 2) {
                w = rand() % 700;                /* long lines, thumbnails, config dumps */
                fputc(';', f);
                put_len(f, w, 'a' + rand() % 26);
                w++;
            } else {
                w = fprintf(f, "G1 X%d Y%d E%.4f", rand() % 300, rand() % 300, rand() % 1000 / 1000.0);
            }
            fputs(crlf ? "\r\n" : "\n", f);
            n += w + 1;
        }
        if (rand() % 3 == 0) fputs("G1 X0 Z99.9", f);     /* no final newline */
        fclose(f);
        char name[64];
        snprintf(name, sizeof(name), "random file %d", t);
        check(name, path);
    }
    remove(path);

    if (access("../../real.gcode", R_OK) == 0) check("real.gcode", "../../real.gcode");

    printf("scan: %d passed, %d failed\n", s_passed, s_failed);
    return s_failed ? 1 : 0;
}
