#!/bin/sh
# Builds and runs the PC tests: the G-code scan (test_scan.c), the stops in a
# file sent to Klipper (test_holds.c), then the Marlin protocol code (test_marlin.c).
# Needs gcc and ESP-IDF at $IDF_PATH or ~/esp/esp-idf (only for cJSON's source).
# Arguments go to the test program: -v shows the log of each test.
set -e
cd "$(dirname "$0")"
IDF=${IDF_PATH:-$HOME/esp/esp-idf}
CJSON=$IDF/components/json/cJSON
OUT=${TMPDIR:-/tmp}/esp32fdm-test-host
mkdir -p "$OUT"
CFLAGS="-std=gnu11 -g -O0 -Wall -Wno-unused-function -Wno-format-security -include stubs/host_compat.h -Istubs -I. -I../../main -I$CJSON"
gcc $CFLAGS test_scan.c -lm -o "$OUT/test_scan"
"$OUT/test_scan"
gcc $CFLAGS test_holds.c -o "$OUT/test_holds"
"$OUT/test_holds"
# host_print.c and the test (which includes printer_comm.c) each have their own statics
gcc $CFLAGS -c ../../main/host_print.c -o "$OUT/host_print.o"
gcc $CFLAGS -c sim.c -o "$OUT/sim.o"
gcc $CFLAGS -c test_marlin.c -o "$OUT/test_marlin.o"
gcc $CFLAGS -c "$CJSON/cJSON.c" -o "$OUT/cJSON.o"
gcc "$OUT/test_marlin.o" "$OUT/host_print.o" "$OUT/sim.o" "$OUT/cJSON.o" -lm -o "$OUT/test_marlin"
exec "$OUT/test_marlin" "$@"
