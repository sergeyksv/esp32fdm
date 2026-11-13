# ESP32 FDM Printer Bridge

WiFi-to-printer bridge firmware for **Freenove ESP32-S3-WROOM** (OV2640/OV3660 camera, dual USB-C).
Dual backend: Marlin (USB serial) or Klipper (Moonraker HTTP).

## Features

- **MJPEG video stream** from OV2640/OV3660 (`/stream`, `/capture`)
- **USB Host** serial bridge to FDM printer (CDC-ACM + CH34x/CP210x/FT23x VCP)
- **Marlin backend**: direct serial, host printing from SD, GCode terminal
- **Klipper backend**: Moonraker HTTP API, file upload + print, temp/progress tracking
- **Bed leveling** (Marlin): probe mesh via G29, color-coded grid visualization, mesh history, screw adjustment calculator with 1/8-turn precision
- **Obico integration**: AI failure detection, remote monitoring
- **RFC 2217 server** — OctoPrint connects via `rfc2217://<ip>:2217` (Marlin only)
- **Wait for user** (Marlin): `M0`/`M1` or a `; @pause <text>` comment in a file printed from SD stops the print without parking, `/print` is the phone page to adjust Z and continue, and the hub sends a Telegram notification
- **Cold pull** (Marlin, Utils): pick a plastic and the board runs a built-in nozzle-cleaning script as a host print, stopping with a notification each time the user has to pull or reload the filament

## Build

Requires ESP-IDF v5.4 installed at `~/esp/esp-idf`.

```bash
source ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py menuconfig   # Set WiFi SSID/password under "ESP32 FDM Bridge Configuration"
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Project Structure

```
main/
  main.c                — Entry point: NVS → WiFi → Camera → HTTP → USB Host → backend
  wifi.c/h              — WiFi STA, EventGroup blocking, auto-reconnect
  camera.c/h            — OV2640/OV3660, Freenove pin mapping, SVGA JPEG Q12, triple-buffered PSRAM
  httpd.c/h             — HTTP server :80, dashboard, settings, camera, API endpoints
  usb_serial.cpp/h      — USB Host CDC-ACM + VCP drivers (C++ for VCP headers)
  printer_backend.h     — Backend enum (Marlin/Klipper), shared by layout.h
  printer_comm.c/h      — Marlin serial protocol, host printing, temp history, state machine
  printer_comm_klipper.c/h — Klipper/Moonraker HTTP backend, file upload to Moonraker
  terminal.c/h          — Web GCode terminal (Marlin only)
  sdcard.c/h            — SD card mount/unmount
  sdcard_httpd.c/h      — SD card web UI, backend-aware print/pause/resume/cancel
  cache.c/h             — LittleFS cache housekeeping (LRU eviction by mtime)
  bedlevel.c/h          — Bed leveling: mesh probe, visualization, screw adjustment (Marlin only)
  obico_client.c/h      — Obico WebSocket, snapshot upload, Janus signaling
  rfc2217.c/h           — RFC 2217 Telnet server (Marlin only)
  gcode_wait.h          — M0/M1 parsing (pure, testable on a PC)
  print_page.c/h        — /print: phone page for the current print (continue, Z, pause, stop) and "Print again" for the last 5 SD files
  cold_pull.c/h         — /utils/coldpull: builds the cold pull script and starts it as a host print
  hub_client.c/h        — Sends events to the notification hub (URL set in Settings)
  layout.h/c            — Shared HTML layout, nav bar (hides Marlin-only items for Klipper)
  i18n.c/h              — Translations, gettext style: _(), N_(), NT_(), TR(); language per request
i18n/                   — <lang>.po translations and messages.pot (tools/i18n.py update / init / stats)
tools/i18n.py           — Extracts marked text, refreshes .po files, builds i18n_tables.c at build time
  Kconfig.projbuild     — Menuconfig options
hub/                    — Notification hub: LAN service that forwards events to Telegram (see hub/README.md)
```

## Hardware

- **Right USB-C:** UART bridge — flashing/debug
- **Left USB-C:** ESP32-S3 native USB (GPIO19/20) — OTG Host to printer
- **Camera:** OV2640 or OV3660 on DVP (GPIOs 4-18, no conflict with USB)
- **PSRAM:** 8MB OPI — camera frame buffers
- **Flash:** 16MB — 4MB app (dual OTA), ~12MB LittleFS cache partition

## Core Affinity

- **Core 0:** WiFi, HTTP server, RFC 2217 server + TX drain
- **Core 1:** USB Host lib, device poll, CDC-ACM driver (priority 20)

## Architecture Notes

- `usb_serial.cpp` must be C++ because VCP driver headers are `.hpp`
- C++ exceptions enabled (`CONFIG_COMPILER_CXX_EXCEPTIONS=y`) — required by VCP component
- Camera XCLK at 20 MHz, SVGA JPEG Q12, 2 DMA buffers with GRAB_LATEST
- RFC 2217 uses FreeRTOS StreamBuffer to decouple USB RX callback from TCP send
- Single RFC 2217 client at a time (matches OctoPrint usage pattern)
- OTG port may not supply 5V VBUS — user may need powered USB hub
- `printer_backend_t` enum lives in `printer_backend.h` to break circular include between `layout.h` and `printer_comm.h`
- Klipper SD print: uploads from local SD to Moonraker with size-based dedup, auto-deletes previous upload
- Marlin host print uses line numbering + checksums; M110 N0 reset on finish to prevent resend loops
- Terminal and RFC 2217 hidden from UI when Klipper backend is selected
- GCode file info (layers, time, filament, thumbnail) cached in `/cache/gcode/` on LittleFS — keyed by filename + file size, LRU eviction at 50 entries
- Bed mesh auto-captured from any G29 output (probe button, terminal, printer UI); history stored in `/cache/beds/`, LRU eviction at 100 entries
- Screw adjustment: 4-screw centered layout, configurable bed size/origin/pitch, tighten or loosen mode, 1/4 + 1/8 turn display
- `M0`/`M1` in a host print are never sent to the printer, and `; @pause` is the form for start files shared with other printers. A hold is `paused` plus `holding`: no parking, no M76 (the printer's UI must stay in print mode), and no `M84 S<n>`: on the Kobra 2 build it makes Marlin drop every babystep, from `M290` and from the printer's own Z buttons, until the next `G28`. It cancels the print after `CONFIG_HOLD_TIMEOUT_MIN`
- Continue from `/print` carries the hold id, so a double tap cannot skip the next M0
- Z taps on `/print` skip the command queue and accumulate in `host_print.c`, then go out as one `M290` when the printer is ready. A step sent without a reply counts as applied and is never resent, since it may have moved the nozzle
- A host print starts at once: the whole-file scan runs in its own low-priority task with its own file handle, and `preparing` is set until it finishes. Progress and layer counts are unknown until then
- A built-in script (cold pull) is a host print from a malloc'd buffer read through `fmemopen`, so no SD card is needed and every print feature applies. The buffer waits in `host_print.c` while `PCMD_HOST_SCRIPT` carries only the name, belongs to the print from then on, and is scanned in place rather than in the scan task, which could outlive it
- Marlin runs M104/M106 as soon as they are read, not after the moves queued before them: a script that must heat or cool after a move (the cold pull purge) needs M400 in between
- "Print again" on /print starts only a file from the recent list (NVS, newest first), since the page is opened from the hub and Telegram
- Each board with a hub URL also reports its status every 10 s and on every state change (`/api/v1/status`), for the tile dashboard the hub serves at `/`
- The web UI is translated gettext style: the English text in the code is the key, and `i18n/<lang>.po` holds the translations, turned into `i18n_tables.c` at build time. `HTTPD_REGISTER` wraps every handler so the language (cookie `lang`, else `Accept-Language`) is set per request in a thread-local. Big page templates mark text with `N_()` and go through `i18n_page()`, which builds each template once per language in PSRAM. The generator fails the build on a translation whose `%` formats differ or that adds `"`, `<`, `>`, `&` or `\`, and turns `'` into `’` so a translation cannot close a JavaScript string. After changing marked text run `python3 tools/i18n.py update` and translate the new entries
- The language lives in a thread-local, so text built in another task (an async handler, a probe task) is English unless the handler passes `i18n_lang()` along and the task calls `i18n_set_lang()` (see `logbuf.c`, `ota.c`), or the text is built later in the request that shows it (see `bedlevel.c`)
- On a phone (under 560 px) the nav is the brand, the page name and a menu button that opens the links as a list; on wider screens the brand is the Home link. CPU bars and the language switch are in the footer
- Boards send events, not Telegram messages: the bot token and chat exist only on the hub
- The `/logs` ring lives in PSRAM `.noinit` (`CONFIG_SPIRAM_ALLOW_NOINIT_SEG_EXTERNAL_MEMORY`), so a panic, watchdog or `esp_restart` keeps it and each boot adds a `==== reboot #N: reason=... ====` line. Power loss or brownout wipes it, and so may a new firmware that moves the segment. Each write is flushed from the cache, or the last lines before a crash would be lost

## OctoPrint Configuration

```
Settings → Webcam & Timelapse:
  Stream URL:   http://<esp32-ip>/stream
  Snapshot URL: http://<esp32-ip>/capture

Settings → Serial Connection → Additional serial ports:
  rfc2217://<esp32-ip>:2217

Connection panel:
  Port: rfc2217://<esp32-ip>:2217
  Baudrate: 115200
```
