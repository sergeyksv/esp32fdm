# ESP32 FDM Bridge

A WiFi-to-printer bridge firmware for generic **ESP32-S3 camera modules** (Freenove ESP32-S3-WROOM and compatible clones), turning a ~$7 microcontroller into a full-featured wireless print server with camera monitoring and cloud AI failure detection — at a fraction of the cost of a Raspberry Pi setup.

## Why This Exists

Traditional wireless 3D printing setups require a single-board computer (Raspberry Pi ~$50-75), a USB webcam (~$20-30), an SD card, a power supply, and software installation (OctoPrint, Klipper, etc.). Total cost: **$80-120+**, plus ongoing maintenance of a full Linux system.

This project replaces all of that with a **single ~$7 ESP32-S3 board** that has everything built in:

| | Raspberry Pi Setup | ESP32 FDM Bridge |
|---|---|---|
| **Cost** | $80-120+ | ~$7 |
| **Hardware** | SBC + camera + PSU + case | Single board |
| **Setup** | OS install, SSH, packages | Flash and go |
| **Boot time** | 30-60 seconds | ~3 seconds |
| **Power** | 5W+ | <1W |
| **Camera** | Separate USB webcam | Built-in OV2640/OV3660 |
| **USB Host** | Native | OTG port |

The ESP32-S3's dual cores, 8MB PSRAM, and native USB OTG make it powerful enough to handle simultaneous MJPEG streaming, printer communication, and cloud connectivity — all in real time.

## Screenshots

| Dashboard | SD Card Printing |
|---|---|
| ![Dashboard](docs/screenshots/dashboard.png) | ![SD Print](docs/screenshots/sd-print.png) |

| Phone Page: Waiting for You | Printer Tiles (Hub) |
|---|---|
| ![Print page](docs/screenshots/print-phone.png) | ![Printer tiles](docs/screenshots/hub-tiles.png) |

| Cold Pull | GCode Terminal |
|---|---|
| ![Cold pull](docs/screenshots/cold-pull.png) | ![Terminal](docs/screenshots/terminal.png) |

| Obico Integration |
|---|
| ![Obico](docs/screenshots/obico.png) |

## Web Installer

Flash the firmware directly from your browser — no tools or setup required:

**[Open Web Installer](https://sergeyksv.github.io/esp32fdm/)**

Requires Chrome or Edge (Web Serial API). Connect the board via the **right USB-C port** (UART).

## Features

### Camera & Streaming
- **MJPEG stream** at 5 FPS (800x600 JPEG) — works with OctoPrint, Obico, and any browser
- **Snapshot endpoint** for timelapse and AI analysis
- **WebRTC streaming** via optional Janus proxy sidecar for low-latency viewing through Obico

### Printer Communication
- **USB Host serial bridge** — connects to any Marlin or Klipper printer via USB
- Supports **CDC-ACM** and popular VCP chips: **CH34x, CP210x, FTDI FT23x**
- **Marlin backend**: direct serial with auto-report parsing, temperature tracking, SD print progress, layer detection
- **Klipper backend**: HTTP API to Moonraker — full status, control, and object queries
- Runtime backend switching (Marlin/Klipper) via web UI

### Cloud Integration (Obico)
- **AI failure detection** — periodic JPEG snapshots analyzed server-side for spaghetti, layer shifts, etc.
- **WebSocket status** — live printer state pushed to Obico cloud (or self-hosted)
- **Remote monitoring & control** — view camera, check progress, pause/cancel from anywhere
- **WebRTC video** — optional low-latency streaming through Janus gateway sidecar
- **Simple linking** — 6-digit code pairs the device with your Obico account
- Works with [app.obico.io](https://app.obico.io) or self-hosted Obico server

### OctoPrint Compatibility
- **RFC 2217 Telnet serial bridge** — OctoPrint connects via `rfc2217://<ip>:2217`
- Camera endpoints work directly with OctoPrint's webcam settings
- Zero-config: just add the IP address

### SD Card & Printing
- **On-board SD card** file management — upload, download, delete via web UI
- **Marlin**: host-based GCode streaming from SD with line numbering, checksums, and resend handling
- **Klipper**: uploads from SD to Moonraker with size-based dedup (skips if unchanged), auto-deletes previous upload to prevent storage bloat
- **GCode scan cache** — file metadata (layers, time, filament, thumbnail) cached on flash so repeated prints load instantly instead of re-scanning
- Pause, resume, and cancel work with both backends
- **Print again** — the phone page lists the last 5 files printed from SD; a tap (after "Is the bed clear?") starts one again

### Waiting for the User (Marlin only)
- A `; @pause <text>` comment or `M0`/`M1` in a file printed from SD stops the print **without parking**: the nozzle stays where it is, the heaters stay on
- Use it in a start file, e.g. to tune the first layer by eye on a slow test line: the comment form is harmless on printers that do not run through the bridge
- `/print` is a page made for a phone: the stop message, **Continue**, and Z buttons (±0.01/±0.05 mm, sent as `M290` babysteps, limited to 1 mm per print)
- The printer beeps, and the [notification hub](#notification-hub) sends a Telegram message with a link to `/print`
- If nobody continues within 30 minutes (`HOLD_TIMEOUT_MIN`), the print is cancelled and the heaters are turned off

### Web Terminal (Marlin only)
- **Browser-based GCode terminal** — send commands and see raw printer responses
- Auto-locked during prints to prevent interference
- Hidden when Klipper backend is selected (use Mainsail/Fluidd instead)

### Utilities

#### Bed Leveling (Marlin only)
- **One-click bed probing** — sends G28 + G29, visualizes the result as a color-coded mesh grid (blue = low, green = zero, red = high)
- **Auto-capture** — mesh is captured whenever G29 output appears, whether triggered from the web UI, terminal, printer LCD, or OctoPrint
- **Mesh history** — up to 100 past meshes stored on flash with prev/next browsing
- **Quality stats** — range, max deviation, standard deviation, and a Good/Fair/Poor verdict badge
- **Screw adjustment calculator** — choose Tighten (CW) or Loosen (CCW) mode, get per-corner corrections in 1/4 + 1/8 turn increments
- **Configurable** — bed size, screw spacing, screw pitch, and origin corner (FL/FR/BL/BR) persisted across reboots

#### Cold Pull (Marlin only)
- **Nozzle cleaning** — pick a plastic (PLA, PETG, ABS, Nylon; temperatures editable) and the number of pulls
- The board runs a built-in script as a host print, so no SD card is needed: home, lift to Z50, melt and purge 30 mm, cool with the fan, then **hold the pull temperature** until you come
- It stops with a beep and a notification each time you need to pull or reload the filament, and you continue on `/print`

### Notifications & Printer Dashboard
- Boards send events to a small **[hub](hub/README.md)** on your LAN, which forwards them to **Telegram**: the bot token lives in one place, each board only needs the hub address
- Each board also reports its status every 10 s, and the hub shows a **tile per printer** for a phone: waiting for you (orange, first), printing (filled to the percentage), idle, error, printer not connected, board offline. A tap opens that board's `/print`

### Languages
- The web UI is translated **gettext style**: the English text in the code is the key, `i18n/<lang>.po` holds the translations (edit with any PO editor, e.g. Poedit)
- The language follows the browser, and a link in the footer switches it (kept in a cookie)
- Shipped: English, Russian. See [Translations](#translations) to add one

### WiFi & Configuration
- **Auto AP fallback** — if WiFi credentials are missing or connection fails, the board creates its own access point (`ESP32FDM-XXXX`) with a captive portal for setup
- All settings configurable via web UI — no reflashing needed
- NVS-persisted configuration survives reboots

### Web Dashboard
- Real-time temperature display (hotend + bed, actual/target)
- **Temperature history graph** — 1 hour of data, auto-refreshing canvas chart
- Print progress bar with layer count, elapsed time, and ETA
- Live camera snapshot (auto-refreshing)
- Responsive nav: Cam, SD, Term (Marlin only), Utils (Marlin only), Logs, Settings; on a phone it folds into a menu button

### Diagnostics
- **Logs page** with filters (printer, system, heap)
- The log lives in PSRAM that survives a **panic, watchdog or software reset**: after a crash the lines before it are still there, behind a `==== reboot #N: reason=... ====` marker (a power loss clears it)

## Hardware

### Target Board: ESP32-S3 Camera Module

Developed on the **Freenove ESP32-S3-WROOM**, but works with compatible clones that share the same pin layout (many generic ESP32-S3-CAM boards on AliExpress/Amazon are near-identical). Some clones may require holding a BOOT button during flashing. Required features:

- **ESP32-S3** dual-core with **8MB PSRAM** (OPI) and **16MB flash**
- **OV2640 or OV3660 camera** on DVP bus
- **SD card slot** (1-bit SDMMC on GPIO 38/39/40)
- **Two USB-C ports:**
  - **Right:** UART bridge — for flashing and debug console
  - **Left:** ESP32-S3 native USB (GPIO 19/20) — **OTG Host to printer**

> **Note:** The OTG port may not supply 5V VBUS. If your printer doesn't enumerate, use a powered USB hub between the ESP32 and the printer.

### Pinout

| Function | GPIOs |
|---|---|
| Camera DVP | D0-D7: 11,9,8,10,12,18,17,16 |
| Camera control | XCLK:15, SIOD:4, SIOC:5, VSYNC:6, HREF:7, PCLK:13 |
| SD card | CLK:39, CMD:38, D0:40 |
| USB OTG | D+:20, D-:19 |

## Build & Flash

Requires [ESP-IDF v5.4](https://docs.espressif.com/projects/esp-idf/en/v5.4/esp32s3/get-started/index.html).

```bash
source ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py menuconfig   # Set WiFi SSID/password under "ESP32 FDM Bridge Configuration"
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Or skip `menuconfig` entirely — on first boot with no saved credentials, the board starts in AP mode with a captive portal for WiFi setup.

## Configuration

### Web UI (recommended)

After connecting to WiFi, open `http://<esp32-ip>/settings` to configure:

- **Camera** — rotation, stream/snapshot URLs
- **Printer** — backend (Marlin/Klipper), Moonraker host:port, pause command (M25 vs M524, Marlin only)
- **Obico** — link device with 6-digit code
- **Notifications** — hub URL, and a button to send a test message
- **WiFi** — reset credentials (triggers AP mode on reboot)

### Menuconfig

Additional compile-time options under `ESP32 FDM Bridge Configuration`:

| Option | Default | Description |
|---|---|---|
| WiFi SSID / Password | — | Initial credentials (overridden by NVS after AP setup) |
| Printer Baud Rate | 115200 | USB serial baud rate |
| Obico Server URL | `https://app.obico.io` | Cloud or self-hosted server |
| Snapshot Interval (printing) | 10s | JPEG upload frequency during prints |
| Snapshot Interval (idle) | 120s | JPEG upload frequency when idle |
| WebSocket Status Interval | 30s | Status push frequency |
| Janus Proxy Host/Port | disabled | Sidecar for WebRTC streaming |
| RFC 2217 Port | 2217 | Telnet serial bridge port |
| Hold Timeout | 30 min | Cancel a print waiting for the user (`; @pause`, `M0`/`M1`) after this time |

## OctoPrint Setup

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

## Obico Setup

1. Create an account at [app.obico.io](https://app.obico.io) (or your self-hosted instance)
2. Open `http://<esp32-ip>/settings` and find the Obico section
3. Click "Link" and enter the 6-digit code from the Obico app
4. Done — snapshots upload automatically, AI detection starts on next print

For WebRTC streaming (optional), run the Janus proxy sidecar on a Linux machine on the same network. See `janus_proxy/` for details.

## Notification Hub

`hub/` is a small Python service (standard library only) for a machine on the same LAN: a home server, a NAS, a Raspberry Pi.

1. Create a Telegram bot with [@BotFather](https://t.me/BotFather) and get the chat id
2. Put both in `telegram.env` and start the hub with Docker Compose or systemd — see [hub/README.md](hub/README.md)
3. On each board: **Settings → Notifications → Hub URL**, e.g. `http://192.168.1.10:8787`, then **Send test**
4. Open `http://<hub>:8787/` on your phone for the printer tiles, and add it to the home screen

The hub has no authentication and is meant for a trusted local network: **never expose its port to the internet**.

## Translations

```bash
python3 tools/i18n.py init et Eesti     # start a new language: i18n/et.po
python3 tools/i18n.py update            # after changing text in the code
python3 tools/i18n.py stats             # how much of each language is translated
```

Translate the entries in `i18n/<lang>.po` and build: the build turns the `.po` files into tables. Untranslated text shows in English. The build fails on a translation whose `%d`/`%s` differ from the English, or that adds `"`, `<`, `>`, `&` or `\`; an apostrophe becomes `’` automatically.

In the code, mark text with `_("...")`, or with `N_("...")` inside a page template that goes through `i18n_page()` — see `main/i18n.h`.

## HTTP API

| Endpoint | Method | Description |
|---|---|---|
| `/` | GET | Dashboard |
| `/camera` | GET | Full-screen MJPEG stream |
| `/stream` | GET | Raw MJPEG multipart stream |
| `/capture` | GET | Single JPEG snapshot |
| `/terminal` | GET | GCode terminal |
| `/terminal/send` | POST | Send GCode command |
| `/terminal/poll` | GET | Poll for new printer output |
| `/sd` | GET | SD card file manager |
| `/sd/files` | GET | JSON file listing |
| `/sd/upload` | POST | Upload file to SD |
| `/sd/print` | POST | Start host print |
| `/sd/pause` | POST | Pause print |
| `/sd/resume` | POST | Resume print |
| `/sd/cancel` | POST | Cancel print |
| `/utils` | GET | Utilities index page |
| `/bedlevel` | GET | Bed leveling page |
| `/bedlevel/probe` | POST | Start G28+G29 probing |
| `/bedlevel/status` | GET | Probing state + mesh JSON |
| `/bedlevel/config` | GET/POST | Bed config (size, screws, origin) |
| `/utils/coldpull` | GET/POST | Cold pull page / start it |
| `/print` | GET | Phone page for the current print |
| `/print/continue` | POST | Continue after a stop (`id=<hold id>`) |
| `/print/z` | POST | Babystep Z during a print (`d=±0.01` or `±0.05`) |
| `/print/recent` | GET | JSON list of recent SD files |
| `/print/start` | POST | Print a recent file again (`name=<file>`) |
| `/api/status` | GET | JSON printer status |
| `/api/temp_history` | GET | Temperature history (1h, 4s intervals) |
| `/settings` | GET | Settings page |
| `/printer/config` | GET/POST | Printer backend config |
| `/obico/link` | GET/POST | Obico device linking |
| `/hub/config` | POST | Set the notification hub URL |
| `/hub/test` | POST | Send a test notification |
| `/logs` | GET | Log page (`?filter=printer\|system\|heap`) |
| `/ota/upload` | POST | Firmware update (raw `.bin` body) |

## Architecture

```
Core 0: WiFi, HTTP server, RFC 2217, Obico WS/snapshot
Core 1: USB Host library, CDC-ACM/VCP driver (priority 20)

USB RX → printer_comm_rx_cb() → StreamBuffer → printer_comm_task (parser)
                                └→ terminal ring buffer → browser poll

Browser/Obico → printer_comm_send_cmd() → FreeRTOS queue → USB TX
```

## Project Structure

```
main/
  main.c               — Entry point, init sequence
  wifi.c/h             — STA + AP fallback with captive portal
  camera.c/h           — OV2640/OV3660 driver, Freenove pin mapping, triple-buffered PSRAM
  httpd.c/h            — HTTP server, dashboard, settings, camera pages
  usb_serial.cpp/h     — USB Host CDC-ACM + VCP drivers
  printer_backend.h    — Backend enum (Marlin/Klipper), avoids circular includes
  printer_comm.c/h     — Marlin serial protocol, state machine, host printing, temp history
  printer_comm_klipper.c/h — Klipper/Moonraker HTTP backend, file upload + print
  terminal.c/h         — Web serial terminal with ring buffer (Marlin only)
  sdcard.c/h           — SD card mount/unmount, file operations
  sdcard_httpd.c/h     — SD card web UI, backend-aware print/pause/resume/cancel
  cache.c/h            — LittleFS cache housekeeping (LRU eviction, mtime touch)
  bedlevel.c/h         — Bed leveling: mesh probe, visualization, screw adjustment (Marlin only)
  obico_client.c/h     — Obico WebSocket, snapshot upload, Janus signaling
  rfc2217.c/h          — RFC 2217 Telnet COM-PORT-OPTION server (Marlin only)
  dns_server.c         — Captive portal DNS responder
  layout.h/c           — Shared HTML layout, nav bar (adapts to backend and screen width)
  host_print.c         — Host printing from SD or a built-in script, stops for the user
  gcode_wait.h         — `M0`/`M1` and `; @pause` parsing
  print_page.c/h       — /print phone page, recent files
  cold_pull.c/h        — Cold pull script and page
  hub_client.c/h       — Events and status to the notification hub
  logbuf.c/h           — Log ring in PSRAM that survives resets
  i18n.c/h             — Translations (gettext style)
  Kconfig.projbuild    — Menuconfig options

hub/                   — Notification hub: Telegram, printer tiles (Python, Docker)
i18n/                  — Translations: <lang>.po, messages.pot
tools/i18n.py          — Extracts marked text, updates .po files, builds the tables

janus_proxy/           — Python sidecar for WebRTC streaming via Obico
  proxy.py             — Main orchestrator
  janus_manager.py     — Janus gateway lifecycle
  signaling.py         — TCP ↔ Janus WebSocket bridge
  stream_worker.py     — MJPEG → UDP data channel injector
```

## License

MIT
