# ESP32 FDM Printer Bridge

WiFi-to-printer bridge firmware for **Freenove ESP32-S3-WROOM** (OV2640/OV3660 camera, dual USB-C).
Dual backend: Marlin (USB serial) or Klipper (Moonraker HTTP).

## Features

- **MJPEG video stream** from OV2640/OV3660 (`/stream`, `/capture`)
- **USB Host** serial bridge to FDM printer (CDC-ACM + CH34x/CP210x/FT23x VCP)
- **Marlin backend**: direct serial, host printing from SD, GCode terminal
- **Klipper backend**: Moonraker HTTP API, file upload + print, temp/progress tracking
- **Obico integration**: AI failure detection, remote monitoring
- **RFC 2217 server** — OctoPrint connects via `rfc2217://<ip>:2217` (Marlin only)

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
  obico_client.c/h      — Obico WebSocket, snapshot upload, Janus signaling
  rfc2217.c/h           — RFC 2217 Telnet server (Marlin only)
  layout.h              — Shared HTML layout, nav bar (hides Marlin-only items for Klipper)
  Kconfig.projbuild     — Menuconfig options
```

## Hardware

- **Right USB-C:** UART bridge — flashing/debug
- **Left USB-C:** ESP32-S3 native USB (GPIO19/20) — OTG Host to printer
- **Camera:** OV2640 or OV3660 on DVP (GPIOs 4-18, no conflict with USB)
- **PSRAM:** 8MB OPI — camera frame buffers

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
