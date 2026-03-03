# ESP32 FDM Printer Bridge

WiFi-to-printer bridge firmware for **Freenove ESP32-S3-WROOM** (OV2640 camera, dual USB-C).
Designed for OctoPrint integration.

## Features

- **MJPEG video stream** from OV2640 (`/stream`, `/capture`)
- **USB Host** serial bridge to FDM printer (CDC-ACM + CH34x/CP210x/FT23x VCP)
- **RFC 2217 server** — OctoPrint connects via `rfc2217://<ip>:2217`

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
  main.c            — Entry point: NVS → WiFi → Camera → HTTP → USB Host → RFC 2217
  wifi.c/h          — WiFi STA, EventGroup blocking, auto-reconnect (10 retries)
  camera.c/h        — OV2640, Freenove pin mapping, VGA JPEG, 2 PSRAM buffers
  httpd.c/h         — HTTP server :80, /stream (MJPEG multipart), /capture (JPEG)
  usb_serial.cpp/h  — USB Host CDC-ACM + VCP drivers (C++ for VCP headers, extern "C" API)
  rfc2217.c/h       — RFC 2217 Telnet COM-PORT-OPTION server, IAC state machine
  Kconfig.projbuild — Menuconfig: WiFi SSID/pass, RFC2217 port, baud rate
  idf_component.yml — Component dependencies
```

## Hardware

- **Right USB-C:** UART bridge — flashing/debug
- **Left USB-C:** ESP32-S3 native USB (GPIO19/20) — OTG Host to printer
- **Camera:** OV2640 on DVP (GPIOs 4-18, no conflict with USB)
- **PSRAM:** 8MB OPI — camera frame buffers

## Core Affinity

- **Core 0:** WiFi, HTTP server, RFC 2217 server + TX drain
- **Core 1:** USB Host lib, device poll, CDC-ACM driver (priority 20)

## Architecture Notes

- `usb_serial.cpp` must be C++ because VCP driver headers are `.hpp`
- C++ exceptions enabled (`CONFIG_COMPILER_CXX_EXCEPTIONS=y`) — required by VCP component
- Camera XCLK at 20 MHz (not 16) to avoid PSRAM bus contention
- RFC 2217 uses FreeRTOS StreamBuffer to decouple USB RX callback from TCP send
- Single RFC 2217 client at a time (matches OctoPrint usage pattern)
- OTG port may not supply 5V VBUS — user may need powered USB hub

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
