"""
Per-printer MJPEG stream fetcher and Janus UDP injector.

Fetches MJPEG multipart stream from ESP32, base64-encodes each frame,
and sends to Janus streaming plugin via UDP (data channel mode).

Wire format for Janus data channel (from moonraker-obico):
  \r\n{b64_len}:{jpg_len}\r\n  (header)
  followed by 1400-byte UDP chunks of base64 data
"""

import base64
import logging
import socket
import threading
import time
import urllib.request

from config import PrinterConfig

log = logging.getLogger(__name__)

CHUNK_SIZE = 1400
CHUNK_DELAY = 0.004  # 4ms between UDP packets
BOUNDARY_MARKER = b"--"


class StreamWorker(threading.Thread):
    def __init__(self, printer: PrinterConfig, target_fps: float = 5.0):
        super().__init__(daemon=True, name=f"stream-{printer.name}")
        self.printer = printer
        self.target_fps = target_fps
        self._min_interval = 1.0 / target_fps
        self._stop_event = threading.Event()
        self._udp_sock: socket.socket | None = None
        self._janus_addr = ("127.0.0.1", printer.data_port)

    def stop(self):
        self._stop_event.set()

    def run(self):
        log.info("[%s] Stream worker started (→ UDP %d, target %.1f FPS)",
                 self.printer.name, self.printer.data_port, self.target_fps)

        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while not self._stop_event.is_set():
            try:
                self._stream_loop()
            except Exception as e:
                log.warning("[%s] Stream error: %s, retrying in 5s",
                            self.printer.name, e)
                time.sleep(5)

        if self._udp_sock:
            self._udp_sock.close()

    def _stream_loop(self):
        """Connect to ESP32 MJPEG stream and forward frames."""
        url = f"http://{self.printer.ip}:{self.printer.stream_port}/stream"
        log.info("[%s] Connecting to %s", self.printer.name, url)

        req = urllib.request.Request(url)
        resp = urllib.request.urlopen(req, timeout=10)

        # Read multipart MJPEG stream
        boundary = None
        content_type = resp.headers.get("Content-Type", "")
        if "boundary=" in content_type:
            boundary = content_type.split("boundary=")[1].strip().encode()

        buf = b""
        last_frame_time = 0.0

        while not self._stop_event.is_set():
            chunk = resp.read(4096)
            if not chunk:
                break

            buf += chunk

            # Find JPEG frames between boundaries
            while True:
                # Find start of JPEG (after headers)
                jpeg_start = buf.find(b"\xff\xd8")
                if jpeg_start < 0:
                    break

                # Find end of JPEG
                jpeg_end = buf.find(b"\xff\xd9", jpeg_start + 2)
                if jpeg_end < 0:
                    break

                jpeg_end += 2  # include the marker
                jpeg_data = buf[jpeg_start:jpeg_end]
                buf = buf[jpeg_end:]

                # Rate limit
                now = time.monotonic()
                if now - last_frame_time < self._min_interval:
                    continue
                last_frame_time = now

                self._inject_frame(jpeg_data)

    def _inject_frame(self, jpeg_data: bytes):
        """Base64-encode a JPEG frame and send to Janus via UDP."""
        b64_data = base64.b64encode(jpeg_data)
        b64_len = len(b64_data)
        jpg_len = len(jpeg_data)

        # Header: \r\n{b64_len}:{jpg_len}\r\n
        header = f"\r\n{b64_len}:{jpg_len}\r\n".encode()

        try:
            self._udp_sock.sendto(header, self._janus_addr)

            # Send base64 data in chunks
            offset = 0
            while offset < b64_len:
                end = min(offset + CHUNK_SIZE, b64_len)
                self._udp_sock.sendto(b64_data[offset:end], self._janus_addr)
                offset = end
                if offset < b64_len:
                    time.sleep(CHUNK_DELAY)

        except OSError as e:
            log.warning("[%s] UDP send error: %s", self.printer.name, e)
