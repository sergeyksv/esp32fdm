"""
Per-printer signaling bridge: ESP32 TCP ↔ Janus WebSocket.

Each ESP32 connects via TCP to its assigned port. Messages are
length-prefixed (4-byte big-endian length + JSON payload).

The bridge forwards:
  ESP32 TCP RX → parse JSON → send to Janus WS
  Janus WS RX  → serialize JSON → send to ESP32 TCP
"""

import asyncio
import json
import logging
import struct

from config import PrinterConfig
from janus_manager import JanusManager

log = logging.getLogger(__name__)


class SignalingBridge:
    """Bridges one ESP32's Janus signaling over TCP ↔ Janus WS."""

    def __init__(self, printer: PrinterConfig, janus: JanusManager):
        self.printer = printer
        self.janus = janus
        self._server: asyncio.Server | None = None
        self._esp_writer: asyncio.StreamWriter | None = None
        self._janus_session_id: int | None = None
        self._janus_handle_id: int | None = None

    async def start(self):
        """Start TCP server listening for ESP32 connection."""
        self._server = await asyncio.start_server(
            self._handle_esp_connection,
            "0.0.0.0",
            self.printer.tcp_port,
        )
        log.info("[%s] Signaling bridge listening on TCP port %d",
                 self.printer.name, self.printer.tcp_port)

    async def stop(self):
        if self._server:
            self._server.close()
            await self._server.wait_closed()

    async def _handle_esp_connection(self, reader: asyncio.StreamReader,
                                     writer: asyncio.StreamWriter):
        """Handle a single ESP32 TCP connection."""
        addr = writer.get_extra_info("peername")
        log.info("[%s] ESP32 connected from %s", self.printer.name, addr)
        self._esp_writer = writer

        try:
            while True:
                # Read length-prefixed message
                hdr = await reader.readexactly(4)
                msg_len = struct.unpack(">I", hdr)[0]
                if msg_len <= 0 or msg_len > 65536:
                    log.warning("[%s] Bad message length: %d", self.printer.name, msg_len)
                    break

                data = await reader.readexactly(msg_len)
                msg_str = data.decode("utf-8")

                log.debug("[%s] ESP32→Janus: %s", self.printer.name, msg_str[:200])

                # Parse and forward to Janus
                try:
                    msg = json.loads(msg_str)
                    await self.janus.send_to_janus(msg)
                except json.JSONDecodeError:
                    log.warning("[%s] Invalid JSON from ESP32: %s",
                                self.printer.name, msg_str[:100])

        except asyncio.IncompleteReadError:
            log.info("[%s] ESP32 disconnected", self.printer.name)
        except Exception as e:
            log.warning("[%s] ESP32 connection error: %s", self.printer.name, e)
        finally:
            writer.close()
            self._esp_writer = None

    async def forward_to_esp(self, msg: dict):
        """Send a Janus message to the connected ESP32."""
        if not self._esp_writer:
            return

        try:
            data = json.dumps(msg).encode("utf-8")
            hdr = struct.pack(">I", len(data))
            self._esp_writer.write(hdr + data)
            await self._esp_writer.drain()
        except Exception as e:
            log.warning("[%s] Failed to send to ESP32: %s", self.printer.name, e)
            self._esp_writer = None


class SignalingRouter:
    """
    Routes Janus WS messages to the correct printer's SignalingBridge.

    Since all signaling goes through one WS connection to Janus,
    we route responses by matching session_id to printer.
    """

    def __init__(self, janus: JanusManager):
        self.janus = janus
        self._bridges: dict[str, SignalingBridge] = {}  # name -> bridge
        self._session_to_printer: dict[int, str] = {}  # janus session_id -> printer name

    def add_bridge(self, bridge: SignalingBridge):
        self._bridges[bridge.printer.name] = bridge

    def register_session(self, session_id: int, printer_name: str):
        """Map a Janus session to a printer for routing responses."""
        self._session_to_printer[session_id] = printer_name
        log.info("Registered Janus session %d for %s", session_id, printer_name)

    async def run_janus_rx_loop(self):
        """Read messages from Janus WS and route to correct bridge."""
        while True:
            msg = await self.janus.recv_from_janus()
            if msg is None:
                log.warning("Janus WS disconnected, waiting for reconnect...")
                await asyncio.sleep(2)
                continue

            # Try to route by session_id
            session_id = msg.get("session_id")
            if session_id and session_id in self._session_to_printer:
                printer_name = self._session_to_printer[session_id]
                bridge = self._bridges.get(printer_name)
                if bridge:
                    await bridge.forward_to_esp(msg)
                    continue

            # Broadcast to all bridges if we can't route
            # (e.g., session create responses before we know the mapping)
            sender_txn = msg.get("transaction", "")
            for name, bridge in self._bridges.items():
                # For now, forward to all — the ESP32 will ignore irrelevant ones
                await bridge.forward_to_esp(msg)
