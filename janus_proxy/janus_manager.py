"""
Janus gateway process lifecycle management.

Spawns Janus, monitors for crashes, and maintains a WebSocket connection
to the local Janus instance for signaling relay.
"""

import asyncio
import json
import logging
import os
import signal
import subprocess
import shutil

import websockets

log = logging.getLogger(__name__)

JANUS_WS_SUBPROTOCOL = "janus-protocol"


class JanusManager:
    def __init__(self, config_dir: str, ws_port: int = 17730):
        self.config_dir = config_dir
        self.ws_port = ws_port
        self.ws_uri = f"ws://127.0.0.1:{ws_port}"
        self._process: subprocess.Popen | None = None
        self._ws: websockets.WebSocketClientProtocol | None = None
        self._ws_lock = asyncio.Lock()
        self._msg_callbacks: list = []  # (session_id) -> callback
        self._running = False

    def _kill_orphaned(self):
        """Kill any leftover Janus processes."""
        try:
            result = subprocess.run(
                ["pkill", "-f", f"janus.*{self.config_dir}"],
                capture_output=True, timeout=5
            )
            if result.returncode == 0:
                log.info("Killed orphaned Janus process")
        except Exception:
            pass

    def start_process(self):
        """Spawn the Janus gateway process."""
        self._kill_orphaned()

        janus_bin = shutil.which("janus")
        if not janus_bin:
            raise RuntimeError("Janus binary not found in PATH. Install with: apt install janus")

        cmd = [
            janus_bin,
            f"--configs-folder={self.config_dir}",
            "--stun-server=stun.l.google.com:19302",
        ]

        log.info("Starting Janus: %s", " ".join(cmd))
        self._process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        self._running = True

    def stop_process(self):
        """Stop the Janus process."""
        self._running = False
        if self._process:
            log.info("Stopping Janus (pid %d)", self._process.pid)
            self._process.send_signal(signal.SIGTERM)
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
            self._process = None

    async def connect_ws(self, max_retries: int = 30, retry_delay: float = 1.0):
        """Connect to Janus WebSocket, retrying until it's ready."""
        for attempt in range(max_retries):
            try:
                self._ws = await websockets.connect(
                    self.ws_uri,
                    subprotocols=[JANUS_WS_SUBPROTOCOL],
                    ping_interval=30,
                    ping_timeout=10,
                )
                log.info("Connected to Janus WebSocket at %s", self.ws_uri)
                return
            except (ConnectionRefusedError, OSError) as e:
                if attempt < max_retries - 1:
                    await asyncio.sleep(retry_delay)
                else:
                    raise RuntimeError(
                        f"Failed to connect to Janus WS after {max_retries} attempts"
                    ) from e

    async def send_to_janus(self, msg: dict) -> None:
        """Send a JSON message to Janus via WebSocket."""
        async with self._ws_lock:
            if self._ws:
                await self._ws.send(json.dumps(msg))

    async def recv_from_janus(self) -> dict | None:
        """Receive a JSON message from Janus WebSocket."""
        if not self._ws:
            return None
        try:
            raw = await self._ws.recv()
            return json.loads(raw)
        except websockets.ConnectionClosed:
            log.warning("Janus WebSocket closed")
            self._ws = None
            return None

    async def get_info(self) -> dict | None:
        """Send an info request and return the response."""
        import secrets
        txn = secrets.token_hex(8)
        await self.send_to_janus({"janus": "info", "transaction": txn})
        # Simple: just read next message (in production, match by transaction)
        return await self.recv_from_janus()

    @property
    def is_alive(self) -> bool:
        return self._process is not None and self._process.poll() is None

    async def monitor_loop(self):
        """Monitor Janus process and restart if it crashes."""
        while self._running:
            if not self.is_alive:
                log.error("Janus process died, restarting...")
                self.start_process()
                await asyncio.sleep(2)
                await self.connect_ws()
            await asyncio.sleep(5)
