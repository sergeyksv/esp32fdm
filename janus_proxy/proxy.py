#!/usr/bin/env python3
"""
Janus/WebRTC streaming proxy sidecar for ESP32 FDM printers.

Manages a local Janus gateway to provide WebRTC streaming for Obico UI.
One instance handles multiple ESP32 boards on the same LAN.

Usage:
    python proxy.py --printer 192.168.1.101 --printer 192.168.1.102
    python proxy.py --config printers.yaml
"""

import argparse
import asyncio
import logging
import os
import signal
import sys
import tempfile

import yaml

from config import PrinterConfig, build_printer_list, generate_janus_configs
from janus_manager import JanusManager
from signaling import SignalingBridge, SignalingRouter
from stream_worker import StreamWorker

log = logging.getLogger("janus_proxy")


def load_config_file(path: str) -> list[PrinterConfig]:
    """Load printer list from YAML config file."""
    with open(path) as f:
        data = yaml.safe_load(f)

    printers = []
    for i, entry in enumerate(data.get("printers", [])):
        printers.append(PrinterConfig(
            name=entry.get("name", f"printer{i+1}"),
            ip=entry["ip"],
            stream_id=entry.get("stream_id", 2 + i * 100),
            data_port=entry.get("data_port", 17732 + i * 100),
            tcp_port=entry.get("tcp_port", 17800 + i),
            stream_port=entry.get("stream_port", 80),
        ))
    return printers


async def main(printers: list[PrinterConfig], ws_port: int = 17730,
               target_fps: float = 5.0):
    # Create temp dir for Janus configs
    config_dir = os.path.join(tempfile.gettempdir(), "janus_proxy_configs")
    generate_janus_configs(config_dir, printers, ws_port=ws_port)
    log.info("Generated Janus configs in %s", config_dir)

    # Start Janus
    janus = JanusManager(config_dir, ws_port=ws_port)
    janus.start_process()
    await asyncio.sleep(2)  # Give Janus time to initialize
    await janus.connect_ws()

    # Verify Janus is running
    info = await janus.get_info()
    if info:
        log.info("Janus info: %s", info.get("janus", "unknown"))
    else:
        log.error("Failed to get Janus info — is it running?")

    # Start signaling bridges
    router = SignalingRouter(janus)
    for printer in printers:
        bridge = SignalingBridge(printer, janus)
        await bridge.start()
        router.add_bridge(bridge)

    # Start stream workers
    workers = []
    for printer in printers:
        worker = StreamWorker(printer, target_fps=target_fps)
        worker.start()
        workers.append(worker)

    # Run signaling router + Janus monitor
    stop_event = asyncio.Event()

    def handle_signal():
        log.info("Shutting down...")
        stop_event.set()

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, handle_signal)

    try:
        # Run Janus WS rx loop and monitor concurrently; stop when signal received
        tasks = [
            asyncio.create_task(router.run_janus_rx_loop()),
            asyncio.create_task(janus.monitor_loop()),
        ]
        await stop_event.wait()
        for t in tasks:
            t.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
    except asyncio.CancelledError:
        pass
    finally:
        # Cleanup
        for worker in workers:
            worker.stop()
        for worker in workers:
            worker.join(timeout=5)
        for bridge in router._bridges.values():
            await bridge.stop()
        janus.stop_process()


def parse_args():
    parser = argparse.ArgumentParser(
        description="Janus/WebRTC proxy sidecar for ESP32 FDM printers"
    )
    parser.add_argument(
        "--printer", action="append", metavar="IP",
        help="ESP32 printer IP address (can be specified multiple times)"
    )
    parser.add_argument(
        "--config", metavar="FILE",
        help="YAML config file with printer definitions"
    )
    parser.add_argument(
        "--ws-port", type=int, default=17730,
        help="Janus WebSocket port (default: 17730)"
    )
    parser.add_argument(
        "--fps", type=float, default=5.0,
        help="Target stream FPS (default: 5.0)"
    )
    parser.add_argument(
        "--log-level", default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level (default: INFO)"
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    if args.config:
        printers = load_config_file(args.config)
    elif args.printer:
        printers = build_printer_list(args.printer)
    else:
        print("Error: specify --printer IP or --config FILE", file=sys.stderr)
        sys.exit(1)

    log.info("Managing %d printer(s):", len(printers))
    for p in printers:
        log.info("  %s: ip=%s stream_id=%d data_port=%d tcp_port=%d",
                 p.name, p.ip, p.stream_id, p.data_port, p.tcp_port)

    asyncio.run(main(printers, ws_port=args.ws_port, target_fps=args.fps))
