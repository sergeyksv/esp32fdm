"""
Printer registry and Janus config file generation.

Generates the three config files Janus needs:
  - janus.jcfg              (main config)
  - janus.plugin.streaming.jcfg  (one mountpoint per printer)
  - janus.transport.websockets.jcfg
"""

import os
import textwrap
from dataclasses import dataclass, field


@dataclass
class PrinterConfig:
    name: str
    ip: str
    stream_id: int        # Janus streaming mountpoint ID
    data_port: int        # UDP port for MJPEG data injection
    tcp_port: int         # TCP port ESP32 connects to for signaling relay
    stream_port: int = 80 # HTTP port on ESP32 for /stream


def build_printer_list(ips: list[str], base_stream_id: int = 2,
                       base_data_port: int = 17732,
                       base_tcp_port: int = 17800) -> list[PrinterConfig]:
    """Build printer configs from a list of IP addresses."""
    printers = []
    for i, ip in enumerate(ips):
        printers.append(PrinterConfig(
            name=f"printer{i+1}",
            ip=ip,
            stream_id=base_stream_id + i * 100,
            data_port=base_data_port + i * 100,
            tcp_port=base_tcp_port + i,
        ))
    return printers


def generate_janus_configs(config_dir: str, printers: list[PrinterConfig],
                           ws_port: int = 17730,
                           stun_server: str = "stun.l.google.com:19302"):
    """Write the three Janus config files to config_dir."""
    os.makedirs(config_dir, exist_ok=True)

    # --- janus.jcfg ---
    janus_cfg = textwrap.dedent(f"""\
        general: {{
            configs_folder = "{config_dir}"
            plugins_folder = "/usr/lib/x86_64-linux-gnu/janus/plugins"
            transports_folder = "/usr/lib/x86_64-linux-gnu/janus/transports"
            log_level = 4
            admin_secret = "janus_proxy_admin"
        }}

        nat: {{
            stun_server = "{stun_server.split(':')[0]}"
            stun_port = {stun_server.split(':')[1] if ':' in stun_server else 19302}
            nice_debug = false
            ice_lite = false
        }}

        media: {{
        }}

        plugins: {{
            disable = "libjanus_audiobridge.so,libjanus_echotest.so,libjanus_nosip.so,libjanus_recordplay.so,libjanus_sip.so,libjanus_textroom.so,libjanus_videocall.so,libjanus_videoroom.so,libjanus_voicemail.so"
        }}

        transports: {{
            disable = "libjanus_http.so,libjanus_pfunix.so,libjanus_rabbitmq.so,libjanus_mqtt.so,libjanus_nanomsg.so"
        }}
    """)

    with open(os.path.join(config_dir, "janus.jcfg"), "w") as f:
        f.write(janus_cfg)

    # --- janus.transport.websockets.jcfg ---
    ws_cfg = textwrap.dedent(f"""\
        general: {{
            ws = true
            ws_port = {ws_port}
            ws_interface = "lo"
            ws_ip = "127.0.0.1"
            wss = false
        }}

        admin: {{
            admin_ws = false
        }}
    """)

    with open(os.path.join(config_dir, "janus.transport.websockets.jcfg"), "w") as f:
        f.write(ws_cfg)

    # --- janus.plugin.streaming.jcfg ---
    mountpoints = []
    for p in printers:
        mp = textwrap.dedent(f"""\
            {p.name}: {{
                type = "rtp"
                id = {p.stream_id}
                description = "{p.name} MJPEG stream"
                is_private = false

                data = true
                datatype = "text"
                dataport = {p.data_port}
                dataiface = "lo"
                databuffermsg = true
            }}
        """)
        mountpoints.append(mp)

    streaming_cfg = "\n".join(mountpoints)

    with open(os.path.join(config_dir, "janus.plugin.streaming.jcfg"), "w") as f:
        f.write(streaming_cfg)

    return config_dir
