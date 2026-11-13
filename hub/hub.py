#!/usr/bin/env python3
"""Notification hub for ESP32 FDM bridge boards.

Boards POST events here and this service forwards them to Telegram, so the bot
credentials live in one place. It is meant for a trusted local network and has
no authentication: never expose the port to the internet.
"""

import argparse
import ipaddress
import json
import logging
import os
import re
import tempfile
import threading
import time
import urllib.error
import urllib.request
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

log = logging.getLogger("hub")

DEFAULT_ENV = Path(__file__).resolve().parent.parent / "telegram.env"
DEFAULT_PORT = 8787
TELEGRAM_API = "https://api.telegram.org"
TELEGRAM_TIMEOUT_S = 6  # Shorter than the board's HTTP timeout
MAX_BODY = 16 * 1024
SEEN_EVENTS_MAX = 256

HOSTNAME_RE = re.compile(r"^[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?$")
DEVICE_ID_RE = re.compile(r"^[0-9a-f]{12}$")
EVENT_TYPES = ("hold", "hold_timeout", "test")

# Board status, for the dashboard
STATES = ("idle", "printing", "paused", "hold", "error", "no_printer")
DEFAULT_INTERVAL_S = 10
MISSED_REPORTS = 3          # A board is offline after this many missed reports
SAVE_LAST_SEEN_S = 300      # How stale the saved "last seen" may get
MAX_PRINTERS = 64
STATUS_STRINGS = {"filename": 64, "hold_msg": 64, "version": 16}
STATUS_NUMBERS = ("progress", "time_left_s", "hotend", "hotend_target", "bed", "bed_target")


class TelegramError(Exception):
    pass


def load_env(path):
    """Read KEY=VALUE lines. Values already in the environment take priority."""
    values = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.removeprefix("export ").strip()] = value.strip().strip("\"'")
    values.update({k: v for k, v in os.environ.items() if k in values})
    return values


def device_link(device, domain):
    """Link to the board's print page, or None if the event has no usable address."""
    name = str(device.get("name") or "")
    if domain and HOSTNAME_RE.match(name):
        return f"http://{name}.{domain}/print"
    try:
        return f"http://{ipaddress.IPv4Address(str(device.get('ip') or ''))}/print"
    except ValueError:
        return None


def format_message(event, domain):
    device = event.get("device") or {}
    name = str(device.get("name") or device.get("id") or "printer")
    kind = event["type"]
    text = str(event.get("message") or "")
    filename = str(event.get("filename") or "")

    if kind == "hold":
        lines = [f"⏸ {name} ждёт вас", text]
    elif kind == "hold_timeout":
        lines = [f"⛔ {name}: никто не продолжил, печать отменена, нагрев выключен"]
    else:
        lines = [f"✅ {name}: тест уведомлений"]

    if filename:
        lines.append(f"Файл: {filename}")
    if kind != "hold_timeout":
        link = device_link(device, domain)
        if link:
            lines.append(link)
    return "\n".join(line for line in lines if line)


class Telegram:
    def __init__(self, token, chat_id, api_url=TELEGRAM_API, thread_id=None):
        self._url = f"{api_url}/bot{token}/sendMessage"
        self._chat_id = chat_id
        self._thread_id = thread_id

    def send(self, text):
        payload = {"chat_id": self._chat_id, "text": text, "disable_web_page_preview": True}
        if self._thread_id:
            payload["message_thread_id"] = self._thread_id
        request = urllib.request.Request(
            self._url,
            data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"},
        )
        # Errors are built from Telegram's description only: the request URL
        # contains the token and must never reach the logs.
        try:
            with urllib.request.urlopen(request, timeout=TELEGRAM_TIMEOUT_S):
                return
        except urllib.error.HTTPError as err:
            try:
                reply = json.load(err)
            except ValueError:
                reply = {}
            raise TelegramError(
                f"HTTP {err.code}: {reply.get('description')} {reply.get('parameters') or ''}"
            ) from None
        except (urllib.error.URLError, OSError) as err:
            reason = getattr(err, "reason", err)
            raise TelegramError(f"network error: {type(reason).__name__}") from None


class Hub:
    def __init__(self, telegram, domain=""):
        self._telegram = telegram
        self._domain = domain
        self._seen = OrderedDict()
        # One at a time, so a board's retry cannot race the send it is retrying.
        self._lock = threading.Lock()

    def handle_event(self, event):
        event_id = str(event.get("event_id") or "")
        with self._lock:
            if event_id and event_id in self._seen:
                log.info("Duplicate event %s ignored", event_id)
                return
            self._telegram.send(format_message(event, self._domain))
            if event_id:
                self._seen[event_id] = True
                while len(self._seen) > SEEN_EVENTS_MAX:
                    self._seen.popitem(last=False)


DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#263238">
<link rel="manifest" href="/manifest.webmanifest">
<link rel="icon" href="/icon.svg" type="image/svg+xml">
<link rel="apple-touch-icon" href="/icon.svg">
<title>Принтеры</title>
<style>
:root{color-scheme:light dark;--bg:#f4f5f7;--fg:#222;--dim:#777;
--idle:#dcefdc;--idle-fg:#1b5e20;--run:#e3ebf5;--run-fill:#b9d0ec;--run-fg:#0d3c6e;
--pause:#fff1c2;--pause-fg:#6d5200;--hold:#ef6c00;--hold-fg:#fff;--err:#e53935;--err-fg:#fff;
--nop:#eceff1;--nop-line:#90a4ae;--nop-fg:#546e7a;--off:#dcdcdc;--off-fg:#888}
@media(prefers-color-scheme:dark){:root{--bg:#121212;--fg:#eee;--dim:#999;
--idle:#1d3a22;--idle-fg:#a5d6a7;--run:#18273a;--run-fill:#25486f;--run-fg:#cfe3fb;
--pause:#3b3211;--pause-fg:#ffe082;--hold:#e65100;--hold-fg:#fff;--err:#c62828;--err-fg:#fff;
--nop:#1e2326;--nop-line:#546e7a;--nop-fg:#90a4ae;--off:#262626;--off-fg:#777}}
*{box-sizing:border-box}
body{margin:0;padding:12px 12px max(12px,env(safe-area-inset-bottom));background:var(--bg);color:var(--fg);
font-family:system-ui,-apple-system,sans-serif}
header{display:flex;justify-content:space-between;align-items:baseline;margin:0 2px 10px}
h1{font-size:18px;margin:0}
#up{font-size:12px;color:var(--dim)}
#err{background:var(--err);color:var(--err-fg);border-radius:8px;padding:8px 10px;margin-bottom:10px;font-size:14px}
#grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
@media(min-width:600px){#grid{grid-template-columns:repeat(4,1fr);gap:10px}}
@media(min-width:1000px){#grid{grid-template-columns:repeat(6,1fr)}}
.t{position:relative;display:flex;flex-direction:column;min-height:96px;padding:9px 9px 8px;border-radius:10px;
color:inherit;text-decoration:none;overflow:hidden;-webkit-tap-highlight-color:transparent;
transition:transform .08s,filter .08s}
a.t:active{transform:scale(.96);filter:brightness(.9)}
.n{font-weight:700;font-size:16px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.s{font-size:13px;margin-top:2px;font-weight:600}
.pc{font-size:24px;font-weight:700;line-height:1.1;margin-top:2px}
.m{font-size:12px;margin-top:3px;opacity:.85;overflow:hidden;display:-webkit-box;-webkit-line-clamp:2;
-webkit-box-orient:vertical;word-break:break-word}
.x{font-size:11px;margin-top:auto;padding-top:4px;opacity:.75;white-space:nowrap}
.idle{background:var(--idle);color:var(--idle-fg)}
.printing{color:var(--run-fg);background:linear-gradient(to right,var(--run-fill) var(--p,0%),var(--run) var(--p,0%))}
.paused{background:var(--pause);color:var(--pause-fg)}
.hold{background:var(--hold);color:var(--hold-fg);animation:pulse 1.6s ease-in-out infinite}
.error{background:var(--err);color:var(--err-fg)}
.no_printer{background:var(--nop);color:var(--nop-fg);box-shadow:inset 0 0 0 2px var(--nop-line)}
.offline{background:var(--off);color:var(--off-fg)}
@keyframes pulse{50%{filter:brightness(1.18)}}
@media(prefers-reduced-motion:reduce){.hold{animation:none}}
.rm{position:absolute;top:2px;right:2px;border:0;background:none;color:inherit;font-size:16px;
padding:4px 7px;opacity:.6;cursor:pointer}
#empty{color:var(--dim);font-size:14px;line-height:1.5}
</style></head><body>
<header><h1>Принтеры</h1><span id="up"></span></header>
<div id="err" hidden>Хаб не отвечает, данные могут быть устаревшими</div>
<div id="grid"></div>
<p id="empty" hidden>Пока ни одна плата не прислала статус. На плате: Settings &rarr; Notifications &rarr; Hub URL.</p>
<script>
var LABEL={idle:"свободен",printing:"печать",paused:"пауза",hold:"ждёт вас",error:"ошибка",
no_printer:"принтер не подключён",offline:"нет связи"};
function $(id){return document.getElementById(id)}
function el(tag,cls,text){var e=document.createElement(tag);if(cls)e.className=cls;
if(text!=null)e.textContent=text;return e}
function ago(s){if(s==null)return"";if(s<90)return s+" с";var m=Math.round(s/60);
if(m<90)return m+" мин";var h=Math.round(s/3600);return h<48?h+" ч":Math.round(s/86400)+" дн"}
function left(s){if(s==null)return"";var m=Math.round(s/60);
return m<60?"ещё "+m+" мин":"ещё "+Math.floor(m/60)+" ч "+(m%60)+" мин"}
function temps(p){if(p.hotend==null)return"";var t="сопло "+p.hotend+"\u00b0";
if(p.bed!=null)t+=" \u00b7 стол "+p.bed+"\u00b0";return t}
function tile(p){
 var st=p.state in LABEL?p.state:"offline";
 var t=el(p.online&&p.link?"a":"div","t "+st);
 if(p.online&&p.link)t.href=p.link;
 t.appendChild(el("div","n",p.name));
 var active=st=="printing"||st=="paused"||st=="hold";
 var pct=typeof p.progress=="number"?Math.floor(p.progress):null;
 if(st=="printing"&&p.preparing)t.appendChild(el("div","s","подготовка"));
 else t.appendChild(el("div","s",LABEL[st]));
 if(active&&pct!=null&&!p.preparing){t.appendChild(el("div","pc",pct+"%"));
  if(st=="printing")t.style.setProperty("--p",pct+"%")}
 if(st=="hold")t.appendChild(el("div","m",p.hold_msg||""));
 else if(st=="offline")t.appendChild(el("div","m",p.seen_s_ago!=null?ago(p.seen_s_ago)+" назад":""));
 else if(active)t.appendChild(el("div","m",st=="printing"&&p.time_left_s!=null?left(p.time_left_s):(p.filename||"")));
 if(p.online)t.appendChild(el("div","x",temps(p)));
 if(!p.online){var b=el("button","rm","\u00d7");b.title="Убрать из списка";
  b.onclick=function(){if(confirm("Убрать "+p.name+" из списка? Плата вернётся, если снова пришлёт статус."))
   fetch("/api/v1/printers/"+p.id,{method:"DELETE"}).then(load)};t.appendChild(b)}
 return t}
function draw(list){
 /* Boards waiting for the user first, the rest keep their place by name */
 list.sort(function(a,b){return(b.state=="hold")-(a.state=="hold")||a.name.localeCompare(b.name)});
 var g=$("grid");g.textContent="";list.forEach(function(p){g.appendChild(tile(p))});
 $("empty").hidden=list.length>0;
 var n=list.filter(function(p){return p.state=="hold"}).length;
 document.title=(n?"("+n+") ":"")+"Принтеры";
 var now=new Date();$("up").textContent=now.getHours()+":"+("0"+now.getMinutes()).slice(-2)}
var busy=false;
function load(){if(busy)return;busy=true;
 fetch("/api/v1/printers",{cache:"no-store"}).then(function(r){if(!r.ok)throw r;return r.json()})
 .then(function(d){$("err").hidden=true;draw(d.printers)})
 .catch(function(){$("err").hidden=false})
 .then(function(){busy=false})}
load();setInterval(load,3000);
document.addEventListener("visibilitychange",function(){if(!document.hidden)load()});
</script></body></html>
"""

MANIFEST = {
    "name": "Принтеры", "short_name": "Принтеры", "start_url": "/", "display": "standalone",
    "background_color": "#121212", "theme_color": "#263238",
    "icons": [{"src": "/icon.svg", "sizes": "any", "type": "image/svg+xml"}],
}

ICON_SVG = """<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 64 64">
<rect width="64" height="64" rx="14" fill="#263238"/>
<rect x="10" y="10" width="20" height="20" rx="4" fill="#81c784"/>
<rect x="34" y="10" width="20" height="20" rx="4" fill="#64b5f6"/>
<rect x="10" y="34" width="20" height="20" rx="4" fill="#ffb74d"/>
<rect x="34" y="34" width="20" height="20" rx="4" fill="#90a4ae"/>
</svg>
"""


class Printers:
    """Last status of every board that has reported, kept in memory.

    Names, addresses and when each board was last seen are also saved to a file,
    if one is given, so that a board that is switched off still shows up as
    offline after the hub restarts.
    """

    def __init__(self, path=None, clock=time.monotonic, wall=time.time):
        self._path = Path(path) if path else None
        self._clock, self._wall = clock, wall
        self._lock = threading.Lock()
        self._boards = {}     # id -> {"device", "status", "received", "seen"}
        self._saved_at = 0.0
        self._load()

    def _load(self):
        if not self._path or not self._path.exists():
            return
        try:
            saved = json.loads(self._path.read_text(encoding="utf-8"))
            for board in saved.get("printers", []):
                device = board["device"]
                if DEVICE_ID_RE.match(device.get("id", "")):
                    self._boards[device["id"]] = {
                        "device": device, "status": None, "received": None,
                        "seen": float(board.get("seen") or 0),
                    }
        except (OSError, ValueError, KeyError, TypeError) as err:
            log.warning("Could not read %s: %s", self._path, err)

    def _save(self):
        """Write the file in one step, so a crash cannot leave half of it.  Lock held."""
        if not self._path:
            return
        data = {"printers": [{"device": b["device"], "seen": b["seen"]}
                             for b in self._boards.values()]}
        try:
            fd, tmp = tempfile.mkstemp(dir=self._path.parent, prefix=".printers-")
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=1)
            os.replace(tmp, self._path)
            self._saved_at = self._clock()
        except OSError as err:
            log.warning("Could not save %s: %s", self._path, err)

    def update(self, status):
        device = status["device"]
        now = self._clock()
        with self._lock:
            board = self._boards.get(device["id"])
            if board is None:
                if len(self._boards) >= MAX_PRINTERS:
                    raise ValueError("too many printers")
                log.info("New printer %s (%s)", device["name"], device["ip"])
                board = self._boards[device["id"]] = {"device": device}
                save = True
            else:
                save = (board["device"] != device or
                        now - self._saved_at > SAVE_LAST_SEEN_S)
            board.update(device=device, status=status, received=now, seen=self._wall())
            if save:
                self._save()

    def remove(self, device_id):
        """Forget a board, e.g. one taken out of service.  It comes back if it reports."""
        with self._lock:
            if self._boards.pop(device_id, None) is None:
                return False
            self._save()
            return True

    def snapshot(self, domain=""):
        now = self._clock()
        wall = self._wall()
        out = []
        with self._lock:
            boards = [dict(b) for b in self._boards.values()]
        for b in boards:
            device, status = b["device"], b["status"]
            item = {
                "id": device["id"], "name": device["name"], "ip": device["ip"],
                "link": device_link(device, domain),
                "seen_s_ago": max(0, int(wall - b["seen"])) if b["seen"] else None,
            }
            online = False
            if status is not None:
                interval = status.get("interval_s") or DEFAULT_INTERVAL_S
                online = now - b["received"] <= interval * MISSED_REPORTS + 2
            item["online"] = online
            if online:
                item.update({k: v for k, v in status.items()
                             if k not in ("device", "interval_s")})
            else:
                item["state"] = "offline"
            out.append(item)
        out.sort(key=lambda i: i["name"].lower())
        return out


def clean_status(status):
    """Return the status reduced to known fields, or raise ValueError."""
    if not isinstance(status, dict):
        raise ValueError("status must be an object")
    device = status.get("device")
    if not isinstance(device, dict):
        raise ValueError("device is required")
    device_id = str(device.get("id") or "")
    name = str(device.get("name") or "")
    if not DEVICE_ID_RE.match(device_id):
        raise ValueError("bad device id")
    if not HOSTNAME_RE.match(name):
        name = device_id
    try:
        ip = str(ipaddress.IPv4Address(str(device.get("ip") or "")))
    except ValueError:
        raise ValueError("bad device ip") from None
    if status.get("state") not in STATES:
        raise ValueError("unknown state")

    out = {"device": {"id": device_id, "name": name, "ip": ip}, "state": status["state"]}
    interval = status.get("interval_s")
    if isinstance(interval, (int, float)) and not isinstance(interval, bool) and 1 <= interval <= 600:
        out["interval_s"] = interval
    for key, limit in STATUS_STRINGS.items():
        if isinstance(status.get(key), str):
            out[key] = status[key][:limit]
    for key in STATUS_NUMBERS:
        value = status.get(key)
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            out[key] = value
    if isinstance(status.get("preparing"), bool):
        out["preparing"] = status["preparing"]
    return out


def validate_event(event):
    if not isinstance(event, dict):
        return "event must be an object"
    if event.get("type") not in EVENT_TYPES:
        return "unknown event type"
    if not isinstance(event.get("device"), dict):
        return "device is required"
    return None


def make_handler(hub, printers=None, domain=""):
    printers = printers if printers is not None else Printers()

    class Handler(BaseHTTPRequestHandler):
        server_version = "fdmhub"
        protocol_version = "HTTP/1.1"
        timeout = 15

        def _reply(self, status, **body):
            data = json.dumps(body).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _send(self, status, content_type, data, cache="no-store"):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", cache)
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path == "/healthz":
                self._reply(200, ok=True)
            elif path == "/api/v1/printers":
                self._reply(200, ok=True, printers=printers.snapshot(domain))
            elif path in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", DASHBOARD_HTML.encode())
            elif path == "/manifest.webmanifest":
                self._send(200, "application/manifest+json",
                           json.dumps(MANIFEST, ensure_ascii=False).encode(), "max-age=86400")
            elif path == "/icon.svg":
                self._send(200, "image/svg+xml", ICON_SVG.encode(), "max-age=86400")
            else:
                self._reply(404, ok=False, error="not found")

        def do_DELETE(self):
            prefix = "/api/v1/printers/"
            device_id = self.path[len(prefix):] if self.path.startswith(prefix) else ""
            if not DEVICE_ID_RE.match(device_id):
                self._reply(404, ok=False, error="not found")
            elif printers.remove(device_id):
                log.info("Printer %s removed from the list", device_id)
                self._reply(200, ok=True)
            else:
                self._reply(404, ok=False, error="unknown printer")

        def do_POST(self):
            try:
                length = int(self.headers.get("Content-Length", ""))
            except ValueError:
                length = -1
            if length < 0 or length > MAX_BODY:
                self.close_connection = True
                self._reply(400, ok=False, error="bad length")
                return
            body = self.rfile.read(length)

            if self.path == "/api/v1/status":
                self._post_status(body)
                return
            if self.path != "/api/v1/events":
                self._reply(404, ok=False, error="not found")
                return
            try:
                event = json.loads(body)
            except ValueError:
                self._reply(400, ok=False, error="bad json")
                return
            problem = validate_event(event)
            if problem:
                self._reply(400, ok=False, error=problem)
                return

            device = event["device"]
            try:
                hub.handle_event(event)
            except TelegramError as err:
                log.error("%s from %s not delivered: %s", event["type"], device.get("name"), err)
                self._reply(502, ok=False, error="telegram failed")
                return
            log.info("%s from %s delivered", event["type"], device.get("name"))
            self._reply(200, ok=True)

        def _post_status(self, body):
            try:
                status = clean_status(json.loads(body))
                printers.update(status)
            except ValueError as err:   # json.JSONDecodeError is a ValueError too
                self._reply(400, ok=False, error=str(err))
                return
            self._reply(200, ok=True)

        def log_message(self, fmt, *args):
            log.debug("%s %s", self.address_string(), fmt % args)

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--env", default=str(DEFAULT_ENV), help="settings file")
    parser.add_argument("--state", help="file to keep the list of printers in")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    env = load_env(args.env)
    missing = [k for k in ("TELEGRAM_BOT_TOKEN", "TELEGRAM_CHAT_ID") if not env.get(k)]
    if missing:
        parser.error(f"{args.env} is missing: {', '.join(missing)}")

    port = int(env.get("HUB_PORT") or DEFAULT_PORT)
    telegram = Telegram(
        env["TELEGRAM_BOT_TOKEN"],
        env["TELEGRAM_CHAT_ID"],
        api_url=env.get("TELEGRAM_API_URL") or TELEGRAM_API,
        thread_id=env.get("TELEGRAM_THREAD_ID") or None,
    )
    domain = env.get("PRINTER_DOMAIN", "")
    hub = Hub(telegram, domain=domain)
    printers = Printers(args.state or env.get("HUB_STATE_FILE") or None)

    server = ThreadingHTTPServer(("0.0.0.0", port), make_handler(hub, printers, domain))
    log.info("Listening on port %d", port)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
