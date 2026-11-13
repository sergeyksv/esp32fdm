#!/usr/bin/env python3
"""Tests for the hub, against a fake Telegram server. Run: python3 hub/test_hub.py"""

import json
import logging
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hub as hubmod  # noqa: E402

TOKEN = "123:SECRET-TOKEN"


class FakeTelegram:
    def __init__(self):
        self.messages, self.paths = [], []
        self.fail_next = 0
        outer = self

        class H(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                outer.paths.append(self.path)
                if outer.fail_next > 0:
                    outer.fail_next -= 1
                    data = json.dumps({"ok": False, "description": "chat not found"}).encode()
                    self.send_response(400)
                else:
                    outer.messages.append(body)
                    data = b'{"ok":true}'
                    self.send_response(200)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

            def log_message(self, *a):
                pass

        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.srv.server_port}"


def event(**kw):
    ev = {
        "event_id": "dev-1",
        "type": "hold",
        "device": {"id": "e072a1d5606c", "name": "fdm03", "ip": "192.168.3.131"},
        "message": "Проверьте высоту сопла",
        "filename": "benchy.gcode",
    }
    ev.update(kw)
    return ev


class HubTest(unittest.TestCase):
    def setUp(self):
        self.tg = FakeTelegram()
        hub = hubmod.Hub(hubmod.Telegram(TOKEN, "-5", api_url=self.tg.url), domain="lan")
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), hubmod.make_handler(hub))
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.base = f"http://127.0.0.1:{self.srv.server_port}"
        for srv in (self.srv, self.tg.srv):
            self.addCleanup(srv.server_close)
            self.addCleanup(srv.shutdown)

    def post(self, body, path="/api/v1/events"):
        data = body if isinstance(body, bytes) else json.dumps(body).encode()
        try:
            with urllib.request.urlopen(urllib.request.Request(self.base + path, data=data)) as r:
                return r.status
        except urllib.error.HTTPError as e:
            return e.code

    def test_hold_uses_short_name_link(self):
        self.assertEqual(self.post(event()), 200)
        msg = self.tg.messages[0]
        self.assertEqual(msg["chat_id"], "-5")
        self.assertIn("fdm03", msg["text"])
        self.assertIn("Проверьте высоту сопла", msg["text"])
        self.assertIn("benchy.gcode", msg["text"])
        self.assertIn("http://fdm03.lan/print", msg["text"])

    def test_timeout_has_no_link_to_act_on(self):
        self.assertEqual(self.post(event(type="hold_timeout", message="")), 200)
        self.assertIn("отменена", self.tg.messages[0]["text"])
        self.assertNotIn("http://", self.tg.messages[0]["text"])

    def test_retry_is_not_sent_twice(self):
        self.assertEqual(self.post(event()), 200)
        self.assertEqual(self.post(event()), 200)
        self.assertEqual(len(self.tg.messages), 1)
        self.assertEqual(self.post(event(event_id="dev-2")), 200)
        self.assertEqual(len(self.tg.messages), 2)

    def test_failed_send_can_be_retried(self):
        self.tg.fail_next = 1
        self.assertEqual(self.post(event()), 502)
        self.assertEqual(self.post(event()), 200)
        self.assertEqual(len(self.tg.messages), 1)

    def test_bad_requests(self):
        self.assertEqual(self.post(b"{not json"), 400)
        self.assertEqual(self.post(event(type="nope")), 400)
        self.assertEqual(self.post({"type": "hold"}), 400)
        self.assertEqual(self.post(b"[]"), 400)
        self.assertEqual(self.post(event(), path="/other"), 404)
        self.assertEqual(self.tg.messages, [])

    def test_oversized_body_rejected(self):
        self.assertEqual(self.post(b"x" * (hubmod.MAX_BODY + 1)), 400)

    def test_token_not_in_logs_on_failure(self):
        self.tg.fail_next = 1
        with self.assertLogs("hub", level=logging.ERROR) as logs:
            self.post(event())
        joined = "\n".join(logs.output)
        self.assertIn("chat not found", joined)
        self.assertNotIn(TOKEN, joined)
        self.assertNotIn("SECRET", joined)

    def test_network_error_does_not_leak_token(self):
        tele = hubmod.Telegram(TOKEN, "-5", api_url="http://127.0.0.1:9")
        with self.assertRaises(hubmod.TelegramError) as ctx:
            tele.send("x")
        self.assertNotIn("SECRET", str(ctx.exception))


class LinkTest(unittest.TestCase):
    dev = {"name": "fdm03", "ip": "192.168.3.131"}

    def test_short_name(self):
        self.assertEqual(hubmod.device_link(self.dev, "lan"), "http://fdm03.lan/print")

    def test_no_domain_uses_ip(self):
        self.assertEqual(hubmod.device_link(self.dev, ""), "http://192.168.3.131/print")

    def test_bad_name_cannot_redirect_the_link(self):
        for bad in ("evil.com/x?", "a b", "-x", "x" * 64, ""):
            self.assertEqual(
                hubmod.device_link({"name": bad, "ip": "192.168.3.131"}, "lan"),
                "http://192.168.3.131/print",
            )

    def test_no_usable_address(self):
        self.assertIsNone(hubmod.device_link({"name": "", "ip": "evil.com"}, "lan"))


def status(**kw):
    st = {
        "device": {"id": "e072a1d5606c", "name": "fdm03", "ip": "192.168.3.131"},
        "version": "2.3", "interval_s": 10, "state": "printing",
        "filename": "benchy.gcode", "progress": 42.5, "time_left_s": 600,
        "hotend": 210, "hotend_target": 210, "bed": 60, "bed_target": 60,
    }
    st.update(kw)
    return st


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


class PrintersTest(unittest.TestCase):
    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.dir.cleanup)
        self.path = Path(self.dir.name) / "printers.json"
        self.clock, self.wall = Clock(), Clock()

    def make(self):
        return hubmod.Printers(self.path, clock=self.clock, wall=self.wall)

    def test_status_then_offline(self):
        p = self.make()
        p.update(hubmod.clean_status(status()))
        [item] = p.snapshot("lan")
        self.assertTrue(item["online"])
        self.assertEqual(item["state"], "printing")
        self.assertEqual(item["progress"], 42.5)
        self.assertEqual(item["link"], "http://fdm03.lan/print")
        self.assertNotIn("device", item)

        self.clock.t += 40
        self.wall.t += 40
        [item] = p.snapshot("lan")
        self.assertFalse(item["online"])
        self.assertEqual(item["state"], "offline")
        self.assertNotIn("progress", item)      # A stale percentage would mislead
        self.assertEqual(item["seen_s_ago"], 40)

    def test_offline_board_survives_restart(self):
        self.make().update(hubmod.clean_status(status()))
        self.wall.t += 3600
        [item] = self.make().snapshot("")
        self.assertEqual(item["name"], "fdm03")
        self.assertEqual(item["state"], "offline")
        self.assertEqual(item["seen_s_ago"], 3600)

    def test_rename_is_saved_at_once(self):
        p = self.make()
        p.update(hubmod.clean_status(status()))
        dev = {"id": "e072a1d5606c", "name": "fdm90", "ip": "192.168.3.204"}
        p.update(hubmod.clean_status(status(device=dev)))
        [item] = self.make().snapshot("")
        self.assertEqual((item["name"], item["ip"]), ("fdm90", "192.168.3.204"))

    def test_sorted_by_name(self):
        p = self.make()
        for i, name in enumerate(("fdm06", "fdm03", "FDM05")):
            dev = {"id": f"00000000000{i}", "name": name, "ip": f"10.0.0.{i + 1}"}
            p.update(hubmod.clean_status(status(device=dev)))
        self.assertEqual([i["name"] for i in p.snapshot()], ["fdm03", "FDM05", "fdm06"])

    def test_corrupt_file_is_ignored(self):
        self.path.write_text("{oops")
        with self.assertLogs("hub", level=logging.WARNING):
            self.assertEqual(self.make().snapshot(), [])

    def test_clean_status(self):
        clean = hubmod.clean_status(status(extra="x", progress="50", hold_msg="m" * 500,
                                           preparing=1))
        self.assertNotIn("extra", clean)
        self.assertNotIn("progress", clean)
        self.assertNotIn("preparing", clean)
        self.assertEqual(len(clean["hold_msg"]), 64)
        bad_name = status(device={"id": "e072a1d5606c", "name": "<b>", "ip": "10.0.0.1"})
        self.assertEqual(hubmod.clean_status(bad_name)["device"]["name"], "e072a1d5606c")
        for bad in (status(state="burning"),
                    status(device={"id": "../x", "name": "a", "ip": "10.0.0.1"}),
                    status(device={"id": "e072a1d5606c", "name": "a", "ip": "evil"}),
                    status(device=None), []):
            with self.assertRaises(ValueError):
                hubmod.clean_status(bad)


class StatusHttpTest(unittest.TestCase):
    def setUp(self):
        hub = hubmod.Hub(hubmod.Telegram(TOKEN, "-5", api_url="http://127.0.0.1:9"))
        srv = ThreadingHTTPServer(("127.0.0.1", 0), hubmod.make_handler(hub, domain="lan"))
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.addCleanup(srv.server_close)
        self.addCleanup(srv.shutdown)
        self.base = f"http://127.0.0.1:{srv.server_port}"

    def test_status_round_trip(self):
        req = urllib.request.Request(self.base + "/api/v1/status",
                                     data=json.dumps(status(state="hold", hold_msg="Z?")).encode())
        with urllib.request.urlopen(req) as r:
            self.assertEqual(r.status, 200)
        with urllib.request.urlopen(self.base + "/api/v1/printers") as r:
            [item] = json.load(r)["printers"]
        self.assertEqual((item["state"], item["hold_msg"]), ("hold", "Z?"))

    def test_dashboard_and_forget(self):
        with urllib.request.urlopen(self.base + "/?x=1") as r:
            self.assertIn("text/html", r.headers["Content-Type"])
            self.assertIn("/api/v1/printers", r.read().decode())
        with urllib.request.urlopen(self.base + "/manifest.webmanifest") as r:
            self.assertEqual(json.load(r)["start_url"], "/")
        urllib.request.urlopen(urllib.request.Request(
            self.base + "/api/v1/status", data=json.dumps(status()).encode()))
        delete = urllib.request.Request(self.base + "/api/v1/printers/e072a1d5606c", method="DELETE")
        with urllib.request.urlopen(delete) as r:
            self.assertEqual(r.status, 200)
        with urllib.request.urlopen(self.base + "/api/v1/printers") as r:
            self.assertEqual(json.load(r)["printers"], [])
        for path in ("/api/v1/printers/e072a1d5606c", "/api/v1/printers/../x"):
            with self.assertRaises(urllib.error.HTTPError) as ctx:
                urllib.request.urlopen(urllib.request.Request(self.base + path, method="DELETE"))
            self.assertEqual(ctx.exception.code, 404)

    def test_bad_status_rejected(self):
        req = urllib.request.Request(self.base + "/api/v1/status", data=b'{"state":"idle"}')
        with self.assertRaises(urllib.error.HTTPError) as ctx:
            urllib.request.urlopen(req)
        self.assertEqual(ctx.exception.code, 400)


if __name__ == "__main__":
    unittest.main()
