from __future__ import annotations

import json
import os
import socket
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

DEVTOOL = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DEVTOOL))

import server  # noqa: E402

PLATFORM_TEMPLATES = {p: f"env BILLEE_TEST_RUNNER={p} {{cmd}}" for p in server.PLATFORMS}


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def fixture_catalog(busy_port: int) -> dict:
    host = {"kind": "task", "runner": "host", "group": "basic", "status": "verified"}
    return {
        "schema": 1,
        "kinds": {k: k for k in ("task", "service", "stream", "gui", "interactive", "link")},
        "runners": {
            "ros": {"label": "ROS", "templates": PLATFORM_TEMPLATES, "needs": {"x86-native": ["pixi"]}},
            "host": {"label": "Host", "templates": {"*": "{cmd}"}},
            "host-sudo": {"label": "Host + sudo", "templates": {"*": "{cmd}"}},
            "link": {"label": "Link", "templates": {"*": "{cmd}"}},
        },
        "platforms": {p: f"{p} test platform" for p in server.PLATFORMS},
        "needs": {
            "display": "A display.",
            "nvidia": "An NVIDIA GPU.",
            "pixi": "Pixi.",
            "sudo": "Sudo.",
            "terminal": "A terminal.",
            "drivetrain": "A drivetrain.",
        },
        "blocking": {"nvidia": "This machine has no NVIDIA GPU.", "pixi": "Pixi isn't installed."},
        "resources": {"lock": "One at a time.", f"port-{busy_port}": "A port.", "shared": "Shared."},
        "exclusive": ["lock", f"port-{busy_port}"],
        "groups": [
            {"id": "basic", "label": "Basics", "description": "Quick tasks."},
            {"id": "long", "label": "Services", "description": "Things that keep running."},
            {"id": "other", "label": "Other", "description": "Terminals and links."},
        ],
        "items": [
            {**host, "id": "t.echo", "label": "Say hello", "cmd": "echo hello {{word}}",
             "args": [{"name": "word", "type": "string", "default": "world", "style": "template",
                       "help": "Who to greet"}],
             "notes": "Prints a greeting. Second sentence."},
            {**host, "id": "t.fail", "label": "Fail on purpose", "shell": True, "cmd": "echo bad; exit 3",
             "status": "untested"},
            {**host, "id": "t.lines", "label": "Many lines", "cmd": "seq 1 {{n}}",
             "args": [{"name": "n", "type": "number", "default": 50, "style": "template"}]},
            {**host, "id": "t.ros", "label": "ROS task", "runner": "ros", "cmd": "echo ros {{n}}",
             "args": [
                 {"name": "n", "type": "number", "default": 1, "style": "template"},
                 {"name": "rviz", "type": "bool", "default": False, "style": "ros"},
                 {"name": "mode", "type": "enum", "choices": ["real", "mock"], "default": "real", "style": "ros"},
                 {"name": "loop", "type": "bool", "default": False, "style": "flag", "flag": "--loop"},
                 {"name": "MODE", "type": "enum", "choices": ["", "a", "b"], "default": "", "style": "env",
                  "env": "MODE"},
                 {"name": "FAST", "type": "bool", "default": False, "style": "env", "env": "FAST"},
             ]},
            {**host, "id": "t.shell", "label": "Shell ROS task", "runner": "ros", "shell": True,
             "cmd": "echo 'a b' && echo $HOME"},
            {**host, "id": "t.broken", "label": "Broken with workaround", "status": "broken",
             "cmd": "false", "workaround": "echo workaround {{x}}",
             "args": [{"name": "x", "type": "string", "default": "1", "style": "template"}]},
            {**host, "id": "t.confirm", "label": "Dangerous", "confirm": True, "cmd": "echo confirmed",
             "notes": "Deletes nothing, really."},
            {**host, "id": "t.gpu", "label": "Needs a GPU", "cmd": "echo gpu", "needs": ["nvidia"]},
            {**host, "id": "t.display", "label": "Wants a display", "cmd": "echo display", "needs": ["display"]},
            {**host, "id": "t.choice", "label": "Pick a target", "cmd": "echo {{target}}",
             "args": [{"name": "target", "type": "enum", "choices": ["cpu", "gpu"], "default": "cpu",
                       "style": "template"}],
             "needsWhen": [{"arg": "target", "is": ["gpu"], "needs": ["nvidia"]}],
             "blockedHint": "Pick cpu."},
            {**host, "id": "s.svc", "group": "long", "label": "Slow service", "kind": "service",
             "shell": True, "cmd": "echo booting; sleep 0.4; echo READY now; sleep 60",
             "ready": "READY", "uses": ["lock"]},
            {**host, "id": "s.other", "group": "long", "label": "Rival service", "kind": "service",
             "cmd": "sleep 60", "uses": ["lock"]},
            {**host, "id": "s.optional", "group": "long", "label": "Optional lock", "kind": "service",
             "cmd": "sleep 60", "args": [{"name": "grab", "type": "bool", "default": False, "style": "env",
                                          "env": "GRAB"}],
             "usesWhen": {"grab": ["lock"]}},
            {**host, "id": "s.stubborn", "group": "long", "label": "Ignores Ctrl-C", "kind": "service",
             "shell": True, "cmd": "trap '' INT; echo up; sleep 60 & wait"},
            {**host, "id": "s.port", "group": "long", "label": "Needs a busy port", "kind": "service",
             "cmd": "sleep 60", "uses": [f"port-{busy_port}"]},
            {**host, "id": "s.stream", "group": "long", "label": "Ticker", "kind": "stream", "shell": True,
             "cmd": "i=0; while true; do i=$((i+1)); echo tick $i; sleep 0.1; done"},
            {**host, "id": "i.term", "group": "other", "label": "Interactive", "kind": "interactive",
             "cmd": "read -r x"},
            {**host, "id": "i.sudo", "group": "other", "label": "Sudo thing", "runner": "host-sudo",
             "confirm": True, "cmd": "echo sudo", "needs": ["sudo"]},
            {**host, "id": "l.web", "group": "other", "label": "A web page", "kind": "link", "runner": "link",
             "cmd": "http://localhost:9/"},
            {**host, "id": "l.copy", "group": "other", "label": "A websocket", "kind": "link", "runner": "link",
             "cmd": "ws://localhost:8765"},
        ],
        "notBootable": [],
    }


class TestServer:
    """A real billee-dev server on a free port, serving a fixture catalog."""

    def __init__(self, needs: dict | None = None, catalog: dict | None = None):
        self.busy = socket.socket()
        self.busy.bind(("127.0.0.1", 0))
        self.busy.listen()
        self.busy_port = self.busy.getsockname()[1]
        self.tmp = tempfile.TemporaryDirectory()
        self.catalog_path = Path(self.tmp.name) / "catalog.json"
        self.write_catalog(catalog or fixture_catalog(self.busy_port))
        self.port = free_port()
        self.base = f"http://127.0.0.1:{self.port}"
        self.app = server.App("x86-native", self.port, self.catalog_path)
        base_needs = {k: True for k in ("display", "vcan0", "can", "zed", "gamepad", "docker", "terminal", "pixi",
                                        "x86-64-linux")}
        base_needs.update(nvidia=False, drivetrain=False, jetson=False)
        base_needs.update(needs or {})
        self.needs = base_needs
        self.app.needs_status = lambda: dict(self.needs)
        self.httpd = ThreadingHTTPServer(("127.0.0.1", self.port), server.make_handler(self.app))
        self.httpd.daemon_threads = True
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def write_catalog(self, data) -> None:
        text = data if isinstance(data, str) else json.dumps(data)
        self.catalog_path.write_text(text)
        self._bump = getattr(self, "_bump", 0) + 1
        stamp = time.time() + self._bump
        os.utime(self.catalog_path, (stamp, stamp))

    def close(self) -> None:
        self.app.registry.stop_all(timeout=5)
        self.httpd.shutdown()
        self.httpd.server_close()
        self.busy.close()
        self.tmp.cleanup()

    def call(self, path: str, body=None, headers: dict | None = None, raw: bytes | None = None):
        data = raw if raw is not None else (None if body is None else json.dumps(body).encode())
        req = urllib.request.Request(self.base + path, data=data, method="GET" if data is None else "POST")
        if data is not None and not any(k.lower() == "content-type" for k in (headers or {})):
            req.add_header("Content-Type", "application/json")
        for key, value in (headers or {}).items():
            req.add_header(key, value)
        try:
            with urllib.request.urlopen(req, timeout=10) as res:
                return res.status, self._decode(res)
        except urllib.error.HTTPError as err:
            return err.code, self._decode(err)

    @staticmethod
    def _decode(res):
        body = res.read()
        if "json" in (res.headers.get("Content-Type") or ""):
            return json.loads(body or b"null")
        return body

    def start(self, item_id: str, args: dict | None = None, **extra):
        return self.call("/api/start", {"id": item_id, "args": args or {}, **extra})

    def run(self, run_id: str) -> dict:
        return next(r for r in self.call("/api/runs")[1] if r["id"] == run_id)

    def wait(self, run_id: str, wanted: set, timeout: float = 10) -> str:
        deadline = time.time() + timeout
        status = "?"
        while time.time() < deadline:
            status = self.run(run_id)["status"]
            if status in wanted:
                return status
            time.sleep(0.05)
        return status

    def log(self, run_id: str, query: str = "", headers: dict | None = None, until_end: bool = True,
            max_lines: int | None = None) -> tuple:
        req = urllib.request.Request(f"{self.base}/api/runs/{run_id}/log{query}")
        for key, value in (headers or {}).items():
            req.add_header(key, value)
        lines, ids, end = [], [], None
        with urllib.request.urlopen(req, timeout=20) as res:
            event = None
            for raw in res:
                line = raw.decode().rstrip("\n")
                if line.startswith("id: "):
                    ids.append(int(line[4:]))
                elif line.startswith("event: "):
                    event = line[7:]
                elif line.startswith("data: "):
                    if event == "end":
                        end = json.loads(line[6:])
                        break
                    lines.append(json.loads(line[6:]))
                    if max_lines and len(lines) >= max_lines and not until_end:
                        break
                elif line == "":
                    event = None
        return lines, ids, end
