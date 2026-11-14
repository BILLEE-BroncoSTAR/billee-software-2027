#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import os
import platform as pyplatform
import re
import shlex
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
REPO_ROOT = HERE.parent
WEB_DIR = HERE / "web"
CATALOG_PATH = HERE / "catalog.json"

PLATFORMS = ("x86-native", "x86-docker", "mac", "linux-aarch64", "l4t")
MAX_LINES = 5000
KEEP_FINISHED = 50
ACTIVE = ("starting", "running", "stopping")
STOP_GRACE_S = (8.0, 4.0)

ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
PLACEHOLDER = re.compile(r"\{\{(\w+)\}\}")
NUMBER = re.compile(r"^-?\d+(\.\d+)?([eE][-+]?\d+)?$")
SAFE_TEXT = re.compile(r"^[\w./:#=+@%,-]+( [\w./:#=+@%,-]+)*$", re.ASCII)
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".svg": "image/svg+xml",
    ".woff2": "font/woff2",
    ".txt": "text/plain; charset=utf-8",
}


class Catalog:
    def __init__(self, path: Path):
        self.path = path
        self._mtime = 0.0
        self._data: dict = {}
        self._lock = threading.Lock()

    @property
    def data(self) -> dict:
        with self._lock:
            mtime = self.path.stat().st_mtime
            if mtime != self._mtime:
                try:
                    self._data = json.loads(self.path.read_text())
                except ValueError as exc:
                    if not self._data:
                        raise
                    print(f"billee-dev: {self.path.name} is not valid JSON ({exc}); keeping the last good copy",
                          file=sys.stderr, flush=True)
                self._mtime = mtime
            return self._data

    def item(self, item_id: str) -> dict | None:
        return next((i for i in self.data["items"] if i["id"] == item_id), None)


class UnknownItem(LookupError):
    pass


class ArgError(ValueError):
    def __init__(self, errors: dict):
        super().__init__("invalid arguments")
        self.errors = errors


def coerce(spec: dict, raw):
    kind = spec["type"]
    default = spec.get("default")
    if kind == "bool":
        if isinstance(raw, bool):
            value = raw
        elif raw in ("true", "false"):
            value = raw == "true"
        else:
            raise ValueError("must be true or false")
        return value, "true" if value else "false"
    if kind == "enum":
        if raw not in spec["choices"]:
            raise ValueError("must be one of: " + ", ".join(c or "(auto)" for c in spec["choices"]))
        return raw, raw
    if kind == "number":
        if isinstance(raw, bool):
            raise ValueError("must be a number")
        text = repr(raw) if isinstance(raw, (int, float)) else str(raw).strip()
        if not NUMBER.match(text):
            raise ValueError("must be a number")
        return float(text), text
    text = "" if raw is None else str(raw)
    if text == default:
        return text, text
    if text == "":
        raise ValueError("can't be empty")
    if not SAFE_TEXT.match(text):
        raise ValueError("letters, digits, spaces and . / : # = + @ % , - _ only")
    if spec.get("style") in ("ros", "env") and " " in text:
        raise ValueError("no spaces")
    return text, text


def resolve_args(item: dict, given) -> dict:
    if given is None:
        given = {}
    if not isinstance(given, dict):
        raise ArgError({"_": "args must be an object"})
    specs = {a["name"]: a for a in item.get("args", [])}
    errors = {name: "not an argument of this item" for name in given if name not in specs}
    out = {}
    for name, spec in specs.items():
        try:
            out[name] = coerce(spec, given.get(name, spec.get("default")))
        except ValueError as exc:
            errors[name] = str(exc)
    if errors:
        raise ArgError(errors)
    return out


def is_default(spec: dict, value) -> bool:
    if spec["type"] == "number":
        return float(value) == float(spec["default"])
    return value == spec.get("default")


def build_command(catalog: dict, item: dict, args: dict, platform: str) -> str:
    template = item["cmd"]
    if item.get("status") == "broken" and item.get("workaround"):
        template = item["workaround"]
    cmd = PLACEHOLDER.sub(lambda m: args[m.group(1)][1], template)
    env, extra = [], []
    for spec in item.get("args", []):
        value, text = args[spec["name"]]
        style = spec.get("style")
        if style == "ros" and not is_default(spec, value):
            extra.append(f"{spec['name']}:={text}")
        elif style == "flag" and value is True:
            extra.append(spec["flag"])
        elif style == "env" and not is_default(spec, value) and value is not False and text != "":
            env.append(f"{spec['env']}={'1' if value is True else text}")
    if extra:
        cmd += " " + " ".join(extra)
    if env:
        cmd = " ".join(env) + " " + cmd
    if item["runner"] == "ros":
        if item.get("shell"):
            cmd = "bash -c " + shlex.quote(cmd)
        cmd = catalog["runners"]["ros"]["templates"][platform].replace("{cmd}", cmd)
    return cmd


def needs_for(catalog: dict, item: dict, args: dict, platform: str) -> list:
    found = [(n, None) for n in item.get("needs", [])]
    found += [(n, None) for n in catalog["runners"].get(item["runner"], {}).get("needs", {}).get(platform, [])]
    for rule in item.get("needsWhen", []):
        if "platform" in rule and platform in rule["platform"]:
            found += [(n, None) for n in rule["needs"]]
        elif "arg" in rule and args[rule["arg"]][0] in rule["is"]:
            found += [(n, f"{rule['arg']} {args[rule['arg']][0]}") for n in rule["needs"]]
    seen, out = set(), []
    for need, via in found:
        if (need, via) not in seen:
            seen.add((need, via))
            out.append((need, via))
    return out


def resources_for(item: dict, args: dict) -> set:
    uses = set(item.get("uses", []))
    for arg_name, extra in item.get("usesWhen", {}).items():
        value = args[arg_name][0]
        if value is True or value == "true":
            uses.update(extra)
    return uses


def mode_of(item: dict) -> str:
    if item["kind"] == "link":
        return "link"
    if item["kind"] == "interactive" or item["runner"] == "host-sudo" or "sudo" in item.get("needs", []):
        return "terminal"
    return "run"


def has_nvidia() -> bool:
    if Path("/etc/nv_tegra_release").exists():
        return True
    for dev in Path("/sys/bus/pci/devices").glob("*"):
        try:
            vendor = (dev / "vendor").read_text().strip()
            pci_class = (dev / "class").read_text().strip()
        except OSError:
            continue
        if vendor == "0x10de" and pci_class.startswith("0x03"):
            return True
    return False


def machine_facts() -> dict:
    system, arch = pyplatform.system(), pyplatform.machine().lower()
    return {
        "nvidia": has_nvidia(),
        "x86-64-linux": system == "Linux" and arch in ("x86_64", "amd64"),
        "arm64-linux": system == "Linux" and arch in ("aarch64", "arm64"),
        "jetson": Path("/etc/nv_tegra_release").exists(),
        "apple-silicon": system == "Darwin" and arch == "arm64",
    }


def zed_connected() -> bool:
    """A Stereolabs camera (USB vendor 2b03) on the USB bus."""
    for vendor in Path("/sys/bus/usb/devices").glob("*/idVendor"):
        try:
            if vendor.read_text().strip() == "2b03":
                return True
        except OSError:
            pass
    return False


def pixi_ok() -> bool:
    return bool(shutil.which("pixi")) or (Path.home() / ".pixi" / "bin" / "pixi").is_file()


def detect_platform() -> str:
    ws = REPO_ROOT / "ros2_ws"
    marker = ws / ".billee-platform"
    recorded = marker.read_text().strip() if marker.exists() else ""
    if recorded not in ("x86", "mac", "linux-aarch64", "l4t"):
        machine = pyplatform.machine().lower()
        if pyplatform.system() == "Darwin":
            recorded = "mac"
        elif machine in ("x86_64", "amd64"):
            recorded = "x86"
        elif Path("/etc/nv_tegra_release").exists():
            recorded = "l4t"
        else:
            recorded = "linux-aarch64"
    if recorded != "x86":
        return recorded
    if not has_nvidia():
        return "x86-native"
    caches = sorted((ws / "build").glob("*/CMakeCache.txt"))
    host_built = bool(caches) and str(ws) in caches[0].read_text(errors="ignore")
    if host_built and (REPO_ROOT / "tooling" / "host-ros2").exists():
        return "x86-native"
    return "x86-docker"


def port_in_use(port: int) -> bool:
    with socket.socket() as sock:
        sock.settimeout(0.3)
        return sock.connect_ex(("127.0.0.1", port)) == 0


_docker = {"at": 0.0, "ok": False}


def docker_ok() -> bool:
    if time.time() - _docker["at"] > 60:
        ok = False
        if shutil.which("docker"):
            try:
                ok = subprocess.run(["docker", "info"], capture_output=True, timeout=5).returncode == 0
            except subprocess.TimeoutExpired:
                ok = False
        _docker.update(at=time.time(), ok=ok)
    return _docker["ok"]


def terminal_argv(script: str) -> list | None:
    if shutil.which("gnome-terminal"):
        return ["gnome-terminal", f"--working-directory={REPO_ROOT}", "--", "bash", "-c", script]
    if shutil.which("x-terminal-emulator"):
        return ["x-terminal-emulator", "-e", "bash", "-c", script]
    if shutil.which("xterm"):
        return ["xterm", "-e", "bash", "-c", script]
    return None


class Run:
    def __init__(self, run_id: str, item: dict, command: str, uses: set):
        self.id = run_id
        self.item = item
        self.command = command
        self.uses = uses
        self.ready = re.compile(item["ready"]) if item.get("ready") else None
        self.status = "starting" if self.ready else "running"
        self.exit_code = None
        self.started = time.time()
        self.ended = None
        self.stop_requested = False
        self.lines: list = []
        self.first = 0
        self.cond = threading.Condition()
        self.proc = None

    def launch(self) -> None:
        env = dict(
            os.environ,
            PYTHONUNBUFFERED="1",
            RCUTILS_LOGGING_BUFFERED_STREAM="0",
            RCUTILS_COLORIZED_OUTPUT="0",
        )
        pixi_bin = Path.home() / ".pixi" / "bin"
        if pixi_bin.is_dir() and str(pixi_bin) not in env.get("PATH", "").split(os.pathsep):
            env["PATH"] = f"{pixi_bin}{os.pathsep}{env.get('PATH', '')}"
        self.proc = subprocess.Popen(
            ["bash", "-c", self.command],
            cwd=REPO_ROOT,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self) -> None:
        for raw in iter(self.proc.stdout.readline, b""):
            line = ANSI.sub("", raw.decode("utf-8", "replace")).rstrip("\r\n")
            with self.cond:
                self.lines.append(line)
                if len(self.lines) > MAX_LINES:
                    drop = len(self.lines) - MAX_LINES
                    del self.lines[:drop]
                    self.first += drop
                if self.status == "starting" and self.ready.search(line):
                    self.status = "running"
                self.cond.notify_all()
        self.proc.stdout.close()
        code = self.proc.wait()
        with self.cond:
            self.exit_code = code
            self.ended = time.time()
            if self.stop_requested:
                self.status = "stopped"
            else:
                self.status = "finished" if code == 0 else "failed"
            self.cond.notify_all()

    def signal_group(self, sig: int) -> None:
        try:
            os.killpg(self.proc.pid, sig)
        except (ProcessLookupError, PermissionError):
            pass

    def stop(self) -> None:
        with self.cond:
            if self.status not in ACTIVE or self.stop_requested:
                return
            self.stop_requested = True
            self.status = "stopping"
            self.cond.notify_all()
        self.signal_group(signal.SIGINT)

        def escalate():
            for sig, wait in ((signal.SIGTERM, STOP_GRACE_S[0]), (signal.SIGKILL, STOP_GRACE_S[1])):
                try:
                    self.proc.wait(timeout=wait)
                    break
                except subprocess.TimeoutExpired:
                    self.signal_group(sig)
            time.sleep(0.5)
            self.signal_group(signal.SIGKILL)

        threading.Thread(target=escalate, daemon=True).start()

    def to_json(self) -> dict:
        with self.cond:
            return {
                "id": self.id,
                "item": self.item["id"],
                "label": self.item["label"],
                "kind": self.item["kind"],
                "command": self.command,
                "status": self.status,
                "exitCode": self.exit_code,
                "started": self.started,
                "ended": self.ended,
                "lines": self.first + len(self.lines),
            }


class Registry:
    def __init__(self, catalog: Catalog):
        self.catalog = catalog
        self.lock = threading.Lock()
        self.runs: dict = {}
        self.counter = 0

    def active(self) -> list:
        with self.lock:
            return [r for r in self.runs.values() if r.status in ACTIVE]

    def holds(self, resource: str) -> bool:
        return any(resource in r.uses for r in self.active())

    def conflict(self, item: dict, uses: set) -> str | None:
        exclusive = set(self.catalog.data.get("exclusive", []))
        for run in self.active():
            if run.item["id"] == item["id"]:
                return f"{item['label']} is already running. Stop it first."
            shared = uses & run.uses & exclusive
            if shared:
                return (f"{item['label']} needs {', '.join(sorted(shared))}, which "
                        f"{run.item['label']} is using. Stop that first.")
        for res in sorted(uses):
            match = re.fullmatch(r"port-(\d+)", res)
            if match and port_in_use(int(match.group(1))):
                return (f"Port {match.group(1)} is already taken by a program this tool didn't "
                        "start (another launch or terminal). Stop it first.")
        return None

    def start(self, item: dict, command: str, uses: set) -> Run:
        with self.lock:
            self.counter += 1
            run = Run(f"r{self.counter}", item, command, uses)
            self.runs[run.id] = run
            finished = [r for r in self.runs.values() if r.status not in ACTIVE]
            for old in finished[:-KEEP_FINISHED]:
                del self.runs[old.id]
        run.launch()
        return run

    def get(self, run_id: str) -> Run | None:
        with self.lock:
            return self.runs.get(run_id)

    def all_json(self) -> list:
        with self.lock:
            runs = list(self.runs.values())
        return [r.to_json() for r in reversed(runs)]

    def stop_all(self, timeout: float = 8.0) -> None:
        runs = self.active()
        for run in runs:
            run.stop()
        deadline = time.time() + timeout
        for run in runs:
            try:
                run.proc.wait(timeout=max(0.1, deadline - time.time()))
            except subprocess.TimeoutExpired:
                run.signal_group(signal.SIGKILL)


class App:
    def __init__(self, platform: str, port: int, catalog_path: Path = CATALOG_PATH):
        self.platform = platform
        self.port = port
        self.catalog = Catalog(catalog_path)
        self.registry = Registry(self.catalog)
        self.machine = machine_facts()
        self.start_lock = threading.Lock()

    def needs_status(self) -> dict:
        net = Path("/sys/class/net")
        dev_input = Path("/dev/input")
        return {
            "display": bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")),
            "vcan0": (net / "vcan0").exists(),
            "can": any(net.glob("can[0-9]*")),
            "zed": zed_connected(),
            "gamepad": dev_input.exists() and any(dev_input.glob("js*")),
            "docker": docker_ok(),
            "terminal": terminal_argv("true") is not None,
            "drivetrain": self.registry.holds("drivetrain") or port_in_use(8765),
            "pixi": pixi_ok(),
            **self.machine,
        }

    def need_block(self, item: dict, args: dict, needs: dict | None = None) -> str | None:
        needs = self.needs_status() if needs is None else needs
        blocking = self.catalog.data.get("blocking", {})
        for need, via in needs_for(self.catalog.data, item, args, self.platform):
            if need in blocking and needs.get(need) is False:
                if via:
                    return f"{via}: {blocking[need]} {item.get('blockedHint', '')}".strip()
                return blocking[need]
        return None

    def prepare(self, body: dict) -> tuple:
        item = self.catalog.item(str(body.get("id", "")))
        if item is None:
            raise UnknownItem(f"no catalog item {body.get('id')!r}")
        args = resolve_args(item, body.get("args"))
        command = build_command(self.catalog.data, item, args, self.platform)
        return item, args, command, resources_for(item, args)

    def refusal(self, item: dict, args: dict, uses: set) -> str | None:
        blocked = self.need_block(item, args)
        if blocked is None and mode_of(item) == "run":
            blocked = self.registry.conflict(item, uses)
        return blocked

    def preview(self, body: dict) -> dict:
        try:
            item, args, command, uses = self.prepare(body)
        except ArgError as exc:
            return {"ok": False, "errors": exc.errors}
        mode = mode_of(item)
        needs = self.needs_status()
        catalog = self.catalog.data
        blocking = catalog.get("blocking", {})
        warnings = [f"Missing here ({n}): {catalog['needs'][n]}"
                    for n, _via in needs_for(catalog, item, args, self.platform)
                    if n not in blocking and needs.get(n) is False]
        blocked = self.need_block(item, args, needs)
        if blocked is None and mode == "run":
            blocked = self.registry.conflict(item, uses)
        return {"ok": True, "command": command, "mode": mode, "uses": sorted(uses),
                "warnings": warnings, "blocked": blocked}


def make_handler(app: App):
    allowed_hosts = {f"127.0.0.1:{app.port}", f"localhost:{app.port}"}
    allowed_origins = {f"http://{h}" for h in allowed_hosts}

    class Handler(BaseHTTPRequestHandler):
        server_version = "billee-dev/1"

        def log_message(self, fmt, *args):
            pass

        def send_json(self, status: int, obj) -> None:
            data = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def host_ok(self) -> bool:
            if self.headers.get("Host") in allowed_hosts:
                return True
            self.send_json(403, {"error": "unexpected Host header"})
            return False

        def do_GET(self):
            if self.host_ok():
                self.guarded(self.get)

        def do_POST(self):
            if self.host_ok():
                self.guarded(self.post)

        def guarded(self, handler):
            try:
                handler()
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                print(f"[billee-dev] {self.command} {self.path} failed: {exc!r}", file=sys.stderr, flush=True)
                self.send_json(500, {"error": f"Server error: {exc}"})

        def get(self):
            url = urlparse(self.path)
            path = url.path
            if path == "/api/info":
                return self.send_json(200, {
                    "platform": app.platform,
                    "platformDescription": app.catalog.data["platforms"].get(app.platform, ""),
                    "repoRoot": str(REPO_ROOT),
                    "needs": app.needs_status(),
                })
            if path == "/api/catalog":
                return self.send_json(200, app.catalog.data)
            if path == "/api/runs":
                return self.send_json(200, app.registry.all_json())
            match = re.fullmatch(r"/api/runs/(r\d+)/log", path)
            if match:
                run = app.registry.get(match.group(1))
                if run is None:
                    return self.send_json(404, {"error": "no such run"})
                query = parse_qs(url.query)
                try:
                    if "from" in query:
                        first = int(query["from"][0])
                    else:
                        first = int(self.headers.get("Last-Event-ID", "-1")) + 1
                except ValueError:
                    first = 0
                return self.stream_log(run, max(0, first))
            return self.serve_static(path)

        def serve_static(self, path: str):
            name = "index.html" if path in ("", "/") else path.lstrip("/")
            target = (WEB_DIR / name).resolve()
            if WEB_DIR not in target.parents or not target.is_file():
                return self.send_json(404, {"error": "not found"})
            data = target.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", CONTENT_TYPES.get(target.suffix, "application/octet-stream"))
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def stream_log(self, run: Run, index: int):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                while True:
                    with run.cond:
                        run.cond.wait_for(
                            lambda: run.first + len(run.lines) > index or run.status not in ACTIVE,
                            timeout=15,
                        )
                        if index < run.first:
                            index = run.first
                        new = run.lines[index - run.first:]
                        done = run.status not in ACTIVE
                    chunk = []
                    for offset, line in enumerate(new):
                        chunk.append(f"id: {index + offset}\ndata: {json.dumps(line)}\n\n")
                    index += len(new)
                    if done:
                        chunk.append(f"event: end\ndata: {json.dumps(run.to_json())}\n\n")
                    elif not new:
                        chunk.append(": keep-alive\n\n")
                    self.wfile.write("".join(chunk).encode())
                    self.wfile.flush()
                    if done:
                        return
            except (BrokenPipeError, ConnectionResetError):
                return

        def post(self):
            origin = self.headers.get("Origin")
            if origin is not None and origin not in allowed_origins:
                return self.send_json(403, {"error": "requests must come from the dev tool page"})
            if not (self.headers.get("Content-Type") or "").startswith("application/json"):
                return self.send_json(415, {"error": "send JSON"})
            try:
                length = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(min(length, 65536)) or b"{}")
                if not isinstance(body, dict):
                    raise ValueError
            except ValueError:
                return self.send_json(400, {"error": "body must be a JSON object"})

            path = urlparse(self.path).path
            try:
                if path == "/api/preview":
                    return self.send_json(200, app.preview(body))
                if path == "/api/start":
                    return self.start(body)
                if path == "/api/stop":
                    run = app.registry.get(str(body.get("run", "")))
                    if run is None:
                        return self.send_json(404, {"error": "no such run"})
                    run.stop()
                    return self.send_json(200, {"run": run.to_json()})
                if path == "/api/terminal":
                    return self.terminal(body)
            except UnknownItem as exc:
                return self.send_json(404, {"error": str(exc)})
            except ArgError as exc:
                return self.send_json(400, {"error": "Check the highlighted arguments.", "errors": exc.errors})
            return self.send_json(404, {"error": "not found"})

        def start(self, body: dict):
            item, args, command, uses = app.prepare(body)
            mode = mode_of(item)
            if mode != "run":
                return self.send_json(409, {"error": f"{item['label']} can't run in the page ({mode})."})
            with app.start_lock:
                blocked = app.refusal(item, args, uses)
                if blocked:
                    return self.send_json(409, {"error": blocked})
                if self.unconfirmed(item, body):
                    return None
                run = app.registry.start(item, command, uses)
            print(f"[billee-dev] {run.id} start {item['id']}: {command}", flush=True)
            return self.send_json(200, {"run": run.to_json()})

        def unconfirmed(self, item: dict, body: dict) -> bool:
            if not item.get("confirm") or body.get("confirmed") is True:
                return False
            about = f"{item['label']}: {item['notes']}" if item.get("notes") else item["label"]
            self.send_json(428, {"error": "confirm", "message": f"{about}\n\nRun it?"})
            return True

        def terminal(self, body: dict):
            item, args, command, uses = app.prepare(body)
            if mode_of(item) == "link":
                return self.send_json(409, {"error": "Links have nothing to run."})
            blocked = app.refusal(item, args, uses)
            if blocked:
                return self.send_json(409, {"error": blocked})
            if self.unconfirmed(item, body):
                return None
            script = (f"cd {shlex.quote(str(REPO_ROOT))} && {command}; echo; "
                      "read -rp '[billee-dev] done. Press Enter to close. ' _")
            argv = terminal_argv(script)
            if argv is None:
                return self.send_json(501, {"error": "No terminal app found. Copy the command instead."})
            proc = subprocess.Popen(argv, cwd=REPO_ROOT, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                    stderr=subprocess.DEVNULL, start_new_session=True)
            threading.Thread(target=proc.wait, daemon=True).start()
            print(f"[billee-dev] terminal {item['id']}: {command}", flush=True)
            return self.send_json(200, {"opened": True, "command": command})

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description="Billee dev tool server")
    parser.add_argument("--port", type=int, default=7700)
    parser.add_argument("--platform", choices=PLATFORMS, help="override detection")
    parser.add_argument("--catalog", type=Path, default=CATALOG_PATH, help=argparse.SUPPRESS)
    opts = parser.parse_args()

    try:
        app = App(opts.platform or detect_platform(), opts.port, opts.catalog)
        if not app.catalog.data.get("items"):
            raise ValueError("no items")
    except (OSError, ValueError) as exc:
        print(f"billee-dev: can't read {opts.catalog}: {exc}", file=sys.stderr)
        return 1
    try:
        httpd = ThreadingHTTPServer(("127.0.0.1", opts.port), make_handler(app))
    except OSError as exc:
        print(f"billee-dev: can't listen on port {opts.port}: {exc.strerror}. "
              "Is the tool already open? Try --port 7701.", file=sys.stderr)
        return 1
    httpd.daemon_threads = True

    signals: list = []
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda signum, _frame: signals.append(signum))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    print(f"billee-dev: http://localhost:{opts.port}  (platform {app.platform}, repo {REPO_ROOT})", flush=True)
    while not signals:
        time.sleep(0.2)
    print("billee-dev: stopping everything it started...", flush=True)
    app.registry.stop_all()
    httpd.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
