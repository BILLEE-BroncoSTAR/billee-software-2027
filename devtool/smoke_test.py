#!/usr/bin/env python3

from __future__ import annotations

import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

PORT = 7799
BASE = f"http://127.0.0.1:{PORT}"
HERE = Path(__file__).resolve().parent
failures = 0


def check(name: str, ok: bool, detail: str = "") -> None:
    global failures
    failures += not ok
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""), flush=True)


def call(path: str, body=None, headers=None):
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(BASE + path, data=data, method="GET" if body is None else "POST")
    if body is not None:
        req.add_header("Content-Type", "application/json")
    for key, value in (headers or {}).items():
        req.add_header(key, value)
    try:
        with urllib.request.urlopen(req, timeout=10) as res:
            return res.status, json.loads(res.read() or b"null")
    except urllib.error.HTTPError as err:
        return err.code, json.loads(err.read() or b"null")


def preview(item_id: str, args: dict) -> dict:
    return call("/api/preview", {"id": item_id, "args": args})[1]


def start(item_id: str, args: dict | None = None, **extra):
    return call("/api/start", {"id": item_id, "args": args or {}, **extra})


def wait_status(run_id: str, wanted: set, timeout: float) -> str:
    deadline = time.time() + timeout
    status = "?"
    while time.time() < deadline:
        status = next(r["status"] for r in call("/api/runs")[1] if r["id"] == run_id)
        if status in wanted:
            return status
        time.sleep(0.3)
    return status


def log_of(run_id: str) -> str:
    lines = []
    with urllib.request.urlopen(f"{BASE}/api/runs/{run_id}/log", timeout=60) as res:
        for raw in res:
            line = raw.decode().rstrip("\n")
            if line.startswith("data: "):
                lines.append(json.loads(line[6:]))
            if line.startswith("event: end"):
                break
    return "\n".join(lines)


def main() -> int:
    server = subprocess.Popen([sys.executable, str(HERE / "server.py"), "--port", str(PORT)])
    try:
        for _ in range(50):
            try:
                status, info = call("/api/info")
                break
            except OSError:
                time.sleep(0.1)
        else:
            print("server didn't start")
            return 1
        check("server answers", status == 200, f"platform {info['platform']}")

        p = preview("drive.pub", {"v": "0.25", "w": "-0.5", "rate": "20"})
        check("number args go into the command", p["ok"] and "x: 0.25" in p["command"]
              and "z: -0.5" in p["command"] and "-r 20" in p["command"], p.get("command", ""))
        p = preview("drivetrain.mock", {"rviz": True, "gear_ratio": "50"})
        check("ros args appended only when changed", p["ok"] and p["command"].endswith("rviz:=true gear_ratio:=50"),
              p.get("command", ""))
        p = preview("drivetrain.mock", {"gear_ratio": "48.0"})
        check("defaults stay out of the command", p["ok"] and "gear_ratio" not in p["command"])
        p = preview("drivetrain.sim-up", {"RVIZ": "false", "HEADLESS": True, "joystick": "false"})
        check("env-style args become a prefix", p["ok"] and p["command"].startswith("RVIZ=false HEADLESS=1 tooling/sim-up true false"),
              p.get("command", ""))
        p = preview("record.play", {"name": "/tmp/billee_run", "loop": True})
        check("flag-style args", p["ok"] and p["command"].endswith("--loop"), p.get("command", ""))
        p = preview("model.sim", {})
        check("shell items are wrapped in bash -c", p["ok"] and "bash -c 'xacro" in p["command"], p.get("command", ""))

        cases = [
            ("drive.pub", {"v": "fast"}, "v"),
            ("drive.pub", {"v": "0.2; rm -rf ~"}, "v"),
            ("can.send", {"frame": "009#00 && reboot"}, "frame"),
            ("record.bag", {"name": "$(whoami)"}, "name"),
            ("drivetrain.real", {"can_interface": "can9"}, "can_interface"),
            ("drivetrain.sim-full", {"world_file": "a b.sdf"}, "world_file"),
            ("drivetrain.mock", {"rviz": "maybe"}, "rviz"),
            ("drivetrain.mock", {"bogus": 1}, "bogus"),
        ]
        for item_id, args, field in cases:
            p = preview(item_id, args)
            check(f"refuses {item_id} {args}", not p["ok"] and field in p["errors"], p.get("errors", {}).get(field, ""))
        status, body = start("drive.pub", {"v": "0.2;reboot"})
        check("start refuses bad args too", status == 400 and "v" in body.get("errors", {}))
        status, _ = call("/api/start", {"cmd": "echo hacked"})
        check("raw commands are not accepted", status == 404)

        status, _ = call("/api/start", {"id": "health.topics"}, {"Origin": "http://evil.example"})
        check("foreign Origin refused", status == 403)
        req = urllib.request.Request(f"{BASE}/api/start", data=b'{"id":"health.topics"}', method="POST")
        req.add_header("Content-Type", "text/plain")
        try:
            urllib.request.urlopen(req, timeout=5)
            check("non-JSON POST refused", False)
        except urllib.error.HTTPError as err:
            check("non-JSON POST refused", err.code == 415)

        status, body = start("drivetrain.mock")
        check("start mock drivetrain", status == 200, body.get("error", ""))
        mock = body["run"]["id"]
        check("mock becomes ready (log match)", wait_status(mock, {"running"}, 60) == "running")

        status, body = start("drivetrain.sim")
        check("second drivetrain is refused", status == 409, body.get("error", ""))
        status, body = start("addons.bridge")
        check("second bridge on :8765 is refused", status == 409, body.get("error", ""))

        status, body = start("health.controllers")
        out = log_of(body["run"]["id"])
        check("task output streams back", out.count("active") >= 2, "both controllers active")

        status, body = start("drive.pub", {"v": "0.2", "w": "0"})
        drive = body["run"]["id"]
        check("drive with arguments", status == 200 and "x: 0.2" in body["run"]["command"])
        time.sleep(2.5)
        status, body = start("drive.stop")
        check("zero command allowed while driving (cmd-vel is shared)", status == 200, body.get("error", ""))
        log_of(body["run"]["id"])
        time.sleep(0.5)
        status, body = start("health.joint-states")
        out = log_of(body["run"]["id"])
        check("the v=0.2 argument reached the wheels", "1.81" in out, "0.2 m/s / 0.11 m = 1.818 rad/s")

        call("/api/stop", {"run": drive})
        check("stop scripted drive", wait_status(drive, {"stopped"}, 15) == "stopped")
        time.sleep(1.0)
        status, body = start("health.joint-states")
        out = log_of(body["run"]["id"])
        check("rover stops after drive stops", "1.81" not in out and "velocity" in out)

        status, body = start("record.bag", {"name": f"/tmp/billee_smoke_{int(time.time())}", "topics": "/joint_states"})
        bag = body["run"]["id"]
        time.sleep(3)
        call("/api/stop", {"run": bag})
        check("bag recording stops cleanly", wait_status(bag, {"stopped"}, 15) == "stopped")

        call("/api/stop", {"run": mock})
        check("mock drivetrain stops", wait_status(mock, {"stopped"}, 20) == "stopped")
        time.sleep(1.0)
        status, body = start("addons.bridge")
        check("port 8765 is free again", status == 200, body.get("error", ""))
        call("/api/stop", {"run": body["run"]["id"]})
        wait_status(body["run"]["id"], {"stopped", "failed"}, 15)
    finally:
        server.terminate()
        server.wait(timeout=20)
    print(f"\n{'All checks passed' if failures == 0 else f'{failures} check(s) failed'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
