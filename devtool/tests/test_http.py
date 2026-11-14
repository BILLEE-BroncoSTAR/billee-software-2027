import http.client
import json
import time
import unittest
from unittest import mock

from harness import TestServer, fixture_catalog, server


class Http(unittest.TestCase):
    def setUp(self):
        self.s = TestServer()
        self.addCleanup(self.s.close)

    def test_info_catalog_runs(self):
        status, info = self.s.call("/api/info")
        self.assertEqual(status, 200)
        self.assertEqual(info["platform"], "x86-native")
        self.assertIn("test platform", info["platformDescription"])
        self.assertFalse(info["needs"]["nvidia"])
        status, catalog = self.s.call("/api/catalog")
        self.assertEqual((status, len(catalog["items"])), (200, len(fixture_catalog(1)["items"])))
        self.assertEqual(self.s.call("/api/runs"), (200, []))

    def test_static_files(self):
        status, body = self.s.call("/")
        self.assertEqual(status, 200)
        self.assertIn(b"<!doctype html>", body.lower())
        for path, ctype in (("/app.js", "text/javascript"), ("/fonts/inter.woff2", "font/woff2")):
            conn = http.client.HTTPConnection("127.0.0.1", self.s.port)
            conn.request("GET", path)
            res = conn.getresponse()
            self.assertEqual(res.status, 200, path)
            self.assertTrue(res.getheader("Content-Type").startswith(ctype), path)
            conn.close()
        for path in ("/../server.py", "/%2e%2e/server.py", "/nope.html", "/fonts"):
            self.assertEqual(self.s.call(path)[0], 404, path)

    def test_host_header_is_checked(self):
        conn = http.client.HTTPConnection("127.0.0.1", self.s.port)
        conn.request("GET", "/api/info", headers={"Host": "evil.example"})
        self.assertEqual(conn.getresponse().status, 403)
        conn.close()

    def test_post_guards(self):
        self.assertEqual(self.s.call("/api/start", {"id": "t.echo"}, {"Origin": "http://evil.example"})[0], 403)
        self.assertEqual(self.s.call("/api/start", raw=b'{"id":"t.echo"}', headers={"Content-Type": "text/plain"})[0],
                         415)
        self.assertEqual(self.s.call("/api/start", raw=b"{nope")[0], 400)
        self.assertEqual(self.s.call("/api/start", raw=b"[1]")[0], 400)
        self.assertEqual(self.s.call("/api/nothing", {})[0], 404)
        self.assertEqual(self.s.call("/api/start", {"id": "missing"})[0], 404)
        self.assertEqual(self.s.call("/api/start", {"cmd": "echo hacked"})[0], 404)
        self.assertEqual(self.s.call("/api/stop", {"run": "r999"})[0], 404)
        origin = {"Origin": f"http://localhost:{self.s.port}"}
        self.assertEqual(self.s.call("/api/preview", {"id": "t.echo"}, origin)[0], 200)

    def test_preview(self):
        _, p = self.s.call("/api/preview", {"id": "t.echo", "args": {"word": "rover"}})
        self.assertEqual(p, {"ok": True, "command": "echo hello rover", "mode": "run", "uses": [],
                             "warnings": [], "blocked": None})
        _, p = self.s.call("/api/preview", {"id": "t.echo", "args": {"word": "a;b"}})
        self.assertFalse(p["ok"])
        self.assertIn("word", p["errors"])
        _, p = self.s.call("/api/preview", {"id": "s.svc"})
        self.assertEqual((p["mode"], p["uses"]), ("run", ["lock"]))
        self.assertEqual(self.s.call("/api/preview", {"id": "i.term"})[1]["mode"], "terminal")
        self.assertEqual(self.s.call("/api/preview", {"id": "l.web"})[1]["mode"], "link")

    def test_task_output_and_exit(self):
        status, body = self.s.start("t.echo", {"word": "rover"})
        self.assertEqual(status, 200)
        run = body["run"]
        self.assertEqual((run["item"], run["command"], run["kind"]), ("t.echo", "echo hello rover", "task"))
        lines, ids, end = self.s.log(run["id"])
        self.assertEqual(lines, ["hello rover"])
        self.assertEqual(ids, [0])
        self.assertEqual((end["status"], end["exitCode"]), ("finished", 0))
        self.assertIsNotNone(end["ended"])

        _, body = self.s.start("t.fail")
        _, _, end = self.s.log(body["run"]["id"])
        self.assertEqual((end["status"], end["exitCode"]), ("failed", 3))

    def test_start_refuses_bad_args(self):
        status, body = self.s.start("t.echo", {"word": "$(id)"})
        self.assertEqual(status, 400)
        self.assertIn("word", body["errors"])

    def test_log_resume(self):
        _, body = self.s.start("t.lines", {"n": 20})
        run_id = body["run"]["id"]
        self.s.wait(run_id, {"finished"})
        lines, ids, _ = self.s.log(run_id, "?from=15")
        self.assertEqual((lines, ids[0]), (["16", "17", "18", "19", "20"], 15))
        lines, _, _ = self.s.log(run_id, headers={"Last-Event-ID": "17"})
        self.assertEqual(lines, ["19", "20"])
        lines, _, _ = self.s.log(run_id, "?from=junk")
        self.assertEqual(len(lines), 20)
        self.assertEqual(self.s.call("/api/runs/r999/log")[0], 404)

    def test_log_keeps_the_last_lines(self):
        with mock.patch.object(server, "MAX_LINES", 10):
            _, body = self.s.start("t.lines", {"n": 50})
            run_id = body["run"]["id"]
            self.s.wait(run_id, {"finished"})
            lines, ids, end = self.s.log(run_id)
        self.assertEqual(lines, [str(n) for n in range(41, 51)])
        self.assertEqual((ids[0], end["lines"]), (40, 50))

    def test_service_ready_conflict_and_stop(self):
        _, body = self.s.start("s.svc")
        svc = body["run"]
        self.assertEqual(svc["status"], "starting")
        self.assertEqual(self.s.wait(svc["id"], {"running"}), "running")

        status, body = self.s.start("s.svc")
        self.assertEqual(status, 409)
        self.assertIn("already running", body["error"])
        status, body = self.s.start("s.other")
        self.assertEqual(status, 409)
        self.assertIn("Slow service is using", body["error"])
        self.assertTrue(self.s.call("/api/preview", {"id": "s.other"})[1]["blocked"])
        self.assertEqual(self.s.start("s.optional")[0], 200)
        self.assertEqual(self.s.start("s.optional", {"grab": True})[0], 409)

        status, body = self.s.call("/api/stop", {"run": svc["id"]})
        self.assertEqual(status, 200)
        self.assertIn(body["run"]["status"], ("stopping", "stopped"))
        self.assertEqual(self.s.wait(svc["id"], {"stopped"}), "stopped")
        self.assertEqual(self.s.call("/api/stop", {"run": svc["id"]})[1]["run"]["status"], "stopped")
        self.assertEqual(self.s.start("s.other")[0], 200)

    def test_stream_and_stop(self):
        _, body = self.s.start("s.stream")
        run_id = body["run"]["id"]
        lines, _, end = self.s.log(run_id, until_end=False, max_lines=3)
        self.assertEqual(lines, ["tick 1", "tick 2", "tick 3"])
        self.assertIsNone(end)
        self.s.call("/api/stop", {"run": run_id})
        self.assertEqual(self.s.wait(run_id, {"stopped"}), "stopped")

    def test_stop_escalates_when_ctrl_c_is_ignored(self):
        with mock.patch.object(server, "STOP_GRACE_S", (0.3, 0.3)):
            _, body = self.s.start("s.stubborn")
            run_id = body["run"]["id"]
            self.s.log(run_id, until_end=False, max_lines=1)
            started = time.time()
            self.s.call("/api/stop", {"run": run_id})
            self.assertEqual(self.s.wait(run_id, {"stopped"}, timeout=5), "stopped")
            self.assertLess(time.time() - started, 3)

    def test_parallel_starts_cannot_both_win(self):
        import threading
        results = []
        threads = [threading.Thread(target=lambda: results.append(self.s.start("s.other")[0])) for _ in range(6)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()
        self.assertEqual(sorted(results), [200, 409, 409, 409, 409, 409])

    def test_confirm(self):
        status, body = self.s.start("t.confirm")
        self.assertEqual(status, 428)
        self.assertIn("Deletes nothing", body["message"])
        self.assertEqual(self.s.call("/api/runs")[1], [])
        status, body = self.s.start("t.confirm", confirmed=True)
        self.assertEqual(status, 200)

    def test_blocking_needs_and_warnings(self):
        status, body = self.s.start("t.gpu")
        self.assertEqual((status, body["error"]), (409, "This machine has no NVIDIA GPU."))
        _, p = self.s.call("/api/preview", {"id": "t.choice", "args": {"target": "gpu"}})
        self.assertEqual(p["blocked"], "target gpu: This machine has no NVIDIA GPU. Pick cpu.")
        self.assertIsNone(self.s.call("/api/preview", {"id": "t.choice"})[1]["blocked"])
        self.s.needs["display"] = False
        _, p = self.s.call("/api/preview", {"id": "t.display"})
        self.assertEqual((p["blocked"], p["warnings"]), (None, ["Missing here (display): A display."]))
        self.assertEqual(self.s.start("t.display")[0], 200)
        self.s.needs["pixi"] = False
        self.assertEqual(self.s.start("t.ros")[1]["error"], "Pixi isn't installed.")

    def test_port_held_by_another_program(self):
        status, body = self.s.start("s.port")
        self.assertEqual(status, 409)
        self.assertIn(f"Port {self.s.busy_port} is already taken", body["error"])

    def test_terminal(self):
        self.assertEqual(self.s.start("i.term")[0], 409)
        self.assertEqual(self.s.call("/api/terminal", {"id": "l.web"})[0], 409)
        with mock.patch.object(server, "terminal_argv", return_value=None):
            self.assertEqual(self.s.call("/api/terminal", {"id": "i.term"})[0], 501)
        with mock.patch.object(server, "terminal_argv", return_value=["true"]) as argv:
            status, body = self.s.call("/api/terminal", {"id": "i.term"})
            self.assertEqual((status, body["command"]), (200, "read -r x"))
            self.assertIn("read -r x;", argv.call_args[0][0])
            self.assertEqual(self.s.call("/api/terminal", {"id": "i.sudo"})[0], 428)
            self.assertEqual(self.s.call("/api/terminal", {"id": "i.sudo", "confirmed": True})[0], 200)
            self.assertEqual(self.s.call("/api/terminal", {"id": "t.gpu"})[0], 409)

    def test_runs_are_newest_first_and_pruned(self):
        with mock.patch.object(server, "KEEP_FINISHED", 2):
            ids = []
            for word in ("a", "b", "c", "d"):
                _, body = self.s.start("t.echo", {"word": word})
                ids.append(body["run"]["id"])
                self.s.wait(ids[-1], {"finished"})
        runs = [r["id"] for r in self.s.call("/api/runs")[1]]
        self.assertEqual(runs, [ids[3], ids[2], ids[1]])

    def test_catalog_hot_reload(self):
        data = fixture_catalog(self.s.busy_port)
        data["items"].append({**data["items"][0], "id": "t.new", "label": "New"})
        self.s.write_catalog(data)
        self.assertEqual(self.s.start("t.new", {"word": "x"})[0], 200)
        with mock.patch("sys.stderr"):
            self.s.write_catalog("{broken")
            self.assertEqual(self.s.call("/api/catalog")[0], 200)

    def test_server_errors_are_reported(self):
        data = fixture_catalog(self.s.busy_port)
        data["items"].append({**data["items"][0], "id": "t.bad", "cmd": "echo {{missing}}"})
        self.s.write_catalog(data)
        with mock.patch("sys.stderr"):
            status, body = self.s.call("/api/preview", {"id": "t.bad"})
        self.assertEqual(status, 500)
        self.assertIn("Server error", body["error"])


class Main(unittest.TestCase):
    def test_bad_platform_and_catalog(self):
        import subprocess
        import sys
        script = str(server.HERE / "server.py")
        res = subprocess.run([sys.executable, script, "--platform", "x86"], capture_output=True, text=True)
        self.assertEqual(res.returncode, 2)
        self.assertIn("invalid choice", res.stderr)
        res = subprocess.run([sys.executable, script, "--catalog", "/nonexistent.json"], capture_output=True,
                             text=True, timeout=10)
        self.assertEqual(res.returncode, 1)
        self.assertIn("can't read", res.stderr)


if __name__ == "__main__":
    unittest.main()
