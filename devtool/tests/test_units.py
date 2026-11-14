import json
import os
import socket
import tempfile
import time
import unittest
from pathlib import Path
from unittest import mock

from harness import fixture_catalog, server


def item(catalog: dict, item_id: str) -> dict:
    return next(i for i in catalog["items"] if i["id"] == item_id)


class Coerce(unittest.TestCase):
    def test_bool(self):
        spec = {"type": "bool", "default": False}
        self.assertEqual(server.coerce(spec, True), (True, "true"))
        self.assertEqual(server.coerce(spec, "false"), (False, "false"))
        for bad in ("yes", 1, None, "True"):
            with self.assertRaises(ValueError):
                server.coerce(spec, bad)

    def test_enum(self):
        spec = {"type": "enum", "choices": ["", "a"], "default": ""}
        self.assertEqual(server.coerce(spec, "a"), ("a", "a"))
        self.assertEqual(server.coerce(spec, ""), ("", ""))
        with self.assertRaisesRegex(ValueError, r"\(auto\), a"):
            server.coerce(spec, "b")

    def test_number(self):
        spec = {"type": "number", "default": 1}
        for raw, text in ((2, "2"), (0.25, "0.25"), ("-0.5", "-0.5"), (" 3 ", "3"), ("1e-3", "1e-3")):
            self.assertEqual(server.coerce(spec, raw)[1], text)
        for bad in (True, "fast", "1;reboot", "", "nan", "inf", "0x10", "1 2"):
            with self.assertRaises(ValueError, msg=bad):
                server.coerce(spec, bad)

    def test_string(self):
        spec = {"type": "string", "default": "/tmp/run", "style": "template"}
        self.assertEqual(server.coerce(spec, "/a/b.bag"), ("/a/b.bag", "/a/b.bag"))
        self.assertEqual(server.coerce(spec, "/a /b"), ("/a /b", "/a /b"))
        self.assertEqual(server.coerce(spec, "009#00.11"), ("009#00.11", "009#00.11"))
        for bad in ("", "a;b", "$(id)", "`id`", "a|b", "a&b", "a>b", "'x'", '"x"', "a\nb", "é", " lead"):
            with self.assertRaises(ValueError, msg=bad):
                server.coerce(spec, bad)

    def test_empty_default_is_allowed(self):
        self.assertEqual(server.coerce({"type": "string", "default": ""}, ""), ("", ""))

    def test_no_spaces_for_ros_and_env(self):
        for style in ("ros", "env"):
            with self.assertRaisesRegex(ValueError, "no spaces"):
                server.coerce({"type": "string", "default": "x", "style": style}, "a b")


class ResolveArgs(unittest.TestCase):
    def setUp(self):
        self.item = item(fixture_catalog(1), "t.ros")

    def test_defaults(self):
        args = server.resolve_args(self.item, None)
        self.assertEqual(args["n"], (1.0, "1"))
        self.assertEqual(args["rviz"], (False, "false"))

    def test_collects_every_error(self):
        with self.assertRaises(server.ArgError) as ctx:
            server.resolve_args(self.item, {"n": "x", "rviz": "maybe", "bogus": 1})
        self.assertEqual(set(ctx.exception.errors), {"n", "rviz", "bogus"})

    def test_rejects_non_object(self):
        with self.assertRaises(server.ArgError) as ctx:
            server.resolve_args(self.item, ["n"])
        self.assertIn("_", ctx.exception.errors)


class BuildCommand(unittest.TestCase):
    def setUp(self):
        self.catalog = fixture_catalog(1)

    def build(self, item_id, given=None, platform="x86-native"):
        it = item(self.catalog, item_id)
        return server.build_command(self.catalog, it, server.resolve_args(it, given), platform)

    def test_template(self):
        self.assertEqual(self.build("t.echo", {"word": "rover"}), "echo hello rover")

    def test_defaults_stay_out(self):
        self.assertEqual(self.build("t.ros"), "env BILLEE_TEST_RUNNER=x86-native echo ros 1")

    def test_changed_args_are_appended(self):
        cmd = self.build("t.ros", {"n": 2, "rviz": True, "mode": "mock", "loop": True, "MODE": "a", "FAST": True})
        self.assertEqual(cmd, "env BILLEE_TEST_RUNNER=x86-native MODE=a FAST=1 echo ros 2 rviz:=true mode:=mock --loop")

    def test_number_default_compares_as_number(self):
        self.assertNotIn("n:=", self.build("t.ros", {"n": "1.0"}))

    def test_every_platform_template(self):
        for platform in server.PLATFORMS:
            self.assertTrue(self.build("t.ros", platform=platform).startswith(f"env BILLEE_TEST_RUNNER={platform} "))

    def test_shell_items_are_quoted(self):
        self.assertEqual(self.build("t.shell"),
                         "env BILLEE_TEST_RUNNER=x86-native bash -c 'echo '\"'\"'a b'\"'\"' && echo $HOME'")

    def test_broken_item_uses_workaround(self):
        self.assertEqual(self.build("t.broken", {"x": "2"}), "echo workaround 2")


class NeedsAndResources(unittest.TestCase):
    def setUp(self):
        self.catalog = fixture_catalog(1)

    def test_runner_and_item_needs(self):
        it = item(self.catalog, "t.ros")
        needs = server.needs_for(self.catalog, it, server.resolve_args(it, {}), "x86-native")
        self.assertEqual(needs, [("pixi", None)])
        self.assertEqual(server.needs_for(self.catalog, it, server.resolve_args(it, {}), "mac"), [])

    def test_needs_when_arg(self):
        it = item(self.catalog, "t.choice")
        self.assertEqual(server.needs_for(self.catalog, it, server.resolve_args(it, {}), "x86-native"), [])
        args = server.resolve_args(it, {"target": "gpu"})
        self.assertEqual(server.needs_for(self.catalog, it, args, "x86-native"), [("nvidia", "target gpu")])

    def test_needs_when_platform_and_dedupe(self):
        it = {"needs": ["display", "display"], "runner": "host",
              "needsWhen": [{"platform": ["mac"], "needs": ["docker", "display"]}]}
        self.assertEqual(server.needs_for(self.catalog, it, {}, "mac"), [("display", None), ("docker", None)])

    def test_resources(self):
        it = item(self.catalog, "s.optional")
        self.assertEqual(server.resources_for(it, server.resolve_args(it, {})), set())
        self.assertEqual(server.resources_for(it, server.resolve_args(it, {"grab": True})), {"lock"})

    def test_modes(self):
        modes = {i["id"]: server.mode_of(i) for i in self.catalog["items"]}
        self.assertEqual(modes["t.echo"], "run")
        self.assertEqual(modes["i.term"], "terminal")
        self.assertEqual(modes["i.sudo"], "terminal")
        self.assertEqual(modes["l.web"], "link")
        self.assertEqual(server.mode_of({"kind": "task", "runner": "host", "needs": ["sudo"]}), "terminal")


class DetectPlatform(unittest.TestCase):
    def detect(self, marker=None, system="Linux", machine="x86_64", nvidia=False, host_built=False):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            ws = root / "ros2_ws"
            (root / "tooling").mkdir()
            (root / "tooling" / "host-ros2").touch()
            ws.mkdir()
            if marker is not None:
                (ws / ".billee-platform").write_text(marker + "\n")
            if host_built:
                (ws / "build" / "pkg").mkdir(parents=True)
                (ws / "build" / "pkg" / "CMakeCache.txt").write_text(f"CMAKE_HOME={ws}/src/pkg\n")
            with mock.patch.object(server, "REPO_ROOT", root), \
                    mock.patch.object(server, "has_nvidia", return_value=nvidia), \
                    mock.patch.object(server.pyplatform, "system", return_value=system), \
                    mock.patch.object(server.pyplatform, "machine", return_value=machine):
                return server.detect_platform()

    def test_cases(self):
        self.assertEqual(self.detect(), "x86-native")
        self.assertEqual(self.detect("x86"), "x86-native")
        self.assertEqual(self.detect("x86", nvidia=True), "x86-docker")
        self.assertEqual(self.detect("x86", nvidia=True, host_built=True), "x86-native")
        self.assertEqual(self.detect("l4t", machine="aarch64"), "l4t")
        self.assertEqual(self.detect("mac"), "mac")
        self.assertEqual(self.detect(system="Darwin", machine="arm64"), "mac")
        self.assertEqual(self.detect(machine="aarch64"), "linux-aarch64")

    def test_unknown_marker_falls_back_to_detection(self):
        self.assertEqual(self.detect("x86-cpu"), "x86-native")
        self.assertEqual(self.detect("garbage", machine="aarch64"), "linux-aarch64")


class CatalogReload(unittest.TestCase):
    def test_reload_and_bad_json(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "c.json"
            path.write_text(json.dumps({"items": [{"id": "a"}]}))
            cat = server.Catalog(path)
            self.assertIsNotNone(cat.item("a"))
            path.write_text(json.dumps({"items": [{"id": "b"}]}))
            future = time.time() + 5
            os.utime(path, (future, future))
            self.assertIsNotNone(cat.item("b"))
            path.write_text("{not json")
            os.utime(path, (future + 5, future + 5))
            with mock.patch("sys.stderr"):
                self.assertIsNotNone(cat.item("b"))
            self.assertIsNone(cat.item("missing"))

    def test_first_load_must_be_valid(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "c.json"
            path.write_text("{")
            with self.assertRaises(ValueError):
                server.Catalog(path).data


class Machine(unittest.TestCase):
    def test_facts_have_every_blocking_key(self):
        catalog = json.loads(server.CATALOG_PATH.read_text())
        facts = server.machine_facts()
        for key in catalog["blocking"]:
            if key != "pixi":
                self.assertIn(key, facts)

    def test_terminal_argv(self):
        with mock.patch.object(server.shutil, "which", side_effect=lambda n: n == "xterm"):
            self.assertEqual(server.terminal_argv("x")[:2], ["xterm", "-e"])
        with mock.patch.object(server.shutil, "which", return_value=None):
            self.assertIsNone(server.terminal_argv("x"))

    def test_port_in_use(self):
        with socket.socket() as sock:
            sock.bind(("127.0.0.1", 0))
            sock.listen()
            port = sock.getsockname()[1]
            self.assertTrue(server.port_in_use(port))
        self.assertFalse(server.port_in_use(port))


if __name__ == "__main__":
    unittest.main()
