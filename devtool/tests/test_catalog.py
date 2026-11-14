import json
import os
import re
import unittest

from harness import server

CATALOG = json.loads(server.CATALOG_PATH.read_text())
ITEMS = CATALOG["items"]
KINDS = {"task", "service", "stream", "gui", "interactive", "link"}
TYPES = {"bool", "enum", "number", "string"}
STYLES = {"template", "ros", "env", "flag"}
REPO_PATH = re.compile(r"(?<![\w/.-])((?:tooling|devtool)/[\w.-]+)")


class Catalog(unittest.TestCase):
    def test_top_level(self):
        for key in ("kinds", "runners", "platforms", "needs", "blocking", "resources", "exclusive", "groups", "items"):
            self.assertIn(key, CATALOG)
        self.assertEqual(set(CATALOG["kinds"]), KINDS)
        self.assertEqual(set(CATALOG["platforms"]), set(server.PLATFORMS))
        self.assertEqual(set(CATALOG["runners"]["ros"]["templates"]), set(server.PLATFORMS))
        self.assertLessEqual(set(CATALOG["blocking"]), set(CATALOG["needs"]))
        self.assertLessEqual(set(CATALOG["exclusive"]), set(CATALOG["resources"]))
        for runner in CATALOG["runners"].values():
            for template in runner["templates"].values():
                self.assertEqual(template.count("{cmd}"), 1)

    def test_ids_and_groups(self):
        ids = [i["id"] for i in ITEMS]
        self.assertEqual(len(ids), len(set(ids)), "duplicate ids")
        groups = [g["id"] for g in CATALOG["groups"]]
        self.assertEqual(len(groups), len(set(groups)))
        for it in ITEMS:
            self.assertIn(it["group"], groups, it["id"])
        for group in groups:
            self.assertTrue(any(i["group"] == group for i in ITEMS), f"empty group {group}")

    def test_items(self):
        for it in ITEMS:
            with self.subTest(it["id"]):
                for key in ("id", "group", "label", "kind", "runner", "cmd", "status"):
                    self.assertTrue(it.get(key) not in (None, ""), key)
                self.assertIn(it["kind"], KINDS)
                self.assertIn(it["runner"], CATALOG["runners"])
                self.assertIn(it["status"], {"verified", "untested", "broken"})
                self.assertEqual(it["kind"] == "link", it["runner"] == "link")
                if it["status"] == "broken":
                    self.assertTrue(it.get("workaround") or it.get("notes"))
                if it.get("ready"):
                    re.compile(it["ready"])
                for key in ("confirm", "shell"):
                    self.assertIsInstance(it.get(key, False), bool)
                for need in it.get("needs", []):
                    self.assertIn(need, CATALOG["needs"])
                for res in it.get("uses", []):
                    self.assertIn(res, CATALOG["resources"])

    def test_args(self):
        for it in ITEMS:
            names = [a["name"] for a in it.get("args", [])]
            self.assertEqual(len(names), len(set(names)), it["id"])
            for spec in it.get("args", []):
                with self.subTest(f"{it['id']}.{spec['name']}"):
                    self.assertIn(spec["type"], TYPES)
                    self.assertIn(spec["style"], STYLES)
                    self.assertIn("default", spec)
                    if spec["type"] == "enum":
                        self.assertIn(spec["default"], spec["choices"])
                        self.assertEqual(len(spec["choices"]), len(set(spec["choices"])))
                    if spec["style"] == "env":
                        self.assertRegex(spec["env"], r"^[A-Z_][A-Z0-9_]*$")
                    if spec["style"] == "flag":
                        self.assertEqual(spec["type"], "bool")
                        self.assertTrue(spec["flag"].startswith("-"))
                    server.coerce(spec, spec["default"])

    def test_placeholders_match_template_args(self):
        for it in ITEMS:
            templates = {a["name"] for a in it.get("args", []) if a["style"] == "template"}
            for field in ("cmd", "workaround"):
                if field not in it:
                    continue
                used = set(server.PLACEHOLDER.findall(it[field]))
                with self.subTest(f"{it['id']}.{field}"):
                    self.assertLessEqual(used, templates, "placeholder without a template arg")
                    if field == "cmd" or it.get("status") == "broken":
                        self.assertEqual(used, templates, "template arg not used")

    def test_conditional_rules(self):
        platforms = set(server.PLATFORMS)
        for it in ITEMS:
            specs = {a["name"]: a for a in it.get("args", [])}
            for rule in it.get("needsWhen", []):
                with self.subTest(it["id"]):
                    for need in rule["needs"]:
                        self.assertIn(need, CATALOG["needs"])
                    if "platform" in rule:
                        self.assertLessEqual(set(rule["platform"]), platforms)
                    else:
                        spec = specs[rule["arg"]]
                        self.assertEqual(spec["type"], "enum")
                        self.assertLessEqual(set(rule["is"]), set(spec["choices"]))
            for arg, resources in it.get("usesWhen", {}).items():
                self.assertIn(arg, specs, it["id"])
                self.assertLessEqual(set(resources), set(CATALOG["resources"]), it["id"])

    def test_every_command_builds_on_every_platform(self):
        for it in ITEMS:
            args = server.resolve_args(it, {})
            for platform in server.PLATFORMS:
                cmd = server.build_command(CATALOG, it, args, platform)
                self.assertNotIn("{{", cmd, it["id"])
                self.assertNotIn("{cmd}", cmd, it["id"])
                server.needs_for(CATALOG, it, args, platform)
                server.resources_for(it, args)

    def test_repo_scripts_exist_and_are_executable(self):
        texts = [i["cmd"] + " " + i.get("workaround", "") for i in ITEMS if i["runner"] != "link"]
        texts += [t for r in CATALOG["runners"].values() for t in r["templates"].values()]
        for text in texts:
            for rel in REPO_PATH.findall(text):
                path = server.REPO_ROOT / rel
                with self.subTest(rel):
                    self.assertTrue(path.exists(), "missing")
                    if path.is_file() and not rel.endswith((".service", ".json", ".sh")):
                        self.assertTrue(os.access(path, os.X_OK), "not executable")

    def test_not_bootable(self):
        for entry in CATALOG.get("notBootable", []):
            self.assertTrue(entry.get("what") and entry.get("why"))


if __name__ == "__main__":
    unittest.main()
