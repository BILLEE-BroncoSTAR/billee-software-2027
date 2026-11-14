import re
import unittest
from unittest import mock

from harness import TestServer, server

try:
    from playwright.sync_api import expect, sync_playwright
except ImportError:
    sync_playwright = None


EXPECTED = re.compile(r"Failed to load resource: the server responded with a status of (400|409|428)")


def card(page, item_id):
    return page.locator(f'[data-item="{item_id}"]')


@unittest.skipIf(sync_playwright is None, "pip install playwright && playwright install chromium")
class Ui(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.pw = sync_playwright().start()
        cls.browser = cls.pw.chromium.launch()

    @classmethod
    def tearDownClass(cls):
        cls.browser.close()
        cls.pw.stop()

    def setUp(self):
        self.s = TestServer()
        self.addCleanup(self.s.close)
        patcher = mock.patch.object(server, "terminal_argv", return_value=["true"])
        patcher.start()
        self.addCleanup(patcher.stop)
        self.ctx = self.browser.new_context(viewport={"width": 1440, "height": 900})
        self.ctx.grant_permissions(["clipboard-read", "clipboard-write"], origin=self.s.base.replace("127.0.0.1", "localhost"))
        self.addCleanup(self.ctx.close)
        self.errors = []
        self.page = self.open()

    def open(self, path="/", width=None):
        page = self.ctx.new_page()
        if width:
            page.set_viewport_size({"width": width, "height": 900})
        page.on("pageerror", lambda e: self.errors.append(str(e)))
        page.on("console", lambda m: m.type == "error" and not EXPECTED.search(m.text) and self.errors.append(m.text))
        page.goto(f"http://localhost:{self.s.port}{path}")
        page.wait_for_selector("body[data-ready='1']")
        return page

    def tearDown(self):
        self.assertEqual(self.errors, [])

    def run_item(self, item_id):
        c = card(self.page, item_id)
        c.locator("[data-action=main]").click()
        return c

    def test_layout_and_machine(self):
        p = self.page
        expect(p.locator("#platform")).to_contain_text("x86-native")
        expect(p.locator(".group h2")).to_have_text(["Basics", "Services", "Other"])
        expect(p.locator('#nav [data-group="all"] .n')).to_have_text(str(len(self.s.app.catalog.data["items"])))
        expect(p.locator('#checks [data-need="nvidia"]')).to_have_attribute("data-ok", "false")
        expect(p.locator('#checks [data-need="display"]')).to_have_attribute("data-ok", "true")
        expect(card(p, "t.echo")).to_contain_text("Prints a greeting.")
        expect(card(p, "t.echo").locator(".badge")).to_have_text(["task", "verified"])
        expect(card(p, "t.confirm").locator(".badge.confirm")).to_have_text("asks first")

    def test_search_groups_and_filters(self):
        p = self.page
        p.keyboard.press("/")
        expect(p.locator("#q")).to_be_focused()
        p.keyboard.type("greeting")
        expect(p.locator(".card:visible")).to_have_count(1)
        expect(p.locator('#nav [data-group="basic"] .n')).to_have_text("1")
        expect(p.locator('#nav [data-group="long"] .n')).to_have_text("0")
        p.keyboard.type(" zzz")
        expect(p.locator("#empty")).to_be_visible()
        p.keyboard.press("Escape")
        expect(p.locator("#empty")).to_be_hidden()
        p.click('#nav [data-group="long"]')
        expect(p.locator('#nav [data-group="long"]')).to_have_attribute("aria-current", "true")
        expect(p.locator(".group:visible h2")).to_have_text(["Services"])
        p.click('#nav [data-group="all"]')
        expect(card(p, "t.gpu")).to_be_visible()
        p.check("#runnable-only")
        expect(card(p, "t.gpu")).to_be_hidden()
        expect(card(p, "l.web")).to_be_hidden()
        expect(card(p, "t.echo")).to_be_visible()
        p.reload()
        p.wait_for_selector("body[data-ready='1']")
        expect(p.locator("#runnable-only")).to_be_checked()
        expect(card(p, "t.gpu")).to_be_hidden()

    def test_args_preview_and_validation(self):
        p = self.page
        c = card(p, "t.echo")
        c.locator(".expand").click()
        expect(c.locator(".expand")).to_have_attribute("aria-expanded", "true")
        expect(c.locator(".cmd pre")).to_have_text("echo hello world")
        p.fill("#t\\.echo--word", "rover")
        expect(c.locator(".cmd pre")).to_have_text("echo hello rover")
        p.fill("#t\\.echo--word", "a;b")
        expect(c.locator(".err")).to_contain_text("letters, digits")
        expect(c.locator("[data-action=main]")).to_be_disabled()
        p.fill("#t\\.echo--word", "ok")
        expect(c.locator(".err")).to_be_hidden()
        expect(c.locator("[data-action=main]")).to_be_enabled()

        r = card(p, "t.ros")
        r.locator(".expand").click()
        r.locator("#t\\.ros--rviz").check()
        r.locator("#t\\.ros--MODE").select_option("a")
        expect(r.locator(".cmd pre")).to_have_text(
            "env BILLEE_TEST_RUNNER=x86-native MODE=a echo ros 1 rviz:=true")
        r.locator(".expand").click()
        expect(r.locator(".card-body")).to_be_hidden()

    def test_run_task_and_output(self):
        p = self.page
        card(p, "t.echo").locator(".expand").click()
        p.fill("#t\\.echo--word", "rover")
        self.run_item("t.echo")
        expect(p.locator("#log")).to_contain_text("hello rover")
        expect(p.locator("#log .end")).to_contain_text("finished (exit code 0)")
        expect(p.locator("#out-title")).to_have_text("Say hello")
        expect(p.locator("#out-cmd")).to_have_text("$ echo hello rover")
        expect(p.locator("#runs li .st")).to_have_text("finished")
        p.click("#out-copy")
        expect(p.locator("#toast")).to_have_text("Output copied")
        self.assertIn("hello rover", p.evaluate("navigator.clipboard.readText()"))

        self.run_item("t.fail")
        expect(p.locator("#log .end")).to_contain_text("failed (exit code 3)")
        expect(p.locator("#runs li").first).to_contain_text("exit 3")
        p.locator("#runs li", has_text="Say hello").click()
        expect(p.locator("#log")).to_contain_text("hello rover")
        expect(p.locator("#runs li", has_text="Say hello")).to_have_attribute("aria-selected", "true")

    def test_service_lifecycle_and_conflict(self):
        p = self.page
        svc = self.run_item("s.svc")
        expect(svc.locator(".card-actions .st")).to_have_text("running", timeout=5000)
        expect(svc).to_have_class(re.compile("is-active"))
        expect(svc.locator("[data-action=main]")).to_have_text("Stop")
        expect(p.locator('#nav [data-group="long"] .live')).to_be_visible()
        expect(p.locator("#log")).to_contain_text("READY now")
        other = card(p, "s.other")
        expect(other.locator("[data-action=main]")).to_be_disabled()
        other.locator(".expand").click()
        expect(other.locator(".msg.block")).to_contain_text("Slow service is using")
        p.click("#out-stop")
        expect(svc.locator("[data-action=main]")).to_have_text("Start", timeout=5000)
        expect(p.locator("#runs li .st").first).to_have_text("stopped")
        expect(other.locator("[data-action=main]")).to_be_enabled()
        expect(p.locator('#nav [data-group="long"] .live')).to_be_hidden()

    def test_stop_from_runs_list(self):
        p = self.page
        self.run_item("s.stream")
        expect(p.locator("#log")).to_contain_text("tick 3")
        p.locator("#runs li .btn.danger").click()
        expect(p.locator("#runs li .st")).to_have_text("stopped", timeout=5000)
        expect(p.locator("#log .end")).to_contain_text("stopped")

    def test_confirm_dialog(self):
        p = self.page
        self.run_item("t.confirm")
        expect(p.locator("#confirm")).to_be_visible()
        expect(p.locator("#confirm-text")).to_contain_text("Deletes nothing")
        p.click("#confirm button[value=cancel]")
        expect(p.locator("#confirm")).to_be_hidden()
        p.wait_for_timeout(300)
        expect(p.locator("#runs .empty")).to_be_visible()
        self.run_item("t.confirm")
        p.click("#confirm-ok")
        expect(p.locator("#log")).to_contain_text("confirmed")

    def test_blocked_items(self):
        p = self.page
        gpu = card(p, "t.gpu")
        expect(gpu.locator("[data-action=main]")).to_be_disabled()
        gpu.locator(".expand").click()
        expect(gpu.locator(".msg.block")).to_have_text("Blocked: This machine has no NVIDIA GPU.")
        expect(gpu.locator(".chip.no")).to_have_text("✗ nvidia")
        option = p.locator("#t\\.choice--target option[value=gpu]")
        expect(option).to_be_disabled()
        expect(option).to_have_text("gpu (not on this machine)")
        disp = card(p, "t.display")
        self.s.needs["display"] = False
        p.click("#recheck")
        disp.locator(".expand").click()
        expect(disp.locator(".msg.warn")).to_contain_text("Missing here (display)")
        expect(disp.locator("[data-action=main]")).to_be_enabled()
        expect(p.locator('#checks [data-need="display"]')).to_have_attribute("data-ok", "false")

    def test_terminal_and_links(self):
        p = self.page
        term = card(p, "i.term")
        expect(term.locator("[data-action=main]")).to_have_text("Open in terminal")
        term.locator("[data-action=main]").click()
        expect(p.locator("#toast")).to_contain_text('Opened a terminal for "Interactive"')
        card(p, "i.sudo").locator("[data-action=main]").click()
        expect(p.locator("#confirm")).to_be_visible()
        p.click("#confirm-ok")
        expect(p.locator("#toast")).to_contain_text('Opened a terminal for "Sudo thing"')

        copy_link = card(p, "l.copy").locator("[data-action=main]")
        expect(copy_link).to_have_text("Copy")
        copy_link.click()
        expect(p.locator("#toast")).to_have_text("Address copied")
        self.assertEqual(p.evaluate("navigator.clipboard.readText()"), "ws://localhost:8765")
        self.ctx.route("http://localhost:9/", lambda route: route.fulfill(body="ok"))
        with p.expect_popup() as popup:
            card(p, "l.web").locator("[data-action=main]").click()
        self.assertEqual(popup.value.url, "http://localhost:9/")
        popup.value.close()

        c = card(p, "t.echo")
        c.locator(".expand").click()
        c.locator(".cmd .btn").click()
        expect(p.locator("#toast")).to_have_text("Command copied")
        self.assertEqual(p.evaluate("navigator.clipboard.readText()"), "echo hello world")

    def test_theme_help_and_deep_link(self):
        p = self.page
        p.click("#theme")
        theme = p.evaluate("document.documentElement.dataset.theme")
        self.assertIn(theme, ("light", "dark"))
        p.reload()
        p.wait_for_selector("body[data-ready='1']")
        self.assertEqual(p.evaluate("document.documentElement.dataset.theme"), theme)
        p.click("#help-open")
        expect(p.locator("#help")).to_be_visible()
        p.keyboard.press("Escape")
        expect(p.locator("#help")).to_be_hidden()
        deep = self.open("/#item-s.port")
        expect(card(deep, "s.port").locator(".card-body")).to_be_visible()
        expect(card(deep, "s.port").locator(".msg.block")).to_contain_text("already taken")

    def test_phone_width(self):
        p = self.open(width=390)
        self.assertLessEqual(p.evaluate("document.documentElement.scrollWidth"), 390)
        expect(p.locator("#nav")).to_be_hidden()
        p.click('#mobile-nav [data-group="other"]')
        expect(p.locator(".group:visible h2")).to_have_text(["Other"])
        self.assertLessEqual(p.evaluate("document.documentElement.scrollWidth"), 390)

    def test_server_gone(self):
        p = self.page
        self.s.httpd.shutdown()
        self.s.httpd.server_close()
        expect(p.locator("#toast")).to_contain_text("Can't reach the dev tool server", timeout=5000)
        self.errors.clear()
        self.s.httpd.shutdown = lambda: None
        self.s.httpd.server_close = lambda: None
        p.close()


if __name__ == "__main__":
    unittest.main()
