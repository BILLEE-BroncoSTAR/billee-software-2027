const $ = (sel, root = document) => root.querySelector(sel);
const el = (tag, { dataset, ...props } = {}, ...kids) => {
  const node = Object.assign(document.createElement(tag), props);
  if (dataset) Object.assign(node.dataset, dataset);
  node.append(...kids.filter((k) => k !== null && k !== undefined && k !== false));
  return node;
};
const code = (text) => el("code", { textContent: text });
const svg = (path) => {
  const wrap = document.createElement("span");
  wrap.innerHTML = `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${path}</svg>`;
  return wrap.firstChild;
};

const ACTIVE = new Set(["starting", "running", "stopping"]);
const CHECKS = {
  display: "Display",
  vcan0: "vcan0",
  can: "CAN bus (can2)",
  zed: "ZED camera",
  gamepad: "Gamepad",
  docker: "Docker",
  terminal: "Terminal app",
  drivetrain: "Drivetrain running",
  nvidia: "NVIDIA GPU",
  pixi: "Pixi",
};
const MAX_LOG_LINES = 4000;

const state = {
  info: null,
  catalog: null,
  cards: [],
  runs: [],
  group: "all",
  selected: null,
  source: null,
  activeKey: null,
  serverDown: false,
};

const store = {
  get(key) {
    try { return localStorage.getItem(`billee-dev.${key}`); } catch { return null; }
  },
  set(key, value) {
    try { localStorage.setItem(`billee-dev.${key}`, value); } catch {}
  },
};

const root = document.documentElement;
if (store.get("theme")) root.dataset.theme = store.get("theme");
$("#theme").addEventListener("click", () => {
  const dark = root.dataset.theme ? root.dataset.theme === "dark" : matchMedia("(prefers-color-scheme: dark)").matches;
  root.dataset.theme = dark ? "light" : "dark";
  store.set("theme", root.dataset.theme);
});

async function api(path, body) {
  const init = body === undefined
    ? {}
    : { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) };
  try {
    const res = await fetch(path, init);
    let data = null;
    try { data = await res.json(); } catch {}
    return { ok: res.ok, status: res.status, data };
  } catch {
    return { ok: false, status: 0, data: { error: "Can't reach the dev tool server. Is billee-dev still running?" } };
  }
}

let toastTimer;
function toast(text) {
  const t = $("#toast");
  t.textContent = text;
  t.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove("show"), 4500);
}

async function copy(text, what = "Command") {
  try {
    await navigator.clipboard.writeText(text);
    toast(`${what} copied`);
  } catch {
    toast(`Couldn't copy. Select the ${what.toLowerCase()} and copy it by hand.`);
  }
}

function ask(message) {
  const dlg = $("#confirm");
  $("#confirm-text").textContent = message;
  dlg.returnValue = "";
  dlg.showModal();
  $("#confirm-ok").focus();
  return new Promise((resolve) => dlg.addEventListener("close", () => resolve(dlg.returnValue === "ok"), { once: true }));
}

$("#help-open").addEventListener("click", () => $("#help").showModal());

async function refreshInfo() {
  const r = await api("/api/info");
  if (!r.ok) return;
  state.info = r.data;
  const pill = $("#platform");
  pill.replaceChildren("Platform ", code(r.data.platform));
  pill.title = r.data.platformDescription;
  $("#checks").replaceChildren(
    ...Object.entries(CHECKS).map(([key, label]) => {
      const ok = Boolean(r.data.needs[key]);
      return el(
        "li",
        { title: state.catalog?.needs?.[key] ?? "", ariaLabel: `${label}: ${ok ? "yes" : "no"}`, dataset: { need: key, ok } },
        el("span", { className: `dot ${ok ? "yes" : "no"}` }),
        label,
      );
    }),
  );
  for (const card of state.cards) card.paintNeeds();
  applyFilters();
}

$("#recheck").addEventListener("click", async () => {
  await refreshInfo();
  for (const card of state.cards) card.schedulePreview();
  toast("Re-checked display, CAN, gamepad, Docker, GPU, Pixi and running drivetrains");
});

function needsHere(item) {
  const platform = state.info?.platform;
  const runner = state.catalog.runners[item.runner]?.needs?.[platform] ?? [];
  const when = (item.needsWhen ?? []).filter((r) => r.platform?.includes(platform)).flatMap((r) => r.needs);
  return [...new Set([...(item.needs ?? []), ...runner, ...when])];
}

function choiceBlocked(item, argName, choice) {
  const needs = state.info?.needs ?? {};
  const blocking = state.catalog.blocking ?? {};
  const missing = (n) => n in blocking && needs[n] === false;
  const rule = (item.needsWhen ?? []).find((r) => r.arg === argName && r.is.includes(choice) && r.needs.some(missing));
  return rule ? blocking[rule.needs.find(missing)] : null;
}

function modeOf(item) {
  if (item.kind === "link") return "link";
  if (item.kind === "interactive" || item.runner === "host-sudo" || (item.needs ?? []).includes("sudo")) return "terminal";
  return "run";
}

function firstSentence(text) {
  if (!text) return "";
  const m = text.match(/^.*?[.!?](\s|$)/);
  return (m ? m[0] : text).trim();
}

function argControl(item, spec, values, onChange) {
  const id = `${item.id}--${spec.name}`;
  let input;
  if (spec.type === "bool") {
    input = el("input", { type: "checkbox", id, checked: Boolean(spec.default) });
    input.addEventListener("change", () => onChange(input.checked));
    values[spec.name] = Boolean(spec.default);
  } else if (spec.type === "enum") {
    const label = (c) => (c === "" ? "(auto)" : c);
    input = el("select", { id }, ...spec.choices.map((c) => el("option", { value: c, textContent: label(c), dataset: { label: label(c) } })));
    input.value = spec.default;
    input.addEventListener("change", () => onChange(input.value));
    values[spec.name] = spec.default;
  } else {
    const wide = spec.type === "string" && String(spec.default).length > 22;
    input = el("input", { type: "text", id, value: String(spec.default), spellcheck: false, autocomplete: "off", className: wide ? "wide" : "" });
    if (spec.type === "number") input.inputMode = "decimal";
    input.addEventListener("input", () => onChange(input.value));
    values[spec.name] = String(spec.default);
  }
  const error = el("span", { className: "err", hidden: true, id: `${id}-err` });
  input.setAttribute("aria-describedby", error.id);
  const label = el("label", { htmlFor: id, textContent: spec.style === "env" ? `${spec.env}=` : spec.name });
  const control = el("div", { className: "control" }, input, spec.help ? el("span", { className: "help", textContent: spec.help }) : null, error);
  return { label, control, error, input };
}

const CHEVRON = '<path d="m9 6 6 6-6 6"/>';
const COPY = '<rect x="9" y="9" width="12" height="12" rx="2"/><path d="M5 15V5a2 2 0 0 1 2-2h10"/>';

function makeCard(item) {
  const catalog = state.catalog;
  const mode = modeOf(item);
  const values = {};
  const args = {};
  let seq = 0;
  let timer = null;
  let activeRun = null;
  let blocked = null;
  let hasErrors = false;

  const commandText = el("pre", { textContent: "…" });
  const copyBtn = el("button", { type: "button", className: "btn sm icon ghost", title: "Copy command", ariaLabel: "Copy command" }, svg(COPY));
  copyBtn.addEventListener("click", () => copy(commandText.textContent));
  const messages = el("div", { className: "msgs" });
  const status = el("span", { className: "st", hidden: true });
  const primary = el("button", { type: "button", className: "btn primary", dataset: { action: "main" } });
  const outputBtn = el("button", { type: "button", className: "btn sm", textContent: "Output", hidden: true });
  outputBtn.addEventListener("click", () => activeRun && selectRun(activeRun.id));

  const argGrid = item.args?.length ? el("div", { className: "args" }) : null;
  for (const spec of item.args ?? []) {
    const ctl = argControl(item, spec, values, (v) => {
      values[spec.name] = v;
      schedulePreview();
    });
    args[spec.name] = ctl;
    argGrid.append(ctl.label, ctl.control);
  }

  const meta = el("dl", { className: "meta" });
  const needsDd = el("dd", { className: "chips" });
  const needsDt = el("dt", { textContent: "Needs" });
  meta.append(needsDt, needsDd);
  const uses = [
    ...(item.uses ?? []),
    ...Object.entries(item.usesWhen ?? {}).flatMap(([arg, res]) => res.map((u) => `${u} (when ${arg} is on)`)),
  ];
  if (uses.length) {
    meta.append(el("dt", { textContent: "Uses" }), el("dd", { className: "chips" }, ...uses.map((u) => el("span", { className: "chip", textContent: u, title: catalog.resources?.[u.split(" ")[0]] ?? "" }))));
  }
  if (item.ready) meta.append(el("dt", { textContent: "Ready when" }), el("dd", {}, "output matches ", code(item.ready)));
  meta.append(
    el("dt", { textContent: "Tested" }),
    el("dd", {}, item.verified ?? (item.status === "untested" ? "Not tested yet." : item.status)),
    el("dt", { textContent: "ID" }),
    el("dd", {}, code(item.id), ` · ${catalog.runners[item.runner]?.label ?? item.runner}`),
  );

  let hint = null;
  if (mode === "run") {
    primary.addEventListener("click", () => (activeRun ? stopRun(activeRun.id) : start(card)));
  } else if (mode === "terminal") {
    primary.textContent = "Open in terminal";
    primary.addEventListener("click", () => openTerminal(card));
    hint = (item.needs ?? []).includes("sudo") || item.runner === "host-sudo"
      ? "Opens a terminal; it asks for your sudo password there."
      : "Opens a terminal; it needs keyboard input.";
    primary.title = hint;
  } else {
    const target = /^https?:/.test(item.cmd) ? item.cmd : null;
    primary.textContent = target ? "Open" : "Copy";
    primary.classList.remove("primary");
    primary.addEventListener("click", () => (target ? window.open(target, "_blank", "noopener") : copy(item.cmd, "Address")));
  }

  const badges = [
    el("span", { className: "badge", textContent: item.kind }),
    el("span", { className: `badge ${item.status}`, textContent: item.status }),
    item.confirm ? el("span", { className: "badge confirm", textContent: "asks first" }) : null,
  ];
  const summaryText = firstSentence(item.notes) || firstSentence(item.verified);
  const bodyId = `body-${item.id}`;
  const expand = el(
    "button",
    { type: "button", className: "expand", ariaExpanded: "false" },
    svg(CHEVRON),
    el(
      "span",
      {},
      el("span", { className: "card-title" }, el("h3", { textContent: item.label }), ...badges),
      el("p", { className: "summary" }, summaryText || code(item.cmd)),
    ),
  );
  expand.firstChild.classList.add("chev");
  expand.setAttribute("aria-controls", bodyId);

  const body = el(
    "div",
    { className: "card-body", id: bodyId, hidden: true },
    item.notes ? el("p", { className: "notes", textContent: item.notes }) : null,
    argGrid ? el("div", {}, el("p", { className: "label", textContent: "Arguments" }), argGrid) : null,
    el("div", {}, el("p", { className: "label", textContent: mode === "link" ? "Address" : "Command" }), el("div", { className: "cmd" }, commandText, copyBtn)),
    messages,
    hint ? el("p", { className: "hint", textContent: hint }) : null,
    meta,
  );

  const root = el(
    "article",
    { className: "card", id: `item-${item.id}`, dataset: { item: item.id } },
    el("div", { className: "card-head" }, expand, el("div", { className: "card-actions" }, status, outputBtn, primary)),
    body,
  );

  function setOpen(open) {
    root.classList.toggle("open", open);
    body.hidden = !open;
    expand.ariaExpanded = String(open);
  }
  expand.addEventListener("click", () => setOpen(body.hidden));

  function showErrors(errors = {}) {
    hasErrors = false;
    for (const [name, ctl] of Object.entries(args)) {
      const msg = errors[name];
      ctl.control.classList.toggle("has-error", Boolean(msg));
      ctl.input.ariaInvalid = msg ? "true" : "false";
      ctl.error.hidden = !msg;
      ctl.error.textContent = msg ?? "";
      if (msg) hasErrors = true;
    }
    if (hasErrors) setOpen(true);
    if (errors._) toast(errors._);
  }

  function paintButtons() {
    if (mode === "run") {
      if (activeRun) {
        primary.textContent = activeRun.status === "stopping" ? "Stopping…" : "Stop";
        primary.className = "btn danger";
        primary.disabled = activeRun.status === "stopping";
      } else {
        primary.textContent = item.kind === "task" ? "Run" : "Start";
        primary.className = "btn primary";
        primary.disabled = hasErrors || Boolean(blocked);
      }
    } else if (mode === "terminal") {
      primary.disabled = hasErrors || Boolean(blocked);
    }
    primary.title = blocked && !activeRun ? blocked : hint ?? "";
  }

  async function preview() {
    const mine = ++seq;
    const r = await api("/api/preview", { id: item.id, args: values });
    if (mine !== seq || !r.ok) return;
    const p = r.data;
    showErrors(p.errors);
    messages.replaceChildren();
    if (!p.ok) {
      commandText.textContent = "Fix the highlighted argument to see the command.";
      blocked = null;
    } else {
      commandText.textContent = p.command;
      blocked = activeRun ? null : p.blocked;
      for (const w of p.warnings) messages.append(el("p", { className: "msg warn" }, el("b", { textContent: "Heads up: " }), w));
      if (blocked) messages.append(el("p", { className: "msg block" }, el("b", { textContent: "Blocked: " }), blocked));
    }
    paintButtons();
  }

  function schedulePreview() {
    clearTimeout(timer);
    timer = setTimeout(preview, 120);
  }

  const card = {
    item,
    mode,
    root,
    values,
    showErrors,
    schedulePreview,
    preview,
    setOpen,
    get active() {
      return activeRun;
    },
    paintNeeds() {
      for (const [name, ctl] of Object.entries(args)) {
        if (ctl.input.tagName !== "SELECT") continue;
        for (const option of ctl.input.options) {
          const why = choiceBlocked(item, name, option.value);
          option.disabled = Boolean(why);
          option.title = why ?? "";
          option.textContent = why ? `${option.dataset.label} (not on this machine)` : option.dataset.label;
        }
      }
      const needs = state.info?.needs ?? {};
      const list = needsHere(item);
      needsDt.hidden = needsDd.hidden = list.length === 0;
      needsDd.replaceChildren(
        ...list.map((n) => {
          const known = n in needs;
          const cls = known ? (needs[n] ? " yes" : " no") : "";
          const mark = known ? (needs[n] ? "✓ " : "✗ ") : "";
          return el("span", { className: `chip${cls}`, title: catalog.needs[n] ?? "", textContent: `${mark}${n}` });
        }),
      );
    },
    runnableHere() {
      const needs = state.info?.needs ?? {};
      return mode !== "link" && !needsHere(item).some((n) => needs[n] === false);
    },
    setActive(run) {
      activeRun = run;
      root.classList.toggle("is-active", Boolean(run));
      status.hidden = !run;
      if (run) {
        status.className = `st st-${run.status}`;
        status.textContent = run.status;
      }
      outputBtn.hidden = !run;
      paintButtons();
    },
    haystack: [item.id, item.label, item.cmd, item.notes, item.verified, item.kind, ...(item.needs ?? []), ...(item.uses ?? [])]
      .join(" ")
      .toLowerCase(),
  };
  paintButtons();
  return card;
}

async function start(card, confirmed = false) {
  const r = await api("/api/start", { id: card.item.id, args: card.values, confirmed });
  if (r.status === 428) {
    if (await ask(r.data.message)) return start(card, true);
    return;
  }
  if (!r.ok) {
    if (r.data?.errors) card.showErrors(r.data.errors);
    toast(r.data?.error ?? `Error ${r.status}`);
    return;
  }
  await refreshRuns();
  selectRun(r.data.run.id);
}

async function stopRun(runId) {
  const r = await api("/api/stop", { run: runId });
  if (!r.ok) toast(r.data?.error ?? `Error ${r.status}`);
  await refreshRuns();
}

async function openTerminal(card, confirmed = false) {
  const r = await api("/api/terminal", { id: card.item.id, args: card.values, confirmed });
  if (r.status === 428) {
    if (await ask(r.data.message)) return openTerminal(card, true);
    return;
  }
  if (!r.ok) {
    if (r.data?.errors) card.showErrors(r.data.errors);
    toast(r.data?.error ?? `Error ${r.status}`);
    return;
  }
  toast(`Opened a terminal for "${card.item.label}". Finish there, then press Re-check.`);
}

function age(run) {
  const end = run.ended ?? Date.now() / 1000;
  const s = Math.max(0, Math.round(end - run.started));
  if (s < 60) return `${s}s`;
  if (s < 3600) return `${Math.floor(s / 60)}m ${String(s % 60).padStart(2, "0")}s`;
  return `${Math.floor(s / 3600)}h ${String(Math.floor(s / 60) % 60).padStart(2, "0")}m`;
}

const runRows = new Map();

function runRow(id) {
  const status = el("span", { className: "st" });
  const name = el("span", { className: "name" });
  const time = el("span", { className: "time" });
  const stop = el("button", { className: "btn sm danger", type: "button", textContent: "Stop" });
  stop.addEventListener("click", (e) => {
    e.stopPropagation();
    stopRun(id);
  });
  const li = el("li", { role: "option", tabIndex: 0, dataset: { run: id } }, status, name, time, stop);
  li.addEventListener("click", () => selectRun(id));
  li.addEventListener("keydown", (e) => {
    if (e.key === "Enter" || e.key === " ") {
      e.preventDefault();
      selectRun(id);
    }
  });
  return {
    li,
    update(run) {
      status.className = `st st-${run.status}`;
      status.textContent = run.status;
      name.textContent = run.label;
      li.title = run.command;
      const exit = run.exitCode !== null && run.status !== "stopped" ? ` · exit ${run.exitCode}` : "";
      time.textContent = `${age(run)}${exit}`;
      stop.hidden = !ACTIVE.has(run.status) || run.status === "stopping";
      li.ariaSelected = String(run.id === state.selected);
    },
  };
}

function renderRuns() {
  const list = $("#runs");
  const active = state.runs.filter((r) => ACTIVE.has(r.status)).length;
  $("#runs-count").textContent = state.runs.length ? `${active} active · ${state.runs.length} total` : "";
  const ids = new Set(state.runs.map((r) => r.id));
  for (const id of runRows.keys()) if (!ids.has(id)) runRows.delete(id);
  if (!state.runs.length) {
    list.replaceChildren(el("li", { className: "empty", textContent: "Nothing started yet. Press Run or Start on an item." }));
    return;
  }
  const rows = state.runs.map((run) => {
    if (!runRows.has(run.id)) runRows.set(run.id, runRow(run.id));
    const row = runRows.get(run.id);
    row.update(run);
    return row.li;
  });
  const current = [...list.children];
  if (current.length !== rows.length || rows.some((li, i) => current[i] !== li)) list.replaceChildren(...rows);
}

function paintLogHead() {
  const run = state.runs.find((r) => r.id === state.selected);
  const status = $("#out-status");
  $("#out-stop").hidden = !run || !ACTIVE.has(run.status) || run.status === "stopping";
  $("#out-copy").disabled = !run;
  if (!run) {
    $("#out-title").textContent = "Output";
    $("#out-cmd").textContent = "";
    status.hidden = true;
    return;
  }
  $("#out-title").textContent = run.label;
  $("#out-cmd").textContent = `$ ${run.command}`;
  status.hidden = false;
  status.className = `st st-${run.status}`;
  status.textContent = run.status;
}
$("#out-stop").addEventListener("click", () => state.selected && stopRun(state.selected));
$("#out-copy").addEventListener("click", () => copy($("#log").textContent, "Output"));

const log = $("#log");
let pending = [];
let flushQueued = false;
let lineCount = 0;

function appendLine(text, className) {
  pending.push([text, className]);
  if (!flushQueued) {
    flushQueued = true;
    requestAnimationFrame(flushLines);
  }
}

function flushLines() {
  flushQueued = false;
  const frag = document.createDocumentFragment();
  for (const [text, className] of pending) {
    frag.append(className ? el("span", { className, textContent: `${text}\n` }) : document.createTextNode(`${text}\n`));
  }
  lineCount += pending.length;
  pending = [];
  log.append(frag);
  while (lineCount > MAX_LOG_LINES && log.firstChild) {
    log.firstChild.remove();
    lineCount--;
  }
  if ($("#follow").checked) log.scrollTop = log.scrollHeight;
}

log.addEventListener("scroll", () => {
  $("#follow").checked = log.scrollHeight - log.scrollTop - log.clientHeight < 24;
});
$("#follow").addEventListener("change", (e) => e.target.checked && (log.scrollTop = log.scrollHeight));

function selectRun(runId) {
  state.source?.close();
  state.selected = runId;
  pending = [];
  lineCount = 0;
  log.textContent = "";
  $("#follow").checked = true;
  const source = new EventSource(`/api/runs/${runId}/log`);
  source.onmessage = (e) => appendLine(JSON.parse(e.data));
  source.addEventListener("end", (e) => {
    source.close();
    const run = JSON.parse(e.data);
    const exit = run.exitCode !== null ? ` (exit code ${run.exitCode})` : "";
    appendLine(`── ${run.status}${exit} ──`, "end");
    refreshRuns();
  });
  state.source = source;
  renderRuns();
  paintLogHead();
}

async function refreshRuns() {
  const r = await api("/api/runs");
  if (!r.ok) {
    if (!state.serverDown) toast(r.data?.error ?? "Lost the dev tool server");
    state.serverDown = true;
    return;
  }
  state.serverDown = false;
  state.runs = r.data;
  renderRuns();
  paintLogHead();
  const active = new Map(state.runs.filter((run) => ACTIVE.has(run.status)).map((run) => [run.item, run]));
  for (const card of state.cards) card.setActive(active.get(card.item.id) ?? null);
  paintNav();

  const key = [...active.keys()].sort().join(",");
  if (key !== state.activeKey) {
    state.activeKey = key;
    for (const card of state.cards) card.schedulePreview();
    refreshInfo();
  }
}

const navButtons = new Map();

function buildNav() {
  const entries = [{ id: "all", label: "All items" }, ...state.catalog.groups];
  const nav = $("#nav");
  const mobile = $("#mobile-nav");
  for (const g of entries) {
    const count = el("span", { className: "n" });
    const live = el("span", { className: "live", hidden: true, title: "Something here is running" });
    const btn = el("button", { type: "button", dataset: { group: g.id } }, el("span", { textContent: g.label }), live, count);
    const chip = el("button", { type: "button", textContent: g.label, dataset: { group: g.id } });
    btn.addEventListener("click", () => setGroup(g.id));
    chip.addEventListener("click", () => setGroup(g.id));
    nav.append(el("li", {}, btn));
    mobile.append(chip);
    navButtons.set(g.id, { btn, chip, count, live });
  }
}

function paintNav(counts) {
  for (const [id, n] of navButtons) {
    const current = String(id === state.group);
    n.btn.setAttribute("aria-current", current);
    n.chip.setAttribute("aria-current", current);
    if (counts) n.count.textContent = counts.get(id) ?? 0;
    n.live.hidden = !state.cards.some((c) => c.active && (id === "all" || c.item.group === id));
  }
}

function applyFilters() {
  const words = $("#q").value.trim().toLowerCase().split(/\s+/).filter(Boolean);
  const runnableOnly = $("#runnable-only").checked;
  const counts = new Map([["all", 0]]);
  let shown = 0;
  for (const card of state.cards) {
    const match = words.every((w) => card.haystack.includes(w)) && (!runnableOnly || card.runnableHere());
    if (match) {
      counts.set("all", counts.get("all") + 1);
      counts.set(card.item.group, (counts.get(card.item.group) ?? 0) + 1);
    }
    const ok = match && (state.group === "all" || card.item.group === state.group);
    card.root.hidden = !ok;
    if (ok) shown++;
  }
  for (const section of document.querySelectorAll(".group")) {
    section.hidden = !section.querySelector(".card:not([hidden])");
  }
  $("#empty").hidden = shown > 0;
  paintNav(counts);
}

let filterTimer;
$("#q").addEventListener("input", () => {
  clearTimeout(filterTimer);
  filterTimer = setTimeout(applyFilters, 60);
});
$("#q").addEventListener("keydown", (e) => {
  if (e.key === "Escape") {
    e.target.value = "";
    applyFilters();
  }
});
$("#runnable-only").addEventListener("change", (e) => {
  store.set("runnable", e.target.checked ? "1" : "");
  applyFilters();
});
document.addEventListener("keydown", (e) => {
  if (e.key !== "/" || e.target.closest("input, select, textarea, dialog")) return;
  e.preventDefault();
  $("#q").focus();
});

function setGroup(groupId) {
  state.group = navButtons.has(groupId) ? groupId : "all";
  store.set("group", state.group);
  applyFilters();
  $("#main").scrollTo({ top: 0 });
}

async function boot() {
  const r = await api("/api/catalog");
  if (!r.ok) {
    toast(r.data?.error ?? "Couldn't load the catalog");
    return;
  }
  state.catalog = r.data;
  const groups = $("#groups");
  for (const group of state.catalog.groups) {
    const cards = state.catalog.items.filter((it) => it.group === group.id).map(makeCard);
    state.cards.push(...cards);
    groups.append(
      el(
        "section",
        { className: "group", id: `group-${group.id}`, ariaLabel: group.label },
        el("div", { className: "group-head" }, el("h2", { textContent: group.label }), el("p", { textContent: group.description })),
        el("div", { className: "cards" }, ...cards.map((c) => c.root)),
      ),
    );
  }
  buildNav();
  $("#runnable-only").checked = store.get("runnable") === "1";
  state.group = store.get("group") ?? "all";
  if (!navButtons.has(state.group)) state.group = "all";
  await refreshInfo();
  await refreshRuns();
  await Promise.all(state.cards.map((card) => card.preview()));
  applyFilters();
  const target = location.hash.match(/^#item-(.+)$/);
  const card = target && state.cards.find((c) => c.item.id === decodeURIComponent(target[1]));
  if (card) {
    card.setOpen(true);
    card.root.scrollIntoView({ block: "start" });
  }
  document.body.dataset.ready = "1";
  setInterval(refreshRuns, 1000);
  setInterval(refreshInfo, 10000);
}

boot();
