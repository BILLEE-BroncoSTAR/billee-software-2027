const $ = (sel) => document.querySelector(sel);
const el = (tag, { dataset, ...props } = {}, ...kids) => {
  const node = Object.assign(document.createElement(tag), props);
  if (dataset) Object.assign(node.dataset, dataset);
  node.append(...kids.filter((k) => k !== null && k !== undefined && k !== false));
  return node;
};
const code = (text) => el("code", { textContent: text });

const ACTIVE = new Set(["starting", "running", "stopping"]);
const NEED_LABELS = {
  display: "display",
  vcan0: "vcan0",
  can0: "can0",
  gamepad: "gamepad",
  docker: "docker",
  terminal: "terminal app",
  drivetrain: "drivetrain running",
};

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

async function copy(text) {
  try {
    await navigator.clipboard.writeText(text);
    toast("Command copied");
  } catch {
    toast("Couldn't copy; select the command and copy it by hand");
  }
}

async function refreshInfo() {
  const r = await api("/api/info");
  if (!r.ok) return;
  state.info = r.data;
  $("#env").textContent = `Platform: ${r.data.platform}`;
  $("#platform").replaceChildren(code(r.data.platform), ` ${r.data.platformDescription}`);
  const parts = Object.entries(NEED_LABELS).flatMap(([key, label], i) => {
    const ok = r.data.needs[key];
    return [i ? ", " : "", label, " ", el("span", { className: ok ? "yes" : "no", textContent: ok ? "(yes)" : "(no)" })];
  });
  $("#needs").replaceChildren(...parts);
  for (const card of state.cards) card.paintNeeds();
  applyFilters();
}

$("#recheck").addEventListener("click", async () => {
  await refreshInfo();
  for (const card of state.cards) card.schedulePreview();
  toast("Re-checked display, CAN, gamepad, Docker and running drivetrains");
});

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
    input = el("select", { id }, ...spec.choices.map((c) => el("option", { value: c, textContent: c === "" ? "(auto)" : c })));
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
  const error = el("span", { className: "err", hidden: true });
  const name = spec.style === "env" ? `${spec.env}=` : spec.name;
  const field = el(
    "dd",
    { className: "field" },
    el("label", { htmlFor: id }, code(name)),
    " - ",
    input,
    spec.help ? ` ${spec.help}` : "",
    error,
  );
  return { field, error, input };
}

function makeItem(item) {
  const catalog = state.catalog;
  const mode = modeOf(item);
  const values = {};
  const args = {};
  let seq = 0;
  let timer = null;
  let activeRun = null;
  let blocked = null;
  let hasErrors = false;

  const signature = el("pre", { className: "signature", textContent: "…" });
  const messages = el("div", { className: "msgs" });
  const runStatus = el("span", { className: "st", hidden: true });
  const primary = el("button", { type: "button", className: "primary" });
  const outputBtn = el("button", { type: "button", className: "secondary", textContent: "Show output", hidden: true });
  const copyBtn = el("button", { type: "button", className: "secondary", textContent: "Copy command" });
  copyBtn.addEventListener("click", () => copy(signature.textContent));
  outputBtn.addEventListener("click", () => activeRun && selectRun(activeRun.id));

  const dl = el("dl");
  if (item.args?.length) {
    dl.append(el("dt", { textContent: "Arguments:" }));
    for (const spec of item.args) {
      const ctl = argControl(item, spec, values, (v) => {
        values[spec.name] = v;
        schedulePreview();
      });
      args[spec.name] = ctl;
      dl.append(ctl.field);
    }
  }
  const needsDd = item.needs?.length ? el("dd") : null;
  if (needsDd) dl.append(el("dt", { textContent: "Needs:" }), needsDd);
  const uses = [
    ...(item.uses ?? []),
    ...Object.entries(item.usesWhen ?? {}).flatMap(([arg, res]) => res.map((u) => `${u} (when ${arg} is on)`)),
  ];
  if (uses.length) dl.append(el("dt", { textContent: "Uses:" }), el("dd", { textContent: uses.join(", ") }));
  if (item.ready) dl.append(el("dt", { textContent: "Ready when the output matches:" }), el("dd", {}, code(item.ready)));
  dl.append(
    el("dt", { textContent: "Tested:" }),
    el("dd", {}, item.status === "broken" ? el("span", { className: "deprecationLabel", textContent: "Broken. " }) : null,
      item.verified ?? (item.status === "untested" ? "Not tested yet." : item.status)),
    el("dt", { textContent: "Identifier:" }),
    el("dd", {}, code(item.id), " · kind ", code(item.kind), " · runner ", code(item.runner)),
  );

  let hint = null;
  if (mode === "run") {
    primary.addEventListener("click", () => (activeRun ? stopRun(activeRun.id) : start(card)));
  } else if (mode === "terminal") {
    primary.textContent = "Open in terminal";
    primary.addEventListener("click", () => openTerminal(card));
    hint = el("span", {
      className: "hint",
      textContent: (item.needs ?? []).includes("sudo") || item.runner === "host-sudo"
        ? "Asks for your sudo password in the terminal."
        : "Needs keyboard input in the terminal.",
    });
  } else {
    const target = item.cmd.startsWith("http") ? item.cmd : null;
    primary.textContent = target ? "Open" : "Copy";
    primary.addEventListener("click", () => (target ? window.open(target, "_blank", "noopener") : copy(item.cmd)));
  }

  const detail = el(
    "li",
    { className: "memberBlock", id: `card-${item.id}` },
    el("h4", { className: "memberName", textContent: item.label }),
    signature,
    item.notes ? el("div", { className: "block", textContent: item.notes }) : null,
    dl,
    messages,
    el("div", { className: "actions" }, primary, mode !== "link" ? copyBtn : null, outputBtn, runStatus, hint),
  );

  const rowStatus = el("span", { className: "st", hidden: true });
  const rowBtn = el("button", { type: "button", className: "small" });
  rowBtn.addEventListener("click", () => primary.click());
  const row = el(
    "tr",
    { id: `row-${item.id}` },
    el("td", { className: "colFirst" }, code(item.kind)),
    el("td", { className: "colSecond" }, el("a", { href: `#card-${item.id}` }, code(item.label))),
    el(
      "td",
      { className: "colLast" },
      el(
        "div",
        { className: "block" },
        item.status === "broken" ? el("span", { className: "deprecationLabel", textContent: "Broken. " }) : null,
        firstSentence(item.notes) || firstSentence(item.verified),
      ),
    ),
    el("td", { className: "colAction" }, rowStatus, " ", rowBtn),
  );

  function showErrors(errors = {}) {
    hasErrors = false;
    for (const [name, ctl] of Object.entries(args)) {
      const msg = errors[name];
      ctl.field.classList.toggle("has-error", Boolean(msg));
      ctl.error.hidden = !msg;
      ctl.error.textContent = msg ?? "";
      if (msg) hasErrors = true;
    }
    if (errors._) toast(errors._);
  }

  function paintButtons() {
    let label = primary.textContent;
    let cls = "primary";
    let disabled = hasErrors;
    if (mode === "run") {
      if (activeRun) {
        label = activeRun.status === "stopping" ? "Stopping…" : "Stop";
        cls = "stop";
        disabled = activeRun.status === "stopping";
      } else {
        label = item.kind === "task" ? "Run" : "Start";
        disabled = hasErrors || Boolean(blocked);
      }
    }
    primary.textContent = label;
    primary.className = cls;
    primary.disabled = disabled;
    rowBtn.textContent = label;
    rowBtn.className = `small${cls === "stop" ? " stop" : ""}`;
    rowBtn.disabled = disabled;
  }

  async function preview() {
    const mine = ++seq;
    const r = await api("/api/preview", { id: item.id, args: values });
    if (mine !== seq || !r.ok) return;
    const p = r.data;
    showErrors(p.errors);
    messages.replaceChildren();
    if (!p.ok) {
      signature.textContent = "Fix the highlighted argument to see the command.";
      blocked = null;
    } else {
      signature.textContent = p.command;
      blocked = activeRun ? null : p.blocked;
      for (const w of p.warnings) {
        messages.append(el("p", { className: "msg warn" }, el("b", { textContent: "Warning: " }), w));
      }
      if (blocked) messages.append(el("p", { className: "msg block-msg" }, el("b", { textContent: "Blocked: " }), blocked));
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
    detail,
    row,
    root: detail,
    values,
    showErrors,
    schedulePreview,
    preview,
    paintNeeds() {
      if (!needsDd) return;
      const needs = state.info?.needs ?? {};
      needsDd.replaceChildren(
        ...item.needs.flatMap((n, i) => {
          const known = n in needs;
          return [
            i ? ", " : "",
            el("span", { title: catalog.needs[n], textContent: n }),
            known ? el("span", { className: needs[n] ? "yes" : "no", textContent: needs[n] ? " (yes)" : " (no)" }) : "",
          ];
        }),
      );
    },
    runnableHere() {
      const needs = state.info?.needs ?? {};
      return mode !== "link" && !(item.needs ?? []).some((n) => needs[n] === false);
    },
    setActive(run) {
      activeRun = run;
      detail.classList.toggle("active", Boolean(run));
      for (const s of [runStatus, rowStatus]) {
        s.hidden = !run;
        if (run) {
          s.className = `st st-${run.status}`;
          s.textContent = run.status;
        }
      }
      outputBtn.hidden = !run;
      paintButtons();
    },
    haystack: [item.id, item.label, item.cmd, item.notes, item.verified, ...(item.needs ?? []), ...(item.uses ?? [])]
      .join(" ")
      .toLowerCase(),
  };
  paintButtons();
  return card;
}

async function start(card, confirmed = false) {
  const r = await api("/api/start", { id: card.item.id, args: card.values, confirmed });
  if (r.status === 428) {
    if (window.confirm(r.data.message)) return start(card, true);
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

async function openTerminal(card) {
  const r = await api("/api/terminal", { id: card.item.id, args: card.values });
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
  return s < 60 ? `${s}s` : `${Math.floor(s / 60)}m${String(s % 60).padStart(2, "0")}s`;
}

function renderRuns() {
  const list = $("#runs");
  const active = state.runs.filter((r) => ACTIVE.has(r.status)).length;
  $("#runs-count").textContent = state.runs.length ? `${active} active, ${state.runs.length} total` : "";
  if (!state.runs.length) {
    list.replaceChildren(el("li", { className: "empty", textContent: "Nothing started yet." }));
    return;
  }
  list.replaceChildren(
    ...state.runs.map((run) => {
      const stop = ACTIVE.has(run.status) && run.status !== "stopping"
        ? el("button", { className: "stop small", type: "button", textContent: "Stop" })
        : null;
      stop?.addEventListener("click", (e) => {
        e.stopPropagation();
        stopRun(run.id);
      });
      const exit = run.exitCode !== null && run.status !== "stopped" ? `, exit ${run.exitCode}` : "";
      const li = el(
        "li",
        { className: run.id === state.selected ? "selected" : "", title: run.command },
        el("span", { className: `st st-${run.status}`, textContent: run.status }),
        el("span", { className: "name", textContent: run.label }),
        el("span", { className: "meta", textContent: `${age(run)}${exit}` }),
        stop,
      );
      li.addEventListener("click", () => selectRun(run.id));
      return li;
    }),
  );
}

function paintLogHead() {
  const run = state.runs.find((r) => r.id === state.selected);
  const status = $("#log-status");
  const stop = $("#log-stop");
  if (!run) {
    $("#log-title").textContent = "no run selected";
    $("#log-cmd").textContent = "";
    status.hidden = true;
    stop.hidden = true;
    return;
  }
  $("#log-title").textContent = run.label;
  $("#log-cmd").textContent = `$ ${run.command}`;
  status.hidden = false;
  status.className = `st st-${run.status}`;
  status.textContent = run.status;
  stop.hidden = !ACTIVE.has(run.status) || run.status === "stopping";
}
$("#log-stop").addEventListener("click", () => state.selected && stopRun(state.selected));

let pending = [];
let flushQueued = false;
let lineCount = 0;
const MAX_LOG_LINES = 4000;

function appendLine(text) {
  pending.push(text);
  if (!flushQueued) {
    flushQueued = true;
    requestAnimationFrame(flushLines);
  }
}

function flushLines() {
  flushQueued = false;
  const log = $("#log");
  const frag = document.createDocumentFragment();
  for (const line of pending) frag.append(document.createTextNode(line + "\n"));
  lineCount += pending.length;
  pending = [];
  log.append(frag);
  while (lineCount > MAX_LOG_LINES && log.firstChild) {
    log.firstChild.remove();
    lineCount--;
  }
  if ($("#follow").checked) log.scrollTop = log.scrollHeight;
}

function selectRun(runId) {
  state.source?.close();
  state.selected = runId;
  pending = [];
  lineCount = 0;
  $("#log").textContent = "";
  const source = new EventSource(`/api/runs/${runId}/log`);
  source.onmessage = (e) => appendLine(JSON.parse(e.data));
  source.addEventListener("end", (e) => {
    source.close();
    const run = JSON.parse(e.data);
    const exit = run.exitCode !== null ? ` (exit code ${run.exitCode})` : "";
    appendLine(`-- ${run.status}${exit} --`);
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

  const key = [...active.keys()].sort().join(",");
  if (key !== state.activeKey) {
    state.activeKey = key;
    for (const card of state.cards) card.schedulePreview();
    refreshInfo();
  }
}

function applyFilters() {
  const words = $("#q").value.trim().toLowerCase().split(/\s+/).filter(Boolean);
  const runnableOnly = $("#runnable-only").checked;
  let shown = 0;
  let stripe = 0;
  for (const card of state.cards) {
    const ok =
      (state.group === "all" || card.item.group === state.group) &&
      words.every((w) => card.haystack.includes(w)) &&
      (!runnableOnly || card.runnableHere());
    card.detail.hidden = !ok;
    card.row.hidden = !ok;
    if (ok) {
      card.row.className = stripe++ % 2 ? "rowColor" : "altColor";
      shown++;
    }
  }
  for (const section of document.querySelectorAll("li.groupBlock")) {
    section.hidden = !section.querySelector("li.memberBlock:not([hidden])");
  }
  $("#count").textContent = `${shown} of ${state.cards.length} items`;
}
$("#q").addEventListener("input", applyFilters);
$("#runnable-only").addEventListener("change", applyFilters);

function setGroup(groupId) {
  state.group = groupId;
  for (const tab of document.querySelectorAll(".tableTabs .tab")) {
    tab.classList.toggle("active", tab.dataset.group === groupId);
  }
  applyFilters();
}

async function boot() {
  const r = await api("/api/catalog");
  if (!r.ok) {
    toast(r.data?.error ?? "Couldn't load the catalog");
    return;
  }
  state.catalog = r.data;
  const tabs = $("#tabs");
  const links = $("#group-links");
  const tabFor = (id, label) => {
    const tab = el("button", { type: "button", className: "tab", textContent: label, dataset: { group: id } });
    tab.addEventListener("click", () => setGroup(id));
    return tab;
  };
  tabs.append(tabFor("all", "All Items"));
  state.catalog.groups.forEach((group, i) => {
    const items = state.catalog.items.filter((it) => it.group === group.id);
    const cards = items.map(makeItem);
    state.cards.push(...cards);
    tabs.append(tabFor(group.id, group.label));
    $("#summary-rows").append(...cards.map((c) => c.row));
    $("#details").append(
      el(
        "li",
        { className: "groupBlock", id: `group-${group.id}` },
        el("h3", { className: "memberBar", textContent: group.label }),
        el("div", { className: "block", textContent: group.description }),
        el("ul", { className: "blockList" }, ...cards.map((c) => c.detail)),
      ),
    );
    links.append(i ? " | " : "", el("a", { href: `#group-${group.id}`, textContent: group.label }));
  });
  setGroup("all");
  await refreshInfo();
  await refreshRuns();
  for (const card of state.cards) card.preview();
  applyFilters();
  setInterval(refreshRuns, 1000);
  setInterval(refreshInfo, 10000);
}

boot();
