# billee-dev

A browser launcher for everything in this repo you'd otherwise type into terminals: sims,
drivetrain modes, builds, health checks, CAN tools, bags. Pick an item, set its arguments,
press Start, and watch its output in the page.

```bash
devtool/billee-dev                 # opens http://localhost:7700
devtool/billee-dev --port 7701     # if 7700 is taken
devtool/billee-dev --no-browser    # server only (e.g. on the Jetson: ssh -L 7700:localhost:7700 <rover>)
```

Ctrl-C in that terminal stops the tool **and everything it started**. Needs only `python3`
(3.8+, standard library). On a new machine, run **Machine setup > Set up this machine** first
(or `devtool/setup-machine` in a terminal): it installs what's missing and builds the workspace.

## Setting up a machine

`devtool/setup-machine` (default `auto`) picks the setup that fits the machine it runs on:

| Machine | Setup | What it does |
|---|---|---|
| x86-64 Linux + NVIDIA GPU | `x86` | `make setup x86`: NVIDIA driver, Docker, NVIDIA toolkit, desktop image |
| x86-64 Linux, no NVIDIA GPU | `x86-cpu` | apt packages (`curl can-utils kmod xvfb`), Pixi, the `default` env and a native build; no Docker |
| NVIDIA Jetson | `l4t` | `make setup l4t` |
| other ARM64 Linux | `linux-aarch64` | `make setup linux-aarch64` |
| Apple-Silicon Mac | `mac` | `make setup mac` |

A platform that doesn't fit the machine is refused before anything is installed, and the page
disables it in the dropdown. `DRY_RUN=1` (the **Preview the setup** item) shows the steps
without changing anything; `SKIP_APT=1` skips the apt packages.

## Files

| File | What it does |
|---|---|
| `billee-dev` | The executable: starts `server.py`, opens the browser, passes Ctrl-C on |
| `setup-machine` | Installs and builds what this machine needs (see above) |
| `server.py` | Checks arguments, builds the command, runs it, streams output, enforces conflicts |
| `catalog.json` | Every item: command, arguments, prerequisites, conflicts, ready signal |
| `web/index.html`, `web/app.js` | The page |
| `tests/` | Unit, catalog, HTTP and browser tests (fixture catalog, no ROS needed) |
| `smoke_test.py` | End-to-end test against the real stack (mock drivetrain) |

## How a click becomes a command

1. The page sends only `{"id": "drive.pub", "args": {"v": "0.2"}}`, never a command.
2. `server.py` looks the item up in `catalog.json` and checks each argument against its spec:
   numbers must be numbers, enums one of their choices, text only letters, digits, spaces and
   `. / : # = + @ % , - _` (no quotes, `$`, `;`, `|`, `&`, `>`, backticks), so a value can't
   break out of the command.
3. It fills the command in (`{{v}}` placeholders, `name:=value` for ROS launch arguments,
   `NAME=value` prefixes, `--flags`) and adds this machine's prefix: `tooling/host-ros2` on a
   natively built x86 laptop, `tooling/desktop-ros2 run pixi run -e …` in Docker, and so on.
4. It refuses to start an item this machine can't run (no NVIDIA GPU, wrong kind of
   machine, Pixi missing), one that would clash with a running one (two drivetrains, two
   bridges on :8765, two gamepad teleops), or one whose port another program already holds.
5. It runs the command in its own process group. Stop sends Ctrl-C (SIGINT) to the whole group,
   then SIGTERM and SIGKILL if it hasn't exited after 8 and 12 seconds.

Items that need a password (CAN setup) or keyboard input (keyboard teleop, container shells)
open in a terminal window instead (`Open in terminal`). Items marked **asks first** show a
confirmation before they run or open.

The server listens on 127.0.0.1 only, and rejects POSTs that aren't JSON or come from another
site's page, so a website open in your browser can't start commands.

## Adding or changing an item

Edit `catalog.json` and reload the page (the server re-reads it when it changes):

```json
{
  "id": "health.odom-echo", "group": "health", "label": "Watch odometry", "kind": "stream", "runner": "ros",
  "cmd": "ros2 topic echo {{topic}} nav_msgs/msg/Odometry",
  "args": [ { "name": "topic", "type": "string", "default": "/diff_drive_controller/odom", "style": "template" } ],
  "needs": ["drivetrain"], "status": "untested"
}
```

- `kind`: `task` (runs to the end), `service` (runs until stopped), `stream`, `gui`, `interactive`, `link`
- `runner`: `ros` (inside the ROS 2 environment), `host`, `host-sudo`, `link`
- argument `type`: `bool`, `enum` (+ `choices`), `number`, `string`
- argument `style`: `template` (`{{name}}` in `cmd`), `ros` (appends `name:=value` when changed),
  `env` (prefixes `ENV=value`), `flag` (appends `flag` when true)
- `needs`: what it expects on the machine (see `catalog.needs`); a missing need is a warning,
  except the ones in `catalog.blocking` (hardware, Pixi), which refuse the start
- `needsWhen`: needs that apply only for one argument value
  (`{"arg": "platform", "is": ["x86"], "needs": ["nvidia"]}`) or on some platforms
  (`{"platform": ["x86-native"], "needs": ["nvidia"]}`). Blocked choices are disabled in the page;
  `blockedHint` adds a sentence to the refusal
- `uses` / `usesWhen`: resources it holds; the ones listed in `exclusive` block each other
- `ready`: regex on the output that means "up" (otherwise it's "running" as soon as it starts)
- `confirm: true`: the page asks before running it (or opening its terminal)

Pass the message type to `ros2 topic echo` (as above): without it, echo asks the ros2 CLI daemon,
which can give stale answers.

## Testing

```bash
python3 -m unittest discover -s devtool/tests -v   # seconds; no ROS needed
python3 devtool/smoke_test.py                       # about a minute; real stack, port 8765 must be free
```

`tests/` runs the server against a fixture catalog of harmless shell commands:

- `test_units.py`: argument checks, command building, needs, resources, platform detection
- `test_catalog.py`: every item in `catalog.json` is valid and builds on every platform, and
  every repo script it calls exists and is executable
- `test_http.py`: the API end to end: runs, log streaming and resume, ready signals,
  conflicts, Stop escalation, confirmation, terminals, Host/Origin checks, hot reload
- `test_ui.py`: the page in headless Chromium (skipped without Playwright:
  `pip install playwright && playwright install chromium`)

`smoke_test.py` starts its own server on port 7799 against the real catalog: mock drivetrain,
a controller check, driving with `v=0.2` and reading 1.818 rad/s back from the wheels,
stopping, and bag recording.

## Known limits

- RViz, Gazebo and rqt open their own windows; they can't be shown inside the page.
