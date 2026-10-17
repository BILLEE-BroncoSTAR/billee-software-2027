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
(3.8+, standard library) and a built workspace (`tooling/host-ros2 build` or `make build`).

## Files

| File | What it does |
|---|---|
| `billee-dev` | The executable: starts `server.py`, opens the browser, passes Ctrl-C on |
| `server.py` | Checks arguments, builds the command, runs it, streams output, enforces conflicts |
| `catalog.json` | Every item: command, arguments, prerequisites, conflicts, ready signal |
| `web/index.html`, `web/app.js` | The page |
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
4. It refuses to start an item that would clash with a running one (two drivetrains, two
   bridges on :8765, two gamepad teleops), or whose port another program already holds.
5. It runs the command in its own process group. Stop sends Ctrl-C (SIGINT) to the whole group,
   then SIGTERM and SIGKILL if it hasn't exited after 8 and 12 seconds.

Items that need a password (CAN setup) or keyboard input (keyboard teleop, container shells)
open in a terminal window instead (`Open in terminal`).

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
- `uses` / `usesWhen`: resources it holds; the ones listed in `exclusive` block each other
- `ready`: regex on the output that means "up" (otherwise it's "running" as soon as it starts)
- `confirm: true`: the page asks before running it

Pass the message type to `ros2 topic echo` (as above): without it, echo asks the ros2 CLI daemon,
which can give stale answers.

## Testing

```bash
python3 devtool/smoke_test.py      # about a minute; nothing may hold port 8765 while it runs
```

It starts its own server on port 7799 and checks argument handling and refusals, the
cross-site protections, conflicts, and real runs: mock drivetrain, a controller check,
driving with `v=0.2` and reading 1.818 rad/s back from the wheels, stopping, and bag
recording.

## Known limits

- On this laptop `make build/shell/sim` fail (Docker isn't reachable); the tool uses `tooling/host-ros2`,
  which is not committed yet. Commit it with this folder.
- `odesc_vcan_emulator.py` lacks the executable bit in git; the catalog runs it with `python3` for now.
- RViz, Gazebo and rqt open their own windows; they can't be shown inside the page.
