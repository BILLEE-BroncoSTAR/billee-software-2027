# Plan: Fix the Remaining `colcon test` Failures

Status 2026-11-02, branch `Prerelease-PoseDev` @ `a2745f8`, run on the dev Orin Nano (`l4t` env).
The same failures appear in the x86 `default` env: the linters and their versions are identical.

```
Summary: 144 tests, 0 errors, 51 failures, 19 skipped
```

**None of these failures is a functional test.** Every functional test passes (gestures, 3D lift,
zed2i, robot_description). The failures come from **linters**: tools that check code style and file
format. ROS packages turn them on with `ament_lint_auto` in `CMakeLists.txt`, and each one reports
as a test. Fixing them changes formatting only, not behaviour.

## 1. What fails, grouped by cause

The 51 counts 15 twice: each failing package also reports a CTest summary entry that fails
whenever any of its linters fails. That leaves **36 actual findings**:

| # | Linter | What it checks | Findings | Packages | Root cause |
| --- | --- | --- | --- | --- | --- |
| A | `xmllint` | `package.xml` against ROS's XML schema | 5 | chassis_bringup, comms, navigation, odesc, teleop | **Environment.** pixi's libxml2 2.15 is built without HTTP, so it can't download the schema the files reference. The files themselves are valid (verified with a local copy). |
| B | `flake8` | Python style (PEP 8) | 16 | chassis_bringup (11), navigation (3), odesc (1), teleop (1) | Real: long lines, missing spaces, comment format, blank lines, one unused import |
| C | `pep257` | Python docstring format | 5 | chassis_bringup | Real: module docstrings start on the first line; ament's convention (D213) wants the summary on the second line |
| D | `uncrustify` | C++ formatting (ROS 2 style) | 9 | comms (2), navigation (1), odesc (5), teleop (1) | Real: brace placement, initialiser lists on their own line, missing final newline |
| E | `lint_cmake` | CMake style | 1 | teleop | Real: trailing whitespace on `CMakeLists.txt:40` |

Not failures, but seen in the same logs (fixed in step 6):

- CMake deprecation warnings `Compatibility with CMake < 3.10 will be removed`: `comms`, `odesc`,
  `navigation`, `zed2i` declare `cmake_minimum_required(VERSION 3.8)`.
- `people/config/gestures.yaml` says `hold_frames ... ~1 Hz on CPU`, a stale comment from the
  removed CPU pose prototype (OpenPose now runs on the GPU at ~6 Hz).

## 2. Ground rules

- **Separate branch**, e.g. `fix/lint` from `Prerelease-PoseDev` (it carries the drivetrain changes
  of `be96a49`, so branching from `main` would conflict). Keep it out of the pose work.
- **One commit per step below**, so a formatting change can be reviewed (or reverted) on its own.
- **Use the linters from the pixi env**, not system ones: the env pins the versions colcon uses
  (uncrustify 0.83, flake8 7.3, pycodestyle 2.14). Different versions format differently.
- **Formatting-only diffs.** If a fix needs a logic change (e.g. the unused import turns out to be
  needed), stop and review it separately.
- All commands below run from `ros2_ws/` in either env; examples use `-e default`.

## 3. Steps

### Step 1: xmllint (A), 5 findings: serve the schema locally

The `<?xml-model href="http://download.ros.org/schema/package_format3.xsd" ...?>` line in each
`package.xml` makes `ament_xmllint` run `xmllint --schema <that URL>`. An **XML catalog** tells
libxml2 to use a local file for that URL. Verified on this env: with the catalog all 5 files
validate.

1. Add the schema and a catalog under `tooling/xml/`:

   ```bash
   mkdir -p tooling/xml
   curl -fsSL -o tooling/xml/package_format3.xsd http://download.ros.org/schema/package_format3.xsd
   sha256sum tooling/xml/package_format3.xsd
   # expect 585201809cbf8c81ec5a4d7d84d827f4676dd62a4bc0a812da7a485dea2ad43a
   ```

   `tooling/xml/catalog.xml`:

   ```xml
   <?xml version="1.0"?>
   <catalog xmlns="urn:oasis:names:tc:entity:xmlns:xml:catalog">
     <uri name="http://download.ros.org/schema/package_format3.xsd" uri="package_format3.xsd"/>
     <system systemId="http://download.ros.org/schema/package_format3.xsd" uri="package_format3.xsd"/>
   </catalog>
   ```

2. Point every pixi env at it, in `ros2_ws/pixi.toml` (top level, so all features get it):

   ```toml
   [activation.env]
   # pixi's libxml2 has no HTTP: ament_xmllint's package.xml schema comes from a local copy
   XML_CATALOG_FILES = "$PIXI_PROJECT_ROOT/../tooling/xml/catalog.xml"
   ```

3. Check: `pixi run -e default ament_xmllint src/teleop/package.xml` → `No problems found`.

Rejected alternative: deleting the `xml-model` line from each `package.xml` also passes, but turns
schema validation off for good.

### Step 2: flake8 (B), 16 findings

| File | Line:col | Code | Fix |
| --- | --- | --- | --- |
| `chassis_bringup/launch/sim_gz.launch.py` | 70:100, 282:100, 314:100 | E501 | wrap to ≤ 99 characters |
| | 139:30 | E231 | space after `,` |
| | 158:90 | W291 | delete trailing whitespace |
| | 218:5, 314–317:5 | E265 | `#comment` → `# comment` |
| | 313:1 | E302 | two blank lines before the function |
| `navigation/launch/pose_estim.launch.py` | 52:25, 111:22 | W291 | delete trailing whitespace |
| | 110:5 | E303 | remove the extra blank line |
| `odesc/scripts/odesc_vcan_emulator.py` | 5:1 | F401 | remove `import math` (first `grep -n math` the file to confirm it is unused) |
| `teleop/launch/teleop.launch.py` | 7:1 | E302 | two blank lines before the function |

Lines 314–317 of `sim_gz.launch.py` are a comment explaining why an `OpaqueFunction` is needed:
keep it, add the space after `#` and wrap line 314.

Check: `pixi run -e default ament_flake8 src/chassis_bringup src/navigation src/odesc src/teleop`.

### Step 3: pep257 (C), 5 findings

Module docstrings in `chassis_bringup/launch/{ground_station,odesc_shadow,real,rover,viz}.launch.py`
start their summary on the opening line. ament's pep257 convention requires (D213):

```python
"""
Summary line.

Details...
"""
```

Edit by hand (one line each) and check with `pixi run -e default ament_pep257 src/chassis_bringup`.

The perception packages (`people`, etc.) use the other style but aren't linted (step 6 decides
whether to start linting them).

### Step 4: uncrustify (D), 9 findings: auto-format

```bash
pixi run -e default ament_uncrustify --reformat \
  src/comms/include/comms/SerialDevice.hpp src/comms/src/GpsNode.cpp \
  src/navigation/include/navigation/straight_line_planner.hpp \
  src/odesc/include/odesc/can.hpp src/odesc/include/odesc/drive_backend.hpp \
  src/odesc/include/odesc/mock_drive_backend.hpp \
  src/odesc/include/odesc/socketcan_drive_backend.hpp src/odesc/src/odesc.cpp \
  src/teleop/src/joy_teleop.cpp
git diff --stat   # expect only these 9 files
```

Then **read the diff**. The two big ones are `comms/src/GpsNode.cpp` (~350-line diff) and
`comms/include/comms/SerialDevice.hpp` (~115 lines); the odesc ones are a few lines each
(e.g. moving `: info_(std::move(info))` to its own line; adding the missing newline at the end of
`straight_line_planner.hpp`). Uncrustify changes whitespace and brace layout only. If a diff touches
anything else (macro bodies, comments that carry meaning), revert that hunk and fix it by hand.

Because odesc and comms drive real hardware, confirm behaviour after formatting (step 7).

### Step 5: lint_cmake (E), 1 finding

Delete the trailing whitespace on `src/teleop/CMakeLists.txt` line 40.
Check: `pixi run -e default ament_lint_cmake src/teleop/CMakeLists.txt`.

### Step 6: cleanups and decisions

1. `cmake_minimum_required(VERSION 3.8)` → `3.10` in `comms`, `odesc`, `navigation` and
   `perception/zed2i` `CMakeLists.txt` (removes the deprecation warnings; the pixi envs ship
   CMake 3.31).
2. `perception/people/config/gestures.yaml`: change the `hold_frames` comment to
   `~0.3 s at ~6 Hz on the Orin Nano`.
3. **Decision for the team:** `people`, `people_msgs` and `zed_openpose` run their functional tests
   but no linters. Turning linters on (`ament_lint_auto` + `test_flake8.py`/`test_pep257.py` for
   `people`) would keep them consistent with the rest, but will surface new findings (e.g. D213 in
   `gesture_node.py`). Recommended: do it in a follow-up on the pose branch, not here.

### Step 7: verify

1. Full lint + test run in one env (x86 is fine; nothing here depends on the Jetson):

   ```bash
   rm -rf build/*/test_results
   pixi run -e default colcon build --symlink-install
   pixi run -e default colcon test
   pixi run -e default colcon test-result --all | tail -1
   # target: "0 errors, 0 failures"
   ```

2. Behaviour check of the reformatted C++ (formatting shouldn't change it, but these nodes drive
   motors):
   - ODESC shadow test on vCAN (`docs/RUN_GUIDE.md` §5, "ODESC shadow test"): drive the sim, confirm
     the ODESC path sees the same commands as before.
   - teleop: gamepad drives the sim rover.
   - comms: `GpsNode` starts, publishes NavSatFix (on the rover if a GPS is attached, otherwise
     confirm it starts and waits on the serial port as before).

3. Re-run in the `l4t` env on the Jetson to confirm the same result there.

## 4. Effort and risk

| Step | Effort | Risk |
| --- | --- | --- |
| 1 xmllint | 15 min | none (test-only environment change) |
| 2 flake8 | 20 min | low; launch files only, plus one import |
| 3 pep257 | 5 min | none |
| 4 uncrustify | 30 min incl. reviewing the diff | low; formatting of drivetrain/GPS code, hence step 7 |
| 5 lint_cmake | 1 min | none |
| 6 cleanups | 10 min | none |
| 7 verify | 30–45 min | catches anything above |

About 2 hours in total, one reviewable PR with 6–7 commits.

## 5. Keeping it at zero afterwards

There is no CI in the repository, so failures accumulate silently. Cheapest guard: a task in the
existing `[tasks]` table of `ros2_ws/pixi.toml` that runs every test and linter, run before pushing
(`colcon test-result` exits non-zero on any failure):

```toml
[tasks]
lint = "colcon test --event-handlers console_cohesion- && colcon test-result"
```

A GitHub Actions job running the same command on the x86 `default` env would catch it on every PR.
