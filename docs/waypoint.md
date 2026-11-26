# Waypoint navigation with Nav2

> **Work in progress.** Tested in the Gazebo sim only. Nothing here has run on the real rover yet.

Code:
- [`ros2_ws/src/navigation/`](../ros2_ws/src/navigation): `launch/nav2.launch.py`, `config/nav2_params.yaml`, `behavior_trees/`, `scripts/`.
- [`ros2_ws/src/chassis_bringup/`](../ros2_ws/src/chassis_bringup): `config/twist_mux.yaml`, `foxglove/navigation.json`.

The rover drives itself to a goal pose (a waypoint) with Nav2, avoiding obstacles seen by the depth camera. You can send goals from Foxglove or from a terminal. The joystick or keyboard can always take over.

## Start up

All terminals are VS Code terminals inside the devcontainer, started in `ros2_ws`. The first time
after pulling, install the new dependency (`twist_mux`) and build:

```bash
pixi install
pixi run build
```

**Terminal 1, the sim.** Leave it running. Wait for Gazebo to show the rover and for
`Managed nodes are active` in the log.

```bash
# Obstacle test world for object avoidence
pixi run ros2 launch chassis_bringup rover.launch.py mode:=sim gui:=true nav_mode:=full world_file:=worlds/nav_test.sdf
# Or the default (empty) world
pixi run ros2 launch chassis_bringup rover.launch.py mode:=sim gui:=true nav_mode:=full
```

**Terminal 2, driving if needed** 

```bash
# Keyboard controlls
pixi run ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_key
# Gamepad controlls
pixi run ros2 launch teleop teleop.launch.py cmd_vel_topic:=/cmd_vel_joy
```

**Foxglove Studio.** Open a connection: **Foxglove WebSocket**, `ws://localhost:8765`. The first time, use **Layouts → Import from file** and pick
`ros2_ws/src/chassis_bringup/foxglove/navigation.json`. After that, just select the layout.

**Shutting down.** Press Ctrl+C in each terminal, in reverse order. If the next start misbehaves
(Nav2 never becomes active, or nodes can't see each other), run this once and relaunch:

```bash
pixi run bash -c "fastdds shm clean"
```

### Nav modes

| `nav_mode` | What runs |
|---|---|
| `full` | Whole Nav2 stack. Accepts goals. |
| `debug` | Costmaps, planner and controller only, no goals. For driving by hand and checking what Nav2 sees. Default in the sim. |
| `off` | No Nav2. Default on the real rover. |

## Using waypoints

A goal is a position in the `map` frame. The origin is where the rover spawned, +x is east (straight ahead at spawn) and +y is north. Units are metres.

**From Foxglove.** In the **Navigation 3D** panel, pick the **Publish pose** tool. Click where you
want the rover to go, then drag to set the direction it should face. The green arrow marks the goal.

**From a terminal.** This gives you live feedback and a result. Press Ctrl+C to cancel.

```bash
pixi run ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 10.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

**Cancel.** Use the **Cancel goal** button in Foxglove, or:

```bash
pixi run ros2 service call /navigate_to_pose/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
```

**Taking over.** Driving with the joystick (holding the deadman) or the keyboard overrides Nav2
straight away. Let go and Nav2 carries on with its goal about 0.5 s later. Taking over does not
cancel the goal.

**Several waypoints in a row.** `nav_mode:=full` also starts Nav2's waypoint follower. It hasn't been
tested yet:

```bash
pixi run ros2 action send_goal --feedback /follow_waypoints nav2_msgs/action/FollowWaypoints \
  "{poses: [{header: {frame_id: map}, pose: {position: {x: 5.0, y: 0.0}, orientation: {w: 1.0}}},
            {header: {frame_id: map}, pose: {position: {x: 5.0, y: 5.0}, orientation: {w: 1.0}}}]}"
```

## Foxglove layout

`chassis_bringup/foxglove/navigation.json` has been updated for navigation. Each window:

| Window | What it shows | How to read it |
|---|---|---|
| Navigation 3D | Top-down view, fixed on the `map` frame: rover model, global costmap (whole area) and local costmap (around the rover), global path (cyan), local path (yellow), rover footprint (magenta), goal (green arrow), local EKF pose (blue) and global EKF pose (orange) | Coloured cells are obstacles and the safety margin around them; the path should bend around them. Blue and orange arrows should sit on the rover. The point cloud and Gazebo ground truth (`/gz/odom`) are hidden; tick them in the panel's topic list to compare. |
| Forward speed | Commanded vs measured forward speed (m/s) | The command line should be followed closely by the wheel odometry and EKF lines. A large gap means slip or a controller problem. |
| Yaw rate | The same for turning (rad/s) | Same as above. |
| Wheel speeds | Each wheel's speed, R1–R3 and L1–L3 (rad/s) | All right wheels together and all left wheels together. One wheel off on its own points to that motor. |
| Command sources | Joystick, Nav2 and the final command to the wheels | Shows who is driving. When the joystick line is active, the output follows it, not Nav2. |
| Nav2 goal progress | Distance remaining (m) and number of recoveries | Distance should count down to about 0. Recoveries going up means Nav2 is stuck and trying to free itself. Empty when no goal is active. |
| Goal status | State of the last goal | 2 = driving, 4 = reached, 5 = cancelled, 6 = failed (aborted). |
| Cancel goal | Button | Cancels every active goal. |
| GPS | Raw GPS and filtered GPS on a map | The two tracks should overlap. |
| Diagnostics | Health of the EKFs and Nav2 | Anything not green needs a look. |
| Log | Warnings and errors from every node | First place to look when a goal fails. |

## How it is integrated

```text
joystick  /cmd_vel_joy  (priority 100) ─┐
keyboard  /cmd_vel_key  (priority 90)  ─┼─ twist_mux ─> /diff_drive_controller/cmd_vel_unstamped ─> wheels
Nav2      /cmd_vel      (priority 10)  ─┘
```

- **One launch.** `rover.launch.py mode:=sim` starts Gazebo, the drive controller, localization (local and global EKF + GPS), `twist_mux`, Nav2 and the Foxglove bridge. `nav_mode` picks how much of Nav2 runs.
- **`navigation/launch/nav2.launch.py`** starts Nav2 from one params file, `navigation/config/nav2_params.yaml`. It is shared by the sim and the rover; the launch fills in the differences (sim time, global frame, point cloud topic).
- **Nav2 setup.**
  - Planner: Smac 2D (plans around obstacles).
  - Controller: DWB, at most 0.5 m/s and 1.0 rad/s, forward only.
  - Goal tolerance: 0.20 m.
  - Costmaps: global 100 × 100 m in `map`, local 6 × 6 m around the rover. Both are fed by the depth camera's point cloud.
  - Footprint: the measured rover outline (1.12 × 0.78 m) plus 5 cm.
- **Recoveries.** When stuck, Nav2 clears the costmaps (except a 4 × 4 m area around the rover), spins, waits and backs up, then retries. It gives up after 6 tries.
- **`twist_mux`** (`chassis_bringup/config/twist_mux.yaml`) decides who drives. The joystick only publishes while its deadman is held, so Nav2 gets control back when you let go.
- **Sim camera** is 320×180. At 1280×720 the point clouds were too large and most never reached Nav2.

## New files

Paths are under `ros2_ws/src/`.

| File | Role |
|---|---|
| `navigation/launch/nav2.launch.py` | Starts Nav2 in `debug` or `full` mode. Included by `rover.launch.py` / `sim_gz.launch.py` |
| `navigation/config/nav2_params.yaml` | All Nav2 settings: planner, controller, costmaps, footprint, recoveries. Replaces the old `costmap.yaml` |
| `navigation/behavior_trees/navigate_to_pose_keep_nearby.xml` | Nav2's standard goal behavior tree, changed so recoveries keep obstacles near the rover |
| `chassis_bringup/config/twist_mux.yaml` | Who drives the wheels: joystick > keyboard > Nav2 |
| `chassis_bringup/foxglove/navigation.json` | Foxglove layout for navigation (see [Foxglove layout](#foxglove-layout)) |
| `robot_description/worlds/nav_test.sdf` | Gazebo world with four obstacle lanes for testing |
| `navigation/scripts/run_nav_tests.sh` | Runs every test scenario N times, each in a fresh sim |
| `navigation/scripts/nav_scenario_test.py` | Runs one test scenario and records the results |
| `navigation/scripts/bag_qos.yaml` | Recording settings for `run_nav_tests.sh -b` |

## Testing

Tests run in `robot_description/worlds/nav_test.sdf`. It has four obstacle lanes, 20 m apart. In each
lane the rover starts at x = 0 facing east, and the goal is 15 m straight ahead.

| Scenario | Lane `y` | Course | Expected |
|---|---|---|---|
| `open` | 0 | nothing | reaches the goal |
| `detour` | 20 | 1 × 3 m block in the way | drives around it |
| `narrow` | 40 | wall with a 1.6 m gap | drives through the gap |
| `blocked` | 60 | goal inside a closed pen | gives up without crashing |
| `cancel` | 0 | goal cancelled after 8 s | stops |

```bash
# Every scenario, 3 runs each, a fresh sim per run (results in nav_test_results/today/results.jsonl)
pixi run ros2 run navigation run_nav_tests.sh -n 3 -o nav_test_results/today
# One scenario, also recording a bag
pixi run ros2 run navigation run_nav_tests.sh -b -n 1 detour
# By hand: start the sim in a lane, then run one scenario
pixi run ros2 launch chassis_bringup sim_gz.launch.py nav_mode:=full world_file:=worlds/nav_test.sdf y:=20
pixi run ros2 run navigation nav_scenario_test.py --scenario detour
```

The tests use their own ROS domain (77) on localhost, so they can't reach a rover. They are measured
against Gazebo's ground truth.

### Results (sim, 3 runs per scenario)

| Scenario | Result | Time to goal | Closest to an obstacle | Final error |
|---|---|---|---|---|
| open | 3/3 reached | 30 s | — | 0.15–0.17 m |
| detour | 3/3 reached | 32 s | 0.20–0.25 m | 0.14–0.18 m |
| narrow | 3/3 through the gap | 30 s | 0.31–0.36 m | 0.14–0.15 m |
| blocked | 3/3 gave up (aborted) | 150–210 s | **0.00–0.04 m** | — |
| cancel | 3/3 cancelled | — | — | — |

- Times are sim seconds; the sim ran at about 0.6× real time.
- Nav2 started driving within 0.25 s of every goal.
- No Nav2 process crashed in any run.
- **Cancel:** the wheels got their last command 0.65–0.75 s after the cancel.
- **Blocked:** Nav2 gave up in a controlled way, but the rover came within 0–4 cm of the pen walls. The camera only looks forward, so Nav2 can't see walls beside the rover. Expect a goal inside an obstacle to possibly touch it.
- **Not tested:** moving obstacles, slopes and rough terrain, obstacles lower than 0.15 m, and the real rover.

## On the rover and future work

The rover config is prepared but **not tested on hardware**. To run it on the Jetson, start with `debug`, then move to `full`:

```bash
pixi run ros2 launch chassis_bringup rover.launch.py mode:=real vio:=true camera:=cloud nav_mode:=debug
```

Then, on the ground station, start the joystick so it feeds `twist_mux`:

```bash
pixi run ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false cmd_vel_topic:=/cmd_vel_joy
```

On the rover, Nav2 uses the `odom` frame (goals relative to where the rover started) and the ZED point
cloud `/zed/points`.

Before the first goal on hardware:
1. Measure the real footprint.
2. Check that obstacles show up in the right place in the local costmap.
3. Test the joystick override and cancel with the wheels off the ground.
4. Check CPU load on the Jetson.

Future additions:
- **GPS waypoints:** send goals as latitude/longitude, convert them to `map` with navsat's `/fromLL` service, and drive them with `follow_waypoints`.
- **Global EKF on the rover:** a `map` frame from GPS, like in the sim. This needs:
  - `GpsNode` to stamp its messages with a time and frame;
  - a measured GPS antenna position;
  - a start-heading step, since the rover won't always start facing east.
- **Safety:**
  - an e-stop lock in `twist_mux`;
  - a check that a lost joystick link doesn't hand control back to Nav2 mid-goal;
  - sensing beside and behind the rover.
- **ZED wrapper migration:** the `zed-wrapper-migration` branch changes the cloud topic. Pass the new one with `cloud_topic:=`.
