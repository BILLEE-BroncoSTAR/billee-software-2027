# Pose estimation and navigation (`navigation`)

Code: [`ros2_ws/src/navigation/`](../ros2_ws/src/navigation). How the EKF fits into the TF tree is
in [architecture.md → Odometry and TF](architecture.md#odometry-and-tf).

| File | |
|---|---|
| `launch/pose_estim.launch.py` | the sim's pose estimator and Nav2 servers. Started by `sim_gz.launch.py navigation:=true` (the default) |
| `config/ekf_sim.yaml` | sim only: local EKF (`ekf_filter_node_odom`), global EKF (`ekf_filter_node_map`) and `navsat_transform_node` |
| `config/ekf_real.yaml` | rover only: one local EKF (`ekf_filter_node`), wheel odometry + the ZED's VIO. Started by `rover.launch.py mode:=real vio:=true` |
| `src/gps_covariance_relay.cpp` | fills a covariance into GPS fixes that arrive without one (`/gps` → `/gps/with_cov`) |
| `config/costmap.yaml` | Nav2 `controller_server`, `planner_server` and both costmaps |
| `src/straight_line_planner.cpp`, `include/navigation/straight_line_planner.hpp`, `global_planner_plugin.xml` | the `nav2_straightline_planner::StraightLine` global planner plugin |

## EKF (`robot_localization`)

The sim and the rover currently run **different** localization setups:

| | Sim (`sim_gz.launch.py navigation:=true`) | Rover (`rover.launch.py mode:=real vio:=true`) |
|---|---|---|
| Local EKF (`odom → base_footprint`) | `ekf_filter_node_odom` (`ekf_sim.yaml`) | `ekf_filter_node` (`ekf_real.yaml`) |
| Global EKF (`map → odom`) | `ekf_filter_node_map` + `navsat_transform_node` | none: the rover has no `map` frame yet |
| GPS | Gazebo NavSat → `/gps` → `gps_covariance_relay` → `/gps/with_cov` | not fused (see [Rover gaps](#rover-gaps)) |
| Outputs | `/odometry/filtered` (local), `/odometry/global` (global), `/odometry/gps`, `/gps/filtered` | `/odometry/filtered` |

All filters are `two_d_mode`, 30 Hz, `base_link_frame: base_footprint`, `publish_tf: true`. There is
no IMU or compass on the rover, so heading comes only from VIO yaw (relative to the start) and wheel
yaw rate.

### Local EKF

`world_frame: odom`. It publishes `odom → base_footprint`.

| Input | Topic | Fused as |
|---|---|---|
| wheel odometry (`diff_drive_controller`) | `/diff_drive_controller/odom` | velocity only. Sim: `vx`, `vy`, `vyaw`. Rover: `vx`, `vyaw` |
| VIO, sim (synthetic, [synthetic-vio.md](synthetic-vio.md)) | `/depth_cam/vio/odom` | pose only: `x`, `y`, `yaw` |
| VIO, rover (ZED 2i, [people-pose.md](people-pose.md#vio)) | `/zed/vio/odom` | pose only: `x`, `y`, `yaw` |

Both VIO inputs are `odom1_relative: true`: the filter uses the VIO pose relative to its first message.

Velocity from the wheels and pose from the VIO, so the same displacement isn't counted twice. The
sim filter also fuses the wheels' `vy`: `diff_drive_controller` always reports `vy = 0`, which acts
as a no-sideslip constraint; it is part of the A/B-tested sim configuration. `ekf_real.yaml` doesn't
fuse it. On the rover, the ZED publishes only while its tracking is `OK`; meanwhile the filter runs
on the wheels alone.

### Global EKF and navsat (sim only)

`ekf_filter_node_map` has `world_frame: map`, publishes `map → odom`, and outputs `/odometry/global`.
It fuses `/odometry/gps` (`x`, `y`) and the wheel velocities (`vx`, `vy`, `vyaw`). The wheels let it
start before the first fix, which `navsat_transform_node` needs.

`navsat_transform_node` converts `/gps/with_cov` into `/odometry/gps` in the `map` frame and
publishes `/gps/filtered`. Settings: `use_odometry_yaw: true` (heading from `/odometry/global`, as
there is no IMU), `zero_altitude: true`, `delay: 3.0`. It removes the antenna lever arm using the
`gps_link` frame from the URDF (`gps.urdf.xacro`).

- **Start heading.** navsat treats yaw 0 as east. The sim rover spawns facing east in
  `worlds/gps_world.sdf` (origin 38.418, -110.777), so this holds in the sim only.
- **Heading drift.** The global filter's only heading input is the wheel yaw rate. After spinning
  in place its heading can drift a long way. Fusing the VIO yaw into it (`differential: true`) is
  the planned fix.
- **Gazebo GPS** has no noise and zero covariance. `gps_covariance_relay` adds 0.5 m horizontal /
  1.0 m vertical (1σ) to fixes whose covariance type is `UNKNOWN`.

### Sim-only wheel odometry tuning

`controllers.yaml` sets `wheel_separation_multiplier: 1.85` (Gazebo skid-steer slip, calibrated
against ground truth) and covariance diagonals for the sim. `real.launch.py` replaces both on the
rover: multiplier 1.0 and `WHEEL_ODOM_COVARIANCE`.

### Rover gaps

What has to happen before the dual EKF can run on the rover:

- `comms/GpsNode` sets no `header.stamp` or `header.frame_id` on `/gps`. navsat and the EKF need
  both (the frame must be `gps_link`).
- The `gps_link` mount position in `gps.urdf.xacro` is a placeholder.
- A start-heading step (GPS heading from a straight drive, or a dual-antenna receiver), since the
  rover won't always start facing east.
- A launch path: `rover.launch.py mode:=real` starts only `ekf_real.yaml`.

`robot_localization` takes each input's covariance from its messages and has no setting to
override it. The real wheel odometry's covariance is therefore set in `real.launch.py`
(`WHEEL_ODOM_COVARIANCE`), and the VIO's is floored in the ZED node (`min_position_var`,
`min_yaw_var`).

## `pose_estim.launch.py`

| Arg | Default | |
|---|---|---|
| `use_sim` | `true` | `use_sim_time` for every node here |
| `sim_vio_profile` / `sim_vio_seed` / `sim_vio_truth_topic` | `validation` / `42` / `/gz/odom` | passed to the synthetic VIO |

It starts the synthetic VIO (`zed2i synthetic_vio_node`), the local and global EKFs and
`navsat_transform_node` with `ekf_sim.yaml`, `gps_covariance_relay`, Nav2's `controller_server` and
`planner_server` with `costmap.yaml`, and a lifecycle manager for each server (`autostart: true`).
The real rover has none of these except, with `vio:=true`, the local EKF from `ekf_real.yaml`.

## Nav2

Only the two servers run: there is no `bt_navigator`, so there is no `NavigateToPose`. Send goals
to the servers' own actions, `compute_path_to_pose` (planner) and `follow_path` (controller). Both
work in `odom`. The sim has a `map` frame (global EKF), but Nav2 doesn't use it yet; the rover
has no `map` frame.

- **Planner** (`planner_server`, plugin `GridBased`): the straight-line planner. It interpolates a
  straight path from start to goal every `interpolation_resolution` (0.1 m), with identity
  orientations except the goal's. It ignores the costmap. Start and goal must be in the costmap's
  global frame (`odom`), or it returns an empty path.
- **Controller** (`controller_server`, plugin `FollowPath`): DWB, 20 Hz, `odom_topic`
  `/diff_drive_controller/odom`, output remapped to `/diff_drive_controller/cmd_vel_unstamped`
  (unstamped, `enable_stamped_cmd_vel: false`). Limits: 0–0.5 m/s forward only, ±1.0 rad/s, no
  sideways motion. Goal tolerance 0.20 m and 0.25 rad. Critics: `BaseObstacle`, `GoalDist`,
  `ObstacleFootprint`, `PreferForward`, `PathDist`. It is idle until it gets a `FollowPath` goal.
- **Costmaps**: both are 3 × 3 m rolling windows in `odom`, footprint 1.3 × 1.0 m, inflation radius
  0.55 m. The global one (0.1 m cells) has only the inflation layer, so it is empty. The local one
  (0.05 m cells) adds a voxel layer fed by `/depth_cam/points`, the **sim's** depth camera; on the
  rover that topic doesn't exist.

When Nav2 drives in the sim, teleop's `joy_tank_drive` publishes on the same command topic (zeros
while the deadman isn't held), so the two compete. Use `joy_control:=false` to test Nav2 alone.
