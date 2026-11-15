# Pose estimation and navigation (`navigation`)

Code: [`ros2_ws/src/navigation/`](../ros2_ws/src/navigation). How the EKF fits into the TF tree is
in [architecture.md → Odometry and TF](architecture.md#odometry-and-tf).

| File | |
|---|---|
| `launch/pose_estim.launch.py` | the sim's pose estimator and Nav2 servers. Started by `sim_gz.launch.py navigation:=true` (the default) |
| `config/ekf_sim.yaml` | EKF for the sim: wheel odometry + synthetic VIO |
| `config/ekf_real.yaml` | EKF for the rover: wheel odometry + the ZED's VIO. Started by `rover.launch.py mode:=real vio:=true` |
| `config/costmap.yaml` | Nav2 `controller_server`, `planner_server` and both costmaps |
| `src/straight_line_planner.cpp`, `include/navigation/straight_line_planner.hpp`, `global_planner_plugin.xml` | the `nav2_straightline_planner::StraightLine` global planner plugin |

## EKF (`robot_localization`)

Both files configure the same filter, `ekf_filter_node`: `two_d_mode`, 30 Hz, `world_frame: odom`,
`base_link_frame: base_footprint`, and `publish_tf: true`, so it is the publisher of
`odom → base_footprint` whenever it runs. It fuses:

| Input | Topic | Fused as |
|---|---|---|
| wheel odometry (`diff_drive_controller`) | `/diff_drive_controller/odom` | velocity only: `vx`, `vyaw` |
| VIO, sim (synthetic, [synthetic-vio.md](synthetic-vio.md)) | `/depth_cam/vio/odom` | pose only: `x`, `y`, `yaw` |
| VIO, rover (ZED 2i, [people-pose.md](people-pose.md#vio)) | `/zed/vio/odom` | pose only: `x`, `y`, `yaw` |

Both VIO inputs are `odom1_relative: true`: the filter uses the VIO pose relative to its first message.

Velocity from the wheels and pose from the VIO, so the same displacement isn't counted twice. The
output is `/odometry/filtered`. On the rover, the ZED publishes only while its tracking is `OK`;
meanwhile the filter runs on the wheels alone.

`robot_localization` takes each input's covariance from its messages and has no setting to
override it. The real wheel odometry's covariance is therefore set in `real.launch.py`
(`WHEEL_ODOM_COVARIANCE`), and the VIO's is floored in the ZED node (`min_position_var`,
`min_yaw_var`).

## `pose_estim.launch.py`

| Arg | Default | |
|---|---|---|
| `use_sim` | `true` | `use_sim_time` for every node here |
| `sim_vio_profile` / `sim_vio_seed` / `sim_vio_truth_topic` | `validation` / `42` / `/gz/odom` | passed to the synthetic VIO |

It starts the synthetic VIO (`zed2i synthetic_vio_node`), the EKF with `ekf_sim.yaml`, Nav2's
`controller_server` and `planner_server` with `costmap.yaml`, and a lifecycle manager for each
server (`autostart: true`). The real rover has none of these except, with `vio:=true`, the EKF.

## Nav2

Only the two servers run: there is no `bt_navigator`, so there is no `NavigateToPose`. Send goals
to the servers' own actions, `compute_path_to_pose` (planner) and `follow_path` (controller). Both
work in `odom`; the rover has no `map` frame.

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
