# chassis_bringup

Launch files for the BILLEE drivetrain. How to use them is in [docs/running.md](../../../docs/running.md).

| Launch | Starts |
|---|---|
| `sim_gz.launch.py` | Gazebo + rover + `diff_drive_controller`, gamepad teleop (`joy_control`). Also `navigation` (default `true`: synthetic VIO + EKF, the `odom → base_footprint` TF source, and the Nav2 controller/planner servers). Optional: `gui` (Gazebo window, default `true`), `foxglove`, `rviz`, `odesc_shadow` (always on `vcan0`) |
| `real.launch.py` | Standalone `controller_manager` + `odesc/OdescSystemHardware` (`can_interface`: `can3` / `vcan0` / `mock`), `gear_ratio`, `foxglove`, `rviz`. `odom_tf` (default `true`): `diff_drive_controller` publishes `odom → base_footprint` from wheel odometry, since there is no EKF on this path |
| `rover.launch.py` | `mode:=sim` → `sim_gz` (`gui` default `false`), `mode:=real` → `real`, always with the Foxglove bridge. `camera:=cloud\|openpose\|both` adds the ZED point cloud, OpenPose, or both through `people/launch/people_rover.launch.py standalone:=false`; skipped with a log line if `zed_openpose_node` isn't built (not the Jetson container). `joy_control` defaults to `false` (the ground station runs teleop) |
| `ground_station.launch.py` | RViz + Foxglove bridge + gamepad teleop. `view:=drivetrain\|cloud\|openpose\|both` picks the RViz layout. Optional: F' GDS browser tab (`fprime_gds:=true`) |
| `viz.launch.py` | RViz and/or the Foxglove bridge on their own |
| `onboard_test.launch.py` | real drivetrain + teleop + viewers, all on the rover (bench tests) |
| `odesc_shadow.launch.py` | included by `sim_gz odesc_shadow:=true`: a namespaced ODESC controller manager on `vcan0` plus the ODrive emulator, fed the same `cmd_vel` as Gazebo |

| Config | |
|---|---|
| `config/config.yaml` | Gazebo → ROS bridge (`/clock`) |
| `config/zed_config.yaml` | Gazebo camera topics (`/depth_cam/*`) and ground-truth `/gz/odom` |
| `config/gz_params.yaml` | Gazebo params |
| `rviz/drivetrain.rviz`, `foxglove/drivetrain.json` | matching viewer layouts (Foxglove: 3D view, `/joy`, drive command). `ground_station.launch.py` sets RViz's fixed frame to `odom` (`fixed_frame:=`). The sim's EKF (`navigation:=true`) or the real path's wheel-odometry TF (`odom_tf:=true`) publishes it |

Notes:

- `sim_gz` ignores `world_file` and always loads `empty.sdf`.
- `real.launch.py` writes a temporary copy of `controllers.yaml` with `use_sim_time: false`, because
  a standalone `ros2_control_node` with sim time and no `/clock` hangs.
- The robot model, controllers and CAN node map come from `robot_description`. Joystick config
  is in `teleop/config/joystick.yaml`.
