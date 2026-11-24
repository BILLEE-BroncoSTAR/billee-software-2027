# Launch files (`chassis_bringup`)

Code: [`ros2_ws/src/chassis_bringup/`](../ros2_ws/src/chassis_bringup). Which one to run on which
machine is in [running.md](running.md); the arguments of `rover.launch.py`, `ground_station.launch.py`
and `sim_gz.launch.py` are in [running.md → Launch options](running.md#launch-options).

| Launch | Starts |
|---|---|
| `sim_gz.launch.py` | Gazebo + rover + `diff_drive_controller`, gamepad teleop (`joy_control`). Also `navigation` (default `true`: synthetic VIO, the local EKF (`odom → base_footprint`), the global EKF + navsat (`map → odom`, from the Gazebo GPS), and the Nav2 controller/planner servers). World: `world_file` (default `worlds/gps_world.sdf`). Optional: `gui` (Gazebo window, default `true`), `foxglove`, `rviz`, `odesc_shadow` (always on `vcan0`) |
| `real.launch.py` | Standalone `controller_manager` + `odesc/OdescSystemHardware` (`can_interface`: `can2` / `vcan0`), `gear_ratio`, `foxglove`, `rviz`. `odom_tf` (default `true`): `diff_drive_controller` publishes `odom → base_footprint` from wheel odometry; `rover.launch.py vio:=true` hands it to the EKF instead |
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
| `foxglove/rover_dashboard.json` | Foxglove dashboard: the five CSI cameras (front tab also shows the ZED 2i), GPS map (`/gps`, nothing publishes it yet), odometry |
| `rviz/drivetrain.rviz`, `foxglove/drivetrain.json` | matching viewer layouts (Foxglove: 3D view, `/joy`, drive command). `ground_station.launch.py` sets RViz's fixed frame to `odom` (`fixed_frame:=`). The sim's EKF (`navigation:=true`) or the real path's wheel-odometry TF (`odom_tf:=true`) publishes it |

## What `rover.launch.py` starts

```text
rover.launch.py
├─ mode:=sim  → sim_gz.launch.py   (foxglove:=true, gui, joy_control, rviz)
│    ├─ Gazebo (world_file, default worlds/gps_world.sdf), ros_gz_bridge (/clock, /gps, /depth_cam/*, /gz/odom), spawn, robot_state_publisher
│    ├─ joint_state_broadcaster + diff_drive_controller spawners
│    ├─ navigation/pose_estim.launch.py     (navigation:=true)   → navigation.md
│    ├─ odesc_shadow.launch.py               (odesc_shadow:=true)
│    ├─ teleop/teleop.launch.py              (joy_control:=true)
│    └─ viz.launch.py                        (foxglove / rviz)
├─ mode:=real → real.launch.py      (foxglove:=true, can_interface, gear_ratio, fault_policy, odom_tf, rviz)
│    ├─ robot_state_publisher, ros2_control_node (OdescSystemHardware), controller spawners
│    └─ viz.launch.py
│  + teleop/teleop.launch.py                 (joy_control:=true)
├─ people/people_rover.launch.py standalone:=false   (camera:=cloud|openpose|both, or vio:=true) → people-pose.md
├─ robot_localization ekf_node, navigation/config/ekf_real.yaml   (mode:=real vio:=true) → navigation.md
└─ camera_stream/csi_cameras.launch.py cameras:=<csi_cameras>     (unless csi_cameras:=0) → cameras.md
```

`camera:=` / `vio:=` are dropped with a log line where `zed_openpose_node` isn't built (any machine
but the Jetson container), and the CSI cameras where there are no NVIDIA camera plugins. `vio:=` is
ignored in `mode:=sim`, whose EKF already fuses the synthetic VIO.

## Other launch files' arguments

| Launch | Option | Default | |
|---|---|---|---|
| `real.launch.py` | `can_interface` / `gear_ratio` / `fault_policy` / `odom_tf` | `can2` / `48.0` / `warn` / `true` | as in `rover.launch.py mode:=real` |
| | `rviz` / `foxglove` | `false` / `false` | viewers via `viz.launch.py` |
| | `description_pkg` / `xacro_file` / `controllers_file` | `robot_description` / `urdf/robot.urdf.xacro` / `config/controllers.yaml` | model and controller sources |
| `onboard_test.launch.py` | `can_interface` / `gear_ratio` / `fault_policy` | `can2` / `48.0` / `warn` | passed to `rover.launch.py mode:=real` |
| | `joy_control` / `rviz` / `csi_cameras` | `true` / `true` / `5` | teleop and RViz on the Jetson itself |
| `viz.launch.py` | `rviz` / `foxglove` | `true` / `true` | RViz2 / foxglove_bridge on `:8765` |
| | `use_sim_time` | `false` | `true` when viewing the sim |
| | `rviz_config` / `fixed_frame` | `rviz/drivetrain.rviz` / config's own | `fixed_frame` overrides the config (`rviz2 -f`) |
| `odesc_shadow.launch.py` | `can_interface` | `vcan0` | the shadow driver's bus |
| `ground_station.launch.py` | `fprime_gds_url` | `http://192.168.4.73:5000` | opened in a browser with `fprime_gds:=true` |
| `sim_gz.launch.py` | `x` / `y` / `z` / `yaw` | `0` / `0` / `0.2` / `0` | spawn pose. Keep `yaw` 0 (facing east) for the GPS: navsat assumes the rover starts facing east |
| | `world_file` | `worlds/gps_world.sdf` | world, relative to the `robot_description` share. It needs the NavSat system and `<spherical_coordinates>` for `/gps` |

`ros2 launch chassis_bringup <file> --show-args` prints every argument with its description.

## Notes

- `sim_gz` loads `world_file` (default `worlds/gps_world.sdf`, installed by `robot_description`). With `gui:=false` it runs the Gazebo server only (`-s`).
- `real.launch.py` writes a temporary copy of `controllers.yaml` with the real-hardware
  overrides (`use_sim_time: false`, the wheel-odometry TF, its covariance); see
  [architecture.md → Odometry and TF](architecture.md#odometry-and-tf).
- `odesc_shadow.launch.py` runs its controller manager in the `/odesc_shadow` namespace, with
  `enable_odom_tf: false` and its own `odesc_shadow_odom` frames, so Gazebo stays the only TF
  source.
- The robot model, controllers and CAN node map come from `robot_description`
  ([robot-model.md](robot-model.md)). Joystick config is in `teleop/config/joystick.yaml`
  ([teleop.md](teleop.md)).
