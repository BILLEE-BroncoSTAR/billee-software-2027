# chassis_bringup

Launch files for the BILLEE drivetrain. How to use them is in [docs/running.md](../../../docs/running.md).

| Launch | Starts |
|---|---|
| `sim_gz.launch.py` | Gazebo + rover + `diff_drive_controller`, gamepad teleop (`joy_control`). Optional: `gui` (Gazebo window, default `true`), `foxglove`, `rviz`, `navigation` (EKF + costmap), `odesc_shadow` |
| `real.launch.py` | Standalone `controller_manager` + `odesc/OdescSystemHardware` (`can_interface`: `can2` / `vcan0` / `mock`), `gear_ratio`, `foxglove`, `rviz` |
| `rover.launch.py` | `mode:=sim` → `sim_gz` (`gui` default `false`), `mode:=real` → `real`, always with the Foxglove bridge |
| `ground_station.launch.py` | RViz + Foxglove bridge + gamepad teleop. Optional: F' GDS browser tab (`fprime_gds:=true`) |
| `viz.launch.py` | RViz and/or the Foxglove bridge on their own |
| `onboard_test.launch.py` | real drivetrain + teleop + viewers, all on the rover (bench tests) |
| `odesc_shadow.launch.py` | included by `sim_gz odesc_shadow:=true`: a namespaced ODESC controller manager on `vcan0` plus the ODrive emulator, fed the same `cmd_vel` as Gazebo |

| Config | |
|---|---|
| `config/config.yaml` | Gazebo → ROS bridge (`/clock`) |
| `config/zed_config.yaml` | Gazebo camera topics (`/depth_cam/*`) and ground-truth `/gz/odom` |
| `config/gz_params.yaml` | Gazebo params |
| `rviz/drivetrain.rviz`, `foxglove/drivetrain.json` | matching viewer layouts (fixed frame `odom`) |

Notes:

- `sim_gz` ignores `world_file` and always loads `empty.sdf`.
- `real.launch.py` writes a temporary copy of `controllers.yaml` with `use_sim_time: false`, because
  a standalone `ros2_control_node` with sim time and no `/clock` hangs.
- The robot model, controllers and CAN node map come from `robot_description`. Joystick config
  is in `teleop/config/joystick.yaml`.
