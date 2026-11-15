# Robot model (`robot_description`)

Code: [`ros2_ws/src/robot_description/`](../ros2_ws/src/robot_description).

The BILLEE rover model: URDF/xacro, meshes, and the `ros2_control` controller config. It has no
nodes. `chassis_bringup` expands the model and feeds it to `robot_state_publisher` and Gazebo.

| File | |
|---|---|
| `urdf/robot.urdf.xacro` | Entry point. Args: `use_sim` (`true` = Gazebo `ign_ros2_control`, `false` = `odesc/OdescSystemHardware`), `can_interface` (default `can2`) |
| `urdf/robot_core.urdf.xacro` | links, joints, inertia, meshes: `base_footprint` → `base_link`, fixed suspension, six continuous wheel joints `joint_wheel_{l,r}{1,2,3}` |
| `urdf/ros2_control.urdf.xacro` | hardware plugin selection, per-wheel CAN `node_id` (must match `odesc/config/node_map.yaml`), `gear_ratio` |
| `urdf/camera.urdf.xacro` | `camera_link` + `camera_link_optical` (ZED 2i mount; `camera_joint` sets its pose) |
| `urdf/robot_gz.urdf.xacro` | Gazebo-only: RGB-D sensor on `/depth_cam` (1280×720, 120° HFOV, 10 Hz), sensors system, `ign_ros2_control` |
| `urdf/macros.xacro` | xacro macros, included by `robot_gz.urdf.xacro` |
| `meshes/` | STL meshes: base, suspension arms, wheels |
| `urdf/materials.xacro`, `urdf/chassis_model.gazebo`, `urdf/chassis_model.trans` | not used: nothing includes `materials.xacro`, and the other two's includes in `robot_core.urdf.xacro` are commented out |
| `config/controllers.yaml` | `diff_drive_controller` (track 0.67 m, wheel radius 0.11 m, `cmd_vel_timeout` 0.25 s, `enable_odom_tf: false`) + `joint_state_broadcaster`, 30 Hz |

Check that the model expands (in `ros2_ws`):

```bash
xacro src/robot_description/urdf/robot.urdf.xacro > /tmp/robot.urdf
xacro src/robot_description/urdf/robot.urdf.xacro use_sim:=false > /tmp/robot_real.urdf
```

`enable_odom_tf: false` is intentional: in sim the EKF (`robot_localization`) owns
`odom → base_footprint`. On the real path `real.launch.py odom_tf:=true` turns it on.
