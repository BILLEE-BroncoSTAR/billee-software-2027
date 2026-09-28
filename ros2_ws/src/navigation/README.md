# navigation

Pose estimation for the rover: the `robot_localization` EKF that owns the
`odom` → `base_link` transform, plus (in simulation) the synthetic VIO that feeds it.

Nav2 itself is not wired up yet — `navigation2` and `nav2_bringup` are installed in
every Pixi environment, but no launch file in this repo starts them.

## Why this package is not optional

`robot_description/config/controllers.yaml` sets:

```yaml
enable_odom_tf: false   # must disable to use robot_localization package
```

so `diff_drive_controller` does **not** publish `odom` → `base_link`. Exactly one node
may own a transform, and here that node is `ekf_filter_node`. Start the drivetrain
without this package and nothing publishes it: RViz and Foxglove cannot place the
robot, Nav2 cannot plan, and `tf2_echo odom base_link` reports
`Invalid frame ID "odom" ... frame does not exist` — while the `/odom` *topics* look
completely healthy, which makes it easy to misdiagnose.

`sim_gz.launch.py` and `real.launch.py` both include this launch by default
(`localization:=true`).

## Running it

```bash
ros2 launch navigation pose_estim.launch.py                 # simulation
ros2 launch navigation pose_estim.launch.py use_sim:=false  # real rover
ros2 launch navigation pose_estim.launch.py use_vio:=false  # EKF on wheel odometry alone
```

| Argument | Default | Meaning |
|---|---|---|
| `use_sim` | `true` | sets `use_sim_time` and enables the synthetic VIO. `false` on hardware |
| `use_vio` | `true` | start the synthetic VIO in sim. `false` shows how far the estimate drifts with no pose source |
| `sim_vio_profile` | `validation` | synthetic VIO noise profile |
| `sim_vio_seed` | `42` | random seed, for reproducible runs |
| `sim_vio_truth_topic` | `/gz/odom` | Gazebo ground truth the synthetic VIO degrades |

## Sensor fusion is additive

[`config/ekf.yaml`](config/ekf.yaml) is **one file for sim and real** — the synthetic
VIO publishes on the same topic a real ZED would, so the filter cannot tell them apart.

A Kalman filter always runs its predict step and only applies an update when a
measurement arrives. Every input is therefore optional except wheel odometry: a topic
nobody publishes is never fused, and the estimate falls back to what is available. Add
a sensor later and the estimate improves with **no config change**, because its entry is
already there waiting for data.

| Input | Topic | Contributes | Status |
|---|---|---|---|
| `odom0` wheel odometry | `/diff_drive_controller/odom` | `vx`, `vyaw` | **required** — the filter needs one source to initialise |
| `odom1` VIO | `/depth_cam/vio/odom` | `x`, `y`, `yaw` — bounds wheel drift | sim: synthetic VIO. Real: remap the ZED driver onto this topic |
| `imu0` IMU | `/imu/data` | `vyaw` | placeholder — nothing publishes it yet |

Wheel odometry contributes **velocities only**. Its pose is those same velocities
already integrated, so fusing both would feed identical information in twice and make
the filter overconfident in a drifting estimate.

Indices must stay **dense**: `robot_localization` scans `odom0`, `odom1`, … and stops at
the first gap. To stop fusing a source, stop publishing it — do not delete its block, or
everything after it is silently ignored.

## GPS is deliberately not fused here

`comms/GpsNode` publishes `sensor_msgs/NavSatFix` on `/gps`. That is global, absolute
and subject to discrete jumps. Fusing it into a `world_frame: odom` filter would make
`odom` → `base_link` jump, which breaks every consumer that assumes odom is smooth and
continuous — Nav2 included.

The supported pattern is two filters:

| Filter | `world_frame` | Publishes | Behaviour |
|---|---|---|---|
| this EKF | `odom` | `odom` → `base_link` | smooth, drifts slowly |
| a second EKF + `navsat_transform_node` | `map` | `map` → `odom` | absolute, may jump |

Adding that does not change `config/ekf.yaml`.

## Checking it

```bash
ros2 run tf2_ros tf2_echo odom base_link     # a Translation, not "does not exist"
ros2 topic echo /odometry/filtered --once    # the EKF's estimate
ros2 run tf2_tools view_frames               # PDF of the whole TF tree
```

Driving forward should advance the transform and `/odometry/filtered` together.

## Gotchas

- **`process_noise_covariance` is a flattened 15×15 matrix (225 values)**, not a
  15-element diagonal. A short one leaves the covariance singular and the filter goes
  NaN on the first update — you get `Critical Error, NaNs were detected in the output
  state of the filter` and `TF_NAN_INPUT`, and the transform silently never appears.
  This file omits it and uses the library defaults; tune from the full upstream matrix.
- **`two_d_mode: true`** pins z, roll and pitch. Without an IMU those axes are
  unobservable, and letting them float makes the estimate wander.
- A bare `LaunchConfiguration` object is **always truthy**. `if LaunchConfiguration("use_sim"):`
  is true even for `use_sim:=false` — which used to start the *simulated* VIO on the
  real rover and feed the filter fabricated odometry. Resolve with `.perform(context)`.
