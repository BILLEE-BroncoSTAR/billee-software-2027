# Synthetic ZED 2i VIO

`synthetic_vio_node` publishes a controllably degraded VIO-like `nav_msgs/Odometry`
measurement from Gazebo **ground truth**. It is for EKF simulation, not a ZED SDK or
camera emulator. It intentionally does not publish TF; the EKF should be the only
publisher of `odom -> base_link`.

See the [Synthetic VIO User Guide](docs/SYNTHETIC_VIO_USER_GUIDE.md) for required
Gazebo wiring, profile behaviour, all parameters, EKF configuration, and experiment
workflow.

## Run

Ground truth has to reach ROS first. In this repo that is already wired up: the
`OdometryPublisher` plugin in `robot_description/urdf/robot_gz.urdf.xacro` emits
`/gz/odom` on the Gazebo side, and `chassis_bringup/config/config.yaml` bridges it into
ROS as `nav_msgs/Odometry`. Without that bridge entry this node's subscription is
silent and it publishes nothing — the symptom is an EKF quietly running on wheel
odometry alone.

Normally you do not launch this node directly: `sim_gz.launch.py` starts it through
`navigation/launch/pose_estim.launch.py` whenever `localization:=true` (the default)
and `use_vio:=true`. To run it standalone — the supplied launch file sets
`use_sim_time:=true`, so latency is measured against Gazebo's `/clock`:

```bash
ros2 launch zed2i synthetic_vio.launch.py \
  truth_topic:=/gz/odom profile:=validation seed:=42
```

Use `seed:=-1` to select a new seed; the resolved seed is printed at startup and can
be reused to reproduce the run.

The node's built-in default output topic is `/sim/zed/vio/odom`, but
[`config/synthetic_vio.yaml`](config/synthetic_vio.yaml) overrides it to
**`/depth_cam/vio/odom`**, which is what `navigation/config/ekf.yaml` reads as `odom1`
and what a real ZED driver should be remapped onto. Change one and you must change the
other.

Fuse only its `x`, `y` and `yaw` fields; fuse wheel odometry as twist (`vx`, `vyaw`) so
the EKF is not given the same wheel measurement twice. That is how
[`navigation/config/ekf.yaml`](../../navigation/config/ekf.yaml) is already set up.

## Profiles

`calibration` uses the configured model exactly and is suitable for initial measurement
covariance tuning. `validation` independently samples the configured noise,
drift, and scale-error standard deviations between the validation multipliers for
each process. `stress` additionally enables latency, dropouts, outliers, and a
deliberate covariance mismatch. `nominal` remains an alias for `calibration`. Do not
tune the EKF on validation or stress runs.

The model is:

```text
VIO pose = truth pose + per-run scale error + random-walk drift + Gaussian jitter
```

The source and output topics, all model parameters, profile ranges, and fault values
are documented in `config/synthetic_vio.yaml`.
