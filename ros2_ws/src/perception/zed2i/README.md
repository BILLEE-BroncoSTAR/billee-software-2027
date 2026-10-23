# Synthetic ZED 2i VIO

`synthetic_vio_node` publishes a controllably degraded VIO-like `nav_msgs/Odometry`
measurement from Gazebo **ground truth**. It is for EKF simulation, not a ZED SDK or
camera emulator. It intentionally does not publish TF; the EKF should be the only
publisher of `odom -> base_link`.

See the [Synthetic VIO User Guide](docs/SYNTHETIC_VIO_USER_GUIDE.md) for required
Gazebo wiring, profile behaviour, all parameters, EKF configuration, and experiment
workflow.

## Run

Bridge the Gazebo model's ground-truth odometry to a ROS 2 `nav_msgs/Odometry` topic,
then launch the node. The exact Gazebo topic is world/model dependent, so pass it as
an argument instead of assuming a wheel-odometry topic. The supplied launch file sets
`use_sim_time:=true`, so latency is measured against Gazebo's `/clock`:

```bash
ros2 launch zed2i synthetic_vio.launch.py \
  truth_topic:=/gz/odom profile:=validation seed:=42
```

Use `seed:=-1` to select a new seed; the resolved seed is printed at startup and can
be reused to reproduce the run.

The output defaults to `/sim/zed/vio/odom`. Fuse only its `x`, `y`, and `yaw` fields;
fuse wheel odometry as twist (`vx`, `vyaw`) so the EKF is not given the same wheel
measurement twice.

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
