# Synthetic ZED 2i VIO

`synthetic_vio_node` publishes a controllably degraded VIO-like `nav_msgs/Odometry`
measurement from Gazebo **ground truth**, for testing the EKF in simulation. It is not a ZED
SDK or camera emulator and doesn't process images. Starting from truth (not wheel odometry)
keeps its errors independent of the wheel-odometry input the EKF also fuses. It doesn't publish
TF; the EKF is the only publisher of `odom -> base_footprint`.

```text
VIO pose = truth pose + per-run scale error + random-walk drift + Gaussian jitter
           (+ latency, dropouts, outliers in the stress profile)
```

## Run

`sim_gz.launch.py navigation:=true` (the default) already starts it through
`navigation/launch/pose_estim.launch.py`. To run it on its own:

```bash
ros2 launch zed2i synthetic_vio.launch.py \
  truth_topic:=/gz/odom profile:=validation seed:=42
```

`truth_topic` must be a ground-truth odometry topic bridged from Gazebo, never
`/diff_drive_controller/odom`. The launch file sets `use_sim_time:=true`, so latency follows
Gazebo's `/clock`. The node logs the resolved seed at startup; `seed:=-1` picks a new one.
The same profile, parameters and seed replay a run exactly.

The output topic is `/depth_cam/vio/odom` (frame `odom`, child `base_footprint`), set by
`config/synthetic_vio.yaml`, which both launch files load. The bare executable without that
file falls back to `/sim/zed/vio/odom` and child `base_link`.

## EKF wiring

`navigation/config/ekf_sim.yaml` fuses this topic as pose only (`x`, `y`, `yaw`,
`odom1_relative: true`) and wheel odometry as velocity only (`vx`, `vyaw`), so the same
measurement isn't counted twice. Don't fuse the VIO twist fields; they carry a deliberately huge
covariance, as do z, roll and pitch.

## Profiles

| Profile | Use | Behaviour |
|---|---|---|
| `calibration` (alias `nominal`) | initial EKF covariance tuning | the configured model exactly, no faults |
| `validation` | evaluating a fixed filter | each noise, drift and scale std sampled per run between `validation_{min,max}_multiplier` |
| `stress` | resilience, innovation rejection | larger errors, plus latency, dropouts, outliers and an overconfident reported covariance |

Tune only on `calibration` runs, then freeze the EKF settings and evaluate over several held-out
`validation` seeds and routes. Use `stress` to find brittle behaviour, not to fit a trace. Every
parameter, with units, is commented in `config/synthetic_vio.yaml`.

## Troubleshooting

| Symptom | Check |
|---|---|
| No `/depth_cam/vio/odom` output | `truth_topic` exists, is `nav_msgs/msg/Odometry`, and has non-zero stamps |
| Latency setting has no effect | the node runs with `use_sim_time:=true` and Gazebo publishes `/clock` |
| EKF implausibly certain | the truth source isn't wheel odometry; VIO is fused as pose only |
