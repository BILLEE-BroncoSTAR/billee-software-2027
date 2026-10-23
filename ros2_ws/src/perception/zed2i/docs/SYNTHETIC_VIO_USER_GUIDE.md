# Synthetic VIO User Guide

## Purpose

`synthetic_vio_node` creates a VIO-like ROS 2 odometry measurement from Gazebo
ground truth. It is intended to test state estimation when a real ZED 2i and its
VIO output are unavailable in simulation.

It is not a ZED SDK emulator and it does not process the simulated camera images.
Instead, it starts with independent simulator truth and applies a configurable error
model. This prevents the visual-odometry input from sharing wheel-encoder error with
the wheel-odometry input fused by the EKF.

The generated measurement is:

```text
synthetic VIO pose = ground-truth pose
                    + per-run scale error
                    + time-correlated random-walk drift
                    + per-sample Gaussian jitter
                    + optional fault effects
```

Fault effects are delivery latency, dropped measurements, and pose outliers. They
are used only when configured, normally through the `stress` profile.

## Prerequisites

1. Gazebo must publish a **ground-truth** `nav_msgs/msg/Odometry` topic. Do not use
   `/odom` or `/diff_drive_controller/odom` if they are based on wheel encoders.
2. Bridge that Gazebo topic into ROS 2 and note its ROS topic name.
3. Run the simulation with `/clock`; the supplied launch file sets
   `use_sim_time:=true` so configured latency follows simulation time.
4. Build and source the workspace:

   ```bash
   cd ros2_ws
   pixi run --environment default -- colcon build \
     --packages-select zed2i --symlink-install
   source install/setup.bash
   ```

The node does not publish TF. Let the EKF remain the sole publisher of
`odom -> base_link`.

## Launch

The default output is `/sim/zed/vio/odom`.

```bash
ros2 launch zed2i synthetic_vio.launch.py \
  truth_topic:=/gz/odom \
  profile:=validation \
  seed:=42
```

The node logs the chosen seed at startup. Use the same profile, parameters, and seed
to replay a run exactly. Set `seed:=-1` to choose a new seed automatically; record
the logged resolved seed with experiment results.

You can also run the executable directly:

```bash
ros2 run zed2i synthetic_vio_node --ros-args \
  -p use_sim_time:=true \
  -p truth_topic:=/gz/odom \
  -p vio_topic:=/sim/zed/vio/odom \
  -p profile:=calibration \
  -p seed:=42
```

The default parameter file is
[`../config/synthetic_vio.yaml`](../config/synthetic_vio.yaml).

## EKF wiring

Treat the synthetic VIO measurement as pose only. Treat wheel odometry as velocity
only. For `robot_localization`, a representative planar configuration is:

```yaml
odom0: /diff_drive_controller/odom
odom0_config: [false, false, false,
               false, false, false,
               true, false, false,
               false, false, true,
               false, false, false]

odom1: /sim/zed/vio/odom
odom1_config: [true, true, false,
               false, false, true,
               false, false, false,
               false, false, false,
               false, false, false]
odom1_relative: true
```

This fuses wheel forward velocity and yaw rate with VIO `x`, `y`, and yaw. Adapt the
frame names and available wheel fields to the rover configuration. Do not also fuse
the synthetic VIO twist fields; they intentionally have very large covariance.

The synthetic VIO message uses `output_frame_id` (`odom` by default) and
`child_frame_id` (`base_link` by default). These names must be compatible with the
EKF frame configuration.

## Profiles

Profiles are separate experiment categories, not successive noise stages applied to
one run.

| Profile | Intended use | Behaviour |
| --- | --- | --- |
| `calibration` | Choose initial EKF measurement/process noise | Uses the configured base model without profile randomization or injected faults. |
| `validation` | Evaluate a fixed filter across plausible VIO variation | Independently samples each noise, drift, and scale-error standard deviation between the validation multiplier bounds. |
| `stress` | Check resilience and innovation rejection | Samples larger errors and enables configured latency, dropout, outlier, and reported-covariance mismatch values. |
| `nominal` | Compatibility alias | Same as `calibration`. |

Tune against calibration data only. Hold validation seeds, routes, and profiles out
of covariance tuning. Use stress results to find brittle behaviour, not to fit a
single worst-case trace.

## Parameters

The settings below are ROS parameters in `synthetic_vio.yaml`.

### Topics and frames

| Parameter | Default | Meaning |
| --- | --- | --- |
| `truth_topic` | `/gz/odom` | ROS 2 ground-truth `nav_msgs/msg/Odometry` input. |
| `vio_topic` | `/sim/zed/vio/odom` | Synthetic VIO odometry output. |
| `output_frame_id` | `odom` | Output `Odometry.header.frame_id`. |
| `child_frame_id` | `base_link` | Output `Odometry.child_frame_id`. |
| `profile` | `calibration` | `calibration`, `validation`, `stress`, or alias `nominal`. |
| `seed` | `-1` | Non-negative deterministic seed; `-1` chooses a seed and logs it. |

### Base measurement model

| Parameter | Units | Meaning |
| --- | --- | --- |
| `position_noise_std` | m | Independent Gaussian standard deviation applied to x and y at each sample. |
| `yaw_noise_std` | rad | Independent Gaussian standard deviation applied to yaw at each sample. |
| `position_drift_rw_std` | m / sqrt(s) | Random-walk standard deviation for each horizontal drift component. |
| `yaw_drift_rw_std` | rad / sqrt(s) | Random-walk standard deviation for yaw drift. |
| `translation_scale_std` | fraction | Standard deviation used to sample one translation scale error for a run. |
| `yaw_scale_std` | fraction | Standard deviation used to sample one yaw scale error for a run. |
| `reported_covariance_scale` | multiplier | Scales the covariance reported in the synthetic `Odometry` message. `1.0` reports the base model's predicted covariance. |

The output covariance includes white measurement noise, uncertainty caused by the
sampled scale term as distance from the starting pose increases, and accumulated
random-walk drift variance. It is diagonal: x/y share one variance and yaw has its
own variance. z, roll, pitch, and all output twist fields are marked with a large
covariance because this simulator is intended for planar pose fusion.

### Optional base faults

These are normally zero in calibration and validation. They remain available to make
a custom scenario without using the stress profile.

| Parameter | Units | Meaning |
| --- | --- | --- |
| `dropout_probability` | probability | Probability of discarding each input truth sample. |
| `outlier_probability` | probability | Probability of adding an outlier to each generated VIO sample. |
| `outlier_position_std` | m | Gaussian standard deviation of an injected x/y outlier. |
| `outlier_yaw_std` | rad | Gaussian standard deviation of an injected yaw outlier. |
| `latency_ms` | ms | Delay between generating and publishing a measurement. The original measurement timestamp is retained. |

### Validation and stress controls

| Parameter | Default | Meaning |
| --- | --- | --- |
| `validation_min_multiplier` | `0.5` | Lower bound for each independently sampled validation error multiplier. |
| `validation_max_multiplier` | `2.0` | Upper bound for each independently sampled validation error multiplier. |
| `stress_min_multiplier` | `2.0` | Lower bound for each independently sampled stress error multiplier. |
| `stress_max_multiplier` | `5.0` | Upper bound for each independently sampled stress error multiplier. |
| `stress_dropout_probability` | `0.05` | Dropout probability used by the stress profile. |
| `stress_outlier_probability` | `0.01` | Outlier probability used by the stress profile. |
| `stress_latency_ms` | `150` | Measurement delivery delay used by the stress profile. |
| `stress_reported_covariance_scale` | `0.7` | Covariance multiplier used by the stress profile; below `1.0` deliberately reports overconfidence. |

## Reproducible experiment workflow

1. Run calibration routes over the expected speed and turn-rate range. Set initial
   EKF covariances from the source model and check innovation consistency.
2. Freeze those EKF settings.
3. Run validation over several seeds and routes. Record seed, profile, parameter
   file revision, route, and estimated-vs-ground-truth error.
4. Run stress scenarios. Examine dropped/late/outlier behaviour and verify that the
   filter is not silently overconfident.
5. Replay an interesting run using its logged seed and exact parameter file.

Do not choose covariance values by minimizing error on one seed. A fixed seed is for
debugging and reproducibility; a set of held-out seeds is for evaluating the filter.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| No `/sim/zed/vio/odom` output | Confirm `truth_topic` exists, is type `nav_msgs/msg/Odometry`, and has non-zero timestamps. |
| VIO publishes immediately despite configured latency | Confirm the node has `use_sim_time:=true` and Gazebo is publishing `/clock`. |
| EKF becomes implausibly certain | Verify the truth source is not wheel odometry, fuse VIO pose only, and do not report covariance below the actual nominal model during calibration. |
| Two `odom -> base_link` TF publishers | Disable TF output on measurement sources; only the EKF should publish that transform. |
| A result cannot be reproduced | Use the resolved seed printed at node startup and the same profile and parameter values. |
