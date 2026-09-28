# comms

Serial device driver and a GPS node that publishes `sensor_msgs/NavSatFix`.

> **Status: work in progress — the node is not built.**
> `CMakeLists.txt` has no `add_executable`, no `find_package` for `rclcpp` or
> `sensor_msgs`, and no install rule, so `src/GpsNode.cpp` and
> `include/comms/SerialDevice.hpp` are not compiled into anything. `colcon build`
> succeeds and produces only the `TestGpsMsgPayloads` test. `package.xml` likewise
> declares no runtime dependencies.
>
> To finish it, the package needs `find_package(rclcpp)` / `find_package(sensor_msgs)`,
> an `add_executable(gps_node src/GpsNode.cpp)` with `ament_target_dependencies`, an
> `install(TARGETS ...)`, and matching `<depend>` entries in `package.xml`.

## Contents

| File | What it is |
|---|---|
| `include/comms/SerialDevice.hpp` | Read-only serial device wrapper |
| `include/comms/Gps.hpp` | Wire-layout structs for the GPS position and covariance payloads |
| `src/GpsNode.cpp` | Node that reads the device and publishes `NavSatFix` on `/gps` |
| `test/TestGpsMsgPayloads.cpp` | Asserts the payload structs match the wire layout (28 and 64 bytes) |

The payload test is the one thing here that builds and runs:

```bash
colcon test --packages-select comms && colcon test-result --verbose
```

## How GPS is meant to be used

`/gps` is **not** an input to the localization EKF in `navigation`. Global, absolute,
jump-prone data does not belong in a `world_frame: odom` filter — it would make
`odom` → `base_link` jump and break every consumer that assumes odom is continuous.

GPS belongs in a second `robot_localization` instance with `world_frame: map`, fed
through `navsat_transform_node`, publishing `map` → `odom`. See
[`navigation/README.md`](../navigation/README.md).
