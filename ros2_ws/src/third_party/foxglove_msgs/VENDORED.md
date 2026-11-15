# foxglove_msgs (vendored)

Foxglove's ROS message package, built from source in this workspace.

- Source: https://github.com/foxglove/foxglove-sdk, `ros/src/foxglove_msgs`
- Tag: `ros-v3.6.0` (commit `115b582496834d07e6ac6e8acf5676a883c6aec6`), unmodified.
  Only the ROS 2 parts are copied (`ros1/` is left out); `LICENSE` is the SDK's (MIT).
- `msg/` is generated from `ros2/` at configure time (see `CMakeLists.txt`) and is gitignored.

Why not the conda package: the pixi environment is locked to robostack build `_18`, and
`ros-humble-foxglove-msgs` exists only as build `_22`. Adding it re-solves the whole environment
(Python 3.12 -> 3.14, ~1800 lock entries). Switch to the conda package when the environment moves
to that generation, and delete this directory.

Used by `camera_stream` (`foxglove_msgs/msg/CompressedVideo`, H.264 for Foxglove's Image panel).
