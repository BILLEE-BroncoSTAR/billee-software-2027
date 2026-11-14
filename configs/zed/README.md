# ZED factory calibration

`SN<serial>.conf` is a ZED camera's factory calibration. The Jetson dev container copies these
files into `/usr/local/zed/settings/` at every start (`postStartCommand`), so the camera opens
without internet. Without the file, the ZED SDK downloads it from Stereolabs on the first open.

`SN36243153.conf` is the team's ZED 2i.

To add another camera: open it once in the Jetson container with internet (for example
`ros2 launch people people_rover.launch.py openpose:=false`), then copy
`/usr/local/zed/settings/SN<its serial>.conf` into this folder and commit it.
