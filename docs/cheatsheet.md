# Cheat sheet

## pixi (run from `ros2_ws`)

| | |
|---|---|
| `pixi shell -e <env>` | enter the env (`default`, `l4t`, `mac-cpu`) |
| `pixi run build` | build everything (inside the shell) |
| `pixi run build --packages-select <pkg>` | build one package |
| `pixi run -e <env> <cmd>` | run one command without entering the shell |
| `pixi run fmt` | `ruff check src/ --fix` |
| `pixi install -e <env>` | install or refresh the env |
| `pixi add <pkg>` / `pixi remove <pkg>` | change dependencies (`pixi.toml`) |
| `pixi task list` | list tasks |

After building a new package in an open shell: `source install/local_setup.bash`.

## ROS 2

| | |
|---|---|
| `ros2 node list` / `ros2 node info <node>` | nodes |
| `ros2 topic list -t` | topics with types |
| `ros2 topic echo <topic>` / `hz` / `info --verbose` | data, rate, publishers and QoS |
| `ros2 topic pub <topic> <type> '<yaml>'` | publish (`-r 10` for 10 Hz, `--once`) |
| `ros2 interface show <type>` | message/service/action definition |
| `ros2 service list -t` / `ros2 service call <srv> <type> '<yaml>'` | services |
| `ros2 action list -t` / `ros2 action send_goal <action> <type> '<yaml>'` | actions |
| `ros2 param list` / `get <node> <p>` / `set <node> <p> <v>` / `dump <node>` | parameters |
| `ros2 control list_controllers` / `list_hardware_components` | ros2_control state |
| `ros2 run tf2_tools view_frames` | TF tree → `frames.pdf` |
| `ros2 run rqt_graph rqt_graph` | node graph |
| `ros2 run rqt_console rqt_console` | log viewer |
| `ros2 run <pkg> <node> --ros-args --log-level debug` | per-node log level |
| `ros2 bag record -o <name> <topics...>` / `ros2 bag info` / `ros2 bag play <bag>` | record and replay |
| `ros2 pkg create --build-type ament_cmake --license Apache-2.0 <pkg>` | new package (`ament_python` for Python) |

## CAN

| | |
|---|---|
| `tooling/can-up` / `tooling/can-up status` | bring up the drivetrain bus (`can2`, 500 kbit/s) |
| `tooling/can-up vcan0` / `tooling/can-up down vcan0` | virtual bus (run on the host) |
| `candump -L <iface>` | watch frames |
