# Pixi Cheatsheet

| Task | Command |
|------|---------|
| Activate environment in terminal | `pixi shell` |
| Install dependencies | `pixi install` |
| Add a dependency | `pixi add <package>` |
| Remove a dependency | `pixi remove <package>` |
| List installed packages | `pixi list` |
| Build a ROS2 package | `pixi run colcon build` |
| Build a single package only | `pixi run colcon build --packages-select <package>` |
| Source the workspace overlay | `source install/setup.bash` |
| Run a node | `pixi run ros2 run <package> <node>` |
| Launch a launch file | `pixi run ros2 launch <package> <launch_file>` |
| List available packages | `pixi run ros2 pkg list` |
| List active nodes | `pixi run ros2 node list` |
| List active topics | `pixi run ros2 topic list` |
| Echo a topic | `pixi run ros2 topic echo <topic>` |
| List available tasks (pixi) | `pixi task list` |

