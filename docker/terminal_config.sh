# Initial shell configuration, which should be run as the user after dependencies have been loaded

# Initialize ROS 2 toolchain from the Pixi Environment
source /workspaces/URC-2027/ros2_ws/.pixi/envs/default/setup.bash
# Initialize colcon autocomplete from the Pixi environment
source /workspaces/URC-2027/ros2_ws/.pixi/envs/default/share/colcon_argcomplete/hook/colcon-argcomplete.bash
# Source the local workspace overlay to enable autocomplete
source /workspaces/URC-2027/ros2_ws/install/setup.bash
# Override GNU Readline defaults to display ambiguous autocomplete matches on first tab (instead of second)
bind 'set show-all-if-ambiguous on'
# Navigate to ros2_ws
cd ros2_ws