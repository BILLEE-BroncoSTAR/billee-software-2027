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

# Dynamic welcome message based on the active Pixi environment
if [ -n "$PIXI_ENVIRONMENT_NAME" ]; then
  echo -e "\e[1;32m➜ Active Pixi Environment: $PIXI_ENVIRONMENT_NAME\e[0m"
else
echo "========================================="
echo "          Project BILL-EE 2027           "
echo "========================================="
echo "To get started, activate an environment:"
echo "  $ pixi shell              | Default x86 Environment"
echo "  $ pixi shell -e mac-cpu   | Apple-Silicon Mac Environment"
echo "  $ pixi shell -e l4t       | Jetson Environment"
echo ""
fi