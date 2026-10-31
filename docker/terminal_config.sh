# Initial shell configuration, which should be run as the user after dependencies have been loaded

# Devcontainers mount the repo at /workspaces/<clone folder name>, so look ros2_ws up rather than
# hardcoding it. BILLEE_PIXI_ENV is set per devcontainer (mac-cpu, l4t); the x86 desktop uses default.
ROS2_WS=$(ls -d /workspaces/*/ros2_ws 2>/dev/null | head -n 1)
PIXI_ENV_DIR="$ROS2_WS/.pixi/envs/${BILLEE_PIXI_ENV:-default}"

if [ -n "$ROS2_WS" ]; then
  # Initialize ROS 2 toolchain from the Pixi Environment
  [ -f "$PIXI_ENV_DIR/setup.bash" ] && source "$PIXI_ENV_DIR/setup.bash"
  # Initialize colcon autocomplete from the Pixi environment
  [ -f "$PIXI_ENV_DIR/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ] && source "$PIXI_ENV_DIR/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
  # Source the local workspace overlay to enable autocomplete
  [ -f "$ROS2_WS/install/setup.bash" ] && source "$ROS2_WS/install/setup.bash"
  # Navigate to ros2_ws
  cd "$ROS2_WS"
fi
# Override GNU Readline defaults to display ambiguous autocomplete matches on first tab (instead of second)
bind 'set show-all-if-ambiguous on'

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
