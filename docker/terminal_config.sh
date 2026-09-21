# Use the configured workspace, or ~/ros2_ws for standalone containers.
_rover_ws="${ROVER_WS:-$HOME/ros2_ws}"
_rover_prefix="$_rover_ws/.pixi/envs/${PIXI_ENVIRONMENT_NAME:-default}"

if [ -d "$_rover_ws" ]; then
    # Load the Pixi ROS environment when installed.
    if [ -f "$_rover_prefix/setup.bash" ]; then
        source "$_rover_prefix/setup.bash"
    fi

    # Load the workspace overlay after a successful build.
    if [ -f "$_rover_ws/install/setup.bash" ]; then
        source "$_rover_ws/install/setup.bash"
    fi

    # Use this environment's executables and completion helper.
    if [ -d "$_rover_prefix/bin" ]; then
        export PATH="$_rover_prefix/bin:$PATH"
    fi

    if [ -f "$_rover_prefix/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ]; then
        source "$_rover_prefix/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
    fi

    cd "$_rover_ws"
fi

bind 'set show-all-if-ambiguous on'
unset _rover_ws _rover_prefix

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