# Initial shell configuration, which should be run as the user after dependencies have been loaded

# Initialize ROS 2 from the Pixi environment selected for this shell.  The
# fallback supports opening a regular dev-container terminal before `pixi
# shell`; a Jetson `pixi shell -e l4t` selects the l4t prefix automatically.
_billee_ws=/workspaces/URC-2027/ros2_ws
_billee_pixi_env=${PIXI_ENVIRONMENT_NAME:-default}
_billee_pixi_prefix=$_billee_ws/.pixi/envs/$_billee_pixi_env

if [ -f "$_billee_pixi_prefix/setup.bash" ]; then
  source "$_billee_pixi_prefix/setup.bash"
fi

# Keep the overlay separate from the Pixi underlay.  local_setup.bash contains
# only workspace packages; setup.bash may replay an underlay recorded by a
# build performed on a different platform.
if [ -f "$_billee_pixi_prefix/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ]; then
  source "$_billee_pixi_prefix/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
fi
if [ -f "$_billee_ws/install/local_setup.bash" ]; then
  source "$_billee_ws/install/local_setup.bash"
fi

unset _billee_ws _billee_pixi_env _billee_pixi_prefix
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
