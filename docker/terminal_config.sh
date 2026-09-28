# Initial shell configuration, which should be run as the user after dependencies have been loaded

# Resolve the workspace + Pixi environment (set by `make setup <platform>`, else detected).
# The Dockerfiles copy tooling/billee-env.sh here.
if [ -f /usr/local/lib/billee/billee-env.sh ]; then
  source /usr/local/lib/billee/billee-env.sh
fi

if [ -n "$BILLEE_WS" ]; then
  _billee_env_dir="$BILLEE_WS/.pixi/envs/${PIXI_ENVIRONMENT_NAME:-$BILLEE_PIXI_ENV}"
  # Initialize ROS 2 toolchain from the Pixi Environment
  [ -f "$_billee_env_dir/setup.bash" ] && source "$_billee_env_dir/setup.bash"
  # Initialize colcon autocomplete from the Pixi environment
  [ -f "$_billee_env_dir/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ] &&
    source "$_billee_env_dir/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
  # Source the local workspace overlay to enable autocomplete
  [ -f "$BILLEE_WS/install/setup.bash" ] && source "$BILLEE_WS/install/setup.bash"
  unset _billee_env_dir
  # Navigate to ros2_ws
  cd "$BILLEE_WS" || true
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
echo "Platform: ${BILLEE_PLATFORM:-unknown}  (Pixi environment: ${BILLEE_PIXI_ENV:-default})"
echo "To get started, activate the environment:"
echo "  $ pixi shell -e ${BILLEE_PIXI_ENV:-default}"
echo ""
fi
