# Initial shell configuration, which should be run as the user after dependencies have been loaded

# Resolve the workspace + Pixi environment (set by `make setup <platform>`, else detected).
# The desktop Dockerfiles copy tooling/billee-env.sh here.
if [ -f /usr/local/lib/billee/billee-env.sh ]; then
  source /usr/local/lib/billee/billee-env.sh
fi
# Without it, the workspace is wherever the repo is mounted: <repo>/ros2_ws (x86
# devcontainer, /workspaces/<repo-folder>), or ros2_ws itself (Mac devcontainer).
if [ -z "$BILLEE_WS" ]; then
  for _d in "$PWD/ros2_ws" "$PWD" /workspaces/*/ros2_ws "$HOME/ros2_ws"; do
    if [ -f "$_d/pixi.toml" ]; then BILLEE_WS=$_d; break; fi
  done
  unset _d
fi

if [ -n "$BILLEE_WS" ]; then
  # The Pixi environment selected for this shell (`pixi shell -e l4t` sets it), else this
  # machine's default.
  _billee_env_dir="$BILLEE_WS/.pixi/envs/${PIXI_ENVIRONMENT_NAME:-${BILLEE_PIXI_ENV:-default}}"
  # Initialize ROS 2 toolchain from the Pixi Environment
  [ -f "$_billee_env_dir/setup.bash" ] && source "$_billee_env_dir/setup.bash"
  # Initialize colcon autocomplete from the Pixi environment
  [ -f "$_billee_env_dir/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ] &&
    source "$_billee_env_dir/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
  # Keep the overlay separate from the Pixi underlay: local_setup.bash contains only
  # workspace packages; setup.bash may replay an underlay recorded by a build performed
  # on a different platform.
  [ -f "$BILLEE_WS/install/local_setup.bash" ] && source "$BILLEE_WS/install/local_setup.bash"
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
