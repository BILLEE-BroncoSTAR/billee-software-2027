# Load the environment prepared by container startup.
_rover_ws="${ROVER_WS:-$HOME/ros2_ws}"
_rover_env="${PIXI_ENVIRONMENT_NAME:-${ROVER_PIXI_ENVIRONMENT:-default}}"

if [ -d "$_rover_ws" ]; then
    cd "$_rover_ws"
    _rover_prefix="$(pwd -P)/.pixi/envs/$_rover_env"

    if [ -r "$_rover_prefix/.rover-ready" ] &&
       [ "$(cat "$_rover_prefix/.rover-ready")" = "$_rover_prefix" ]; then

        source "$_rover_prefix/setup.bash"

        if [ -f "$_rover_ws/install/setup.bash" ]; then
            source "$_rover_ws/install/setup.bash"
        fi

        export PATH="$_rover_prefix/bin:$PATH"
        source "$_rover_prefix/share/colcon_argcomplete/hook/colcon-argcomplete.bash"
    else
        echo "Pixi setup is not ready. Check the container startup log."
    fi
fi

bind 'set show-all-if-ambiguous on'
unset _rover_ws _rover_env _rover_prefix

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
