#!/usr/bin/env bash
# Repeatable Nav2 sim tests: for each scenario and run, start a fresh headless sim in
# worlds/nav_test.sdf at the scenario's lane, run nav_scenario_test.py, shut the sim down.
#
#   pixi run bash src/navigation/scripts/run_nav_tests.sh [-n RUNS] [-o OUTDIR] [-b] [scenario ...]
#   (installed: ros2 run navigation run_nav_tests.sh ...)
#
# Scenarios: open detour narrow blocked cancel (default: all). Each run appends one JSON line
# to OUTDIR/results.jsonl and keeps its launch log; a node that crashed ("process has died") is
# recorded as such, separately from a goal that was rejected or aborted.
# -b also records a bag per run (OUTDIR/bag_<scenario>_<run>): poses (truth, EKFs), commands,
# plans, costmaps, footprint, BT log, camera_info (the depth camera's rate) and TF.
#
# Runs on an isolated DDS domain (ROS_DOMAIN_ID=77, localhost only) unless NAV_TEST_DOMAIN is set,
# so it never talks to a rover or another sim on the network. Needs an X display for the depth
# camera (see docs/gazebo.md).
set -u

runs=3
bag=0
outdir="nav_test_results/$(date +%Y%m%d_%H%M%S)"
while getopts "n:o:b" opt; do
  case $opt in
    n) runs=$OPTARG ;;
    o) outdir=$OPTARG ;;
    b) bag=1 ;;
    *) sed -n '2,13p' "$0"; exit 2 ;;
  esac
done
shift $((OPTIND - 1))
scenarios=("$@")
[ ${#scenarios[@]} -eq 0 ] && scenarios=(open detour narrow blocked cancel)

export ROS_DOMAIN_ID=${NAV_TEST_DOMAIN:-77}
export ROS_LOCALHOST_ONLY=1
mkdir -p "$outdir"
results="$outdir/results.jsonl"

nav2_active() {  # bt_navigator reaches 'active' within 180 s
  for _ in $(seq 1 60); do
    timeout 10 ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState 2>/dev/null \
      | grep -q "label='active'" && return 0
    sleep 3
  done
  return 1
}

lane_y() {
  case $1 in
    open|cancel) echo 0 ;; detour) echo 20 ;; narrow) echo 40 ;; blocked) echo 60 ;;
    *) echo "unknown scenario $1" >&2; exit 2 ;;
  esac
}

stop_sim() {  # $1 = launch process group id
  kill -INT -- "-$1" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 -- "-$1" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- "-$1" 2>/dev/null
  sleep 2
  # A SIGKILLed node leaves Fast DDS shared-memory segments behind; enough of them break
  # discovery for the next run (nodes start but never see each other). This removes only
  # segments no live process holds.
  command -v fastdds >/dev/null && fastdds shm clean >/dev/null 2>&1
}

for scenario in "${scenarios[@]}"; do
  y=$(lane_y "$scenario")
  for run in $(seq 1 "$runs"); do
    tag="${scenario}_${run}"
    log="$outdir/launch_${tag}.log"
    echo "== $tag (lane y=$y)"
    # Start-up failures are relaunched (up to 3 times, recorded), never scored as navigation:
    #  - Gazebo's render thread occasionally never initialises (seen under WSLg): no /clock;
    #  - a lifecycle change_state reply can be lost under start-up load: Nav2 never activates.
    for attempt in 1 2 3; do
      ros2 daemon stop >/dev/null 2>&1
      setsid ros2 launch chassis_bringup sim_gz.launch.py gui:=false joy_control:=false \
        nav_mode:=full world_file:=worlds/nav_test.sdf y:="$y" > "$log" 2>&1 &
      pgid=$!
      if timeout 120 ros2 topic echo --once --no-daemon /clock >/dev/null 2>&1 && nav2_active; then
        break
      fi
      echo "   sim or Nav2 did not start, relaunching"
      mv "$log" "${log%.log}_nostart_${attempt}.log"
      stop_sim "$pgid"
    done
    bpid=""
    if [ "$bag" = 1 ]; then
      setsid ros2 bag record -o "$outdir/bag_${tag}" --use-sim-time \
        --qos-profile-overrides-path "$(dirname "$(readlink -f "$0")")/bag_qos.yaml" \
        /gz/odom /odometry/filtered /odometry/global /odometry/gps /diff_drive_controller/odom \
        /diff_drive_controller/cmd_vel_unstamped /cmd_vel /plan /local_plan /goal_pose \
        /local_costmap/costmap /global_costmap/costmap /local_costmap/published_footprint \
        /behavior_tree_log /depth_cam/camera_info /joint_states /tf /tf_static /clock /rosout \
        > "$outdir/bag_${tag}.log" 2>&1 &
      bpid=$!
    fi
    if ! timeout 900 ros2 run navigation nav_scenario_test.py --scenario "$scenario" \
        --run "$run" --out "$results" > "$outdir/test_${tag}.log" 2>&1; then
      echo "{\"scenario\": \"$scenario\", \"run\": \"$run\", \"result\": \"TEST_SCRIPT_FAILED\"}" \
        >> "$results"
    fi
    # Count deaths before the shutdown: SIGINT itself can make processes exit non-zero.
    died=$(grep -c "process has died" "$log")
    grep "process has died" "$log" | sed 's/^/   crash: /'
    [ -n "$bpid" ] && { kill -INT -- "-$bpid" 2>/dev/null; sleep 3; }
    stop_sim "$pgid"
    echo "{\"scenario\": \"$scenario\", \"run\": \"$run\", \"launch_process_deaths\": $died, \"sim_launch_attempts\": $attempt}" \
      >> "$outdir/crashes.jsonl"
    tail -n 1 "$results"
  done
done
echo "results: $results"
