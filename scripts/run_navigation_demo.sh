#!/usr/bin/env bash
# One terminal controls the simulation stack; graphics appear on the simulation host's desktop.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help ]]; then
    echo "Usage: bash scripts/run_navigation_demo.sh [--slam] [--provisional]"
    echo "Starts Isaac, Nav2 and RViz on ubuntu@192.0.2.33. Ctrl-C stops all three."
    echo "--provisional explicitly accepts the unmeasured simulation sensor mounts."
    echo "--slam builds a map with stereo-assisted RTAB-Map and LiDAR."
    exit 0
fi
sim_args=()
slam=false
for arg in "$@"; do
    case "$arg" in
        --provisional) sim_args=(--provisional);;
        --slam) slam=true;;
        *) echo "Usage: $0 [--slam] [--provisional]" >&2; exit 2;;
    esac
done
if [[ ! -d /home/ubuntu/isaacsim ]]; then
    exec ssh -t "$ISAAC_SSH_TARGET" \
        bash /home/ubuntu/robomaster-s1-jetson/scripts/run_navigation_demo.sh "$@"
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
    echo "Open a fresh terminal without another ROS distribution sourced." >&2; exit 1
fi
command -v setsid >/dev/null
command -v flock >/dev/null
mkdir -p "$HOME/.cache/robomaster-navigation"
exec 9>"$HOME/.cache/robomaster-navigation/launcher.lock"
flock -n 9 || { echo "The navigation demo launcher is already running." >&2; exit 1; }
if pgrep -f '/kit/python/bin/python3 .*simulation/isaac_navigation.py|/opt/ros/jazzy/bin/ros2 launch .*robomaster_nav2.launch.xml' >/dev/null; then
    echo "Isaac navigation or Nav2 is already running. Close its launch terminal before starting this demo." >&2
    exit 1
fi
export DISPLAY="${DISPLAY:-:1}"
export ROBOMASTER_WEBRTC="${ROBOMASTER_WEBRTC:-1}"
log_dir="$(mktemp -d "$HOME/.cache/robomaster-navigation/run-XXXXXXXX")"
export ROBOMASTER_SLAM=0
if [[ "$slam" == true ]]; then
    export ROBOMASTER_SLAM=1 ROBOMASTER_SLAM_DB="$log_dir/rtabmap.db"
fi
echo "Simulation domain: 43. Windows appear on the simulation host's desktop ($DISPLAY)."
echo "Logs: $log_dir"
pids=()
cleanup() {
    trap - EXIT INT TERM HUP
    if (( ${#pids[@]} )); then
        echo "Stopping navigation, RViz and Isaac..."
        for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
        for ((i=0; i<10; i++)); do
            alive=false
            for pid in "${pids[@]}"; do kill -0 -- "-$pid" 2>/dev/null && alive=true; done
            [[ "$alive" == false ]] && break
            sleep 1
        done
        for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
        sleep 1
        for pid in "${pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
start() {
    local mode="$1"
    shift
    setsid bash "$script_dir/run_isaac_navigation.sh" "$mode" "$@" >"$log_dir/$mode.log" 2>&1 9>&- &
    pids+=("$!")
}
wait_ready() {
    local mode="$1" pattern="$2" limit="$3" started=$SECONDS
    until grep -q "$pattern" "$log_dir/$mode.log"; do
        for pid in "${pids[@]}"; do
            if ! kill -0 "$pid" 2>/dev/null; then
                echo "A component exited during startup. See $log_dir" >&2
                tail -30 "$log_dir/$mode.log" >&2
                return 1
            fi
        done
        if (( SECONDS - started >= limit )); then
            echo "Timed out waiting for $mode. See $log_dir/$mode.log" >&2; return 1
        fi
        if (( (SECONDS - started) % 30 == 0 )); then echo "Waiting for $mode..."; fi
        sleep 1
    done
}
echo "Starting Isaac (first shader compilation may take several minutes)..."
start sim "${sim_args[@]}"
wait_ready sim 'RoboMaster planar navigation prototype ready' 600
if [[ "$slam" == true ]]; then
    start slam
    wait_ready slam 'rtabmap (' 120
    echo "Waiting for the first SLAM occupancy map..."
    bash "$script_dir/run_isaac_navigation.sh" wait-map >"$log_dir/map-ready.log" 2>&1 || {
        echo "SLAM did not publish a map. See $log_dir/slam.log" >&2; exit 1;
    }
    grep -Eq '^[1-9][0-9]*$' "$log_dir/map-ready.log" || {
        echo "SLAM map has no cells. See $log_dir/slam.log" >&2; exit 1;
    }
fi
start nav2
wait_ready nav2 'Managed nodes are active' 120
critical_pids=("${pids[@]}")
start rviz
echo "Ready. Use Nav2 Goal in RViz. Ctrl-C here stops the entire demo."
# RViz is only a viewer: closing it or a GUI crash must not stop navigation.
# Keep supervising the simulator and Nav2; either exiting stops the stack.
wait -n "${critical_pids[@]}"
