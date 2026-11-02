#!/usr/bin/env bash
# Stationary bring-up only: no cmd_vel publishers, no automatic patrol health.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-$JETSON_HOST}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
unset ROS_LOCALHOST_ONLY CONDA_PREFIX CONDA_DEFAULT_ENV
mkdir -p "$repo/maps/hardware"
exec 9>"$repo/maps/hardware/.mapping.lock"
flock -n 9 || { echo 'Stop existing hardware mapping first' >&2; exit 1; }
pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
    sleep 2
    for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
    wait || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
setsid bash "$repo/scripts/run_stereo_rectified_preview.sh" --headless & pids+=("$!")
setsid env PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}" "$repo/.venv/bin/python" "$repo/scripts/fusion_input_gate.py" & pids+=("$!")
setsid ros2 launch "$repo/config/fusion_odometry.launch.xml" repo:="$repo" & pids+=("$!")
wait -n "${pids[@]}"
