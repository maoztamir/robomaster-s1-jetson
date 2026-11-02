#!/usr/bin/env bash
# Jetson-only mapping patrol launcher for a non-interactive SSH session.
# Pauses the user-owned teleop processes because sudo is unavailable over SSH.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
unset FASTDDS_DEFAULT_PROFILES_FILE

systemctl is-active --quiet robomaster-chassis
systemctl is-active --quiet robomaster-lidar

mkdir -p "$HOME/.cache"
exec 9>"$HOME/.cache/robomaster-patrol.lock"
flock -n 9 || { echo 'A robot patrol session is already running' >&2; exit 1; }

teleop_cgroup=/sys/fs/cgroup/system.slice/robomaster-teleop.service/cgroup.procs
teleop_pids=()
if systemctl is-active --quiet robomaster-teleop; then
    [[ -r "$teleop_cgroup" ]] || { echo 'Cannot inspect the teleop service' >&2; exit 1; }
    mapfile -t teleop_pids < "$teleop_cgroup"
    ((${#teleop_pids[@]} > 0)) || { echo 'Teleop service has no processes' >&2; exit 1; }
    for pid in "${teleop_pids[@]}"; do
        [[ "$(stat -c %U "/proc/$pid")" == "$(id -un)" ]] || {
            echo "Refusing to pause teleop process $pid owned by another user" >&2
            exit 1
        }
    done
fi

patrol_pid=''
cleanup() {
    trap - EXIT INT TERM HUP
    if [[ -n "$patrol_pid" ]]; then
        kill -TERM "$patrol_pid" 2>/dev/null || true
        wait "$patrol_pid" 2>/dev/null || true
    fi
    timeout 3 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
        '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
    sleep .5
    for pid in "${teleop_pids[@]}"; do kill -CONT "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM HUP

for pid in "${teleop_pids[@]}"; do kill -STOP "$pid"; done

: "${PATROL_LIDAR_YAW:=0.0}"
: "${PATROL_FORWARD_COMMAND:=0.5}"
: "${PATROL_TURN_COMMAND:=0.5}"
: "${PATROL_RADIUS:=0.4}"
: "${PATROL_CLEARANCE:=0.75}"

/usr/bin/python3 "$repo_dir/s1_can/patrol_node.py" --ros-args \
    -p enabled:=true -p duration:=120.0 -p require_tracking:=true \
    -p lidar_yaw:="$PATROL_LIDAR_YAW" -p forward:="$PATROL_FORWARD_COMMAND" \
    -p turn:="$PATROL_TURN_COMMAND" -p radius:="$PATROL_RADIUS" \
    -p clearance:="$PATROL_CLEARANCE" &
patrol_pid=$!
wait "$patrol_pid"
patrol_pid=''
