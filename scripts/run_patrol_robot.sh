#!/usr/bin/env bash
# Manual Jetson patrol. Default is a non-driving, 15-second scan check.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:---check}"
[[ "$mode" == --check || "$mode" == --drive || "$mode" == --mapping ]] || { echo 'Use --check, --drive or --mapping' >&2; exit 2; }
require_tracking=false
if [[ "$mode" == --mapping ]]; then require_tracking=true; fi
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
unset FASTDDS_DEFAULT_PROFILES_FILE
[[ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]] || { echo 'Missing Jetson DDS configuration' >&2; exit 1; }
mkdir -p "$HOME/.cache"
exec 9>"$HOME/.cache/robomaster-patrol.lock"
flock -n 9 || { echo 'A robot patrol session is already running' >&2; exit 1; }
if [[ "$mode" == --check ]]; then
    exec /usr/bin/python3 "$repo_dir/s1_can/patrol_node.py" --ros-args \
        -p enabled:=true -p duration:=15.0 -r /cmd_vel:=/patrol/check_cmd_vel
fi
# September 15 supervised pulse results: .5 forward -> ~10 cm;
# .5 positive yaw -> ~20 degrees LEFT. Not calibrated SI velocities.
# Clearances are provisional margins around the recorded stock footprint.
: "${PATROL_LIDAR_YAW:=0.0}"
: "${PATROL_FORWARD_COMMAND:=0.5}"
: "${PATROL_TURN_COMMAND:=0.5}"
: "${PATROL_RADIUS:=0.4}"
: "${PATROL_CLEARANCE:=0.75}"
: "${PATROL_DURATION:=120.0}"  # patrol_node rejects more than 600 s
systemctl is-active --quiet robomaster-chassis
systemctl is-active --quiet robomaster-lidar
sudo -v
restore_teleop=false
if systemctl is-active --quiet robomaster-teleop; then restore_teleop=true; fi
patrol_pid=''
cleanup() {
    if [[ -n "$patrol_pid" ]]; then
        kill -TERM "$patrol_pid" 2>/dev/null || true
        wait "$patrol_pid" 2>/dev/null || true
    fi
    # Give the chassis watchdog time to expire before restoring manual driving.
    sleep .5
    if [[ "$restore_teleop" == true ]]; then sudo -n systemctl start robomaster-teleop; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
sudo -n systemctl stop robomaster-teleop
/usr/bin/python3 "$repo_dir/s1_can/patrol_node.py" --ros-args \
    -p enabled:=true -p duration:="$(printf %.1f "$PATROL_DURATION")" -p require_tracking:="$require_tracking" \
    -p lidar_yaw:="$PATROL_LIDAR_YAW" -p forward:="$PATROL_FORWARD_COMMAND" \
    -p turn:="$PATROL_TURN_COMMAND" -p radius:="$PATROL_RADIUS" \
    -p clearance:="$PATROL_CLEARANCE" &
patrol_pid=$!
wait "$patrol_pid"
patrol_pid=''
