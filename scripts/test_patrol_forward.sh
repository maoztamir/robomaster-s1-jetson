#!/usr/bin/env bash
# Supervised short pulse; command values are not calibrated physical velocities.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
command_value="${1:-0.3}"
case "$command_value" in 0.3|0.5) ;; *) echo 'Use command 0.3 or 0.5' >&2; exit 2;; esac
motion="${2:-forward}"
case "$motion" in
    forward) twist="{linear: {x: $command_value}}" ;;
    turn) twist="{angular: {z: $command_value}}" ;;
    *) echo 'Use forward or turn' >&2; exit 2 ;;
esac
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
unset FASTDDS_DEFAULT_PROFILES_FILE
systemctl is-active --quiet robomaster-chassis
sudo -v
restore=false
if systemctl is-active --quiet robomaster-teleop; then restore=true; fi
cleanup() {
    timeout 5 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}' >/dev/null 2>&1 || true
    sleep .5
    if [[ "$restore" == true ]]; then sudo -n systemctl start robomaster-teleop; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
sudo -n systemctl stop robomaster-teleop
count=$(ros2 topic info /cmd_vel | sed -n 's/^Publisher count: //p')
[[ "$count" == 0 ]] || { echo "Another velocity publisher exists ($count); aborting." >&2; exit 1; }
echo "$motion pulse: command $command_value, ten messages at 10 Hz. Keep gamepad L1 released."
timeout --signal=INT --kill-after=2 5 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "$twist" --rate 10 --times 10
echo 'Pulse finished; sending zero and restoring gamepad. Record distance or turn angle.'
