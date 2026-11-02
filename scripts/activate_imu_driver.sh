#!/usr/bin/env bash
# Activate staged IMU reception with rollback; keep robot stationary, L1 released.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
stage="$repo_dir/.staging/imu-driver"
for file in chassis.py ros_node.py imu.py; do test -s "$stage/$file"; done
mkdir -p "$HOME/.cache" "$HOME/can-debug"
exec 9>"$HOME/.cache/robomaster-patrol.lock"
flock -n 9 || { echo 'Stop patrol first.' >&2; exit 1; }
if pgrep -x probe_attitude >/dev/null; then echo 'Stop the manual IMU probe first.' >&2; exit 1; fi
systemctl is-active --quiet robomaster-chassis
sudo -v
restore_teleop=false
systemctl is-active --quiet robomaster-teleop && restore_teleop=true
backup=$(mktemp -d "$HOME/can-debug/imu-driver-backup-XXXXXXXX")
cp "$repo_dir/s1_can/chassis.py" "$repo_dir/s1_can/ros_node.py" "$repo_dir/config/s1_chassis.yaml" "$backup/"
if [[ -f "$repo_dir/s1_can/imu.py" ]]; then cp "$repo_dir/s1_can/imu.py" "$backup/"; fi
success=false
modified=false
capture_pid=''
cleanup() {
    if [[ -n "$capture_pid" ]]; then
        kill -TERM "$capture_pid" 2>/dev/null || true
        wait "$capture_pid" 2>/dev/null || true
    fi
    journalctl -u robomaster-chassis --since "$capture_started" --no-pager > "$backup/chassis-journal.log" || true
    if [[ "$modified" == true && "$success" != true ]]; then
        echo "Activation failed: restoring previous driver from $backup" >&2
        sudo -n systemctl stop robomaster-chassis || true
        cp "$backup/chassis.py" "$backup/ros_node.py" "$repo_dir/s1_can/"
        cp "$backup/s1_chassis.yaml" "$repo_dir/config/"
        if [[ -f "$backup/imu.py" ]]; then cp "$backup/imu.py" "$repo_dir/s1_can/"; else rm -f "$repo_dir/s1_can/imu.py"; fi
    fi
    sudo -n systemctl start robomaster-chassis
    if [[ "$restore_teleop" == true ]]; then sudo -n systemctl start robomaster-teleop; fi
}
capture_started=$(date '+%Y-%m-%d %H:%M:%S')
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
sudo -n systemctl stop robomaster-teleop
sleep .5
sudo -n systemctl stop robomaster-chassis
# Record the outgoing subscription and incoming controller replies, without TX.
candump -L can1,201:7FF,202:7FF > "$backup/can-activation.log" &
capture_pid=$!
modified=true
cp "$stage/chassis.py" "$stage/ros_node.py" "$stage/imu.py" "$repo_dir/s1_can/"
sed -i '/^[[:space:]]*enable_imu:/d' "$repo_dir/config/s1_chassis.yaml"
printf '\n    enable_imu: true\n' >> "$repo_dir/config/s1_chassis.yaml"
sudo -n systemctl start robomaster-chassis
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
unset FASTDDS_DEFAULT_PROFILES_FILE
timeout 25 ros2 topic echo /robomaster/imu/yaw sensor_msgs/msg/Imu --once \
    | tee "$backup/imu-topic.log"
systemctl is-active --quiet robomaster-chassis
success=true
echo "IMU driver active; previous version saved in $backup"
