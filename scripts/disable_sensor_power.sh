#!/usr/bin/env bash
# Run on the Jetson with the robot parked and L1 released.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$repo" == /home/ubuntu/robomaster-s1-jetson ]] || { echo 'Run on the Jetson checkout.' >&2; exit 1; }
(( EUID == 0 )) || { echo 'Run: sudo bash scripts/disable_sensor_power.sh' >&2; exit 1; }
backup="$(mktemp -d /etc/systemd/system/robomaster-sensor-sleep-disabled.XXXXXX)"
teleop_active=false
systemctl is-active --quiet robomaster-teleop && teleop_active=true
restore_teleop() {
    [[ "$teleop_active" == false ]] || systemctl start robomaster-teleop
}
trap restore_teleop EXIT
systemctl stop robomaster-teleop
# Stop the manager before starting LiDAR: it may hold the serial port exclusively.
systemctl disable --now robomaster-sensor-power.service
dropin=/etc/systemd/system/robomaster-chassis.service.d/sensor-sleep.conf
if [[ -f "$dropin" ]]; then mv "$dropin" "$backup/"; fi
if [[ -f /etc/sudoers.d/robomaster-sensors ]]; then
    mv /etc/sudoers.d/robomaster-sensors "$backup/"
fi
systemctl daemon-reload
# Restart loads the default sensor_sleep_guard=false; setting it live was insufficient.
systemctl restart robomaster-chassis.service
systemctl enable --now robomaster-lidar.service
if systemctl is-active --quiet robomaster-sensor-power.service; then
    echo 'Sensor sleep is unexpectedly still running.' >&2; exit 1
fi
systemctl is-active --quiet robomaster-chassis.service
systemctl is-active --quiet robomaster-lidar.service
echo "Sensor sleep disabled; LiDAR stays running. Configuration backup: $backup"
echo 'Jetson power mode and camera configuration are unchanged.'
