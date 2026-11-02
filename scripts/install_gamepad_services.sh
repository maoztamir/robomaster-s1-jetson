#!/usr/bin/env bash
# Install the Jetson gamepad and chassis boot services.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$script_dir")"
if [[ "$project_dir" != /home/ubuntu/robomaster-s1-jetson ]]; then
    echo "Run this installer from /home/ubuntu/robomaster-s1-jetson on the Jetson." >&2
    exit 1
fi
for required in /opt/ros/humble/setup.bash "$project_dir/ros2_install/setup.bash" \
    "$project_dir/config/calibration_dds.xml" \
    /opt/ros/humble/lib/joy_linux/joy_linux_node \
    /opt/ros/humble/lib/teleop_twist_joy/teleop_node; do
    [[ -f "$required" ]] || { echo "Missing: $required" >&2; exit 1; }
done
if (( EUID != 0 )); then
    exec sudo bash "$0" "$@"
fi
systemctl cat robomaster-can.service >/dev/null
systemctl cat NetworkManager-wait-online.service >/dev/null
# network-online.target alone does not wait when its provider is disabled.
systemctl enable NetworkManager-wait-online.service
backup_dir="$(mktemp -d /etc/systemd/system/robomaster-backup.XXXXXX)"
for unit in robomaster-chassis robomaster-joy robomaster-teleop; do
    if [[ -f "/etc/systemd/system/$unit.service" ]]; then
        cp -p "/etc/systemd/system/$unit.service" "$backup_dir/"
    fi
done
echo "Previous units saved in $backup_dir"
systemctl stop robomaster-teleop.service robomaster-joy.service 2>/dev/null || true
systemctl stop robomaster-chassis.service
if pgrep -u ubuntu -f '[/](s1_chassis_node|joy_linux_node|teleop_node)( |$)' >/dev/null; then
    echo "Stop manual chassis/gamepad ROS nodes before running this installer." >&2
    exit 1
fi
for unit in robomaster-chassis robomaster-joy robomaster-teleop; do
    install -D -m 0644 "$project_dir/systemd/$unit.service" "/etc/systemd/system/$unit.service"
done
systemctl daemon-reload
systemctl enable --now robomaster-can.service robomaster-chassis.service robomaster-joy.service robomaster-teleop.service
systemctl --no-pager --full status robomaster-chassis.service robomaster-joy.service robomaster-teleop.service
