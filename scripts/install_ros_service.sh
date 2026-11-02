#!/usr/bin/env bash
# Install and start the Jetson ROS 2 chassis node with systemd.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$script_dir")"
service_name="robomaster-chassis.service"

if [[ "$project_dir" != "/home/ubuntu/robomaster-s1-jetson" ]]; then
    echo "Expected the Jetson checkout at /home/ubuntu/robomaster-s1-jetson." >&2
    exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ROS 2 Humble is not installed." >&2
    exit 1
fi
if [[ ! -f "$project_dir/ros2_install/setup.bash" ]]; then
    echo "Build s1_chassis_driver with colcon before installing the service." >&2
    exit 1
fi
if [[ ! -f "$project_dir/config/calibration_dds.local.xml" ]]; then
    echo "Missing config/calibration_dds.local.xml." >&2
    exit 1
fi

if ((EUID != 0)); then
    exec sudo -- "$0" "$@"
fi

# Stop an older installed instance cleanly before replacing its unit. SIGINT
# lets the node send its final zero wheel command during shutdown.
systemctl cat NetworkManager-wait-online.service >/dev/null
systemctl enable NetworkManager-wait-online.service
systemctl stop "$service_name" 2>/dev/null || true

if pgrep -u ubuntu -f "s1_chassis_node" >/dev/null; then
    echo "A manually started s1_chassis_node is still running." >&2
    echo "Stop it with Ctrl-C, then run this installer again." >&2
    exit 1
fi

install -D -m 0644 \
    "$project_dir/systemd/$service_name" \
    "/etc/systemd/system/$service_name"

systemctl daemon-reload
systemctl enable --now "$service_name"

echo "Installed and started $service_name."
systemctl --no-pager --full status "$service_name"
