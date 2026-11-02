#!/usr/bin/env bash
set -euo pipefail
sensor_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$sensor_repo" == /home/ubuntu/robomaster-s1-jetson ]] || {
    echo "Run this installer from the Jetson checkout." >&2; exit 1;
}
for required in /opt/ros/humble/setup.bash \
    /opt/ros/humble/lib/gscam/gscam_node /opt/ros/humble/lib/rplidar_ros/rplidar_node \
    "$sensor_repo/output/calibration/refined-20260913/left.yaml" \
    "$sensor_repo/output/calibration/refined-20260913/right.yaml"; do
    [[ -f "$required" ]] || { echo "Missing $required" >&2; exit 1; }
done
if (( EUID != 0 )); then exec sudo bash "$0" "$@"; fi
# Use a non-sticky, user-owned directory: Linux protected_regular can reject
# root opening another user's existing file for writing in /tmp.
install -d -m 0755 -o ubuntu -g ubuntu /home/ubuntu/.cache/robomaster
runuser -u ubuntu -- touch /home/ubuntu/.cache/robomaster/stereo-start.lock
exec 9>>/home/ubuntu/.cache/robomaster/stereo-start.lock
flock -w 15 9
systemctl stop robomaster-cameras.service robomaster-lidar.service 2>/dev/null || true
# Retire only the project's manual camera launch and sensor nodes, never motors.
pkill -TERM -u ubuntu -f '^/usr/bin/python3 /opt/ros/humble/bin/ros2 launch /home/ubuntu/robomaster-s1-jetson/config/stereo_camera.launch.xml$' || true
pkill -TERM -u ubuntu -x gscam_node || true
pkill -TERM -u ubuntu -x rplidar_node || true
for attempt in {1..10}; do
    if ! pgrep -u ubuntu -x 'gscam_node|rplidar_node' >/dev/null; then break; fi
    sleep 1
done
if pgrep -u ubuntu -x 'gscam_node|rplidar_node' >/dev/null; then
    echo "Manual sensor processes did not stop; no new instances started." >&2; exit 1
fi
for unit in robomaster-cameras robomaster-lidar; do
    install -m 0644 "$sensor_repo/systemd/$unit.service" "/etc/systemd/system/$unit.service"
done
systemctl daemon-reload
systemctl enable --now robomaster-cameras.service robomaster-lidar.service
systemctl --no-pager --full status robomaster-cameras.service robomaster-lidar.service
