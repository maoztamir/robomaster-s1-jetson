#!/usr/bin/env bash
# Optional Jetson sensor sleep. Run with L1 released and the robot parked.
set -eo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
[[ "$repo" == /home/ubuntu/robomaster-s1-jetson ]] || { echo 'Run on the Jetson checkout.' >&2; exit 1; }
(( EUID == 0 )) || { echo 'Run: sudo bash scripts/install_sensor_power.sh' >&2; exit 1; }
for unit in robomaster-chassis robomaster-lidar robomaster-cameras robomaster-teleop; do
    systemctl cat "$unit.service" >/dev/null
done
backup="$(mktemp -d /etc/systemd/system/robomaster-sensor-power-backup.XXXXXX)"
echo "Service backup: $backup"
if [[ -d /etc/systemd/system/robomaster-chassis.service.d ]]; then
    cp -a /etc/systemd/system/robomaster-chassis.service.d "$backup/"
fi
if [[ -f /etc/systemd/system/robomaster-sensor-power.service ]]; then
    cp -a /etc/systemd/system/robomaster-sensor-power.service "$backup/"
fi
teleop_active=false
systemctl is-active --quiet robomaster-teleop && teleop_active=true
systemctl stop robomaster-teleop
# Build as ubuntu with the Jetson's ROS Humble interpreter, not the Dell venv.
if ! runuser -u ubuntu -- /bin/bash -c '
    set -e
    cd /home/ubuntu/robomaster-s1-jetson
    source /opt/ros/humble/setup.bash
    colcon --log-base ros2_log build --packages-select s1_chassis_driver --symlink-install \
      --build-base ros2_build --install-base ros2_install
'; then
    [[ "$teleop_active" == false ]] || systemctl start robomaster-teleop
    exit 1
fi
mkdir -p /etc/systemd/system/robomaster-chassis.service.d
cat > /etc/systemd/system/robomaster-chassis.service.d/sensor-sleep.conf <<'UNIT'
[Service]
ExecStart=
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/ubuntu/robomaster-s1-jetson/ros2_install/setup.bash && exec ros2 run s1_chassis_driver s1_chassis_node --ros-args --params-file /home/ubuntu/robomaster-s1-jetson/config/s1_chassis.yaml -p sensor_sleep_guard:=true"
UNIT
# Grant only the four exact sensor start/stop operations needed by the manager.
cat > "$backup/sudoers.new" <<'SUDOERS'
ubuntu ALL=(root) NOPASSWD: /usr/bin/systemctl start robomaster-lidar.service, /usr/bin/systemctl stop robomaster-lidar.service, /usr/bin/systemctl start robomaster-cameras.service, /usr/bin/systemctl stop robomaster-cameras.service
SUDOERS
visudo -cf "$backup/sudoers.new"
install -m 0440 "$backup/sudoers.new" /etc/sudoers.d/robomaster-sensors
install -m 0644 "$repo/systemd/robomaster-sensor-power.service" /etc/systemd/system/
systemctl daemon-reload
systemctl start robomaster-lidar
systemctl enable robomaster-sensor-power
systemctl restart robomaster-chassis robomaster-sensor-power
[[ "$teleop_active" == false ]] || systemctl start robomaster-teleop
systemctl --no-pager status robomaster-sensor-power robomaster-chassis
echo 'Installed. Keep L1 released; after 5 minutes idle sensors sleep.'
echo 'Gamepad input wakes LiDAR; wheel motion waits for fresh scans.'
