#!/usr/bin/env bash
# Support ordinary always-on LiDAR and the optional sensor power manager.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
ssh -o BatchMode=yes -o ConnectTimeout=5 "$JETSON_SSH_TARGET" /bin/bash <<'REMOTE'
set -e
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/robomaster-s1-jetson/config/calibration_dds.local.xml
if systemctl is-active --quiet robomaster-sensor-power; then
    timeout 20 ros2 service call /robomaster/sensors/wake std_srvs/srv/Trigger '{}'
elif systemctl is-active --quiet robomaster-lidar; then
    echo 'LiDAR is running; sensor sleep is disabled.'
else
    echo 'LiDAR is stopped. On the Jetson run: sudo systemctl start robomaster-lidar' >&2
    exit 1
fi
REMOTE
