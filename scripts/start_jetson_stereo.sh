#!/usr/bin/env bash
# Ensure the calibrated camera publishers are started after a Jetson reboot.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
ssh -o BatchMode=yes -o ConnectTimeout=8 "$JETSON_SSH_TARGET" 'bash -s' <<'REMOTE'
set -eo pipefail
mkdir -p /home/ubuntu/.cache/robomaster
exec 9>>/home/ubuntu/.cache/robomaster/stereo-start.lock
flock -w 10 9
if systemctl is-enabled --quiet robomaster-cameras.service 2>/dev/null; then
    if systemctl is-active --quiet robomaster-cameras.service; then
        echo "Jetson cameras are managed by the boot service."
        exit 0
    fi
    echo "Camera boot service is not active. Check: journalctl -u robomaster-cameras.service" >&2
    exit 1
fi
camera_count=$(pgrep -cx gscam_node || true)
if [[ "$camera_count" == 2 ]]; then
    echo "Both Jetson camera processes are already running."
    exit 0
fi
if [[ "$camera_count" != 0 ]]; then
    echo "Only part of the camera stack is running; inspect it before restarting." >&2
    exit 1
fi
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
nohup ros2 launch /home/ubuntu/robomaster-s1-jetson/config/stereo_camera.launch.xml \
    >/tmp/robomaster-stereo-calibrated.log 2>&1 </dev/null 9>&- &
camera_launcher=$!
sleep 4
if ! kill -0 "$camera_launcher" 2>/dev/null || [[ $(pgrep -cx gscam_node || true) != 2 ]]; then
    tail -30 /tmp/robomaster-stereo-calibrated.log >&2
    exit 1
fi
echo "Started calibrated cameras on the Jetson. Log: /tmp/robomaster-stereo-calibrated.log"
REMOTE
