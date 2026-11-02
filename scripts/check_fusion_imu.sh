#!/usr/bin/env bash
# Stationary 15-second IMU capture; never sends wheel commands.
set -eo pipefail
mode="${1:---stationary}"
[[ "$mode" == --stationary || "$mode" == --yaw ]] || { echo 'Use --stationary or --yaw' >&2; exit 2; }
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
probe="$repo_dir/build-dds-check/probe_attitude"
[[ -x "$probe" ]] || { echo 'IMU probe binary is missing' >&2; exit 1; }
mkdir -p "$HOME/.cache" "$HOME/can-debug"
exec 9>"$HOME/.cache/robomaster-patrol.lock"
flock -n 9 || { echo 'Stop patrol before the stationary IMU check' >&2; exit 1; }
if pgrep -x probe_attitude >/dev/null; then
    echo 'An existing sensor probe is running; stop it first.' >&2; exit 1
fi
sudo -v
restore_chassis=false
restore_teleop=false
systemctl is-active --quiet robomaster-chassis && restore_chassis=true
systemctl is-active --quiet robomaster-teleop && restore_teleop=true
cleanup() {
    if [[ "$restore_chassis" == true ]]; then sudo -n systemctl start robomaster-chassis; fi
    if [[ "$restore_teleop" == true ]]; then sudo -n systemctl start robomaster-teleop; fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP
sudo -n systemctl stop robomaster-teleop
sleep .5
sudo -n systemctl stop robomaster-chassis
log_file="$HOME/can-debug/fusion-imu-${mode#--}-$(date +%Y%m%d-%H%M%S).log"
if [[ "$mode" == --yaw ]]; then
    echo 'Driving services are paused. Keep L1 released.'
    echo 'During the 15-second capture: hold still for 3 seconds, slowly rotate'
    echo 'the whole robot LEFT 90 degrees by hand over about 5 seconds, then leave'
    echo 'it stationary and level on the floor until capture finishes.'
else
    echo 'Keep robot stationary and gamepad L1 released. Capturing 15 seconds.'
fi
echo "Log: $log_file"
"$probe" can1 --imu | tee "$log_file"
echo 'Capture finished; restoring previously active driving services.'
