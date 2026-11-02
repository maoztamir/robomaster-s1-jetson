#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
if [[ ! -x build-dds-check/probe_attitude ]]; then
    echo "Missing build-dds-check/probe_attitude; build the sensor probe first." >&2
    exit 1
fi
echo "Close the attitude stream and other manual chassis tests before continuing."
sudo systemctl stop robomaster-chassis.service
mkdir -p "$HOME/can-debug"
log_file="$HOME/can-debug/imu-$(date +%Y%m%d-%H%M%S).log"
echo "Keep the chassis stationary. Saving to $log_file. Ctrl-C stops."
./build-dds-check/probe_attitude can1 --imu-stream | tee "$log_file"
