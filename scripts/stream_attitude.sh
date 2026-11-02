#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
if [[ ! -x build-dds-check/probe_attitude ]]; then
    echo "Missing build-dds-check/probe_attitude; build the orientation probe first." >&2
    exit 1
fi
echo "Close other manual chassis tests and DDS readers before continuing."
sudo systemctl stop robomaster-chassis.service
exec ./build-dds-check/probe_attitude can1 --stream
