#!/usr/bin/env bash
# Install and start the RoboMaster SocketCAN recovery monitor with systemd.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$script_dir")"

if ((EUID != 0)); then
    exec sudo -- "$0" "$@"
fi

install -D -m 0755 \
    "$project_dir/scripts/watch_can0.sh" \
    /usr/local/sbin/robomaster-can-watch
install -D -m 0644 \
    "$project_dir/systemd/robomaster-can.default" \
    /etc/default/robomaster-can
install -D -m 0644 \
    "$project_dir/systemd/robomaster-can.service" \
    /etc/systemd/system/robomaster-can.service

systemctl daemon-reload
systemctl enable robomaster-can.service
systemctl restart robomaster-can.service

echo "Installed and started robomaster-can.service."
systemctl --no-pager --full status robomaster-can.service
