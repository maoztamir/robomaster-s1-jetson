#!/usr/bin/env bash
# Build the reference C++ implementation and capture a zero-only CAN test.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
command -v candump >/dev/null
command -v g++ >/dev/null
mkdir -p build-keepalive
g++ -std=c++17 -O2 -Wall -Wextra -pthread -I include \
    examples/keepalive_zero.cpp src/chassis.cpp src/protocol.cpp \
    src/crc.cpp src/can_streambuf.cpp -o build-keepalive/keepalive_zero

echo "Keep wheels lifted. Close any other manual wheel-control programs."
sudo systemctl stop robomaster-chassis.service
log_dir="$HOME/can-debug/cpp-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$log_dir"
capture_pid=""
cleanup() {
    if [[ -n "$capture_pid" ]]; then
        kill "$capture_pid" 2>/dev/null || true
        wait "$capture_pid" 2>/dev/null || true
    fi
    ip -details -statistics link show can1 > "$log_dir/can-status.txt" 2>&1 || true
    tc -s qdisc show dev can1 >> "$log_dir/can-status.txt" 2>&1 || true
    echo "Recording saved in: $log_dir"
}
trap cleanup EXIT
candump -L can1 > "$log_dir/keepalive.log" &
capture_pid=$!
sleep 1
kill -0 "$capture_pid"
echo "Recording automatically. Note the elapsed second if the LED changes."
date -Ins > "$log_dir/start-time.txt"
./build-keepalive/keepalive_zero can1 2>&1 | tee "$log_dir/test-output.txt"
