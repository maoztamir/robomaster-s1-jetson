#!/usr/bin/env bash
# Ten-minute, zero-motion endurance test with a complete CAN recording.
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
export KEEPALIVE_DURATION_S="${1:-600}"
if [[ $# -gt 1 || ! "$KEEPALIVE_DURATION_S" =~ ^[1-9][0-9]{0,4}$ ]] || (( KEEPALIVE_DURATION_S > 86400 )); then
    echo "Usage: bash scripts/test_long_keepalive.sh [seconds: 1..86400]" >&2
    exit 2
fi
command -v candump >/dev/null
command -v flock >/dev/null
[[ -x .venv/bin/python ]] || { echo "Project .venv/bin/python is missing." >&2; exit 1; }
mkdir -p "$HOME/can-debug"
exec 9> "$HOME/can-debug/long-keepalive.lock"
flock -n 9 || { echo "A long keepalive test is already running." >&2; exit 1; }
echo "Keep wheels lifted. Close other manual chassis drivers and sensor probes."
service_state="$(systemctl is-active robomaster-chassis.service || true)"
if [[ "$service_state" != "inactive" ]]; then
    sudo systemctl stop robomaster-chassis.service
fi
log_dir="$(mktemp -d "$HOME/can-debug/long-$(date +%Y%m%d-%H%M%S)-XXXXXX")"
capture_pid=""
cleanup() {
    local result=$?
    trap - EXIT
    if [[ -n "$capture_pid" ]]; then
        if ! kill -0 "$capture_pid" 2>/dev/null; then
            echo "CAN recorder ended early; recording may be incomplete." >&2
            result=1
        fi
        kill "$capture_pid" 2>/dev/null || true
        wait "$capture_pid" 2>/dev/null || true
    fi
    ip -details -statistics link show can1 > "$log_dir/can-after.txt" 2>&1 || true
    tc -s qdisc show dev can1 >> "$log_dir/can-after.txt" 2>&1 || true
    date -Ins > "$log_dir/end-time.txt"
    echo "$result" > "$log_dir/exit-code.txt"
    echo "Recording saved in: $log_dir"
    echo "Completion is not proof of controller enablement; review the CAN log and LED observations."
    exit "$result"
}
trap cleanup EXIT
date -Ins > "$log_dir/start-time.txt"
ip -details -statistics link show can1 > "$log_dir/can-before.txt"
candump -L can1 > "$log_dir/keepalive.log" 2> "$log_dir/capture-errors.txt" &
capture_pid=$!
sleep 1
kill -0 "$capture_pid"
echo "Duration: $KEEPALIVE_DURATION_S seconds. Recording: $log_dir"
echo "Watch the LED; note elapsed time if it turns red. Ctrl-C stops early."
bash scripts/test_can_keepalive.sh 2>&1 | tee "$log_dir/test-output.txt"
# Include the expected controller transition after the final heartbeat.
sleep 1
