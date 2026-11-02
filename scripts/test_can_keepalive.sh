#!/usr/bin/env bash
# Heartbeat test: zero wheel commands only; defaults to four minutes.
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_dir"
export KEEPALIVE_DURATION_S="${KEEPALIVE_DURATION_S:-240}"
if [[ ! "$KEEPALIVE_DURATION_S" =~ ^[1-9][0-9]{0,4}$ ]] || (( KEEPALIVE_DURATION_S > 86400 )); then
    echo "KEEPALIVE_DURATION_S must be an integer between 1 and 86400." >&2
    exit 2
fi

if [[ ! -x .venv/bin/python ]]; then
    echo "Missing $repo_dir/.venv/bin/python; create the project virtual environment first." >&2
    exit 1
fi

echo "Keep all wheels lifted and stop any other manual wheel-control programs."
# An already stopped service needs no privileged operation (also supports SSH).
service_state="$(systemctl is-active robomaster-chassis.service || true)"
if [[ "$service_state" != "inactive" ]]; then
    sudo systemctl stop robomaster-chassis.service
fi

show_can_status() {
    ip -details -statistics link show can1 || true
    tc -s qdisc show dev can1 || true
}
trap show_can_status EXIT

.venv/bin/python - <<'PY'
import logging
import os
import time

from s1_can import S1Chassis

logging.basicConfig(level=logging.INFO)
duration = int(os.environ["KEEPALIVE_DURATION_S"])
robot = S1Chassis("can1", heartbeat_profile="legacy_cpp")

try:
    robot.start()
    started = time.monotonic()
    next_report = 0
    print(f"Keeping heartbeat and zero wheel commands active for {duration} seconds.", flush=True)
    print("Ctrl-C stops the test. SocketCAN status does not confirm controller enablement.", flush=True)

    while time.monotonic() - started < duration:
        robot.set_wheel_speeds(0, 0, 0, 0)
        elapsed = time.monotonic() - started
        if elapsed >= next_report:
            print(
                f"{elapsed:5.0f}s | SocketCAN connected={robot.connected}"
                f" | error={robot.last_transport_error}",
                flush=True,
            )
            next_report += 15
        time.sleep(0.05)
except KeyboardInterrupt:
    print("Test interrupted.", flush=True)
finally:
    robot.stop()
    print("Stopped; final zero command sent if CAN was available.", flush=True)
PY
