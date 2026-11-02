#!/usr/bin/env bash
# Jetson-only, same-user process pause with a heartbeat lease on stdin.
# No sudo: systemd's ubuntu-owned ROS processes are resumed/restarted on exit.
set -eo pipefail
if [[ "$USER" != ubuntu ]]; then echo 'Run as ubuntu on the Jetson.' >&2; exit 1; fi
systemctl is-active --quiet robomaster-teleop || { echo 'Teleop is not running.' >&2; exit 1; }
teleop_main="$(systemctl show robomaster-teleop -p MainPID --value)"
mapfile -t teleop_children < <(pgrep -P "$teleop_main")
(( ${#teleop_children[@]} )) || { echo 'No teleop child found.' >&2; exit 1; }
teleop_pids=("$teleop_main" "${teleop_children[@]}")
for pid in "${teleop_pids[@]}"; do
    [[ "$(ps -o user= -p "$pid" | tr -d ' ')" == ubuntu ]] || exit 1
done
paused=false
cleanup() {
    trap - EXIT INT TERM HUP
    if [[ "$paused" == true ]]; then
        # Queue shutdown before resuming: discard input queued during the pause.
        for pid in "${teleop_children[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
        for pid in "${teleop_pids[@]}"; do kill -CONT "$pid" 2>/dev/null || true; done
        sleep 1
        for pid in "${teleop_children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
        # Restart=always restores fresh gamepad teleop through systemd.
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
paused=true
for pid in "${teleop_pids[@]}"; do kill -STOP "$pid"; done
echo PAUSED
while read -r -t 2 heartbeat; do
    [[ "$heartbeat" == keep ]] || break
done
