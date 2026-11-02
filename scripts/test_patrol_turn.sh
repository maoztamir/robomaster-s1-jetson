#!/usr/bin/env bash
# Positive yaw should turn left (counterclockwise viewed from above).
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/test_patrol_forward.sh" 0.5 turn
