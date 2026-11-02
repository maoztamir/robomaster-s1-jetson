#!/usr/bin/env bash
# Convenience entry point for the supervised patrol that requires healthy mapping.
set -e
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$script_dir/run_patrol_robot.sh" --mapping
