#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${CONDA_PREFIX:-}" ]]; then echo 'Run conda deactivate first.' >&2; exit 1; fi
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-$JETSON_HOST}"
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE
export PYTHONPATH="$repo:/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
bash "$repo/scripts/wake_hardware_lidar.sh"
exec "$repo/.venv/bin/python" "$repo/scripts/adjust_hardware_heading.py" "$@"
