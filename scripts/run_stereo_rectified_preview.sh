#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
preview_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "Run conda deactivate before starting the preview." >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-$JETSON_HOST}"
unset ROS_LOCALHOST_ONLY
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$preview_repo/.venv/bin/python" "$preview_repo/scripts/stereo_rectified_preview.py" "$@"
