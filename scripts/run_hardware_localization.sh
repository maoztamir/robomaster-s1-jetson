#!/usr/bin/env bash
# Stationary localization bring-up on Dell. No motor commands are published.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
rviz=true
home_pose="$repo/maps/hardware/home-pose.yaml"
use_home=true
if [[ "${1:-}" == --help ]]; then
    echo 'Usage: bash scripts/run_hardware_localization.sh [--headless] [--manual] [map.yaml]'
    echo 'Loads measured occupancy, LiDAR odometry and AMCL. No navigation controller.'
    exit 0
fi
while [[ "${1:-}" == --* ]]; do
    case "$1" in
        --headless) rviz=false;;
        --manual) use_home=false;;
        *) echo "Unknown option: $1" >&2; exit 2;;
    esac
    shift
done
if [[ $# -gt 1 || "${1:-}" == --* ]]; then exit 2; fi
if [[ -n "${CONDA_PREFIX:-}" ]]; then echo 'Run conda deactivate first.' >&2; exit 1; fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
    echo 'Use a fresh terminal with ROS 2 Jazzy.' >&2; exit 1
fi
map_path="$(realpath -e "${1:-$repo/maps/hardware/refined-area-20261010/refined-area.yaml}")"
source /opt/ros/jazzy/setup.bash
for package in rtabmap_odom nav2_map_server nav2_amcl nav2_lifecycle_manager; do
    ros2 pkg prefix "$package" >/dev/null || { echo "Missing ROS package: $package" >&2; exit 1; }
done
if [[ "$rviz" == true ]]; then
    : "${DISPLAY:?A desktop DISPLAY is required; use --headless otherwise.}"
    ros2 pkg prefix rviz2 >/dev/null
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$ROBOMASTER_DDS_PROFILE}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-$JETSON_HOST}"
unset ROS_LOCALHOST_ONLY FASTDDS_DEFAULT_PROFILES_FILE
mkdir -p "$repo/maps/hardware"
exec 9>"$repo/maps/hardware/.mapping.lock"
flock -n 9 || { echo 'Stop mapping/fusion/localization before starting this session.' >&2; exit 1; }
export PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}"
if [[ "$ROS_DOMAIN_ID" == 42 ]]; then
    bash "$repo/scripts/wake_hardware_lidar.sh"
fi
"$repo/.venv/bin/python" "$repo/scripts/localization_scan_gate.py"
echo "Localization map: $map_path"
echo 'Position refers to the laser centre. In RViz, use 2D Pose Estimate at its approximate location.'
launch_pid=''
cleanup() {
    trap - EXIT INT TERM HUP
    if [[ -n "$launch_pid" ]]; then
        kill -INT -- "-$launch_pid" 2>/dev/null || true
        wait "$launch_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP
setsid ros2 launch "$repo/config/hardware_localization.launch.py" \
    map_path:="$map_path" rviz:="$rviz" home_pose:="$home_pose" use_home:="$use_home" &
launch_pid=$!
wait "$launch_pid"
