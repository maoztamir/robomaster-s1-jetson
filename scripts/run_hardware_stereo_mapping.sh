#!/usr/bin/env bash
# Run on the Dell laptop. Stereo mapping with optional LiDAR occupancy; no motor commands.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
mapping_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
    echo "Usage: bash scripts/run_hardware_stereo_mapping.sh [--lidar] [database.db]"
    echo "--lidar: use /scan for walls/free space, stereo for odometry and loop recognition."
    echo "Close other stereo previews first. Each default run creates a new database."
    exit 0
fi
mapping_lidar=false
mapping_prefix=stereo
if [[ "${1:-}" == --lidar ]]; then
    mapping_lidar=true
    mapping_prefix=lidar-stereo
    shift
fi
if [[ $# -gt 1 || "${1:-}" == --* ]]; then exit 2; fi
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "Run conda deactivate first." >&2; exit 1
fi
source /opt/ros/jazzy/setup.bash
if ! ros2 pkg prefix rtabmap_launch >/dev/null 2>&1; then
    echo "Install on this laptop: sudo apt install ros-jazzy-rtabmap-ros" >&2
    exit 1
fi
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-$JETSON_HOST}"
unset ROS_LOCALHOST_ONLY
mkdir -p "$mapping_repo/maps/hardware"
mapping_db="${1:-$mapping_repo/maps/hardware/$mapping_prefix-$(date +%Y%m%d-%H%M%S)-$$.db}"
mapping_db="$(realpath -m "$mapping_db")"
mkdir -p "$(dirname "$mapping_db")"
exec 9>"$mapping_repo/maps/hardware/.mapping.lock"
flock -n 9 || { echo "Hardware mapping is already running." >&2; exit 1; }
bash "$mapping_repo/scripts/start_jetson_stereo.sh"
pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    wait || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
# Reuse calibrated live rectification; RTAB-Map performs its own stereo matching.
bash "$mapping_repo/scripts/run_stereo_rectified_preview.sh" --headless &
pids+=("$!")
# Small standard-message relay avoids RTAB-Map message-version dependencies on Jetson.
PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}" \
    "$mapping_repo/.venv/bin/python" "$mapping_repo/scripts/mapping_tracking_status.py" &
pids+=("$!")
# Coordinate convention only: origin at the left lens, NOT a measured robot base.
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 \
    --roll -1.5707963267948966 --pitch 0 --yaw -1.5707963267948966 \
    --frame-id stereo_camera_link --child-frame-id left_camera_optical_frame &
pids+=("$!")
mapping_args=()
mapping_tf_args=()
if [[ "$mapping_lidar" == true ]]; then
    # Measured relative mount from isaac_robot_measurements.yaml:
    # scan centre is 85 mm behind, 30 mm right and 80 mm above the LEFT lens.
    # This difference does not depend on the provisional chassis origin.
    # Camera and LiDAR are level; scan zero was verified forward. Scan handedness
    # remains to be checked physically. No base_link origin is invented here.
    ros2 run tf2_ros static_transform_publisher --x -0.085 --y -0.030 --z 0.080 \
        --roll 0 --pitch 0 --yaw 0 \
        --frame-id stereo_camera_link --child-frame-id laser &
    pids+=("$!")
    mapping_tf_args+=(--lidar)
    # Direct stereo callbacks do not attach scans in this RTAB-Map build.
    # Stereo RGBD packets use the callback that synchronizes images with /scan.
    mapping_args+=("rgbd_sync:=true" "subscribe_rgbd:=true"
        "scan_topic:=/scan" "qos_scan:=2"
        "rtabmap_args:=--Grid/Sensor 0 --Grid/CellSize 0.05 --Grid/RangeMin 0.15 --Grid/RangeMax 8.0 --Grid/RayTracing true --Mem/NotLinkedNodesKept false")
    echo "LiDAR occupancy with stereo odometry; mapping frame is the left lens."
fi
PYTHONPATH="/usr/lib/python3/dist-packages:${PYTHONPATH:-}" \
    "$mapping_repo/.venv/bin/python" "$mapping_repo/scripts/mapping_transform_gate.py" "${mapping_tf_args[@]}"
echo "Mapping database: $mapping_db"
ros2 launch rtabmap_launch rtabmap.launch.py \
    stereo:=true depth:=false visual_odometry:=true \
    frame_id:=stereo_camera_link map_frame_id:=stereo_map \
    vo_frame_id:=stereo_odom odom_topic:=/stereo_mapping/odom \
    namespace:=stereo_mapping use_sim_time:=false \
    wait_for_transform:=1.0 \
    left_image_topic:=/stereo/left/camera/image_rect \
    right_image_topic:=/stereo/right/camera/image_rect \
    left_camera_info_topic:=/stereo/left/camera/camera_info \
    right_camera_info_topic:=/stereo/right/camera/camera_info \
    approx_sync:=true approx_sync_max_interval:=0.03 \
    odom_always_process_most_recent_frame:=false \
    qos:=2 qos_image:=1 qos_camera_info:=2 \
    topic_queue_size:=10 sync_queue_size:=10 \
    subscribe_scan:="$mapping_lidar" subscribe_scan_cloud:=false \
    "${mapping_args[@]}" \
    rtabmap_viz:=true rviz:=false database_path:="$mapping_db" &
pids+=("$!")
# Exit and clean up if any component stops; don't leave orphan rectifiers/TFs.
wait -n "${pids[@]}"
