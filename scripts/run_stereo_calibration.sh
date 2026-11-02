#!/usr/bin/env bash
# Hardware stereo calibration from existing compressed feeds; domain 42.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
if [[ "${1:-}" == --docker ]]; then
    shift
    script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
    exec bash "$script_dir/run_stereo_calibration_docker.sh" "$@"
fi
if [[ "${1:-}" == --help ]]; then
    echo "Usage: $0 [--docker] INNER_CORNERS_ACROSSxDOWN SQUARE_SIZE_METRES"
    echo "Example only: $0 8x6 0.025 (use your measured board dimensions)"
    exit 0
fi
if [[ $# != 2 || ! "$1" =~ ^([2-9]|[1-9][0-9]+)x([2-9]|[1-9][0-9]+)$ || ! "$2" =~ ^0\.[0-9]*[1-9][0-9]*$ ]]; then
    echo "Provide inner-corner counts and square size in metres; see --help." >&2
    exit 2
fi
calibration_distro="${ROS_DISTRO:-}"
if [[ -z "$calibration_distro" ]]; then
    if [[ -f /opt/ros/jazzy/setup.bash ]]; then calibration_distro=jazzy
    elif [[ -f /opt/ros/humble/setup.bash ]]; then calibration_distro=humble
    fi
fi
if [[ "$calibration_distro" != jazzy && "$calibration_distro" != humble ]]; then
    echo "This launcher supports native ROS 2 Jazzy or Humble." >&2; exit 1
fi
if [[ -n "${CONDA_PREFIX:-}" ]]; then
    echo "Run conda deactivate before launching the ROS calibration tools." >&2; exit 1
fi
source "/opt/ros/$calibration_distro/setup.bash"
calibration_repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-$JETSON_HOST}"
if [[ "$calibration_distro" == jazzy ]]; then unset ROS_LOCALHOST_ONLY; fi
for package in camera_calibration image_transport compressed_image_transport; do
    ros2 pkg prefix "$package" >/dev/null || {
        echo "Install prerequisites: sudo apt install ros-$calibration_distro-camera-calibration ros-$calibration_distro-image-transport ros-$calibration_distro-compressed-image-transport" >&2; exit 1;
    }
done
: "${DISPLAY:?Run from a graphical desktop terminal.}"
pids=()
cleanup() {
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
    wait || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
if [[ -n "${CALIBRATION_DECODER:-}" ]]; then
    "$CALIBRATION_DECODER" &
    pids+=("$!")
else
for side in left right; do
    transport_args=(compressed raw --ros-args)
    if [[ "$calibration_distro" == jazzy ]]; then
        transport_args=(--ros-args -p in_transport:=compressed -p out_transport:=raw
            -p "qos_overrides./stereo/$side/camera/image_raw/compressed.subscription.reliability:=best_effort")
    fi
    ros2 run image_transport republish "${transport_args[@]}" \
        -r __node:="calibration_decode_$side" \
        -r in/compressed:="/stereo/$side/camera/image_raw/compressed" \
        -r out:="/calibration/$side/image_raw" &
    pids+=("$!")
done
fi
echo "Hold the board still at each pose; approximate pairing does not synchronize capture."
echo "Use CALIBRATE then SAVE. Do not COMMIT until the calibration has been reviewed."
ros2 run camera_calibration cameracalibrator --size "$1" --square "$2" \
    --approximate 0.05 --no-service-check \
    --ros-args -r left:=/calibration/left/image_raw -r right:=/calibration/right/image_raw \
    -r left_camera:=/stereo/left/camera -r right_camera:=/stereo/right/camera
