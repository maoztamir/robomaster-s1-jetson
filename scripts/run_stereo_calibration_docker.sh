#!/usr/bin/env bash
# Humble calibration on the laptop, with direct discovery of the Jetson.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
if [[ $# != 2 || ! "$1" =~ ^([2-9]|[1-9][0-9]+)x([2-9]|[1-9][0-9]+)$ || ! "$2" =~ ^0\.[0-9]*[1-9][0-9]*$ ]]; then
    echo "Usage: $0 INNER_CORNERS_ACROSSxDOWN SQUARE_SIZE_METRES" >&2; exit 2
fi
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
: "${DISPLAY:?Run from the laptop graphical desktop.}"
docker image inspect osrf/ros:humble-desktop-full >/dev/null
if existing_state="$(docker inspect robomaster-calibration --format '{{.State.Status}}' 2>/dev/null)"; then
    echo "A calibration container already exists ($existing_state)."
    if [[ "$existing_state" == running ]]; then
        echo "Use the existing window named 'display' (Alt-Tab)."
        docker inspect robomaster-calibration --format '{{range .Mounts}}{{if eq .Destination "/tmp"}}SAVE location: {{.Source}}/calibrationdata.tar.gz{{end}}{{end}}'
        echo "To restart: SAVE any samples first, then run: docker stop robomaster-calibration"
    else
        echo "Remove the stopped container with: docker rm robomaster-calibration"
    fi
    exit 0
fi
mkdir -p "$repo_dir/output/calibration"
session_dir="$(mktemp -d "$repo_dir/output/calibration/session-XXXXXXXX")"
calibration_user="$(id -un)"
xhost_added=false
if ! xhost | grep -Fq "SI:localuser:$calibration_user"; then
    xhost "+SI:localuser:$calibration_user"
    xhost_added=true
fi
cleanup() {
    if [[ "$xhost_added" == true ]]; then xhost "-SI:localuser:$calibration_user" >/dev/null; fi
}
trap cleanup EXIT
echo "SAVE writes directly to $session_dir/calibrationdata.tar.gz"
docker run --rm --init --name robomaster-calibration --network host \
    --user "$(id -u):$(id -g)" \
    -e DISPLAY -e HOME=/tmp -e ROS_DOMAIN_ID=42 -e ROS_DISTRO=humble \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp -e QT_X11_NO_MITSHM=1 \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/robot-dds.xml \
    -e ROBOMASTER_DDS_PROFILE=/robot-dds.xml \
    --mount "type=bind,src=$ROBOMASTER_DDS_PROFILE,dst=/robot-dds.xml,readonly" \
    --mount "type=bind,src=$repo_dir,dst=/workspace,readonly" \
    --mount "type=bind,src=$session_dir,dst=/tmp" \
    --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
    osrf/ros:humble-desktop-full \
    bash -c 'source /opt/ros/humble/setup.bash &&
      cmake -S /workspace/calibration/decoder -B /tmp/decoder-build &&
      cmake --build /tmp/decoder-build -j2 &&
      export CALIBRATION_DECODER=/tmp/decoder-build/calibration_decoder &&
      exec bash /workspace/scripts/run_stereo_calibration.sh "$@"' calibration "$@"
