#!/usr/bin/env bash
# Run on the remote PC, using native Humble or the project's Humble container.
set -eo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(dirname -- "$script_dir")"
if [[ "${1:-}" == --help ]]; then
    echo "Usage: bash scripts/run_remote_rviz.sh [--docker]"
    echo "Uses native ROS 2 Humble when available; otherwise Docker with X11."
    exit 0
fi
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --docker ) ]]; then
    echo "Usage: $0 [--docker]" >&2
    exit 2
fi
if [[ -f /opt/ros/humble/setup.bash && "${1:-}" != --docker ]]; then
    if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ]]; then
        echo "Open a terminal without another ROS distribution sourced, or use --docker." >&2
        exit 1
    fi
    source /opt/ros/humble/setup.bash
    export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    exec rviz2 -d "$project_dir/config/robomaster_remote.rviz"
fi
: "${DISPLAY:?An X11/XWayland DISPLAY is required for RViz.}"
command -v docker >/dev/null
command -v xhost >/dev/null
docker image inspect osrf/ros:humble-desktop-full >/dev/null
# Preserve an existing X11 permission; revoke only the grant made here.
rviz_xhost_added=false
if ! xhost | grep -q 'SI:localuser:root'; then
    xhost +SI:localuser:root
    rviz_xhost_added=true
fi
cleanup() {
    if [[ "$rviz_xhost_added" == true ]]; then
        xhost -SI:localuser:root >/dev/null
    fi
}
trap cleanup EXIT
docker run --rm --name robomaster-rviz --network host \
    -e DISPLAY -e ROS_DOMAIN_ID=42 -e ROS_LOCALHOST_ONLY=0 \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e QT_X11_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --mount "type=bind,src=$project_dir/config,dst=/robot-config,readonly" \
    osrf/ros:humble-desktop-full \
    rviz2 -d /robot-config/robomaster_remote.rviz
