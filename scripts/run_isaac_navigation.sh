#!/usr/bin/env bash
# Run on simulator simulation host. Hardware control remains on domain 42.
set -eo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mode="${1:-}"
case "$mode" in sim|slam|wait-map|nav2|rviz|goal|blockage) ;; *)
    echo "Usage: $0 sim [--provisional] | nav2 | rviz | goal X Y | blockage true|false" >&2
    echo "For the complete stereo-assisted SLAM stack: run_navigation_demo.sh --slam --provisional" >&2; exit 2;;
esac
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
    echo "Use a fresh terminal without Humble sourced for this Jazzy simulation." >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source /home/ubuntu/IsaacSim-ros_workspaces/jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
case "$mode" in
wait-map)
    exec timeout 90s ros2 topic echo /map nav_msgs/msg/OccupancyGrid --once \
        --qos-durability transient_local --qos-reliability reliable --field info.width
    ;;
slam)
    : "${ROBOMASTER_SLAM_DB:?Start SLAM with run_navigation_demo.sh --slam --provisional}"
    exec ros2 launch "$repo_dir/config/robomaster_slam.launch.xml" \
        config_dir:="$repo_dir/config" database_path:="$ROBOMASTER_SLAM_DB"
    ;;
sim)
    if [[ "${2:-}" == --provisional ]]; then export ROBOMASTER_ALLOW_PROVISIONAL=1; fi
    exec ros2 launch isaacsim run_isaacsim.launch.py \
        install_path:=/home/ubuntu/isaacsim ros_distro:=jazzy use_internal_libs:=true \
        standalone:="$repo_dir/simulation/isaac_navigation.py"
    ;;
nav2)
    exec ros2 launch "$repo_dir/config/robomaster_nav2.launch.xml" config_dir:="$repo_dir/config"
    ;;
rviz)
    exec rviz2 -d "$repo_dir/config/robomaster_navigation.rviz" --ros-args -p use_sim_time:=true
    ;;
goal)
    [[ $# == 3 && "$2" =~ ^-?[0-9]+([.][0-9]+)?$ && "$3" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || { echo "Usage: $0 goal X Y" >&2; exit 2; }
    exec ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
        "{pose: {header: {frame_id: map}, pose: {position: {x: $2, y: $3}, orientation: {w: 1.0}}}}"
    ;;
blockage)
    [[ "${2:-}" == true || "${2:-}" == false ]] || { echo "Use blockage true or blockage false" >&2; exit 2; }
    exec ros2 service call /simulation/blockage std_srvs/srv/SetBool "{data: $2}"
    ;;
esac
