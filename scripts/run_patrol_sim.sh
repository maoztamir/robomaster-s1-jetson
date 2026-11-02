#!/usr/bin/env bash
# Map-free patrol, SIMULATION ONLY. Start Isaac using run_isaac_navigation.sh sim first.
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -d /home/ubuntu/isaacsim ]]; then
    exec ssh -t "$ISAAC_SSH_TARGET" bash /home/ubuntu/robomaster-s1-jetson/scripts/run_patrol_sim.sh
fi
if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
    echo 'Use a fresh terminal without Humble sourced.' >&2; exit 1
fi
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
exec /usr/bin/python3 "$repo_dir/s1_can/patrol_node.py" --ros-args \
    -p use_sim_time:=true -p enabled:=true -p duration:=0.0
