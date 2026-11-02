#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/deployment_env.sh"
if [[ ! -d /home/ubuntu/isaacsim ]]; then
    exec ssh "$ISAAC_SSH_TARGET" bash /home/ubuntu/robomaster-s1-jetson/scripts/stop_patrol_sim.sh
fi
# Match only this example's standalone process, leaving the scene open.
pkill -INT -f '^/usr/bin/python3 /home/ubuntu/robomaster-s1-jetson/s1_can/patrol_node.py' || [[ $? == 1 ]]
