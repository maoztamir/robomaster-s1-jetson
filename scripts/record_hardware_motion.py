"""Record manual commands and LiDAR odometry; never publish motor commands."""
import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--trial', choices=('forward', 'turn'), default='forward')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 120:
        parser.error('--seconds must be between 1 and 120')
    if args.output.exists():
        parser.error('Output already exists; choose a new filename')
    rclpy.init()
    node = Node('hardware_motion_recorder')
    commands, odometry = [], []
    start = time.monotonic()

    def command(msg):
        commands.append(dict(t=time.monotonic() - start,
                             x=msg.linear.x, y=msg.linear.y, yaw=msg.angular.z))

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
            return  # RTAB-Map publishes invalid poses when tracking is lost.
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        odometry.append(dict(t=time.monotonic() - start,
                             stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             frame=msg.header.frame_id, child=msg.child_frame_id,
                             x=p.x, y=p.y, yaw=yaw))

    node.create_subscription(Twist, '/cmd_vel', command, 10)
    node.create_subscription(Odometry, '/localization/odom', odom, qos_profile_sensor_data)
    movement = 'turn' if args.trial == 'turn' else 'straight movement'
    print(f'Recording {args.seconds:g}s. Use the gamepad for one measured {movement}; '
          'release L1 and stop before recording ends.', flush=True)
    try:
        while time.monotonic() - start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as out:
        json.dump(dict(duration=time.monotonic() - start,
                       trial=args.trial,
                       command_units='abstract, not SI',
                       odometry_reference='laser centre; rotation can translate this point',
                       commands=commands, odometry=odometry), out, indent=2)
    print(f'Saved {len(commands)} command and {len(odometry)} valid odometry samples '
          f'to {args.output}', flush=True)
    if not commands or len(odometry) < 2:
        raise SystemExit('Missing data: this recording cannot calibrate motion')


if __name__ == '__main__':
    main()
