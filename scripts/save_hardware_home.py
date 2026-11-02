"""Save the aligned, stationary laser-centre pose as the map-specific home."""
import argparse
from collections import deque
from datetime import datetime, timezone
import math
from pathlib import Path
import tempfile
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from s1_can.home_pose import map_signature


def main():
    repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path,
                        default=repo / 'maps/hardware/refined-area-20261010/refined-area.yaml')
    parser.add_argument('--output', type=Path, default=repo / 'maps/hardware/home-pose.yaml')
    args = parser.parse_args()
    signature = map_signature(args.map)
    rclpy.init()
    node = Node('save_hardware_home')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    poses = deque(maxlen=20)
    scan_at = [None]

    def receive_odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        age = node.get_clock().now().nanoseconds * 1e-9 - (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if not all(math.isfinite(v) for v in (p.x, p.y, q.z, q.w)) or not -.2 <= age <= .7:
            return
        poses.append((time.monotonic(), p.x, p.y, 2 * math.atan2(q.z, q.w)))

    def receive_scan(msg):
        age = node.get_clock().now().nanoseconds * 1e-9 - (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if msg.header.frame_id == 'laser' and -.2 <= age <= .7:
            scan_at[0] = time.monotonic()

    node.create_subscription(Odometry, '/localization/odom', receive_odom, qos_profile_sensor_data)
    node.create_subscription(LaserScan, '/scan', receive_scan, qos_profile_sensor_data)
    result = None
    deadline = time.monotonic() + 15.
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            now = time.monotonic()
            if (scan_at[0] is None or now-scan_at[0] > .7 or len(poses) < 8
                    or now-poses[-1][0] > .7 or poses[-1][0]-poses[0][0] < 1.):
                continue
            a = poses[0]
            if any(math.hypot(p[1]-a[1], p[2]-a[2]) > .02 or abs(math.atan2(
                    math.sin(p[3]-a[3]), math.cos(p[3]-a[3]))) > .03 for p in poses):
                continue
            try:
                tf = buffer.lookup_transform('localization_map', 'laser', rclpy.time.Time())
            except TransformException:
                continue
            age = node.get_clock().now().nanoseconds * 1e-9 - (
                tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
            if not -.6 <= age <= 1.:
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
                continue
            result = {'frame_id': 'localization_map', 'reference': 'laser centre',
                      'map_yaml': str(args.map.resolve()), 'map_signature': signature,
                      'saved_at': datetime.now(timezone.utc).isoformat(),
                      'pose': {'x': float(p.x), 'y': float(p.y), 'yaw': yaw}}
            break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if result is None:
        raise SystemExit('No fresh stationary localized pose. Align in RViz, release L1, '
                         'and ensure LiDAR is awake before saving.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        backup = args.output.with_name(args.output.name + '.' +
                                      datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f') + '.bak')
        backup.write_bytes(args.output.read_bytes())
    with tempfile.NamedTemporaryFile(mode='w', dir=args.output.parent, delete=False) as out:
        yaml.safe_dump(result, out, sort_keys=False)
        temp = Path(out.name)
    temp.replace(args.output)
    print(f'Home saved to {args.output}: {result["pose"]}', flush=True)


if __name__ == '__main__':
    main()
