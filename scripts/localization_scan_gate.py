"""Require a fresh, usable /scan in the expected laser frame before startup."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = Node('localization_scan_gate')
    accepted = False
    last_reason = 'No /scan received'

    def receive(msg):
        nonlocal accepted, last_reason
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        count = sum(math.isfinite(r) and max(.15, msg.range_min) <= r <= min(8., msg.range_max)
                    for r in msg.ranges)
        if msg.header.frame_id != 'laser':
            last_reason = f'Expected laser frame, received {msg.header.frame_id!r}'
        elif not -.2 <= age <= 1.0:
            last_reason = f'Scan timestamp age is {age:.3f}s'
        elif count < 30:
            last_reason = f'Only {count} valid ranges within 8m'
        else:
            accepted = True
            print(f'LiDAR ready: age={age:.3f}s, valid_ranges={count}', flush=True)

    node.create_subscription(LaserScan, '/scan', receive, qos_profile_sensor_data)
    deadline = time.monotonic() + 10
    try:
        while not accepted and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not accepted:
            raise RuntimeError(last_reason + '; check Jetson power, Wi-Fi and LiDAR service')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
