"""Adjust the current localization heading while keeping its requested X/Y."""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--degrees', required=True, type=float,
                        help='Positive rotates counterclockwise; negative clockwise')
    args = parser.parse_args()
    if not math.isfinite(args.degrees) or not 0 < abs(args.degrees) <= 20:
        parser.error('Use a nonzero adjustment of at most 20 degrees')
    rclpy.init()
    node = Node('adjust_hardware_heading')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/localization/initialpose', 10)
    deadline = time.monotonic() + 10.
    sent = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if not pub.get_subscription_count():
                continue
            try:
                tf = buffer.lookup_transform('localization_map', 'laser', rclpy.time.Time())
            except TransformException:
                continue
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
            age = node.get_clock().now().nanoseconds * 1e-9 - stamp
            if not -.6 <= age <= 1.:
                continue
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            if not all(math.isfinite(v) for v in (p.x, p.y, yaw)):
                continue
            yaw += math.radians(args.degrees)
            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'localization_map'
            # Stamp at the current transform to avoid extrapolation past odometry.
            msg.header.stamp = tf.header.stamp
            msg.pose.pose.position.x = p.x
            msg.pose.pose.position.y = p.y
            msg.pose.pose.orientation.z = math.sin(yaw/2)
            msg.pose.pose.orientation.w = math.cos(yaw/2)
            msg.pose.covariance[0] = .01
            msg.pose.covariance[7] = .01
            msg.pose.covariance[35] = math.radians(3.)**2
            pub.publish(msg)
            # Allow delivery before destroying the publisher.
            until = time.monotonic() + .5
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
            print(f'Heading adjusted {args.degrees:+g} degrees; requested X/Y '
                  f'kept at ({p.x:.3f}, {p.y:.3f})', flush=True)
            sent = True
            break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not sent:
        raise SystemExit('No fresh localized pose. Wake LiDAR and set an initial pose first.')


if __name__ == '__main__':
    main()
