"""Relay RTAB-Map tracking state to a small standard ROS message for the Jetson.

No timer: publishing stops when OdomInfo stops, allowing the Jetson watchdog to
expire. Preserve the original stamp so delayed queued messages cannot rearm it.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Vector3Stamped
from rtabmap_msgs.msg import OdomInfo


def main():
    rclpy.init()
    node = Node('mapping_tracking_status')
    publisher = node.create_publisher(Vector3Stamped, '/patrol/tracking_status', 1)

    def receive(info):
        msg = Vector3Stamped()
        msg.header = info.header
        msg.vector.x = 1.0 if not info.lost and info.inliers >= 20 else 0.0
        # Diagnostics for the Jetson log without introducing a custom message:
        # y is the current inlier count and z is one when RTAB-Map reports lost.
        msg.vector.y = float(info.inliers)
        msg.vector.z = 1.0 if info.lost else 0.0
        publisher.publish(msg)

    node.create_subscription(OdomInfo, '/stereo_mapping/odom_info', receive,
                             qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
