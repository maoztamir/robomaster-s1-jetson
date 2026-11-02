"""Keep sensors awake while a stereo workflow runs; the lease expires on exit."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = Node('stereo_sensor_lease')
    pub = node.create_publisher(Bool, '/robomaster/sensors/stereo_lease', 10)
    node.create_timer(1., lambda: pub.publish(Bool(data=True)))
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
