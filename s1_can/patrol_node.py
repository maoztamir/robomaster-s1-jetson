"""Opt-in ROS patrol with scan watchdog and publisher conflict stop."""
import time

import rclpy
from geometry_msgs.msg import Twist, Vector3Stamped
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

if __package__:
    from s1_can.patrol import Patrol
    from s1_can.tracking_guard import TrackingGuard
else:
    from patrol import Patrol
    from tracking_guard import TrackingGuard


class PatrolNode(Node):
    def __init__(self):
        super().__init__('robomaster_patrol')
        defaults = dict(enabled=False, duration=60.0, forward=0.15, turn=0.35,
                        clearance=0.75, radius=0.4, scan_timeout=0.4, lidar_yaw=0.0,
                        require_tracking=False)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.settings = {name: self.get_parameter(name).value for name in defaults}
        p = self.settings
        if not (0 <= p['duration'] <= 600 and 0 < p['scan_timeout'] <= .5):
            raise ValueError('duration must be 0..600 seconds (0 means continuous); scan_timeout 0..0.5 seconds')
        self.policy = Patrol(p['forward'], p['turn'], p['clearance'], p['radius'])
        self.scan = None
        self.received = 0.
        self.started = time.monotonic()
        self.state = None
        self.finished = False
        self.tracking = TrackingGuard()
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        if p['require_tracking']:
            # The relay is a small reliable standard message crossing from the
            # Jazzy mapping PC to the Humble Jetson. Request retransmission;
            # LaserScan remains best effort sensor data below.
            tracking_qos = QoSProfile(depth=10,
                                      reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Vector3Stamped, '/patrol/tracking_status',
                                     self.receive_tracking, tracking_qos)
        self.create_subscription(LaserScan, '/scan', self.receive, qos_profile_sensor_data)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def receive_tracking(self, message):
        age = (self.get_clock().now().nanoseconds -
               (message.header.stamp.sec * 10**9 + message.header.stamp.nanosec)) / 1e9
        self.tracking.update(message.vector.x == 1.0, age, time.monotonic(),
                             explicitly_lost=message.vector.z == 1.0)

    def receive(self, scan):
        # Reject scans already stale in the ROS clock as well as receipt silence.
        age = (self.get_clock().now().nanoseconds - (scan.header.stamp.sec * 10**9 + scan.header.stamp.nanosec)) / 1e9
        if age < -.1 or age > self.settings['scan_timeout']:
            self.scan = None
            return
        self.scan, self.received = scan, time.monotonic()

    def tick(self):
        now = time.monotonic()
        x, z, state = 0., 0., 'disabled'
        if self.settings['duration'] > 0 and now - self.started >= self.settings['duration']:
            self.finished = True
            state = 'duration complete'
        elif self.count_publishers('/cmd_vel') > 1:
            state = 'another cmd_vel publisher exists; stop other controllers'
        elif now - self.started < 3.0:
            state = 'waiting for ROS discovery'
        elif self.settings['enabled']:
            if self.settings['require_tracking'] and not self.tracking.permits(now):
                state = (f'tracking lost; stopped until session restart '
                         f'({self.tracking.reason})' if self.tracking.stopped
                         else f'waiting for healthy mapping tracking '
                              f'({self.tracking.reason})')
            elif self.scan is None or now - self.received > self.settings['scan_timeout']:
                state = 'waiting for fresh scan'
            else:
                s = self.scan
                x, z, state = self.policy.command(s.ranges, s.angle_min, s.angle_increment,
                    s.range_min, s.range_max, now, self.settings['lidar_yaw'])
        msg = Twist()
        msg.linear.x, msg.angular.z = x, z
        self.publisher.publish(msg)
        if state != self.state:
            self.get_logger().info(state)
            self.state = state


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.publisher.publish(Twist())
                time.sleep(.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
