"""Parked sensor sleep, service wake-up and fresh-scan permission heartbeat."""
import math
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener, TransformException

from .sensor_idle import SensorIdlePolicy
from .lidar_motor import LidarMotorHold


class SensorPowerNode(Node):
    def __init__(self):
        super().__init__('sensor_power_manager')
        self.declare_parameter('idle_seconds', 300.)
        self.declare_parameter('lidar_serial_port', '/dev/serial/by-id/'
                               'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.motor = LidarMotorHold(str(self.get_parameter('lidar_serial_port').value))
        self.policy = SensorIdlePolicy(time.monotonic(),
                                      float(self.get_parameter('idle_seconds').value))
        # Restart recovery: explicitly start LiDAR before granting permission,
        # including when the previous manager exited while sensors were asleep.
        self.asleep = True
        self.future = None
        self.action = None
        self.stereo_started = False
        self.manual_hold = False
        self.lease_until = 0.
        self.retry_after = 0.
        self.scan_at = None
        self.scan_count = 0
        self.previous_pose = None
        self.executor_pool = ThreadPoolExecutor(max_workers=1)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.ready = self.create_publisher(Bool, '/robomaster/sensors/ready', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/initialpose',
                                 self.pose_request, 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Bool, '/robomaster/sensors/stereo_lease',
                                 self.stereo_lease, 10)
        self.create_subscription(Odometry, '/localization/odom', self.odom,
                                 qos_profile_sensor_data)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for prefix in ('', '/hardware_navigation'):
            for action in ('navigate_to_pose', 'navigate_through_poses'):
                topic = f'{prefix}/{action}/_action/status'
                self.create_subscription(GoalStatusArray, topic,
                                         lambda m, t=topic: self.goals(m, t), qos)
        self.create_service(SetBool, '/robomaster/sensors/keep_awake', self.keep_awake)
        self.create_service(Trigger, '/robomaster/sensors/sleep_now', self.sleep_now)
        self.create_service(Trigger, '/robomaster/sensors/wake', self.wake)
        self.create_timer(.2, self.tick)
        self.get_logger().info('Sensor sleep after %.0fs idle; stereo requires keep_awake=true'
                               % self.policy.timeout)

    def command(self, msg):
        self.policy.command((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    def pose_request(self, msg):
        if msg.header.frame_id == 'localization_map':
            # A person aligning the map needs scans even while the robot is parked.
            self.policy.activity(time.monotonic())

    def odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x, p.y, q.z, q.w)):
            return
        pose = (p.x, p.y, 2 * math.atan2(q.z, q.w))
        if self.previous_pose:
            a = self.previous_pose
            if math.hypot(pose[0]-a[0], pose[1]-a[1]) > .02 or abs(
                    math.atan2(math.sin(pose[2]-a[2]), math.cos(pose[2]-a[2]))) > .03:
                self.policy.activity(time.monotonic())
                self.previous_pose = pose
        else:
            self.previous_pose = pose

    def goals(self, msg, topic):
        if any(s.status in (1, 2, 3) for s in msg.status_list):
            self.policy.active_goals.add(topic)
            self.policy.activity(time.monotonic())
        else:
            self.policy.active_goals.discard(topic)

    def keep_awake(self, request, response):
        self.manual_hold = request.data
        self.policy.activity(time.monotonic())
        response.success = True
        response.message = ('Stereo requested; sensors will start and stay awake' if request.data
                            else 'Idle timer enabled; cameras stop on the next idle timeout')
        return response

    def stereo_lease(self, msg):
        if msg.data:
            self.lease_until = time.monotonic() + 5.
            self.policy.activity(time.monotonic())

    def sleep_now(self, request, response):
        now = time.monotonic()
        if (self.future is not None or self.manual_hold or now < self.lease_until
                or self.policy.active_goals or now - self.policy.last_activity < 2.):
            response.success = False
            response.message = 'Busy: release controls, stop goals and stereo, then wait two seconds'
        else:
            self.policy.last_activity = now - self.policy.timeout
            response.success = True
            response.message = 'Parked sensor sleep requested; new commands cancel it or wake LiDAR'
        return response

    def wake(self, request, response):
        self.policy.activity(time.monotonic())
        response.success = True
        response.message = 'LiDAR wake requested; motion stays blocked until fresh scans'
        return response

    def scan(self, msg):
        now = time.monotonic()
        age = self.get_clock().now().nanoseconds * 1e-9 - (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        valid = (msg.header.frame_id == 'laser' and -.2 <= age <= .7 and
                 sum(math.isfinite(r) and max(.15, msg.range_min) <= r <=
                     min(8., msg.range_max) for r in msg.ranges) >= 30)
        if not valid:
            self.scan_at = None
            self.scan_count = 0
            return
        if self.scan_at is None or now - self.scan_at > .7:
            self.scan_count = 0
        self.scan_count += 1
        self.scan_at = now

    def switch(self, action):
        if action != 'sleep':
            self.motor.release()
        units = {'sleep': [('stop', 'robomaster-cameras'), ('stop', 'robomaster-lidar')],
                 'wake': [('start', 'robomaster-lidar')],
                 'stereo': [('start', 'robomaster-lidar'), ('start', 'robomaster-cameras')]}
        for verb, unit in units[action]:
            subprocess.run(['sudo', '-n', '/usr/bin/systemctl', verb, unit + '.service'],
                           check=True, timeout=35, capture_output=True, text=True)
        if action == 'sleep':
            self.motor.stop()

    def tick(self):
        now = time.monotonic()
        self.policy.hold = self.manual_hold or now < self.lease_until
        if self.future is not None and self.future.done():
            try:
                self.future.result()
                self.asleep = self.action == 'sleep'
                self.stereo_started = self.action == 'stereo'
                self.get_logger().info('Sensor operation completed: ' + self.action)
            except Exception as exc:
                self.get_logger().error('Sensor service operation failed: ' + str(exc))
                self.retry_after = now + 10
                self.stereo_started = False
            self.future = None
        if self.future is None and now >= self.retry_after:
            action = None
            if self.policy.hold and not self.stereo_started:
                action = 'stereo'
            elif self.asleep and not self.policy.should_sleep(now):
                action = 'wake'
            elif not self.asleep and self.policy.should_sleep(now):
                action = 'sleep'
            if action:
                self.action = action
                self.scan_at = None
                self.scan_count = 0
                self.ready.publish(Bool(data=False))
                self.future = self.executor_pool.submit(self.switch, action)
        permitted = (not self.asleep and self.future is None and self.scan_at is not None
                     and now - self.scan_at <= .7 and self.scan_count >= 2)
        if permitted and self.policy.active_goals:
            try:
                tf = self.tf.lookup_transform('localization_map', 'laser', rclpy.time.Time())
                age = self.get_clock().now().nanoseconds * 1e-9 - (
                    tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9)
                permitted = -.6 <= age <= 1.
            except TransformException:
                permitted = False
        self.ready.publish(Bool(data=permitted))

    def destroy_node(self):
        try:
            if rclpy.ok():
                self.ready.publish(Bool(data=False))
        finally:
            self.executor_pool.shutdown(wait=True)
            self.motor.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorPowerNode()
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
