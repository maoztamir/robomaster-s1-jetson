"""Recover invalid LiDAR odometry after sleep, preserving an AMCL pose seed."""
import math
from pathlib import Path
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


class Recovery(Node):
    def __init__(self):
        super().__init__('localization_scan_recovery')
        self.scan_at = self.valid_at = self.bad_since = None
        self.seed = None
        self.seed_stamp = -1.
        self.future = None
        self.cooldown = 0.
        self.reset = self.create_client(Empty, '/localization/icp_odometry/reset_odom')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/initialpose', 10)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/localization/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/localization/amcl_pose', self.pose,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PoseWithCovarianceStamped, '/localization/initialpose', self.pose, 10)
        self.create_timer(.1, self.tick)

    def age(self, stamp):
        return self.get_clock().now().nanoseconds*1e-9-(stamp.sec+stamp.nanosec*1e-9)

    def scan(self, msg):
        if msg.header.frame_id=='laser' and -.2<=self.age(msg.header.stamp)<=.7:
            self.scan_at=time.monotonic()

    def odom(self, msg):
        q=msg.pose.pose.orientation;p=msg.pose.pose.position
        norm=sum(v*v for v in (q.x,q.y,q.z,q.w))
        if all(math.isfinite(v) for v in (p.x,p.y,norm)) and .8<=norm<=1.2:
            self.valid_at=time.monotonic();self.bad_since=None
        elif self.bad_since is None:self.bad_since=time.monotonic()

    def pose(self, msg):
        if msg.header.frame_id!='localization_map':return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if stamp<=self.seed_stamp:return  # Ignore our own zero-stamped replay.
        p=msg.pose.pose.position;q=msg.pose.pose.orientation
        if not all(math.isfinite(v) for v in (p.x,p.y,q.x,q.y,q.z,q.w)):return
        if not .8<=sum(v*v for v in (q.x,q.y,q.z,q.w))<=1.2:return
        self.seed=msg;self.seed_stamp=stamp

    def tick(self):
        now=time.monotonic()
        if self.future is not None:
            if not self.future.done():return
            if self.future.exception() is None:
                import copy
                seed=copy.deepcopy(self.seed)
                seed.header.stamp.sec=seed.header.stamp.nanosec=0
                self.pub.publish(seed)
                self.get_logger().info('Reset invalid LiDAR odometry and restored the last map pose seed')
            else:self.get_logger().error('LiDAR odometry reset failed')
            self.future=None;self.cooldown=now+5.;self.bad_since=None
            return
        if (now<self.cooldown or self.seed is None or self.scan_at is None
                or now-self.scan_at>.7 or self.bad_since is None or now-self.bad_since<1.
                or not self.reset.service_is_ready()):return
        self.future=self.reset.call_async(Empty.Request())


def main():
    rclpy.init();node=Recovery()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
