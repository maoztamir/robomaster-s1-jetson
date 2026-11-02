"""ROS tests with service operations and CAN hardware replaced by fakes."""
import unittest
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock, patch

try:
    import rclpy
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool
    from std_srvs.srv import Trigger
    from s1_can.sensor_power_node import SensorPowerNode
    from s1_can.ros_node import S1ChassisNode
    ROS = True
except ImportError:
    ROS = False


@unittest.skipUnless(ROS, 'Source ROS and expose system packages for ROS integration checks')
class SensorPowerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_sleep_wake_scan_gate_and_stereo_lease(self):
        node = SensorPowerNode()
        node.asleep = False  # Simulate a completed initial sensor wake.
        node.executor_pool.shutdown()
        now = [0.]
        actions = []
        def submit(fn, action):
            actions.append(action)
            future = Future()
            future.set_result(None)
            return future
        node.executor_pool = Mock(submit=submit)
        node.ready = Mock()
        node.policy.last_activity = 0.
        clock = SimpleNamespace(monotonic=lambda: now[0])
        try:
            with patch('s1_can.sensor_power_node.time', clock):
                now[0] = 301.
                node.tick(); node.tick()
                self.assertEqual(actions, ['sleep'])
                self.assertTrue(node.asleep)
                self.assertFalse(node.ready.publish.call_args.args[0].data)
                cmd = Twist(); cmd.linear.x = 1.
                node.command(cmd); node.tick(); node.tick()
                self.assertEqual(actions, ['sleep', 'wake'])
                self.assertFalse(node.ready.publish.call_args.args[0].data)
                scan = LaserScan()
                scan.header.frame_id = 'laser'
                scan.range_min = .15; scan.range_max = 8.; scan.ranges = [1.] * 50
                scan.header.stamp = node.get_clock().now().to_msg()
                node.scan(scan); node.tick()
                self.assertFalse(node.ready.publish.call_args.args[0].data)
                node.scan(scan); node.tick()
                self.assertTrue(node.ready.publish.call_args.args[0].data)
                now[0] += .8; node.tick()
                self.assertFalse(node.ready.publish.call_args.args[0].data)
                node.stereo_lease(Bool(data=True)); node.tick(); node.tick()
                self.assertEqual(actions[-1], 'stereo')
                now[0] += 301.; node.tick()
                self.assertEqual(actions[-1], 'sleep')
        finally:
            node.destroy_node()

    def test_chassis_blocks_before_ready_and_after_heartbeat_expiry(self):
        robot = Mock()
        with patch('s1_can.ros_node.S1Chassis', return_value=robot):
            node = S1ChassisNode()
        node._sensor_gate.enabled = True
        now = [10.]
        try:
            with patch('s1_can.ros_node.time', SimpleNamespace(monotonic=lambda: now[0])):
                cmd = Twist(); cmd.linear.x = 1.
                node._on_cmd_vel(cmd)
                robot.drive.assert_not_called()
                robot.set_wheel_speeds.assert_called_with(0, 0, 0, 0)
                node._on_sensor_ready(Bool(data=True))
                node._on_cmd_vel(cmd)
                robot.drive.assert_called_once_with(1., 0., 0.)
                now[0] += .7
                node._on_cmd_vel(cmd); node._refresh_command()
                self.assertEqual(robot.drive.call_count, 1)
                robot.set_wheel_speeds.assert_called_with(0, 0, 0, 0)
        finally:
            node.destroy_node()

    def test_immediate_sleep_rejects_recent_commands_and_active_goals(self):
        node = SensorPowerNode()
        try:
            with patch('s1_can.sensor_power_node.time', SimpleNamespace(monotonic=lambda: 10.)):
                node.policy.last_activity = 9.
                response = node.sleep_now(Trigger.Request(), Trigger.Response())
                self.assertFalse(response.success)
                node.policy.last_activity = 0.
                node.policy.active_goals.add('test-goal')
                response = node.sleep_now(Trigger.Request(), Trigger.Response())
                self.assertFalse(response.success)
                node.policy.active_goals.clear()
                response = node.sleep_now(Trigger.Request(), Trigger.Response())
                self.assertTrue(response.success)
                self.assertTrue(node.policy.should_sleep(10.))
        finally:
            node.destroy_node()

    def test_rviz_pose_request_wakes_sleeping_lidar_without_motor_command(self):
        node = SensorPowerNode()
        node.ready = Mock()
        node.executor_pool.shutdown()
        actions = []
        def submit(fn, action):
            actions.append(action)
            future = Future(); future.set_result(None)
            return future
        node.executor_pool = Mock(submit=submit)
        node.policy.last_activity = 0.
        try:
            with patch('s1_can.sensor_power_node.time', SimpleNamespace(monotonic=lambda: 500.)):
                msg = PoseWithCovarianceStamped(); msg.header.frame_id = 'localization_map'
                node.pose_request(msg); node.tick()
                self.assertEqual(actions, ['wake'])
                self.assertFalse(node.ready.publish.call_args.args[0].data)
        finally:
            node.destroy_node()
