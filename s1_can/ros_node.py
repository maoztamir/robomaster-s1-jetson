"""ROS 2 ``/cmd_vel`` adapter for the direct-CAN S1 chassis driver."""

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from .sensor_idle import SensorMotionGate
from .imu import ImuReceiver
from rclpy.node import Node

from .chassis import DEFAULT_HEARTBEAT_PROFILE, HeartbeatProfile, S1Chassis


class S1ChassisNode(Node):
    """Translate ROS body velocities into watchdog-protected S1 commands."""

    def __init__(self) -> None:
        super().__init__("s1_chassis_node")

        self.declare_parameter("can_interface", "can1")
        self.declare_parameter("command_rate", 20.0)
        self.declare_parameter("cmd_vel_timeout", 0.35)
        self.declare_parameter("max_wheel_command", 100)
        self.declare_parameter("linear_scale", 30.0)
        self.declare_parameter("angular_scale", 30.0)
        self.declare_parameter("enable_imu", False)
        self.declare_parameter("sensor_sleep_guard", False)
        self._sensor_gate = SensorMotionGate(
            enabled=bool(self.get_parameter("sensor_sleep_guard").value))
        self.create_subscription(Bool, '/robomaster/sensors/ready',
                                 self._on_sensor_ready, 10)
        self.declare_parameter(
            "heartbeat_profile", DEFAULT_HEARTBEAT_PROFILE.value
        )

        can_interface = str(self.get_parameter("can_interface").value)
        self._command_rate = float(self.get_parameter("command_rate").value)
        self._cmd_vel_timeout = float(self.get_parameter("cmd_vel_timeout").value)
        max_wheel_command = int(self.get_parameter("max_wheel_command").value)
        linear_scale = float(self.get_parameter("linear_scale").value)
        angular_scale = float(self.get_parameter("angular_scale").value)
        heartbeat_profile_name = str(
            self.get_parameter("heartbeat_profile").value
        )
        try:
            heartbeat_profile = HeartbeatProfile(heartbeat_profile_name)
        except ValueError as exc:
            choices = ", ".join(profile.value for profile in HeartbeatProfile)
            raise ValueError(
                f"heartbeat_profile must be one of: {choices}"
            ) from exc
        self._validate_parameters(
            can_interface,
            self._command_rate,
            self._cmd_vel_timeout,
            max_wheel_command,
            linear_scale,
            angular_scale,
        )

        self._robot = S1Chassis(
            can_interface,
            command_timeout=self._cmd_vel_timeout,
            max_wheel_command=max_wheel_command,
            linear_scale=linear_scale,
            angular_scale=angular_scale,
            heartbeat_profile=heartbeat_profile,
            enable_imu=bool(self.get_parameter("enable_imu").value),
        )
        self._imu_receiver = None
        self._imu_last = time.monotonic()
        if self.get_parameter("enable_imu").value:
            self._imu_receiver = ImuReceiver(can_interface)
            self._imu_pub = self.create_publisher(Imu, '/robomaster/imu/yaw', 10)
            self.create_timer(.02, self._publish_imu)
        self._latest_command = (0.0, 0.0, 0.0)
        self._last_command_time: float | None = None
        self._command_timed_out = True
        self._last_connection_state: tuple[bool, str | None] | None = None

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_timer(1.0 / self._command_rate, self._refresh_command)
        self.create_timer(1.0, self._report_connection_state)
        self._robot.start()
        self.get_logger().info(
            "S1 chassis started on %s with heartbeat profile %s; "
            "waiting for /cmd_vel"
            % (can_interface, heartbeat_profile.value)
        )

    def _publish_imu(self):
        now = time.monotonic()
        try:
            sample = self._imu_receiver.poll()
        except OSError as exc:
            self.get_logger().error('IMU receiver unavailable: %s' % exc,
                                    throttle_duration_sec=5.)
            return
        if sample is not None:
            device_clock, values = sample
            # Receive-time ROS stamp: controller boot counters are not Unix time.
            # Only Z has a validated sign/scale; do not fabricate other axes.
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.orientation_covariance[0] = -1.
            msg.linear_acceleration_covariance[0] = -1.
            msg.angular_velocity.z = -values[5]
            msg.angular_velocity_covariance = [1e6, 0., 0., 0., 1e6, 0., 0., 0., 0.0004]
            self._imu_pub.publish(msg)
            self._imu_last = now
        elif now - self._imu_last > 5.:
            self.get_logger().warning('No fresh IMU samples; renewing subscription')
            self._robot.refresh_imu_subscription()
            self._imu_last = now

    @staticmethod
    def _validate_parameters(
        can_interface: str,
        command_rate: float,
        cmd_vel_timeout: float,
        max_wheel_command: int,
        linear_scale: float,
        angular_scale: float,
    ) -> None:
        if not can_interface:
            raise ValueError("can_interface must not be empty")
        if not math.isfinite(command_rate) or command_rate <= 0:
            raise ValueError("command_rate must be positive")
        if not math.isfinite(cmd_vel_timeout) or cmd_vel_timeout <= 0:
            raise ValueError("cmd_vel_timeout must be positive")
        if 1.0 / command_rate >= cmd_vel_timeout:
            raise ValueError("command_rate period must be shorter than cmd_vel_timeout")
        if not 0 < max_wheel_command <= 32767:
            raise ValueError("max_wheel_command must be in [1, 32767]")
        if not math.isfinite(linear_scale) or linear_scale < 0:
            raise ValueError("linear_scale must be finite and non-negative")
        if not math.isfinite(angular_scale) or angular_scale < 0:
            raise ValueError("angular_scale must be finite and non-negative")

    def _on_cmd_vel(self, message: Twist) -> None:
        command = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        )
        if not all(math.isfinite(value) for value in command):
            self.get_logger().error("Rejected non-finite /cmd_vel command")
            self._robot.set_wheel_speeds(0, 0, 0, 0)
            self._last_command_time = None
            self._command_timed_out = True
            return

        self._latest_command = command
        self._last_command_time = time.monotonic()
        self._command_timed_out = False
        if self._sensor_gate.permits(time.monotonic()):
            self._robot.drive(*command)
        else:
            self._robot.set_wheel_speeds(0, 0, 0, 0)

    def _on_sensor_ready(self, message: Bool) -> None:
        self._sensor_gate.update(message.data, time.monotonic())

    def _refresh_command(self) -> None:
        if not self._sensor_gate.permits(time.monotonic()):
            self._robot.set_wheel_speeds(0, 0, 0, 0)
            return
        last_command_time = self._last_command_time
        if (
            last_command_time is None
            or time.monotonic() - last_command_time > self._cmd_vel_timeout
        ):
            if not self._command_timed_out:
                self._robot.set_wheel_speeds(0, 0, 0, 0)
                self.get_logger().warning("/cmd_vel timed out; commanded zero speed")
                self._command_timed_out = True
            return

        self._robot.drive(*self._latest_command)

    def _report_connection_state(self) -> None:
        state = (self._robot.connected, self._robot.last_transport_error)
        if state == self._last_connection_state:
            return
        self._last_connection_state = state
        connected, error = state
        if connected:
            self.get_logger().info("RoboMaster CAN transport is connected")
        else:
            self.get_logger().warning(
                "RoboMaster CAN transport is disconnected: %s"
                % (error or "waiting for interface")
            )

    def destroy_node(self) -> bool:
        self._robot.stop()
        if self._imu_receiver is not None:
            self._imu_receiver.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: S1ChassisNode | None = None
    try:
        node = S1ChassisNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
