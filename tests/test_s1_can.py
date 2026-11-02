from __future__ import annotations

import struct
import time
import unittest
from unittest.mock import patch

from s1_can.chassis import HeartbeatProfile, S1Chassis, mix_ros_velocity
from s1_can.protocol import PacketEncoder, SequenceTracker, crc8, crc16
from s1_can.transport import CanTransport, CanTransportError
from s1_can.test_drive import ZERO_MOTION, drive_for, run_motion_test


class FakeBus:
    def __init__(self) -> None:
        self.messages = []

    def send(self, message) -> None:
        self.messages.append(message)


class FailingBus:
    def send(self, message) -> None:
        raise OSError("adapter removed")


class FakeTransport:
    def __init__(self) -> None:
        self.packets: list[bytes] = []
        self.closed = False

    def send(self, packet: bytes) -> None:
        self.packets.append(packet)

    def close(self) -> None:
        self.closed = True

    def reconnect(self) -> bool:
        return not self.closed


class RecoveringTransport(FakeTransport):
    def __init__(self) -> None:
        super().__init__()
        self.reconnect_calls = 0
        self.fail_next_send = False
        self.available = True

    def send(self, packet: bytes) -> None:
        if self.fail_next_send:
            self.fail_next_send = False
            self.available = False
            raise CanTransportError("simulated USB disconnect")
        if not self.available:
            raise CanTransportError("simulated interface unavailable")
        super().send(packet)

    def reconnect(self) -> bool:
        self.reconnect_calls += 1
        self.available = True
        return True


class FakeRobot:
    def __init__(self) -> None:
        self.commands: list[tuple[float, float, float]] = []
        self.started = False
        self.stopped = False

    def start(self) -> None:
        self.started = True

    def drive(self, vx: float, vy: float, wz: float) -> None:
        self.commands.append((vx, vy, wz))

    def stop(self) -> None:
        self.stopped = True


class ProtocolTests(unittest.TestCase):
    def test_crc_known_vectors(self) -> None:
        self.assertEqual(crc8(bytes.fromhex("550e04")), 0x66)
        self.assertEqual(crc16(bytes.fromhex("550e046609c30000003f1901")), 0x8B9B)

    def test_workmode_packet_matches_cpp(self) -> None:
        packet = PacketEncoder().encode(0x09, 0xC3, 0x3F, 0x19, b"\x01")
        self.assertEqual(packet.hex(), "550e046609c30000003f19019b8b")

    def test_sequence_tracker_mirrors_cpp_first_duplicate(self) -> None:
        tracker = SequenceTracker()
        self.assertEqual([tracker.next(0x3F19) for _ in range(4)], [0, 0, 1, 2])
        self.assertEqual(tracker.next(0x3F20), 0)

    def test_transport_fragments_into_standard_201_frames(self) -> None:
        bus = FakeBus()
        transport = CanTransport(bus=bus)
        transport.send(bytes(range(21)))
        self.assertEqual([len(message.data) for message in bus.messages], [8, 8, 5])
        self.assertTrue(all(message.arbitration_id == 0x201 for message in bus.messages))
        self.assertTrue(all(not message.is_extended_id for message in bus.messages))

    def test_transport_wraps_send_failure_and_marks_disconnected(self) -> None:
        transport = CanTransport(bus=FailingBus())
        with self.assertRaisesRegex(CanTransportError, "adapter removed"):
            transport.send(b"\x55")
        self.assertFalse(transport.connected)


class ChassisTests(unittest.TestCase):
    def test_disconnect_reconnects_and_reinitializes_with_zero(self) -> None:
        transport = RecoveringTransport()
        robot = S1Chassis(
            transport=transport,
            heartbeat_rate=200,
            command_timeout=0.1,
            reconnect_interval=0.01,
        )
        robot.start()
        robot.set_wheel_speeds(30, 30, 30, 30)
        transport.packets.clear()
        transport.fail_next_send = True

        deadline = time.monotonic() + 0.25
        while transport.reconnect_calls < 2 and time.monotonic() < deadline:
            time.sleep(0.005)

        self.assertGreaterEqual(transport.reconnect_calls, 2)
        self.assertTrue(robot.connected)
        self.assertEqual(
            [packet[9:11] for packet in transport.packets[:3]],
            [b"\x3f\x19", b"\x3f\x60", b"\x3f\x20"],
        )
        self.assertEqual(
            struct.unpack("<hhhh", transport.packets[2][11:-2]),
            (0, 0, 0, 0),
        )
        robot.stop()

    def test_default_heartbeat_matches_legacy_cpp_packet(self) -> None:
        transport = FakeTransport()
        robot = S1Chassis(transport=transport)
        with robot._lock:
            robot._send_heartbeat_locked()

        packet = transport.packets[-1]
        self.assertEqual(robot.heartbeat_profile, HeartbeatProfile.LEGACY_CPP)
        self.assertEqual(packet[4:6], b"\x09\x03")
        self.assertEqual(packet[9:11], b"\x3f\x60")
        self.assertEqual(
            packet[11:-2],
            bytes.fromhex("00 04 20 00 01 00 40 00 02 10 04 03 00 04"),
        )

    def test_direct_c3_heartbeat_uses_coherent_direct_profile(self) -> None:
        transport = FakeTransport()
        robot = S1Chassis(
            transport=transport,
            heartbeat_profile=HeartbeatProfile.DIRECT_C3,
        )
        with robot._lock:
            robot._send_heartbeat_locked()

        packet = transport.packets[-1]
        self.assertEqual(packet[4:6], b"\x09\xc3")
        self.assertEqual(packet[9:11], b"\x3f\x60")
        self.assertEqual(
            packet[11:-2],
            bytes.fromhex("00 04 20 00 01 00 40 00 02 10 00 03 00 00"),
        )

    def test_unknown_heartbeat_profile_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "heartbeat_profile must be one of"):
            S1Chassis(transport=FakeTransport(), heartbeat_profile="hybrid")

    def test_packet_debug_log_includes_command_and_sequence(self) -> None:
        robot = S1Chassis(transport=FakeTransport())
        with self.assertLogs("s1_can.chassis", level="DEBUG") as logs:
            with robot._lock:
                robot._send_heartbeat_locked()
                robot._send_heartbeat_locked()
                robot._send_heartbeat_locked()

        messages = [record.getMessage() for record in logs.records]
        self.assertTrue(all("command 0x3F/0x60" in message for message in messages))
        self.assertEqual(
            [message.rsplit("sequence=", 1)[-1] for message in messages],
            ["0", "0", "1"],
        )

    def test_idle_refresh_pairs_heartbeat_with_zero_wheels(self) -> None:
        transport = FakeTransport()
        robot = S1Chassis(transport=transport, heartbeat_rate=200)
        robot.start()
        time.sleep(0.02)
        robot.stop()

        commands = [packet[9:11] for packet in transport.packets]
        self.assertEqual(commands[0], b"\x3f\x19")
        self.assertEqual(commands.count(b"\x3f\x19"), 1)
        self.assertEqual(len(commands[1:]) % 2, 0)
        for index in range(1, len(commands), 2):
            self.assertEqual(commands[index:index + 2], [b"\x3f\x60", b"\x3f\x20"])
        for packet in transport.packets:
            if packet[9:11] != b"\x3f\x20":
                continue
            self.assertEqual(struct.unpack("<hhhh", packet[11:-2]), (0, 0, 0, 0))

    def test_drive_for_refreshes_command_for_full_duration(self) -> None:
        robot = FakeRobot()
        with (
            patch(
                "s1_can.test_drive.time.monotonic",
                side_effect=(0, 0, 0.05, 0.10, 0.15, 0.20),
            ),
            patch("s1_can.test_drive.time.sleep") as sleep,
        ):
            drive_for(robot, (1.0, 0.0, 0.0), 0.20)

        self.assertEqual(robot.commands, [(1.0, 0.0, 0.0)] * 4)
        self.assertEqual(sleep.call_count, 4)

    def test_motion_test_warms_up_moves_and_cools_down(self) -> None:
        robot = FakeRobot()
        motion = (1.0, 0.0, 0.0)
        with patch("s1_can.test_drive.drive_for") as drive:
            run_motion_test(robot, motion, duration=1.0, warmup=1.0, cooldown=1.0)

        self.assertTrue(robot.started)
        self.assertTrue(robot.stopped)
        self.assertEqual(
            [call.args for call in drive.call_args_list],
            [
                (robot, ZERO_MOTION, 1.0),
                (robot, motion, 1.0),
                (robot, ZERO_MOTION, 1.0),
            ],
        )

    def test_ros_mixer_has_confirmed_physical_patterns(self) -> None:
        self.assertEqual(mix_ros_velocity(1, 0, 0), (1, 1, 1, 1))
        self.assertEqual(mix_ros_velocity(0, 1, 0), (1, -1, 1, -1))
        self.assertEqual(mix_ros_velocity(0, 0, 1), (1, -1, -1, 1))

    def test_wheel_payload_applies_cpp_wiring_signs(self) -> None:
        transport = FakeTransport()
        robot = S1Chassis(transport=transport)
        robot.set_wheel_speeds(10, 20, 30, 40)
        with robot._lock:
            robot._send_wheel_speeds_locked(robot._wheel_speeds)
        self.assertEqual(struct.unpack("<hhhh", transport.packets[-1][11:-2]), (10, -20, -30, 40))

    def test_watchdog_replaces_stale_command_with_zero(self) -> None:
        transport = FakeTransport()
        robot = S1Chassis(
            transport=transport, heartbeat_rate=200, command_timeout=0.02
        )
        robot.start()
        robot.set_wheel_speeds(30, 30, 30, 30)
        time.sleep(0.05)
        robot.stop()
        wheel_packets = [p for p in transport.packets if p[9:11] == b"\x3f\x20"]
        self.assertGreater(len(wheel_packets), 2)
        self.assertEqual(struct.unpack("<hhhh", wheel_packets[-2][11:-2]), (0, 0, 0, 0))
        self.assertTrue(transport.closed)

    def test_stop_is_safe_when_final_zero_cannot_be_sent(self) -> None:
        transport = RecoveringTransport()
        robot = S1Chassis(transport=transport)
        robot.start()
        transport.fail_next_send = True
        robot.stop()
        self.assertTrue(transport.closed)
        self.assertFalse(robot.connected)


if __name__ == "__main__":
    unittest.main()
