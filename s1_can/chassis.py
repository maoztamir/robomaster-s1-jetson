"""Threaded RoboMaster S1 wheel driver based on ``src/chassis.cpp``."""

from __future__ import annotations

import logging
import math
import struct
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from .protocol import PacketEncoder
from .transport import CanTransport, CanTransportError

_LOGGER = logging.getLogger(__name__)


class HeartbeatProfile(str, Enum):
    """Coherent heartbeat packets observed in working direct-CAN drivers."""

    LEGACY_CPP = "legacy_cpp"
    DIRECT_C3 = "direct_c3"


@dataclass(frozen=True)
class _HeartbeatSpec:
    receiver: int
    payload: bytes


HEARTBEAT_PROFILES = {
    HeartbeatProfile.LEGACY_CPP: _HeartbeatSpec(
        receiver=0x03,
        payload=bytes.fromhex("00 04 20 00 01 00 40 00 02 10 04 03 00 04"),
    ),
    HeartbeatProfile.DIRECT_C3: _HeartbeatSpec(
        receiver=0xC3,
        payload=bytes.fromhex("00 04 20 00 01 00 40 00 02 10 00 03 00 00"),
    ),
}
DEFAULT_HEARTBEAT_PROFILE = HeartbeatProfile.LEGACY_CPP


class _Transport(Protocol):
    def send(self, packet: bytes) -> None: ...
    def reconnect(self) -> bool: ...
    def close(self) -> None: ...


def mix_ros_velocity(vx: float, vy: float, wz: float) -> tuple[float, float, float, float]:
    """Map ROS body coordinates to physical S1 wheel directions.

    ROS uses +x forward, +y left, and +z counter-clockwise. Wheel order is front
    right, front left, rear left, rear right. This is the sole coordinate/sign
    conversion point; ``send_wheel_speeds`` only applies the C++ protocol's motor
    wiring signs.
    """
    return (
        vx + vy + wz,
        vx - vy - wz,
        vx + vy - wz,
        vx - vy + wz,
    )


class S1Chassis:
    """Direct-CAN S1 chassis with idle keepalive, command refresh, and watchdog."""

    def __init__(
        self,
        can_interface: str = "can0",
        *,
        heartbeat_rate: float = 100.0,
        command_timeout: float = 0.35,
        reconnect_interval: float = 1.0,
        max_wheel_command: int = 100,
        linear_scale: float = 1.0,
        angular_scale: float = 1.0,
        heartbeat_profile: HeartbeatProfile | str = DEFAULT_HEARTBEAT_PROFILE,
        transport: _Transport | None = None,
        enable_imu: bool = False,
    ) -> None:
        if heartbeat_rate <= 0:
            raise ValueError("heartbeat_rate must be positive")
        if command_timeout <= 0:
            raise ValueError("command_timeout must be positive")
        if reconnect_interval <= 0:
            raise ValueError("reconnect_interval must be positive")
        if not 0 < max_wheel_command <= 32767:
            raise ValueError("max_wheel_command must be in [1, 32767]")
        try:
            selected_profile = HeartbeatProfile(heartbeat_profile)
        except ValueError as exc:
            choices = ", ".join(profile.value for profile in HeartbeatProfile)
            raise ValueError(
                f"heartbeat_profile must be one of: {choices}"
            ) from exc
        self.heartbeat_rate = float(heartbeat_rate)
        self.command_timeout = float(command_timeout)
        self.reconnect_interval = float(reconnect_interval)
        self.max_wheel_command = int(max_wheel_command)
        self.linear_scale = float(linear_scale)
        self.angular_scale = float(angular_scale)
        self.heartbeat_profile = selected_profile
        self._heartbeat_spec = HEARTBEAT_PROFILES[selected_profile]
        self._transport = transport or CanTransport(can_interface)
        self._encoder = PacketEncoder()
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._wheel_speeds = (0, 0, 0, 0)
        self._last_command = -math.inf
        self._connected = False
        self._next_reconnect = 0.0
        self._last_transport_error: str | None = None
        self._closed = False
        self.enable_imu = enable_imu

    def refresh_imu_subscription(self) -> None:
        """Serialize the diagnostic subscription with every other CAN write."""
        with self._lock:
            if self.enable_imu and self._connected:
                try:
                    self._subscribe_imu_locked()
                except CanTransportError as exc:
                    self._mark_disconnected_locked(exc, time.monotonic())

    def _subscribe_imu_locked(self) -> None:
        self._send_packet(9, 3, 0x48, 4, bytes((0, 9, 7)))
        self._send_packet(9, 3, 0x48, 3,
                          bytes((9, 7, 3, 0, 1)) +
                          bytes.fromhex('f4 1d 1c dc 03 00 02 00') +
                          struct.pack('<H', 10))

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def connected(self) -> bool:
        """Whether workmode and the initial zero command reached SocketCAN."""
        with self._lock:
            return self._connected

    @property
    def last_transport_error(self) -> str | None:
        with self._lock:
            return self._last_transport_error

    def start(self) -> None:
        """Enter chassis work mode 1 and start the 100 Hz-compatible refresh loop."""
        with self._lock:
            if self.running:
                return
            if self._closed:
                raise RuntimeError("S1Chassis cannot be restarted after stop()")
            _LOGGER.info(
                "Starting RoboMaster chassis with heartbeat profile %s "
                "(receiver 0x%02X)",
                self.heartbeat_profile.value,
                self._heartbeat_spec.receiver,
            )
            self._stop_event.clear()
            self._wheel_speeds = (0, 0, 0, 0)
            self._last_command = -math.inf
            self._try_reconnect_locked(time.monotonic())
            self._thread = threading.Thread(
                target=self._run,
                name="s1-chassis-heartbeat",
                daemon=True,
            )
            self._thread.start()

    def set_wheel_speeds(
        self,
        front_right: float,
        front_left: float,
        rear_left: float,
        rear_right: float,
    ) -> None:
        """Set logical wheel commands in confirmed physical wheel order."""
        values = self._saturate((front_right, front_left, rear_left, rear_right))
        with self._lock:
            self._wheel_speeds = values
            self._last_command = time.monotonic()
            if self.running and self._connected:
                try:
                    self._send_wheel_speeds_locked(values)
                except CanTransportError as exc:
                    self._mark_disconnected_locked(exc, time.monotonic())

    def drive(self, vx: float, vy: float, wz: float) -> None:
        """Drive using ROS coordinate conventions and configurable abstract scales."""
        wheels = mix_ros_velocity(
            float(vx) * self.linear_scale,
            float(vy) * self.linear_scale,
            float(wz) * self.angular_scale,
        )
        self.set_wheel_speeds(*wheels)

    def stop(self) -> None:
        """Stop motion, stop the worker, send a final zero, and close the CAN bus."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, 2.0 / self.heartbeat_rate))
        with self._lock:
            if self._closed:
                return
            self._wheel_speeds = (0, 0, 0, 0)
            if self._connected:
                try:
                    self._send_heartbeat_locked()
                    self._send_wheel_speeds_locked((0, 0, 0, 0))
                    if self.enable_imu:
                        self._send_packet(9, 3, 0x48, 4, bytes((0, 9, 7)))
                except CanTransportError:
                    # Shutdown must continue even if the adapter was removed.
                    pass
            self._connected = False
            self._thread = None
            self._closed = True
        self._transport.close()

    close = stop

    def __enter__(self) -> "S1Chassis":
        self.start()
        return self

    def __exit__(self, *_: object) -> None:
        self.stop()

    def _run(self) -> None:
        period = 1.0 / self.heartbeat_rate
        while not self._stop_event.wait(period):
            now = time.monotonic()
            with self._lock:
                if not self._connected:
                    if now >= self._next_reconnect:
                        self._try_reconnect_locked(now)
                    continue
                if now - self._last_command > self.command_timeout:
                    self._wheel_speeds = (0, 0, 0, 0)
                try:
                    self._send_heartbeat_locked()
                    self._send_wheel_speeds_locked(self._wheel_speeds)
                except CanTransportError as exc:
                    self._mark_disconnected_locked(exc, now)

    def _try_reconnect_locked(self, now: float) -> bool:
        if not self._transport.reconnect():
            error = getattr(self._transport, "last_error", None)
            self._last_transport_error = error or "SocketCAN interface unavailable"
            self._connected = False
            self._next_reconnect = now + self.reconnect_interval
            return False

        # Never replay a pre-disconnect motion command. A fresh command must
        # arrive after recovery before non-zero wheel speeds are sent.
        self._wheel_speeds = (0, 0, 0, 0)
        self._last_command = -math.inf
        try:
            self._send_workmode_locked(1)
            self._send_heartbeat_locked()
            self._send_wheel_speeds_locked((0, 0, 0, 0))
            if self.enable_imu:
                self._subscribe_imu_locked()
        except CanTransportError as exc:
            self._mark_disconnected_locked(exc, now)
            return False

        was_disconnected = self._last_transport_error is not None
        self._connected = True
        self._last_transport_error = None
        self._next_reconnect = 0.0
        if was_disconnected:
            _LOGGER.info("RoboMaster SocketCAN transport recovered; sent safe zero")
        return True

    def _mark_disconnected_locked(self, exc: CanTransportError, now: float) -> None:
        if self._connected or self._last_transport_error != str(exc):
            _LOGGER.warning("RoboMaster SocketCAN transport lost: %s", exc)
        self._connected = False
        self._wheel_speeds = (0, 0, 0, 0)
        self._last_command = -math.inf
        self._last_transport_error = str(exc)
        self._next_reconnect = now + self.reconnect_interval

    def _send_workmode_locked(self, mode: int) -> None:
        if not 0 <= mode <= 0xFF:
            raise ValueError("work mode must fit in one byte")
        # chassis::send_workmode in src/chassis.cpp
        # A power-cycled controller has also lost its command-sequence state.
        # Encode each recovery assertion with a fresh tracker so workmode uses
        # the physically validated sequence zero instead of the long-running
        # process's pre-reboot sequence.
        packet = PacketEncoder().encode(0x09, 0xC3, 0x3F, 0x19, bytes((mode,)))
        self._send_encoded_packet(packet)

    def _send_heartbeat_locked(self) -> None:
        self._send_packet(
            0x09,
            self._heartbeat_spec.receiver,
            0x3F,
            0x60,
            self._heartbeat_spec.payload,
        )

    def _send_wheel_speeds_locked(self, wheels: tuple[int, int, int, int]) -> None:
        # chassis::send_wheel_speed in src/chassis.cpp negates FL and RL when
        # converting confirmed physical wheel directions to controller wiring.
        front_right, front_left, rear_left, rear_right = wheels
        payload = struct.pack(
            "<hhhh", front_right, -front_left, -rear_left, rear_right
        )
        self._send_packet(0xC9, 0xC3, 0x3F, 0x20, payload)

    def _send_packet(
        self, sender: int, receiver: int, command_set: int, command_id: int, payload: bytes
    ) -> None:
        self._send_encoded_packet(
            self._encoder.encode(sender, receiver, command_set, command_id, payload)
        )

    def _send_encoded_packet(self, packet: bytes) -> None:
        sequence = packet[6] | (packet[7] << 8)
        _LOGGER.debug(
            "RoboMaster TX 0x%02X->0x%02X command 0x%02X/0x%02X sequence=%d",
            packet[4],
            packet[5],
            packet[9],
            packet[10],
            sequence,
        )
        self._transport.send(packet)

    def _saturate(self, wheels: tuple[float, float, float, float]) -> tuple[int, int, int, int]:
        if not all(math.isfinite(value) for value in wheels):
            raise ValueError("wheel commands must be finite")
        peak = max(abs(value) for value in wheels)
        scale = min(1.0, self.max_wheel_command / peak) if peak else 1.0
        return tuple(round(value * scale) for value in wheels)  # type: ignore[return-value]
