"""Classic-CAN transport corresponding to ``src/can_streambuf.cpp``."""

from __future__ import annotations

from typing import Any


class CanTransportError(RuntimeError):
    """SocketCAN is unavailable or a CAN frame could not be transmitted."""


class CanTransport:
    """Send the DJI byte stream as <=8-byte standard CAN frames on ID 0x201."""

    def __init__(
        self,
        channel: str = "can0",
        arbitration_id: int = 0x201,
        *,
        send_timeout: float = 0.1,
        bus: Any | None = None,
    ) -> None:
        if send_timeout <= 0:
            raise ValueError("send_timeout must be positive")
        self.channel = channel
        self.arbitration_id = arbitration_id
        self.send_timeout = float(send_timeout)
        self._owns_bus = bus is None
        self._last_error: str | None = None
        if bus is None:
            try:
                import can
            except ImportError as exc:
                raise RuntimeError(
                    "python-can is required; install the project's Python dependencies"
                ) from exc
            self._can = can
            self._message_type = can.Message
            self._bus = None
            # A missing/down interface is recoverable. S1Chassis will keep trying
            # while its worker is alive instead of failing construction forever.
            self.reconnect()
        else:
            self._can = None
            self._message_type = None
            self._bus = bus

    @property
    def connected(self) -> bool:
        return self._bus is not None

    @property
    def last_error(self) -> str | None:
        return self._last_error

    def reconnect(self) -> bool:
        """Open SocketCAN again, returning whether the transport is ready."""
        if self._bus is not None:
            return True
        if not self._owns_bus:
            return False
        try:
            self._bus = self._can.Bus(interface="socketcan", channel=self.channel)
        except Exception as exc:
            # python-can uses several CanError subclasses and may also expose an
            # OSError from the kernel when the interface has disappeared.
            self._last_error = str(exc)
            self._bus = None
            return False
        self._last_error = None
        return True

    def send(self, packet: bytes) -> None:
        """Fragment one serialized DJI packet into classic CAN data frames."""
        if not packet:
            return
        if self._bus is None:
            detail = f": {self._last_error}" if self._last_error else ""
            raise CanTransportError(f"SocketCAN {self.channel} is not connected{detail}")
        if self._message_type is None:
            # Test/injected buses may provide their own message factory.
            try:
                import can
            except ImportError:
                can = None
            message_type = can.Message if can is not None else _SimpleMessage
        else:
            message_type = self._message_type

        try:
            for offset in range(0, len(packet), 8):
                message = message_type(
                    arbitration_id=self.arbitration_id,
                    data=packet[offset : offset + 8],
                    is_extended_id=False,
                )
                if self._owns_bus:
                    self._bus.send(message, timeout=self.send_timeout)
                else:
                    self._bus.send(message)
        except Exception as exc:
            self._last_error = str(exc)
            self._disconnect()
            raise CanTransportError(
                f"SocketCAN {self.channel} transmission failed: {exc}"
            ) from exc

    def close(self) -> None:
        self._disconnect()

    def _disconnect(self) -> None:
        bus = self._bus
        self._bus = None
        if self._owns_bus and bus is not None:
            try:
                bus.shutdown()
            except Exception:
                # The kernel device may already have disappeared.
                pass


class _SimpleMessage:
    """Small python-can-compatible value used only by dependency-free tests."""

    def __init__(self, *, arbitration_id: int, data: bytes, is_extended_id: bool) -> None:
        self.arbitration_id = arbitration_id
        self.data = bytearray(data)
        self.is_extended_id = is_extended_id
