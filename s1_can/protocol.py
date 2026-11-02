"""DJI packet encoding ported from ``src/protocol.cpp`` and ``src/crc.cpp``.

The motion controller receives a byte stream split across classic CAN frames.  This
module only builds the DJI packets; CAN framing lives in :mod:`s1_can.transport`.
"""

from __future__ import annotations

from collections import defaultdict
from threading import Lock

PACKET_START = 0x55
CRC8_SEED = 119
CRC16_SEED = 13970


def crc8(data: bytes, seed: int = CRC8_SEED) -> int:
    """Return the table-equivalent CRC-8 used by ``src/crc.cpp``."""
    crc = seed
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8C if crc & 1 else 0)
    return crc


def crc16(data: bytes, seed: int = CRC16_SEED) -> int:
    """Return the table-equivalent reflected CRC-16 used by ``src/crc.cpp``."""
    crc = seed
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


class SequenceTracker:
    """Mirror C++ ``id_tracker`` sequence behavior independently per command."""

    def __init__(self) -> None:
        self._counts: dict[int, int] = defaultdict(int)
        self._seen: set[int] = set()
        self._lock = Lock()

    def next(self, command: int) -> int:
        # src/protocol.cpp returns zero both when inserting a command and on its
        # next lookup. Preserve that slightly unusual 0, 0, 1, 2, ... sequence.
        with self._lock:
            if command not in self._seen:
                self._seen.add(command)
                return 0
            current = self._counts[command]
            self._counts[command] = (current + 1) & 0xFFFF
            return current


class PacketEncoder:
    """Encode DJI command packets exactly as ``package::write_to`` does."""

    def __init__(self, sequences: SequenceTracker | None = None) -> None:
        self._sequences = sequences or SequenceTracker()

    def encode(
        self,
        sender: int,
        receiver: int,
        command_set: int,
        command_id: int,
        payload: bytes = b"",
        *,
        is_ack: bool = False,
        need_ack: bool = False,
    ) -> bytes:
        packet_size = len(payload) + 13
        if packet_size > 0x3FF:
            raise ValueError("DJI packet exceeds the 10-bit length field")
        for name, value in (
            ("sender", sender),
            ("receiver", receiver),
            ("command_set", command_set),
            ("command_id", command_id),
        ):
            if not 0 <= value <= 0xFF:
                raise ValueError(f"{name} must fit in one byte")

        header = bytearray((
            PACKET_START,
            packet_size & 0xFF,
            ((packet_size >> 8) & 0x03) | 0x04,
        ))
        header.append(crc8(bytes(header)))

        command = (command_set << 8) | command_id
        sequence = self._sequences.next(command)
        packet = header + bytearray((
            sender,
            receiver,
            sequence & 0xFF,
            (sequence >> 8) & 0xFF,
            (int(is_ack) << 7) | (int(need_ack) << 5),
            command_set,
            command_id,
        ))
        packet.extend(payload)
        checksum = crc16(bytes(packet))
        packet.extend((checksum & 0xFF, (checksum >> 8) & 0xFF))
        return bytes(packet)
