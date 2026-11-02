"""Read-only CAN IMU receiver. Subscription writes belong to S1Chassis."""
import math
import socket
import struct

from .protocol import crc8, crc16


class ImuDecoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        samples = []
        while len(self.buffer) >= 4:
            b = self.buffer
            if b[0] != 0x55 or crc8(bytes(b[:3])) != b[3]:
                del b[0]
                continue
            size = b[1] | ((b[2] & 3) << 8)
            if size < 13:
                del b[0]
                continue
            if len(b) < size:
                break
            packet = bytes(b[:size])
            if crc16(packet[:-2]) != int.from_bytes(packet[-2:], 'little'):
                del b[0]
                continue
            del b[:size]
            if (size == 47 and packet[4:6] == bytes((3, 9))
                    and packet[9:11] == bytes((0x48, 8)) and packet[12] == 7):
                clock = struct.unpack_from('<II', packet, 13)
                values = struct.unpack_from('<6f', packet, 21)
                if all(math.isfinite(v) for v in values):
                    samples.append((clock, values))
        return samples


class ImuReceiver:
    def __init__(self, channel):
        self.socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            self.socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER,
                                   # Match standard data frames; CAN_ERR_FLAG in
                                   # this mask routes the filter to error frames.
                                   struct.pack('=II', 0x202, 0xC00007FF))
            self.socket.bind((channel,))
            self.socket.setblocking(False)
        except Exception:
            self.socket.close()
            raise
        self.decoder = ImuDecoder()

    def poll(self):
        latest = None
        # Bound work per ROS callback; drop intermediate samples, never replay a queue.
        for _ in range(256):
            try:
                frame = self.socket.recv(16)
            except BlockingIOError:
                break
            if len(frame) != 16:
                continue
            _, size, data = struct.unpack('=IB3x8s', frame)
            for sample in self.decoder.feed(data[:min(size, 8)]):
                latest = sample
        else:
            # Queue may still be old: wait for a subsequent poll to drain it.
            return None
        return latest

    def close(self):
        self.socket.close()
