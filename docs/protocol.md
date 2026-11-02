# DJI chassis packet protocol

The existing working C++ implementation is the authoritative source for this
Python port. The relevant files are:

- `src/chassis.cpp`
- `src/protocol.cpp`
- `src/crc.cpp`
- `src/can_streambuf.cpp`
- their matching headers under `include/robomaster/`

The Python equivalents are `s1_can/protocol.py`, `s1_can/transport.py`, and
`s1_can/chassis.py`. Values below are copied from those C++ functions rather
than inferred from unrelated DJI SDK implementations.

## CAN transport

Outgoing commands use standard 11-bit CAN arbitration ID `0x201`. A serialized
DJI packet is treated as a byte stream and split, in order, into classic CAN
frames containing at most eight bytes. There is no additional fragment header.

For example:

| DJI packet length | CAN payload lengths |
|---:|---|
| 14 bytes | 8 + 6 |
| 21 bytes | 8 + 8 + 5 |
| 27 bytes | 8 + 8 + 8 + 3 |

`src/can_streambuf.cpp` provides the original stream behavior;
`CanTransport.send()` performs explicit fragmentation with `python-can`.

## Packet layout

Total packet length is `payload length + 13`.

| Offset | Size | Meaning |
|---:|---:|---|
| 0 | 1 | Start byte `0x55` |
| 1 | 1 | Total length, low 8 bits |
| 2 | 1 | Length bits 8-9 plus required `0x04` flag |
| 3 | 1 | CRC8 over bytes 0-2 |
| 4 | 1 | Sender ID |
| 5 | 1 | Receiver ID |
| 6 | 2 | Per-command sequence, little-endian |
| 8 | 1 | ACK flags: `is_ack` bit 7, `need_ack` bit 5 |
| 9 | 1 | Command set |
| 10 | 1 | Command ID |
| 11 | N | Payload |
| last 2 | 2 | CRC16 over every preceding byte, little-endian |

The 10-bit length field limits a DJI packet to 1023 bytes.

## CRC algorithms

The Python code implements the table-equivalent reflected algorithms from
`src/crc.cpp`:

| Check | Seed | Reflected polynomial | Coverage |
|---|---:|---:|---|
| CRC8 | `0x77` | `0x8C` | First three header bytes |
| CRC16 | `0x3692` | `0x8408` | Entire packet except final CRC16 |

Known workmode test vector:

```text
55 0e 04 66 09 c3 00 00 00 3f 19 01 9b 8b
```

The automated tests verify this vector and the CRC values.

## Sequence handling

Sequences are tracked independently for the combined key
`(command_set << 8) | command_id`, wrap at 16 bits, and reproduce the C++
tracker's unusual initial behavior:

```text
0, 0, 1, 2, 3, ...
```

Do not replace this with a single global packet counter without validating the
controller again.

The long-running Python driver intentionally encodes each reconnect work-mode
command with a fresh tracker and sequence zero. A power-cycled Motion
Controller has lost its prior sequence state, and the physically validated
startup work-mode packet also uses sequence zero. Work mode is not sent during
normal heartbeat/wheel-command refresh because repeated assertions can destabilize an
already-enabled controller.

## Working chassis commands

| Function | Sender | Receiver | Command | Payload |
|---|---:|---:|---:|---|
| `send_workmode(mode)` | `0x09` | `0xC3` | `0x3F/0x19` | one unsigned byte |
| `legacy_cpp` heartbeat | `0x09` | `0x03` | `0x3F/0x60` | fixed 14-byte payload |
| `direct_c3` heartbeat | `0x09` | `0xC3` | `0x3F/0x60` | fixed 14-byte payload |
| `send_wheel_speed()` | `0xC9` | `0xC3` | `0x3F/0x20` | four little-endian signed 16-bit values |
| C++ `send_speed()` | `0xC9` | `0xC3` | `0x3F/0x21` | three native/little-endian floats on tested hosts |

Restored stable heartbeat payload:

```text
00 04 20 00 01 00 40 00 02 10 04 03 00 04
```

Isolated direct-controller fallback payload:

```text
00 04 20 00 01 00 40 00 02 10 00 03 00 00
```

The Python/ROS path pairs this original C++ heartbeat with the wheel-speed
packet by default. It sends `(0, 0, 0, 0)` while idle, refreshes the requested wheel values
while moving, and returns to zero on timeout, reconnect, or shutdown. It does
not expose the captured hybrid heartbeat as a runtime option. Select
`direct_c3` only for an isolated A/B test after a controller power cycle.

Do not periodically reassert workmode during an active session. Live testing
showed that doing so can immediately return the controller to a red state.
Workmode is sent only at startup or after a real transport reconnect.

The public wheel API uses physical order `(FR, FL, RL, RR)`. The controller
wiring signs written into the payload are:

```text
(front_right, -front_left, -rear_left, rear_right)
```

This inversion comes directly from `chassis::send_wheel_speed()` in
`src/chassis.cpp`; it is separate from the ROS coordinate mixer.

The Python motion path intentionally uses wheel command `0x3F/0x20`. The C++
float body-speed command is documented for completeness but is not used by
`S1Chassis.drive()`.

## Scope and trust

Initialization, legacy heartbeat/zero refresh, and wheel control are undergoing
the controlled soak validation described in the troubleshooting guide. DDS
encoder, IMU, battery, attitude, and velocity feedback remain experimental.
Validate raw frames and units before publishing them as ROS sensor or odometry
data.
