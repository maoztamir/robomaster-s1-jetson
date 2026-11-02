# Python driver and ROS2 node

## Architecture

```text
/cmd_vel (geometry_msgs/Twist)
  -> s1_chassis_node (rclpy)
  -> S1Chassis.drive(vx, vy, wz)
  -> ROS-coordinate Mecanum mixer and saturation
  -> DJI packet encoder
  -> python-can SocketCAN
  -> can0/can1 -> SH-C31A -> S1 Motion Controller
```

The Python driver does not depend on ROS and can be tested independently before
ROS2 is installed.

## Python-only installation

On Ubuntu:

```bash
sudo apt install -y python3-venv can-utils
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

Configure SocketCAN before creating the driver:

```bash
sudo ./scripts/watch_can0.sh --interface can0 --once
```

Minimal API:

```python
import time

from s1_can import S1Chassis

robot = S1Chassis(
    "can0",
    max_wheel_command=100,
    linear_scale=30.0,
    angular_scale=30.0,
)

try:
    robot.start()
    robot.drive(vx=1.0, vy=0.0, wz=0.0)
    time.sleep(0.2)
finally:
    robot.stop()
```

`drive()` uses ROS coordinates even when ROS is not installed. Callers that run
longer than `command_timeout` must refresh their command. This is deliberate:
if the calling loop fails, the background driver sends zero instead of
continuing the last movement.

## Safe physical test utility

Lift the chassis before running these commands:

```bash
.venv/bin/python -m s1_can.test_drive forward  --interface can0 --command 30 --duration 1
.venv/bin/python -m s1_can.test_drive backward --interface can0 --command 30 --duration 1
.venv/bin/python -m s1_can.test_drive left     --interface can0 --command 30 --duration 1
.venv/bin/python -m s1_can.test_drive right    --interface can0 --command 30 --duration 1
.venv/bin/python -m s1_can.test_drive ccw      --interface can0 --command 30 --duration 1
.venv/bin/python -m s1_can.test_drive cw       --interface can0 --command 30 --duration 1
```

`--command` is the abstract scale used for the unit direction vector. It is not
calibrated metres per second. `--duration` controls how long the utility
refreshes the command at 20 Hz; it then exits through `stop()`.

## Saturation

Mixed commands are scaled together when any wheel would exceed
`max_wheel_command`. This preserves the requested direction instead of clipping
each wheel independently.

Example before saturation:

```text
vx=1, vy=1, wz=1 -> (3, -1, 1, 1)
```

With a maximum of 1, the result is proportional to:

```text
(1, -0.333, 0.333, 0.333)
```

The final protocol values are rounded to signed integers.

## ROS2 package

The repository root is an `ament_python` package named
`s1_chassis_driver`. On ROS2 Humble:

```bash
source /opt/ros/humble/setup.bash

colcon --log-base ros2_log build \
    --packages-select s1_chassis_driver \
    --symlink-install \
    --build-base ros2_build \
    --install-base ros2_install

source ros2_install/setup.bash
```

Start the node:

```bash
ros2 run s1_chassis_driver s1_chassis_node --ros-args \
    --params-file config/s1_chassis.yaml
```

The node initially sends zero and waits for `/cmd_vel`.

## Parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `can_interface` | `can1` | SocketCAN interface |
| `command_rate` | `20.0` Hz | Rate used to refresh a current ROS command |
| `cmd_vel_timeout` | `0.35` s | Maximum age of the last `/cmd_vel` |
| `max_wheel_command` | `100` | Absolute raw wheel-command limit |
| `linear_scale` | `30.0` | Raw command per unit linear input |
| `angular_scale` | `30.0` | Raw command per unit angular input |
| `heartbeat_profile` | `legacy_cpp` | Coherent heartbeat packet profile (`legacy_cpp` or `direct_c3`) |

The defaults are initial abstract scales, not a physical calibration. Until
wheel feedback and geometry are validated, a command such as `linear.x = 1.0`
means “apply the configured scale,” not “guarantee 1 m/s.”

After a transport reconnect, the driver sends work mode with startup sequence
zero, followed by heartbeat and zero speed. The CAN monitor forces that
reconnect if a disconnected controller leaves the transmit queue stalled.
Stale motion still expires through the independent ROS and chassis watchdogs;
recovery does not extend either timeout.

Use `legacy_cpp` for the pre-migration C++ packet (`0x09 -> 0x03`). The
`direct_c3` profile is an isolated fallback for controlled A/B testing after a
Motion Controller power cycle. The experimental hybrid heartbeat is not
available as a runtime option.

## Finite ROS test

With all wheels lifted, publish 20 messages at 20 Hz:

```bash
ros2 topic pub --rate 20 --times 20 \
    /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The chassis should move for about one second and stop approximately
`cmd_vel_timeout` later. Avoid indefinite `ros2 topic pub --rate ...` tests
during initial validation.

## Tests

Run the dependency-light unit suite with the project virtual environment:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

The tests cover known CRC/packet vectors, per-command sequence behavior, CAN
fragmentation, wheel mapping, watchdog zeroing, safe shutdown, and disconnect
recovery.
