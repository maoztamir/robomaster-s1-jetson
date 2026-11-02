# Validation checklist and troubleshooting

## Release checklist

Use this checklist for every new machine, adapter, kernel, or chassis.

### Static checks

- Correct CAN-H, CAN-L, and ground wiring.
- Appropriate chassis power supply.
- Correct termination.
- SH-C31A visible in `lsusb`.
- The selected SocketCAN interface uses `gs_usb`, not native MTTCAN.
- Interface is `UP`, `ERROR-ACTIVE`, and at 1 Mbps.
- Python unit tests pass.
- [Chassis calibration](chassis-calibration.md) verified with stationary orientation
  and a ten-minute zero-command test before motion commissioning.

### Lifted motion checks

- `stop` test causes no movement.
- Forward and backward signs are correct.
- Left and right signs are correct.
- Clockwise and counter-clockwise signs are correct.
- One-second and longer finite durations are respected.
- `Ctrl-C` sends zero.
- Stopping command refresh results in zero after the watchdog timeout.

### Fault-injection checks

- Power-cycle the Motion Controller while the ROS node remains running.
- Confirm work mode is restored automatically and a fresh command moves the
  chassis without restarting `robomaster-chassis.service`.
- Remove the USB adapter while the ROS node runs.
- Confirm the node reports disconnection and clears motion.
- Reconnect and confirm the CAN monitor restores 1 Mbps configuration.
- Confirm recovery sends zero and does not replay the prior command.
- Send a fresh command and confirm motion resumes.
- Reboot the Jetson and confirm both systemd services become active while the
  wheels remain stopped.

## Useful status commands

```bash
ip -brief link
ip -details -statistics link show can1

systemctl --no-pager --full status robomaster-can.service
systemctl --no-pager --full status robomaster-chassis.service

journalctl -b \
    -u robomaster-can.service \
    -u robomaster-chassis.service \
    --no-pager
```

CAN counters should be interpreted together with the CAN state. A receive-drop
counter can grow when no userspace socket is consuming frames; that is different
from `bus-errors`, `error-pass`, or `bus-off` counters increasing.

## Controller LED blinks red while CAN is healthy

Do not treat the Motion Controller LED as the only connection test. The
controller can blink red for its own chassis alarms while still accepting
direct-CAN commands. On the validated gimbal-free setup, CAN ID `0x202` has
reported diagnostics including `[L-SA_ABN]lost connect with gimbal` and
`[L-SA_ABN]roll over`. The former is expected with the gimbal disconnected.
Rollover also occurred with this chassis completely stationary: drifting attitude
was accompanied by `Fail to check mis 4. Reset` and an exit from SDK mode despite
continuous heartbeats. Completing DJI calibration with the original assembly
properly installed resolved that failure. Follow the
[calibration procedure and recorded evidence](chassis-calibration.md); do not
assume the robot moved or dismiss rollover as a missing-gimbal warning. A repeating
`[L-SA_ABN]low power warning` is a separate motion-blocking condition: charge
or replace the chassis battery before evaluating connection persistence.

Confirm actual connectivity using the service log, an empty transmit backlog,
and live controller-originated frames:

```bash
journalctl -b -u robomaster-chassis.service --no-pager -n 20
tc -s qdisc show dev can1
timeout 2 candump can1
```

Traffic on `0x202` and motor IDs `0x211` through `0x214` proves the controller
side is still transmitting. A successful finite motion test is the final
enablement check.

If the driver reports `Transmit buffer full` or `No buffer space available`
while CAN still says `ERROR-ACTIVE`, inspect the qdisc:

```bash
tc -s qdisc show dev can1
```

The SH-C31A can remain nominally error-active with almost all 1000 transmit
slots occupied after its peer disconnects. The CAN monitor treats a queue at
least 90% full as stalled, cycles the interface, and keeps retrying until the
peer returns. This interface cycle makes the chassis driver discard stale
motion before reinitializing with work mode, the legacy heartbeat, and zero speed.

## Repeated `RTNETLINK answers: No such device` while `can1` exists

The `gs_usb` binding can become stale: `ip link` still lists `can1`, but the
driver rejects attempts to set its queue length, bitrate, or link state. Normal
bus-off cycling cannot repair this condition. After three consecutive
configuration failures, the monitor verifies that the interface is bound to
`/sys/bus/usb/drivers/gs_usb`, rebinds only that USB interface, and then resumes
normal configuration. It will never apply this fallback to native MTTCAN.

The defaults are configurable in `/etc/default/robomaster-can`:

```text
CAN_REBIND_AFTER_FAILURES=3
CAN_REBIND_COOLDOWN=30
```

Successful recovery is visible as `Rebinding gs_usb interface`, followed by
`can1 reappeared` and `can1 ready` in the service journal. Motion is not replayed;
the chassis driver reconnects with a zero command and waits for fresh input.

## `No such device` for `can0`

The interface is absent or has a different name.

```bash
ip -brief link
lsusb
```

On the validated Jetson, `can0` is native MTTCAN and the SH-C31A is `can1`.
Always pass `--interface can1` there and keep
`/etc/default/robomaster-can` consistent.

## `ModuleNotFoundError: No module named 'can'`

The Python interpreter does not contain `python-can`.

For the project virtual environment:

```bash
.venv/bin/pip install -r requirements.txt
```

For the ROS2 system interpreter on Ubuntu:

```bash
sudo apt install python3-can
```

Do not run the ROS deb installation through a Conda Python interpreter. ROS2
binary packages are built for Ubuntu's system Python.

## `modprobe: FATAL: Module gs_usb not found`

The Jetson kernel does not ship the driver. Follow the matching-kernel build in
[jetson-orin-nano.md](jetson-orin-nano.md). Check all three match:

```bash
uname -r
modinfo ./gs_usb.ko
readlink -f "/lib/modules/$(uname -r)/build"
```

Never reuse a module with a different `vermagic` after a kernel update.

## Interface is `BUS-OFF` or `STOPPED`

First correct the physical cause: power, ground, CAN-H/L, termination, or an
absent acknowledging node. Then recover without `restart-ms`:

```bash
sudo ./scripts/watch_can0.sh --interface can1 --once
```

For continuous recovery:

```bash
sudo ./scripts/watch_can0.sh --interface can1
```

## Motion lasts only about 0.35 seconds

This is the command watchdog working correctly. A long-running controller must
refresh commands faster than `command_timeout`. `test_drive` refreshes at 20 Hz,
and the ROS node refreshes at its `command_rate` while the last `/cmd_vel`
remains fresh.

Do not increase the timeout to hide a broken command loop. A shorter stale-motion
window is an intentional safety property.

## `sequence size exceeds remaining buffer`

This was observed when native ROS2 Jazzy on Ubuntu 24.04 joined the ROS2 Humble
Jetson domain through Fast DDS. Stop the Jazzy daemon:

```bash
source /opt/ros/jazzy/setup.bash
ros2 daemon stop
```

Use the Humble desktop container described in [remote-rviz.md](remote-rviz.md).
Cross-distribution communication is not a supported deployment configuration.

## Node is connected but RViz is empty

This is expected in the current milestone. The node subscribes to `/cmd_vel` and
publishes only ROS parameter/logging state. DDS feedback decoding, TF, URDF,
joint states, odometry, IMU, and battery topics are future work.

## Updating code on the Jetson

Pull, rebuild, and restart in this order:

```bash
cd /home/ubuntu/robomaster-s1-jetson
git pull --ff-only

source /opt/ros/humble/setup.bash
colcon --log-base ros2_log build \
    --packages-select s1_chassis_driver \
    --symlink-install \
    --build-base ros2_build \
    --install-base ros2_install

sudo systemctl restart robomaster-chassis.service
systemctl --no-pager --full status robomaster-chassis.service
```

The systemd stop path sends `SIGINT`, allowing the node to send a final zero
before it is restarted.
