# Jetson Orin Nano deployment

This guide records the exact deployment validated on a Jetson Orin Nano running
Ubuntu 22.04, L4T R36.4.7, and kernel `5.15.148-tegra`. Other JetPack or kernel
versions may require different kernel-source handling.

## Prerequisite: verify chassis calibration

Complete [chassis calibration and verification](chassis-calibration.md) before
motion validation. Our controller reported rollover and left SDK mode while the
robot was stationary and CAN remained healthy. Calibration with the original
assembly securely installed resolved the observed drift; retain that assembly
for maintenance even though normal operation uses the Jetson alone. After CAN
and the driver are installed below, verify stationary orientation and the
ten-minute zero-command test before commissioning motion or boot services.

## 1. Base packages

```bash
sudo apt update
sudo apt install -y \
    git can-utils python3-pip python3-venv \
    build-essential nvidia-l4t-kernel-headers
```

Clone the repository at the path expected by the included ROS systemd unit:

```bash
cd /home/ubuntu
git clone https://github.com/maoztamir/robomaster-s1-jetson.git
cd robomaster-s1-jetson
```

If a different user or checkout path is required, update
`systemd/robomaster-chassis.service` and the path check in
`scripts/install_ros_service.sh` before installation.

## 2. Determine which CAN interface is the USB adapter

The Jetson native MTTCAN controller normally owns `can0`. Confirm drivers rather
than assuming interface names:

```bash
ip -brief link
readlink -f /sys/class/net/can0/device/driver 2>/dev/null || true
lsusb
```

The validated SH-C31A reports USB ID `1d50:606f` and uses driver `gs_usb`. Once
loaded, it appeared as `can1` because native MTTCAN already occupied `can0`.

Never configure the native controller by accident when the USB adapter is the
one wired to the RoboMaster.

## 3. Provide `gs_usb` when Jetson's kernel omits it

First try the normal module:

```bash
sudo modprobe gs_usb
```

If that reports that the module is missing, inspect the kernel configuration:

```bash
grep CONFIG_CAN_GS_USB "/boot/config-$(uname -r)" 2>/dev/null || true
```

The validated stock kernel reported `CONFIG_CAN_GS_USB` unset and did not ship
`gs_usb.ko`. The following builds the upstream Linux stable driver matching the
base kernel version. Do this only with headers matching the running kernel:

```bash
kernel_release="$(uname -r)"
kernel_base="${kernel_release%%-*}"
module_dir="$PWD/../gs_usb_module_${kernel_base}"

test -e "/lib/modules/${kernel_release}/build"
mkdir -p "$module_dir"
cd "$module_dir"

curl -L \
    "https://git.kernel.org/pub/scm/linux/kernel/git/stable/linux.git/plain/drivers/net/can/usb/gs_usb.c?h=v${kernel_base}" \
    -o gs_usb.c

printf 'obj-m += gs_usb.o\n' > Makefile
make -C "/lib/modules/${kernel_release}/build" M="$PWD" modules

modinfo ./gs_usb.ko
```

For the validated `v5.15.148` source:

```text
gs_usb.c SHA256:
d1ee028fd35e29f1c120577f62d1c3d670731b3b5104f4107278d061a7e5ac06

expected vermagic prefix:
5.15.148-tegra
```

Install it for the running kernel:

```bash
sudo install -D -m 0644 \
    ./gs_usb.ko \
    "/lib/modules/$(uname -r)/extra/gs_usb.ko"

sudo depmod -a
sudo modprobe gs_usb
ip -brief link
```

Do not copy this `.ko` to a different kernel. After any Jetson kernel update,
rebuild the module against the new matching headers before expecting CAN to
work. An out-of-tree module is tied to its kernel ABI and `vermagic`.

## 4. Install Python dependencies and test without ROS

```bash
cd /home/ubuntu/robomaster-s1-jetson
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
.venv/bin/python -m unittest discover -s tests -v
```

Configure the USB adapter:

```bash
sudo ./scripts/watch_can0.sh --interface can1 --once
ip -details -statistics link show can1
```

Expected state includes:

```text
UP
ERROR-ACTIVE
bitrate 1000000
qlen 1000
restart-ms 0
```

Run the zero-only test before motion:

```bash
.venv/bin/python -m s1_can.test_drive stop \
    --interface can1 \
    --duration 1
```

Then follow the lifted-chassis direction tests in
[hardware-and-safety.md](hardware-and-safety.md).

## 5. Install ROS2 Humble

Humble is the ROS2 release built for Ubuntu 22.04. Follow the current official
[ROS2 Ubuntu installation instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
For a headless Jetson, install at least:

```bash
sudo apt install -y ros-humble-ros-base ros-dev-tools python3-can
```

Do not run a blind full system upgrade solely for this project: a new kernel
will require rebuilding the out-of-tree `gs_usb` module.

Verify:

```bash
source /opt/ros/humble/setup.bash
ros2 --help >/dev/null && echo "ROS2 CLI OK"
/usr/bin/python3 -c "import rclpy, can; print('rclpy and python-can OK')"
```

Build the package:

```bash
cd /home/ubuntu/robomaster-s1-jetson
source /opt/ros/humble/setup.bash

colcon --log-base ros2_log build \
    --packages-select s1_chassis_driver \
    --symlink-install \
    --build-base ros2_build \
    --install-base ros2_install
```

## 6. Install automatic services

Install CAN recovery first:

```bash
sudo ./scripts/install_can_service.sh
```

Stop any manually running `s1_chassis_node`, then install the ROS node:

```bash
sudo ./scripts/install_ros_service.sh
```

The service configuration is intentionally explicit:

```text
CAN interface: can1
CAN bitrate: 1 Mbps
ROS domain: 42
ROS RMW: rmw_fastrtps_cpp
service user: ubuntu
checkout: /home/ubuntu/robomaster-s1-jetson
```

Verify:

```bash
systemctl is-enabled robomaster-can.service
systemctl is-active robomaster-can.service
systemctl is-enabled robomaster-chassis.service
systemctl is-active robomaster-chassis.service

journalctl -b \
    -u robomaster-can.service \
    -u robomaster-chassis.service \
    --no-pager
```

The expected boot order is CAN module/interface recovery, then ROS node startup,
then a connected message. The wheels must remain stopped for the entire boot.

## 7. Required deployment validation

Before ground operation, validate all of the following while lifted:

- zero-only startup;
- six directions;
- command duration;
- `/cmd_vel` timeout;
- `Ctrl-C` shutdown;
- USB removal while the node runs;
- recovery without replaying motion;
- recovery from an `ERROR-ACTIVE` but stalled transmit queue;
- a fresh command after recovery;
- complete Jetson reboot with both services enabled.

See [validation-and-troubleshooting.md](validation-and-troubleshooting.md) for
expected symptoms and diagnostics.
