# RPLIDAR A1 with ROS 2 Humble on Jetson Orin Nano

This guide records the RPLIDAR A1 setup physically validated on the project
Jetson Orin Nano. The lidar publishes `sensor_msgs/msg/LaserScan` on `/scan`,
and the scan can be viewed remotely from the ROS 2 Humble Docker environment
described in [remote-rviz.md](remote-rviz.md).

Validated result:

```text
USB adapter: Silicon Labs CP2102 (10c4:ea60)
Linux driver: cp210x
Serial port: /dev/ttyUSB0
Serial baud rate: 115200
ROS topic: /scan
Scan frame: laser
Measured scan rate: approximately 6.98 Hz
```

The driver is the official
[Slamtec `rplidar_ros` ROS 2 package](https://github.com/Slamtec/rplidar_ros/tree/ros2).

## 1. Connect and identify the lidar

Connect the A1 through its USB interface board using a data-capable USB cable.
Power alone is not sufficient: the adapter must enumerate on the Jetson USB
bus.

```bash
lsusb
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

The validated adapter appeared as:

```text
ID 10c4:ea60 Silicon Labs CP210x UART Bridge
/dev/ttyUSB0
```

Check the stable udev path:

```bash
ls -l /dev/serial/by-id/
```

On the validated unit it was:

```text
/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Prefer `/dev/serial/by-id/...` in configuration. A name such as
`/dev/ttyUSB0` can change when USB devices are reconnected. Other adapters may
produce a different by-id name, so discover it on the target Jetson rather
than copying this value blindly.

If the adapter is absent from `lsusb`, ROS configuration is not yet relevant.
Check the interface-board connection, USB cable, and USB port first. Watch
enumeration live with:

```bash
sudo dmesg -w
```

## 2. Grant serial-port access

The serial device normally belongs to `root:dialout`. Add the service/login
user to that group:

```bash
sudo usermod -aG dialout ubuntu
```

Log out and reconnect so the new group membership applies, then verify:

```bash
id
```

The output must include `dialout`. Do not work around missing group membership
with `chmod 777 /dev/ttyUSB0`; that change is overly broad and is lost when the
device reconnects.

## 3. Install the ROS 2 driver

On the Ubuntu 22.04 / ROS 2 Humble Jetson:

```bash
sudo apt update
sudo apt install -y ros-humble-rplidar-ros
```

Verify the installed executables:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg executables rplidar_ros
```

Expected executables include `rplidar_node`, `rplidar_client`, and
`rplidar_composition`.

## 4. Start the A1 manually

The robot uses ROS domain 42. Start the lidar with the stable serial path:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch rplidar_ros rplidar_a1_launch.py \
    serial_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 \
    serial_baudrate:=115200 \
    frame_id:=laser \
    inverted:=false \
    angle_compensate:=true
```

Use the by-id path discovered on the current machine if it differs. The lidar
should begin rotating. Stop it cleanly with `Ctrl-C` when testing is complete.

## 5. Validate `/scan`

In a second Jetson terminal:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

ros2 node list
ros2 topic info /scan
ros2 topic hz /scan
```

Expected results include:

```text
/rplidar_node
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
average rate: approximately 6.98 Hz
```

The exact rate can vary with A1 motor speed. A stable stream without serial or
health errors is more important than matching one decimal value exactly.

## 6. View the scan remotely in RViz2

Keep the lidar node running on the Jetson. On the Ubuntu development PC, enter
the already-running Humble container:

```bash
docker exec -it robomaster-humble bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic info /scan
rviz2
```

In RViz2:

1. Set **Global Options -> Fixed Frame** to `laser` for a lidar-only test.
2. Add a **LaserScan** display.
3. Select `/scan` as its topic.
4. Set its reliability policy to **Best Effort**.

Obstacle points should appear around the sensor in near real time.

## 7. Publish the temporary TF used during validation

ROS navigation needs the lidar located in the robot frame. Before the physical
mounting offset was measured, an identity transform was used only to validate
the TF and RViz path:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 \
    --yaw 0 --pitch 0 --roll 0 \
    --frame-id base_link \
    --child-frame-id laser
```

With this process running, set the RViz fixed frame to `base_link`. The missing
frame warning should disappear.

The zero transform is not a permanent mounting calibration unless the lidar is
actually centered and aligned at the `base_link` origin.

## 8. Measure the permanent transform

Before navigation or mapping, measure the lidar origin relative to the chassis
`base_link` using ROS coordinates:

```text
x: positive forward
y: positive left
z: positive upward
yaw: positive counter-clockwise when viewed from above
```

Record the values in metres and radians. Replace the temporary zeros with those
measurements and verify orientation against a known wall or object.

## 9. Remaining deployment work

The following items were intentionally left for the next integration session:

- Measure and validate the physical `base_link -> laser` transform.
- Add a repository-owned launch file containing the serial and TF parameters.
- Add an automatic Jetson systemd service with `ROS_DOMAIN_ID=42`.
- Verify boot, USB disconnect/reconnect, and clean shutdown behavior.
- Save an RViz configuration for chassis and lidar visualization.
- Add mapping or navigation only after trustworthy TF and odometry are
  available.

Do not treat a visible scan as odometry. The current chassis driver does not
yet publish validated wheel odometry, and that work remains separate from the
lidar driver.

## Troubleshooting

### No USB serial device

```bash
lsusb
sudo dmesg -w
```

If reconnecting produces no USB event, try a known data-capable cable, a direct
Jetson USB port, and the original lidar interface board.

### Permission denied opening the serial port

```bash
ls -l /dev/ttyUSB0
id
```

Confirm the device group is `dialout` and the running user has a fresh login
session containing that group.

### `/scan` exists but RViz reports a missing frame

The lidar driver places `laser` in the message header but does not define its
relationship to `base_link`. Run the static transform publisher and use
`base_link` as the RViz fixed frame, or use `laser` temporarily for a
lidar-only view.

### Scan appears mirrored or rotated

Check the physical sensor orientation and the `inverted` parameter. Correct
the measured static-transform yaw in one place; do not scatter sign changes
across navigation configuration.
