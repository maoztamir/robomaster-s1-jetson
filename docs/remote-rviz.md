# Remote ROS2 and RViz2 from another Ubuntu PC

The Jetson only needs ROS2 Humble Base. RViz2 can run on a separate development
PC over the normal ROS2 DDS network.

## Load the repository preset

On the remote PC, from this checkout:

```bash
bash scripts/run_remote_rviz.sh
```

This opens `config/robomaster_remote.rviz` using native Humble when available,
otherwise `osrf/ros:humble-desktop-full` in Docker with host networking and ROS
domain 42. Docker mode requires Docker access, the image already pulled, `xhost`,
and an X11/XWayland desktop. Use `--docker` to select it explicitly. Keep the PC
and Jetson on the same LAN. The script starts only the viewer.

The preset shows `/scan` in the `laser` frame and both cameras using JPEG
compressed image transport. It uses Best Effort sensor subscriptions. The fixed frame deliberately
remains `laser`: the measured sensor mounting transforms and robot odometry are
not available. The grid is sensor-relative, not a map or a robot trajectory.
TF is included as a disabled display for later use. No invented sensor poses
or odometry are supplied.

Live inventory on 2026-09-10:

| Data | Topic | Availability |
|---|---|---|
| Drive command, not measured velocity | `/cmd_vel` | Published by gamepad teleop |
| Gamepad input | `/robomaster/joy` | Published |
| Joystick diagnostics | `/diagnostics` | Published |
| LiDAR scan | `/scan` | Known sensor; publisher stopped at inspection |
| Left camera image | `/stereo/left/camera/image_raw` | Known sensor; publisher stopped at inspection |
| Odometry, wheel feedback, IMU, battery, robot TF | Not yet established | Chassis ROS node does not publish these |

A config file cannot create missing telemetry. Start the LiDAR publisher using
[the A1 instructions](rplidar-a1-ros2.md#4-start-the-a1-manually) and the camera
publisher on the Jetson before expecting images/scans. Until then, RViz's missing
`laser` frame and no-image warnings are expected. Camera calibration is unknown,
so the preset uses Image displays instead of calibrated Camera displays.

Start both IMX219-83 previews on the Jetson:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch ~/robomaster-s1-jetson/config/stereo_camera.launch.xml
```

This requires the installed `gscam` package and Jetson Argus/GStreamer plugins.
It publishes `/stereo/left/camera/image_raw/compressed` and
`/stereo/right/camera/image_raw/compressed` at 640×360 with a target of 10 Hz per stream.
The preview uses JPEG quality 75 and best-effort sensor QoS. Humble RViz uses the
full `image_raw/compressed` topic to select compressed transport, with queue depth 1. Raw image
publishers are not active in this JPEG mode. This reduces network load compared
with the original RGB previews (roughly 110 Mbps for two streams at 10 Hz).
The remote RViz installation needs the compressed image transport plugin; it is
already present in the tested Humble Desktop Docker image.
Sensor IDs default to left=0, right=1 according to the user's wiring; cover one
lens to verify the assignment. Override with `left_sensor_id:=1 right_sensor_id:=0`
if needed. These are independent, uncalibrated previews, not synchronized depth.
Missing calibration-file warnings do not prevent Image displays from working.
This launch is manual and does not persist across reboot.

After updating the preset, reopen RViz or use File → Open Config and select
`/robot-config/robomaster_remote.rviz` inside the Docker viewer to add the second view.

Standard RViz displays do not show numeric Twist, Joy, or DiagnosticArray data.
While the script's Docker viewer runs, inspect these in another PC terminal:

```bash
docker exec -it robomaster-rviz bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic echo /cmd_vel'
docker exec -it robomaster-rviz bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic echo /robomaster/joy'
docker exec -it robomaster-rviz bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic echo /diagnostics'
```

For native Humble, source its setup, export `ROS_DOMAIN_ID=42`, and run the same
`ros2 topic echo` commands directly. Full robot telemetry requires additional
validated publishers; it is not enabled by this viewing preset.

## Use the same ROS distribution

ROS2 does not guarantee communication between distributions. A native Jazzy
installation on Ubuntu 24.04 partially discovered the Humble Jetson and could
send a simple `Twist`, but node metadata was missing and Fast DDS repeatedly
reported:

```text
sequence size exceeds remaining buffer
```

Do not rely on that combination. A related Humble/Jazzy Fast DDS issue can lead
to excessive memory use. Use Humble on both sides of the robot DDS domain.

On an Ubuntu 24.04 PC, keep native Jazzy for unrelated local work and run Humble
Desktop in the OSRF container. Stop the native Jazzy daemon before joining the
robot network:

```bash
source /opt/ros/jazzy/setup.bash
ros2 daemon stop
```

References:

- [ROS2 cross-distribution communication policy](https://docs.ros.org/en/humble/Releases.html#cross-distribution-communications)
- [Fast DDS Humble/Jazzy issue](https://github.com/ros2/rmw_fastrtps/issues/797)
- [OSRF Docker images](https://github.com/osrf/docker_images)

## Start the Jetson node

The installed systemd service already uses domain 42. Verify it:

```bash
systemctl is-active robomaster-chassis.service
```

For a manual run instead:

```bash
source /opt/ros/humble/setup.bash
source ~/robomaster-s1-jetson/ros2_install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run s1_chassis_driver s1_chassis_node --ros-args \
    --params-file ~/robomaster-s1-jetson/config/s1_chassis.yaml
```

## Run Humble Desktop on an Ubuntu 24.04 PC

Install Docker Engine, then pull the OSRF-maintained desktop image:

```bash
docker pull osrf/ros:humble-desktop-full
docker image inspect osrf/ros:humble-desktop-full \
    --format 'Architecture={{.Architecture}} OS={{.Os}}'
```

For an X11 desktop, grant the container's local root user temporary display
access and launch it with host networking:

```bash
xhost +SI:localuser:root

docker run --rm -it \
    --name robomaster-humble \
    --network host \
    -e DISPLAY="$DISPLAY" \
    -e ROS_DOMAIN_ID=42 \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    osrf/ros:humble-desktop-full \
    bash
```

Host networking is required here so DDS multicast and dynamically selected UDP
ports are not hidden behind Docker's default bridge/NAT.

Inside the container:

```bash
echo "$ROS_DISTRO"
ros2 daemon stop
sleep 3
ros2 node list
ros2 node info /s1_chassis_node
```

Expected nodes include `/s1_chassis_node`; its subscribers should include
`/cmd_vel` with type `geometry_msgs/msg/Twist`.

Launch RViz2:

```bash
rviz2
```

The window will initially contain little robot data. The current driver does
not yet publish TF, URDF, odometry, joint states, IMU, or battery feedback.
Seeing and inspecting `/s1_chassis_node` verifies the network connection even
though RViz has no robot model to render.

After exiting the container, revoke the temporary X11 permission:

```bash
xhost -SI:localuser:root
```

## Publish a finite command from the PC

Keep the chassis lifted. From a second host terminal while the container runs:

```bash
docker exec -it robomaster-humble bash -lc '
    source /opt/ros/humble/setup.bash
    ros2 topic pub --rate 20 --times 20 \
        /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
'
```

The finite publisher exits after roughly one second. The Jetson node then
expires the command and sends zero.

## Network troubleshooting

Both computers must be on the same reachable LAN, use domain 42, and allow DDS
multicast/UDP traffic. Diagnose in this order:

1. Confirm normal IP connectivity and SSH.
2. Confirm both environments report `ROS_DISTRO=humble`.
3. Confirm `ROS_DOMAIN_ID=42` in both processes.
4. Stop stale ROS daemons after changing domain or RMW: `ros2 daemon stop`.
5. Check host firewalls and managed-network multicast filtering.
6. Confirm the container uses `--network host`.

Do not try to carry DDS through a basic SSH port forward; DDS uses discovery and
multiple UDP endpoints, not one fixed TCP port.
