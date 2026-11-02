# RoboMaster navigation prototype — Isaac Sim 5.1

Run on **ubuntu@192.0.2.33**, from `/home/ubuntu/robomaster-s1-jetson`.
Isaac Sim is installed at `/home/ubuntu/isaacsim`; the host uses ROS 2 Jazzy.
All these launch modes force **ROS domain 43**. Hardware remains on domain 42.
Do not use `run_remote_rviz.sh` for this simulation: that viewer is for hardware.

## What this implements

The repaired RoboMaster chassis geometry is placed in a 10×8 m indoor maze.
The outer and internal walls are 6 cm thick and 1 m high. Four alternating
partitions create a winding route, with four side walls adding dead ends.
Main corridors are approximately 2 m wide; alternating end passages are about
1.7 m wide. The robot starts at (-4, -3), facing north. A goal at (4, -3)
exercises the route through the maze. Walls use the same box geometry in
rendering, collision checks, the map and the ideal LiDAR.
Global routes use Smac 2D to handle the long winding maze path.
Nav2 uses Regulated Pure Pursuit to turn toward the path and drive forward,
with lateral navigation velocity held at zero. The simulation still supports
mecanum motion for other controllers. Navigation uses a 0.32×0.24 m reference
footprint with 5 cm padding. Both costmaps include the known walls and live
LiDAR, with a 1 m inflation radius to favor clearance and slow near walls.
A command watchdog stops motion after 0.35 seconds without fresh commands.

The single-goal navigation tree (`config/robomaster_navigate.xml`) bounds recovery
to three outer retries using costmap clearing, a checked 20 cm backup and a short
wait. It does not request recovery spins; the controller still turns to face the
path and the final goal heading. A blocked route may therefore return failure
instead of repeatedly spinning. See the [Nav2 controller documentation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/).

For live diagnostics on domain 43:

```bash
ros2 service call /simulation/status std_srvs/srv/Trigger '{}'
```

The response includes actual pose, body velocity and the number of commands
rejected by the simulator's collision guard.

Validated on the simulation host on 2026-10-31 with the maze and camera panel running: driving
from (-4, -3) to (4, -3) returned `SUCCEEDED` / error code 0. Recorded body lateral
velocities were zero, the collision guard rejected no commands, and Nav2 logged
no progress failures or recovery spins. A subsequent goal inside the wall at
(3, 0) returned `ABORTED` and left the robot stopped, also without recovery spins
or collision-guard rejections. This validates this maze route, not all possible
obstacle arrangements or unmeasured hardware dynamics.

**This is a navigation prototype, not calibrated wheel dynamics.** The chassis
uses ideal planar mecanum integration and a swept-in-small-steps rectangle
collision guard. There are no tyre slip, roller contacts, torque limits or battery
models. Joint-state velocities are ideal inverse-kinematic values. The rendered
wheel geometry does not establish physically realistic mecanum motion.

In the default predefined-map mode, localization is perfect simulation pose: `map -> odom` is identity and
`odom -> base_link` is computed from the ideal model. No AMCL or SLAM is needed
for this initial navigation test. The map and ideal 2D scans use the same box
geometry as the room shown in Isaac. The scan is an analytic box intersection,
not a calibrated RTX RPLIDAR noise model. Cameras are actual Isaac-rendered RGB
views at 320×180, target 5 Hz. They have nominal field of view and baseline, not
measured stereo intrinsics or hardware exposure timing.

## Sensor measurements still required

`config/robomaster_sim.json` contains **explicitly provisional** sensor mounts
and a stock-reference footprint, separate from `isaac_robot_measurements.yaml`.
The launcher refuses to use provisional mounts unless `--provisional` is supplied.
Using the user measurements recorded on 2026-09-13, LiDAR is at (0.005,0,0.250) m
and the camera midpoint at (0.090,0,0.170) m with 0.060 m lens separation.
These positions assume a ground-plane origin at the midpoint of the stock
320 mm length and the same front edge used for the measurements. Hardware
base_link alignment and LiDAR scan yaw remain unverified; simulated LiDAR yaw
stays zero. Camera orientation was reported as straight forward and level.
Update the positions, camera roll/pitch/yaw, LiDAR yaw and status when measured.
This 2D scan prototype assumes a horizontal LiDAR; do not use it for a tilted
sensor without extending the model. Camera offsets use ROS axes +X forward,
+Y left, +Z up; angles are radians. Camera baseline lies along the configured
camera assembly's Y direction (default level/forward-facing mounting).

## Stereo-assisted SLAM

```bash
bash scripts/run_navigation_demo.sh --slam --provisional
```

This starts RTAB-Map before Nav2 and waits for its first occupancy map. Both
rendered cameras publish matching timestamps and pinhole `CameraInfo` derived
from the simulation field of view, resolution and baseline. The ideal cameras
are distortion-free and parallel, so these images are already rectified. A
`stereo_sync` node bundles each exact-time pair; RTAB-Map combines it with `/scan`.
Stereo features support place recognition and loop closure, while LiDAR creates
the 2D occupancy map (`Grid/Sensor=0`). This mode still uses ideal simulation
odometry, **not camera-only visual odometry**. Loop closure quality depends on
visible texture and revisiting recognisable views; enabling SLAM is not evidence
that a loop closure has occurred.

RTAB-Map alone publishes `/map` and `map -> odom`. The predefined room map moves
to `/simulation/reference_map` in the odom frame and is not used by Nav2.
Choose nearby goals in observed free space with Nav2 Goal and work around the
room as the map grows. Unknown space is not traversable with the current planner.
Each launch starts a fresh database at the printed log directory's `rtabmap.db`;
old session databases are preserved. Stop via the launcher so it can save cleanly.

The real IMX219 pair still requires measured intrinsics, stereo extrinsics,
rectification and verified capture synchronization before using this on hardware.
Simulation calibration must not be copied to the real cameras.

Configuration: `config/robomaster_slam.yaml` and `config/robomaster_slam.launch.xml`.
Reference: [RTAB-Map ROS 2](https://github.com/introlab/rtabmap_ros/tree/ros2).

Validated on the simulation host on 2026-09-11: a fresh sensor-built map was published by
RTAB-Map alone, calibrated stereo pairing was active, and Nav2 reached
(-1.2,1.5) with `SUCCEEDED`. RTAB-Map's stored local-map count grew from 1 to
19 during the trip. Visual loop closure and camera-only odometry were not
validated by this test.

## Run

Start the complete stack from this repository on your PC, or from
`/home/ubuntu/robomaster-s1-jetson` on the simulation host:

```bash
bash scripts/run_navigation_demo.sh --provisional
```

From your PC this connects over SSH. Isaac and RViz windows appear on the simulation host's
desktop. The launcher waits for Isaac and Nav2 readiness, then opens RViz.
The combined launcher also enables Isaac WebRTC streaming. In the Isaac Sim
WebRTC Streaming Client on your PC, connect to `192.0.2.33` (signaling port
`49100`). This streams the Isaac window; RViz remains on the host desktop.
The **Robot Cameras** panel inside the Isaac window shows the live left and
right sensor feeds alongside the maze view, using the existing camera renders.
Set `ROBOMASTER_CAMERA_PREVIEW=0` on the simulation host to hide this panel.
Set `ROBOMASTER_WEBRTC=0` on the simulation host to disable streaming.
Press Ctrl-C in the launch terminal to stop all three components.
Closing RViz (or a viewer crash) leaves Isaac and Nav2 running. Reopen it with
`bash scripts/run_isaac_navigation.sh rviz` on the simulation host.
Logs are
saved under `~/.cache/robomaster-navigation/run-*` on the simulation host. Close any previous
manual simulation/Nav2 launch terminals before using this combined launcher.

For separate terminals instead:

Use three terminals on the simulation host's desktop, each starting with:

```bash
cd /home/ubuntu/robomaster-s1-jetson
```

Terminal 1 — simulator (explicitly accept approximate sensor positions):

```bash
bash scripts/run_isaac_navigation.sh sim --provisional
```

First launch can take a few minutes to compile camera shaders. Wait for
`RoboMaster planar navigation prototype ready on domain 43`.

Terminal 2 — navigation:

```bash
bash scripts/run_isaac_navigation.sh nav2
```

Wait for `Managed nodes are active` before sending a goal.

Terminal 3 — simulation RViz:

```bash
bash scripts/run_isaac_navigation.sh rviz
```

Use **Nav2 Goal** to click a free-space target and drag its heading. There is
no initial-pose step because localization is perfect in this prototype. The
robot starts at (-2,0) m. A useful first target is (2,0) m, across the central
obstacle. Alternatively, from a fourth terminal:

```bash
bash scripts/run_isaac_navigation.sh goal 2 0
```

To add/remove a simulated obstruction absent from the static map:

```bash
bash scripts/run_isaac_navigation.sh blockage true
bash scripts/run_isaac_navigation.sh blockage false
```

The service refuses to spawn a blockage overlapping the robot. Watch `/scan`,
the local costmap and `/plan` to see how the obstruction is detected and avoided.
Stop a goal in RViz with **Cancel**, or stop Nav2 with Ctrl-C; the simulation's
watchdog then stops the robot. Close each launch terminal with Ctrl-C when done.
Closing only RViz does not stop navigation.
When restarting the simulator, stop and restart Nav2 too, so its state and
transform buffers start with the new simulation clock.

## Topics and files

Validated on the simulation host on 2026-09-10: Nav2 reached (2,0) from (-2,0), then
returned to (-2,0) with the extra blockage enabled; both actions reported
`SUCCEEDED`. Both 320×180 camera streams, LiDAR, odometry and RViz footprint
subscriptions were verified. Five planar-geometry tests passed. These checks
validate this ideal navigation prototype, not real-robot motion or sensor calibration.

- `/clock`, `/map`, `/odom`, `/tf`, `/tf_static`: simulation time and perfect pose.
- `/scan`: ideal 360-beam scan, 0.15–12 m range, target 10 Hz.
- `/stereo/{left,right}/camera/image_raw`: rendered camera images.
- `/cmd_vel`: final Nav2 commands; `/cmd_vel_nav`: commands before smoothing.
- `/joint_states`: ideal wheel velocities; `/simulation/robot_marker`: RViz footprint.
- `config/robomaster_nav2.yaml`: footprint, clearance, velocity limits and controller.
- `simulation/planar.py`: motion, scan geometry and collision checks.
- `simulation/isaac_navigation.py`: Isaac rendering and ROS simulation interface.

Changing footprint dimensions requires updating both `robomaster_sim.json` and
`robomaster_nav2.yaml`. Camera previews are local to the simulation host by default; run this
Jazzy RViz preset on domain 43 if viewing from a different PC.

## Existing asset preservation

The original URDF/USD remains in `/home/ubuntu/robomaster_isaac/urdf`. Its first
USD import had empty chassis mesh objects, a fixed root, cylinder wheels and an
empty sensor layer. The repaired import resolves the original URDF's mesh paths
against `jeguzzi/robomaster_ros` commit
`c05a39d7f0fa8b3b277aa74826aa92e202efc987`. Assets and MIT license are in
`vendor/` on the simulation host; generated USD is in `assets/robomaster/`. The repaired
composed robot contains 20 mesh instances. One empty base-link visual reference
can still emit an importer warning; chassis meshes are present.

`repair_isaac_robomaster.py` rebuilds this import using Isaac's standalone launcher.
The paused `config/robomaster_inspection.usda` remains available for inspection.
Carter is no longer used by the launcher, map or navigation parameters.
