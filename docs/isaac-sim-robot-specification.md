# Isaac Sim robot measurement and sensor specification

This document is the source-of-truth worksheet for building a digital replica
of the current RoboMaster S1 platform in NVIDIA Isaac Sim. Values marked
**validated** were observed on the physical robot or come from the component
manufacturer. Values marked **TBD — measure** must be measured on the final
assembled robot; do not replace them with visual estimates.

## Coordinate convention and model origin

Use the ROS REP-103 body convention throughout the USD/URDF model:

```text
+X = forward
+Y = left
+Z = up
+yaw = counter-clockwise when viewed from above
```

Define `base_link` at the geometric centre between the four wheel contact
patches, projected onto the floor. This matches the reusable RoboMaster ROS
model. `base_footprint` may be coincident with `base_link`; the physical chassis
is represented by a fixed `chassis_base_link` above it. Put the measured centre
of mass in the body's inertial pose; do not assume it is at a frame origin.

## Current physical configuration

| Subsystem | Current component | Current state |
|---|---|---|
| Mobile base | DJI RoboMaster S1 chassis and Motion Controller | Direct CAN motion physically validated |
| Drive | Four original 100 mm Mecanum wheels | Forward, lateral and rotation signs validated |
| Computer | NVIDIA Jetson Orin Nano Developer Kit | ROS 2 Humble; chassis service validated |
| Storage | 500 GB-class Kingston NVMe mounted at `/data` | Installed and writable |
| CAN interface | DSD TECH SH-C31A (`gs_usb`) | SocketCAN `can1`, 1 Mbit/s |
| Stereo camera | Waveshare-style IMX219-83 stereo board | Left image validated; right image pending |
| Camera IMU | ICM-20948 on stereo board | Not yet wired/validated as a ROS sensor |
| 2D lidar | SLAMTEC RPLIDAR A1 | `/scan` and remote RViz display validated |
| Chassis feedback | DJI encoders/IMU/battery DDS | Not trustworthy yet; exclude from ground truth |

## Chassis and collision measurements

Before collecting dynamic measurements, complete the
[chassis calibration and keepalive checks](chassis-calibration.md). A stationary
orientation-drift fault was resolved by DJI calibration on September 8, 2026.
Diagnostic attitude readings now respond to movement and showed greatly reduced
stationary drift, but this does not validate all DDS feedback as simulation
ground truth. The evidence is in `controller_calibration_validation` in the YAML.

Measure the assembled robot, including its actual sensor brackets, Jetson,
battery and cables. Record all lengths in metres and masses in kilograms.

| Parameter | Value | How to measure |
|---|---:|---|
| Stock complete S1 envelope | 0.320 × 0.240 × 0.270 | Official DJI specification |
| Stock complete S1 mass | Approximately 3.3 | Official DJI specification |
| Reference chassis-base mass, excluding wheel links | 1.934 | Secondary URDF reference |
| Reference chassis-base centre of mass | (-0.0058789, -0.0001128, +0.0294307) | Relative to `chassis_base_link` |
| Reference chassis-base inertia diagonal | (0.01451, 0.00512, 0.01759) | `ixx`, `iyy`, `izz`, kg·m² |
| `base_link -> chassis_base_link` | (0, 0, +0.03465) | Secondary URDF reference |
| Overall length, `+X` | TBD — measure | Rearmost to foremost collision point |
| Overall width, `+Y` | TBD — measure | Outside of left/right wheel or widest bracket |
| Overall height, `+Z` | TBD — measure | Floor to highest fixed component |
| Ground clearance | TBD — measure | Floor to lowest fixed chassis component |
| Main chassis collision box | TBD — measure | Length × width × height, excluding wheels |
| Total running mass | TBD — measure | Robot ready to drive, including all batteries |
| Centre of mass `(x,y,z)` | TBD — measure | Measure or derive from component masses/CAD |
| Body inertia tensor | TBD — derive | Compute from CAD or measured mass distribution |

The official values describe a complete stock S1, including its original upper
assembly. They are a bounding-box reference, not the wheel-centre geometry of
the modified platform. The Jetson, lidar, camera and custom mounts change the
height and mass distribution.

DJI does not publish the axle-centre wheelbase and track in the public S1 spec
sheet. For those values this project uses the MIT-licensed
[`jeguzzi/robomaster_ros`](https://github.com/jeguzzi/robomaster_ros) physical
model at commit `c05a39d7f0fa8b3b277aa74826aa92e202efc987` as a secondary
reference. Its model includes detailed meshes, collision geometry, mass and
inertia and can be imported or adapted rather than recreating the stock chassis
from an empty box.

Prefer several simple collision shapes over one detailed triangle mesh. Keep
high-detail geometry for visuals and low-detail primitives or convex meshes for
physics and collision.

## Wheel and drivetrain specification

The DJI wheel specification gives a `0.100 m` diameter, 12 rollers and `0.157
kg` per wheel. Verify the effective rolling radius under the robot's working
load because tyre compression affects odometry.

| Parameter | Front right | Front left | Rear left | Rear right |
|---|---:|---:|---:|---:|
| Link name | `front_right_wheel` | `front_left_wheel` | `rear_left_wheel` | `rear_right_wheel` |
| Driver order | `w1` | `w2` | `w3` | `w4` |
| Centre `x` from `base_link` | +0.100 | +0.100 | -0.100 | -0.100 |
| Centre `y` from `base_link` | -0.100 | +0.100 | +0.100 | -0.100 |
| Centre `z` from `base_link` | +0.050 | +0.050 | +0.050 | +0.050 |
| Effective radius | TBD; nominal 0.050 | TBD; nominal 0.050 | TBD; nominal 0.050 | TBD; nominal 0.050 |
| Wheel width | 0.042 reference | 0.042 reference | 0.042 reference | 0.042 reference |
| Wheel mass | nominal 0.157 | nominal 0.157 | nominal 0.157 | nominal 0.157 |
| Joint axis/sign | TBD — validate | TBD — validate | TBD — validate | TBD — validate |
| Roller angle/handedness | TBD — record | TBD — record | TBD — record | TBD — record |

Also record these derived dimensions:

| Drivetrain parameter | Value |
|---|---:|
| Wheelbase: front-to-rear wheel-centre distance | 0.200, secondary URDF reference |
| Track: left-to-right wheel-centre distance | 0.200, secondary URDF reference |
| Roller angle relative to wheel plane | TBD — verify; do not assume |
| Stock maximum motor angular velocity | 1000 rpm / 104.72 rad/s; DJI specification |
| Calibrated safe wheel angular velocity | TBD — calibrate |
| Maximum motor/wheel torque | TBD — calibrate or obtain data |
| Joint damping and friction | TBD — tune against coast-down tests |
| Wheel/roller friction coefficients | TBD — tune against straight and lateral tests |

DJI specifies a maximum M3508I motor speed of 1000 rpm, or approximately
104.72 rad/s. Treat that as a stock hard-limit reference, not as a claim that
the direct-CAN raw command equals rpm. Retain the lower measured/calibrated
limit for the assembled robot once motion calibration is complete.

Isaac Sim's holonomic/Mecanum controller needs wheel radii, wheel positions,
wheel orientations and Mecanum roller angles. The current ROS-to-wheel mixing
is already validated as:

```text
w1/front-right = +vx +vy +wz
w2/front-left  = +vx -vy -wz
w3/rear-left   = +vx +vy -wz
w4/rear-right  = +vx -vy +wz
```

Here `vx` is forward, `vy` is left and `wz` is counter-clockwise. The software
commands are currently abstract raw units, not rad/s or m/s. Create calibration
curves before using the same numerical command in simulation and hardware.

## Sensor inventory

### RPLIDAR A1

| Field | Current value |
|---|---|
| ROS topic/type | `/scan`, `sensor_msgs/msg/LaserScan` |
| Frame | `laser` |
| Transport | CP2102 USB serial, 115200 baud |
| Observed scan rate | Approximately 6.98 Hz |
| Manufacturer scan range | 360 degrees |
| Manufacturer distance range | 0.15–12 m for applicable A1 revisions |
| Manufacturer sample frequency | Approximately 8 kHz |
| Manufacturer nominal angular resolution | At most 1 degree at 5.5 Hz |
| Manufacturer housing size | 0.0968 × 0.0703 × 0.055 m |
| Manufacturer mass | 0.170 kg |
| `base_link -> laser` translation | TBD — measure optical origin, not housing edge |
| `base_link -> laser` roll/pitch/yaw | TBD — measure and validate against a wall |
| Range/noise model | TBD — measure against targets at known distances |

The identity transform used during the first RViz test was temporary and must
not be copied into the simulator as a measured pose.

### IMX219-83 stereo camera

Create three links/frames: `stereo_camera`, `left_camera_optical_frame`, and
`right_camera_optical_frame`. Follow the ROS optical convention (`+Z` forward,
`+X` right, `+Y` down) inside each optical frame.

| Field | Current value |
|---|---|
| Sensors | Two Sony IMX219, 8 MP each |
| Native resolution | 3280 × 2464 per sensor |
| Lens field of view | 83° diagonal, 73° horizontal, 50° vertical (manufacturer) |
| Lens focal length | 2.6 mm (manufacturer) |
| Nominal stereo baseline | 0.060 m (manufacturer); verify physically |
| Hardware frame synchronization | Not provided by this board |
| Validated ROS topic | `/stereo/left/camera/image_raw` |
| Validated stream | 1280 × 720 at approximately 14.33 Hz |
| Right camera topic/rate | TBD — start and validate |
| `base_link -> stereo_camera` pose | TBD — measure all six degrees of freedom |
| Left/right optical poses | TBD — obtain from stereo calibration |
| Intrinsics `K`, distortion `D` | TBD — calibrate both cameras |
| Stereo rectification `R`, projection `P` | TBD — stereo calibration |
| Exposure, gain and latency model | TBD — measure |

Do not use only the nominal 60 mm baseline for depth ground truth. A stereo
calibration must determine the effective left-to-right transform, intrinsics
and distortion of the installed camera pair.

### ICM-20948 camera-board IMU

The board includes a three-axis accelerometer, gyroscope and magnetometer, but
the four-wire I2C connection and ROS publishing have not yet been validated.

| Field | Current value |
|---|---|
| ROS IMU topic | TBD; suggested `/stereo/imu/data_raw` |
| ROS magnetic topic | TBD; suggested `/stereo/imu/mag` |
| Frame | TBD; suggested `stereo_imu_link` |
| `base_link -> stereo_imu_link` pose | TBD — measure sensor axes and location |
| Accelerometer range | Configurable ±2/±4/±8/±16 g |
| Gyroscope range | Configurable ±250/±500/±1000/±2000 deg/s |
| Magnetometer range | Nominal ±4900 µT |
| Publish rate | TBD — validate on Jetson |
| Bias, noise density and random walk | TBD — characterize while stationary |
| Camera/IMU time synchronization | None guaranteed by the camera board |

Do not simulate this IMU as a validated measurement source until its axis
mapping, scale, timestamps, covariance and stationary bias have been checked.

### Chassis-integrated feedback

The existing C++ code contains DJI DDS decoders for wheel encoders, chassis
IMU, battery and velocity. Physical tests produced unreliable values, so these
channels are not currently valid sensor specifications. Isaac Sim may publish
perfect or noise-injected ground truth for development, but it must be named
and documented separately from real hardware feedback.

## Required TF/link tree

```text
base_footprint
└── base_link
    └── chassis_base_link
        ├── front_right_wheel
        ├── front_left_wheel
        ├── rear_left_wheel
        ├── rear_right_wheel
        ├── laser
        ├── stereo_camera
        │   ├── left_camera_optical_frame
        │   └── right_camera_optical_frame
        └── stereo_imu_link
```

Add a separate `jetson_link` only if its mass/collision is modelled separately;
it does not need its own ROS sensor frame.

## Physical measurement procedure

1. Lift or immobilize the chassis and switch motor power off.
2. Mark the floor projection of the four wheel centres.
3. Define and mark `base_link` midway between those points.
4. Measure every wheel centre `(x,y,z)` from that origin.
5. Measure the lidar scan-plane origin and camera lens centres—not enclosure
   edges—from the same origin.
6. Photograph top, front and side views with a ruler in frame.
7. Weigh the complete ready-to-run robot and major removable assemblies.
8. Record measurements to at least the nearest millimetre and note uncertainty.
9. Calibrate both cameras and validate the lidar pose against a straight wall.
10. Perform low-speed motion tests to tune wheel radius, friction and dynamics.

## Minimum viable Isaac Sim replica

Build and validate incrementally:

1. Chassis collision body, mass and four wheel joints.
2. Mecanum wheel geometry/controller with the established wheel order.
3. 2D lidar at the measured transform and observed update rate.
4. Left and right cameras with calibrated intrinsics and transforms.
5. IMU only after its real mounting axes and noise are characterized.
6. ROS 2 bridge topics, using the same frame IDs and coordinate conventions as
   the physical robot.

Useful authoritative references:

- [DJI RoboMaster S1 specifications](https://www.dji.com/ae/support/product/robomaster-s1)
- [DJI RoboMaster S1 Mecanum wheel specifications](https://store.dji.com/product/robomaster-s1-mecanum-wheel)
- [SLAMTEC RPLIDAR A1 specifications](https://www.slamtec.com/en/lidar/a1spec)
- [Waveshare IMX219-83 stereo camera documentation](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)
- [Isaac Sim wheeled robot API](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.robot.experimental.wheeled_robots/docs/index.html)

Enter the measured values in the machine-readable
[`config/isaac_robot_measurements.yaml`](../config/isaac_robot_measurements.yaml)
worksheet as they become available.
