# Sensor fusion bring-up

Status: in preparation; no live fused odometry is installed or enabled.

Inventory on September 15: live `/scan` and stereo images/CameraInfo, but no ROS
IMU topic. Laptop has robot_localization and imu_filter_madgwick installed.
The existing CAN IMU probe decodes accelerometer and gyro floats at 10 Hz;
old September 8 logs show stationary acceleration Z near -1.04 raw units.
Do not publish those raw values as SI Imu: unit conversion, axis rotation, bias,
clock handling, covariance and mounting frame still require verification.

First prerequisite: on the stationary Jetson run
`bash scripts/check_fusion_imu.sh`. The deployed wrapper pauses teleop and chassis,
records the existing finite 15-second probe, then restores previously active
services. Sudo needs interactive authentication. It sends DDS subscription
commands, no wheel/work-mode commands. Do not run a separate probe concurrently
with the chassis writer: DJI packets share a fragmented CAN byte stream.

Intended pipeline, subject to sensor validation:

1. One chassis/CAN owner handles serialized outgoing commands and IMU subscription;
   validate CRCs and timestamps on incoming telemetry. Publish standard Imu only
   after confirmed axes/units, mark unavailable orientation rather than inventing it.
2. Independent LiDAR scan matching and stereo visual odometry, with a common
   measured body frame and only one owner of the odom-to-body TF.
3. Planar robot_localization EKF uses selected independent measurements and
   realistic covariances. Reject lost visual estimates. Avoid fusing correlated
   derived pose and velocity as if independent; do not constrain lateral speed
   to zero on this mecanum robot. Begin with verified gyro yaw rate. Acceleration
   requires gravity/sign/bias checks before contributing to translation.
4. RTAB-Map uses fused odometry, stereo for visual loop recognition and LiDAR
   for occupancy. Validate trajectory and wall consistency, not only green status.
5. Mapping patrol health must follow the combined estimator and required sensor
   freshness; current --mapping guard specifically monitors stereo OdomInfo.
   Do not silently reuse that guard as proof of fused-estimator health.

Existing camera-only databases and combined bag remain preserved for comparison.
No new driving is required for the first stationary IMU capture.

References:
- https://docs.ros.org/en/rolling/p/robot_localization/
- https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst
- https://github.com/introlab/rtabmap_ros

## Staged yaw-rate publisher

The driver now has optional `enable_imu` (default false). Subscription writes
use the same S1Chassis lock as wheel/heartbeat packets. A read-only SocketCAN
receiver validates packet CRCs and publishes `/robomaster/imu/yaw` at the observed
10 Hz: angular_velocity.z = -raw gyro Z. Orientation and acceleration covariance
are -1 (unavailable); unverified angular X/Y have large covariance. The yaw
variance .0004 is an initial conservative setting, not a complete noise model.
The message uses host receipt time, not unverified controller boot-clock epoch.
No sample means no publication; timestamps are not refreshed without data.
Full receiver recovery after adapter removal still needs validation.

On Jetson, `bash scripts/activate_imu_driver.sh` activates the staged source,
enables IMU in chassis YAML, restarts while stationary, verifies a message and
restores teleop. It backs up the old version and rolls back if verification fails.
Staged files are not active until that command succeeds. Local tests passed
(17 existing driver tests, 3 IMU decoding/subscription tests). Fusion is not yet
running; verify live freshness/rate/sign before enabling the estimator.

## Experimental estimator setup

`bash scripts/run_fusion_odometry.sh` launches independent LiDAR and stereo
odometry, input quality gates and planar EKF on the laptop. This is an odometry
bring-up test, not a mapping/patrol launcher. It uses the existing mapping lock.
Jetson network dropped during first test; all-input validation is pending.
Covariance floors are provisional. Tracking-reset handling and actual transform
consistency must be verified before moving trials. Do not treat EKF output alone
as proof all sensors are contributing. No fused patrol health publisher exists yet.
