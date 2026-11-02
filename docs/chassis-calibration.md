# Chassis calibration before Jetson-only operation

**Verify chassis calibration before accepting a direct-CAN installation as ready
for motion.** On this robot, incorrect stationary orientation estimates caused
the Motion Controller to leave SDK mode after several minutes, even though the
Jetson continued sending valid heartbeats. Completing DJI calibration with the
original assembly properly installed resolved the observed drift and rollover
failures. The Python driver subsequently passed a ten-minute zero-motion test.

The gimbal/intelligent controller is not required for normal Jetson-only
operation, but retain it for the DJI app calibration procedure. No direct-CAN
calibration command has been validated in this project. This calibration is
separate from stereo-camera calibration and wheel-speed/odometry calibration.

## Recognizing the failure

During the September 8, 2026 investigation:

- The chassis was stationary, but reported pitch and roll changed continuously.
- The indicator initially turned green, then red while the test still ran.
- `can1` remained `ERROR-ACTIVE`, with no bus errors and an empty TX backlog.
- `S1Chassis.connected` remained true because it indicates successful transport
  initialization, not controller acceptance or continued SDK-mode operation.
- Both Python and reference C++ tests reproduced the problem. The often-observed
  195-second transition was not a fixed timeout; one C++ capture showed a
  transition approximately 158 seconds after startup.

Controller diagnostics included:

```text
[L-SA_MOTOR]roll over
[L-N_MIS]Fail to check mis 4. Reset
[L-N_MIS]Exit mis 4. New mis 0
```

Do not dismiss rollover as merely a missing-gimbal warning. In these recordings
it accompanied an explicit controller mode reset. The separate message
`[L-SA_ABN]lost connect with gimbal` persisted during successful Jetson-only tests.

The measured gyro offset was consistent with the observed drift, but its exact
firmware-level cause was not established. The successful intervention was DJI
calibration, not increasing heartbeat frequency, repeatedly forcing work mode,
or subtracting an angle offset in the Jetson display.

## Calibration procedure

1. Stop all manual drive, keepalive, and sensor-probe scripts with `Ctrl-C`.
   Stop the ROS chassis service:

   ```bash
   sudo systemctl stop robomaster-chassis.service
   ```

2. Power the chassis off before changing wiring. Disconnect the Jetson USB-CAN
   adapter so it cannot transmit during calibration. Use a charged chassis battery.
3. Securely reinstall the original gimbal/intelligent-controller assembly in
   its original configuration with the original wiring. A gimbal attached only
   by its cable made the required positioning impractical in our first attempt;
   that attempt did not resolve the drift.
4. Power on and connect the RoboMaster app. Open **Settings → System → Calibration**
   for gimbal and chassis calibration. Start on a level surface and follow the
   app's positioning instructions, including any requested changes of orientation.
   Keep clear of moving parts and secure the Jetson and other added equipment.
5. Wait for an explicit calibration-success indication. Record any failure or
   warning rather than assuming the procedure completed successfully.
6. Power off again. Restore the intended Jetson-only wiring, disconnect the
   original assembly, then power on with the chassis stationary.
7. Perform the stationary and endurance checks below before requesting motion.

DJI documents the app calibration procedure and circumstances requiring
recalibration, including motion-controller reinstallation, in the
[RoboMaster S1 User Manual v1.8, Gimbal and Chassis Calibration](https://dl.djicdn.com/downloads/robomaster-s1/20220429UM/RoboMaster_S1_User_Manual_v1.8_EN.pdf).
The temporary reassembly and direct-CAN verification steps above document this
project's tested workflow. Calibration is not guaranteed to repair every IMU or
controller fault; repeat failures require further diagnosis.

## Verify stationary orientation

On the Jetson, from the repository root, build the diagnostic probe if needed:

```bash
cd /home/ubuntu/robomaster-s1-jetson
mkdir -p build-dds-check
g++ -std=c++17 -O2 -Wall -Wextra -pthread -I include \
    examples/probe_attitude.cpp src/chassis.cpp src/dds.cpp \
    src/protocol.cpp src/crc.cpp src/can_streambuf.cpp \
    -o build-dds-check/probe_attitude
bash scripts/stream_attitude.sh
```

Keep the chassis untouched for at least 30 seconds in one continuous run. Compare
pitch and roll at the start and end, then press `Ctrl-C`. These displayed angles
are in degrees. A fixed offset can reflect mounting or the support surface;
look for persistent drift, not just whether the number is exactly zero.
Restarting the probe resets its elapsed-time display, not the controller's attitude.

For an additional accelerometer/gyro check, stop the attitude probe first:

```bash
bash scripts/stream_imu.sh
```

Leave the chassis stationary for 30 seconds and stop with `Ctrl-C`. The IMU
launcher saves output under `~/can-debug/imu-*.log`. These are raw protocol
values; do not assume their units or use them as calibrated ROS sensor data.
Both probes request telemetry only and send no wheel, work-mode, or heartbeat
commands. Do not run multiple probes simultaneously: they share subscription ID 7.

The C++ DDS callback formerly consumed packet fields in unspecified argument
evaluation order. The corrected decoder reads metadata first, then sensors in
subscription order. Rebuild tools after updating `include/robomaster/dds.hpp`.
Regression tests verify decoding order, but do not establish physical sensor
accuracy or validate every DDS channel.

## Verify continuous controller operation

With all wheels lifted and other manual drivers/probes stopped:

```bash
bash scripts/test_long_keepalive.sh
```

The default is **600 seconds (ten minutes)**. It maintains the Python driver's
legacy heartbeat and zero wheel commands, reports status every 15 seconds, and
records CAN traffic plus interface statistics in `~/can-debug/long-*/`. Keep the
terminal open; `Ctrl-C` stops early. The chassis service remains stopped afterward.

A successful run requires the full duration, continuous outgoing heartbeat and
controller-originated traffic, no rollover or unexpected SDK-mode exit during
the test, and no transport errors or stalled backlog. Record the indicator's
behavior as well. The script's exit code or `connected=True` alone does not
automatically certify these conditions; the CAN recording must be reviewed.

A red indicator, lost-RC messages, and exit from SDK mode **after the final
heartbeat** are expected when the test ends. Treat `low power warning`
separately: charge the battery before evaluating motion.

After successful stationary and endurance checks, use the finite lifted-wheel
tests in [hardware and safety](hardware-and-safety.md#mandatory-first-motion-procedure).
Zero-command endurance does not establish motion, odometry, watchdog, disconnect,
or indefinite-operation reliability.

## Observed results: September 8, 2026

| Check | Observed result |
|---|---|
| Before successful calibration | Stationary roll changed −25.54° and pitch +12.88° in 31.9 s |
| After calibration | Pitch 0.06° → 0.06°, roll −2.26° → −2.36°, yaw −0.15° → −0.11° in 31.8 s |
| Reference C++ keepalive | Four minutes; user observed green until shutdown; no rollover messages |
| Python keepalive, charged battery | Four minutes; no rollover, low-power, or mode-reset messages during test |
| Lifted forward command | User confirmed all four wheels turned for about one second at raw command 30, then stopped |
| Python endurance | 600.04 s; 55,961 heartbeats and 30,609 controller `0x202` packets during test |
| Largest recorded endurance gaps | Heartbeat 15.13 ms; controller `0x202` packets 22.15 ms |
| Endurance errors | No packet CRC errors, bus errors, rollover, low-power, or mode-reset messages during test; empty final TX backlog |

These are observations from this installation, not universal acceptance limits.
The missing-gimbal warning remained present in successful runs. The exact
recording paths and measurements are preserved in
[`config/isaac_robot_measurements.yaml`](../config/isaac_robot_measurements.yaml)
under `controller_calibration_validation`. The YAML records evidence; it does
not apply calibration to the physical controller.
