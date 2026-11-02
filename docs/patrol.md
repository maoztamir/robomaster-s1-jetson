# Map-free patrol in Isaac Sim

This example drives forward, turns toward free space when blocked, and occasionally
chooses a random turn after 25–45 seconds of clear travel. Obstacles can trigger
an earlier turn. It uses `/scan` only: no saved map, SLAM, or navigation goals.
It does not guarantee complete coverage or escape from every confined space.

On the Isaac host (`192.0.2.33`), stop the previous navigation demo with Ctrl-C.
In a fresh terminal start the existing scene:

```bash
cd /home/ubuntu/robomaster-s1-jetson
DISPLAY=:1 bash scripts/run_isaac_navigation.sh sim --provisional
```

Once the scene is playing, use another terminal:

```bash
cd /home/ubuntu/robomaster-s1-jetson
bash scripts/run_patrol_sim.sh
```

Watch the robot in the Isaac viewport. Patrol runs continuously (`duration:=0.0`).
Isaac Pause/Play pauses and resumes movement. Ctrl-C in the patrol terminal stops
the controller; the scene stays open. Run the command again to restart it.
The launcher always selects simulation domain 43. Hardware uses domain 42.
Do not run Nav2 or keyboard teleoperation alongside patrol.

Defaults: forward 0.15 m/s, turn 0.35 rad/s, obstacle threshold 0.75 m,
all-around turning clearance 0.4 m relative to the scanner. Missing, stale, or
incomplete scans stop motion. Another velocity publisher also suppresses patrol
commands, but this is not a command multiplexer or a hardware emergency stop.

The scene uses ideal planar motion and analytic LiDAR. Passing this example does
not validate physical braking, sensor mounting, low obstacles, or wheel dynamics.
Hardware deployment remains pending measured sensor orientation, footprint and
command scaling, and a supervised trial. No hardware startup service is installed.

To stop a patrol started in the background, run `bash scripts/stop_patrol_sim.sh`.
To open RViz alongside Isaac, run `bash scripts/run_isaac_navigation.sh rviz` on
the simulation host. Its map display shows the scene reference map; patrol does
not consume it or build a new map. The watchdog uses a steady clock so pausing or
closing Isaac also stops motion commands while the controller waits for fresh scans.

## Jetson deployment (September 15)

The controller modules and `scripts/run_patrol_robot.sh` are installed in
`/home/ubuntu/robomaster-s1-jetson` on the Jetson. No boot service is installed.
Run `bash scripts/run_patrol_robot.sh --check` for a 15-second scan-processing
check. It publishes only to `/patrol/check_cmd_vel`, leaving driving services alone.

The manual `--drive` mode is a two-minute supervised trial. Defaults are
scanner yaw 0, forward command 0.5, turn command 0.5, turning clearance 0.4 m,
and forward threshold 0.75 m. Short supervised pulses produced approximately
10 cm forward and 20 degrees left respectively. These are initial trial settings,
not calibrated physical velocities or validated braking distances. Override with
`PATROL_LIDAR_YAW`, `PATROL_FORWARD_COMMAND`, `PATROL_TURN_COMMAND`,
`PATROL_RADIUS`, and `PATROL_CLEARANCE` environment settings if needed.
The launcher stops teleop with sudo and restores it on exit if previously active.
Keep the gamepad deadman released when changing modes. Continuous operation
remains enabled only by the simulation launcher until hardware trials are complete.

LiDAR coverage: finite returns beyond the advertised maximum are saturated at
that maximum for local obstacle checks. This handles observed RPLIDAR readings
around 12.4 m with a declared 12 m limit. Missing sectors, NaN, negative, and
below-minimum readings are not treated as clear space; insufficient valid coverage
still stops motion. `duration complete` is the normal end of the timed trial.

## Patrol during stereo mapping

Use `bash scripts/run_patrol_robot.sh --mapping` for mapping trials. Start
`scripts/run_hardware_stereo_mapping.sh` on the laptop first. Its tracking relay
publishes the original OdomInfo stamp and tracking validity (not lost and at
least 20 inliers) on `/patrol/tracking_status`. The Jetson waits for fresh healthy
tracking before moving. A reported loss, stale source timestamp, or more than
1.5 seconds without an update latches a stop; tracking recovery alone cannot
resume movement. Ctrl-C ends the session and restores gamepad control, then
restart the session after tracking is healthy. The 120-second trial limit remains.
`--drive` is standalone LiDAR patrol and does not check visual tracking.
This guard prevents continued mapping patrol after reported loss; it does not
repair stereo tracking or detect all incorrect poses that still report healthy.
