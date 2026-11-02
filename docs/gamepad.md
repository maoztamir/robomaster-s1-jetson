# Gamepad control after Jetson reboot

## Optional parked sensor sleep

**Current hardware testing state (October 12): disabled.** The user disabled
the manager after sleep/wake interrupted localization. LiDAR runs continuously,
the chassis sleep override is removed, and `sensor_sleep_guard` is false.
The optional implementation below is retained for future work. To return to
the current testing state after an installation, with the robot parked and L1
released, run on the Jetson:

```bash
sudo bash /home/ubuntu/robomaster-s1-jetson/scripts/disable_sensor_power.sh
```

This also removes the sensor-only sudoers entry, restarts the chassis without
its sleep gate, and enables/starts LiDAR. It preserves the 7W power mode and
camera configuration. The standard mapping/localization/navigation workflows
do not depend on the manager while it is disabled.

With the robot parked, sticks centred and L1 released, install on the Jetson:

```bash
sudo bash scripts/install_sensor_power.sh
```

The optional manager stops the cameras and LiDAR after five minutes without
nonzero drive commands, observed odometry movement, an active navigation goal,
or a stereo keep-awake lease. Gamepad commands wake only the LiDAR. The chassis
guard holds zero wheel commands until two fresh scans arrive; a missing manager
heartbeat or stale scans blocks movement. Navigation goals also require a fresh
`localization_map` to `laser` transform. It does not automatically recover a lost
map alignment. After sensor wake, check wall alignment before navigation.

The A1 adapter's DTR motor-stop line remains asserted with the serial port open
throughout sleep. Stopping the ROS driver alone was physically tested on October
12 and left the motor spinning, despite its stop log. The serial hold is released
before starting the driver on wake; it does not cut power to the whole USB device.
For a parked diagnostic, `/robomaster/sensors/sleep_now` (`std_srvs/srv/Trigger`)
requests sleep immediately, rejecting requests during service operations, recent
drive commands, stereo use or active goals. Normal inactivity remains five minutes.
RViz `/localization/initialpose` requests also wake LiDAR and reset the idle timer,
so a stationary robot can be aligned without gamepad input. Otherwise a paused
scan stream can leave the displayed estimate stale even though AMCL receives poses.

The installer builds the ROS package, adds a chassis systemd override enabling
the guard, and grants ubuntu only four exact sensor start/stop commands through
`/etc/sudoers.d/robomaster-sensors`. Existing gamepad scaling stays unchanged.
The current mapping/fusion launchers do not send sleep-manager leases. If you
explicitly reinstall the optional manager, keep sensors awake during stereo use:

```bash
ros2 service call /robomaster/sensors/keep_awake std_srvs/srv/SetBool '{data: true}'
```

Call again with `data: false` when finished. Stopping the power-manager service
alone blocks driving because the guard remains enabled. To uninstall, with the
robot parked and L1 released, stop teleop, disable the manager, remove the
`sensor-sleep.conf` chassis override, reload systemd and restart the chassis and
teleop. Keep the CAN/chassis heartbeat running during sensor sleep.

## Standard gamepad services

On the Jetson, with the robot lifted and L1 released, install once:

```bash
cd /home/ubuntu/robomaster-s1-jetson
sudo bash scripts/install_gamepad_services.sh
```

The installer enables CAN, chassis, gamepad input, and teleop at boot. It refuses
to start over manual ROS drive nodes. ROS 2 Humble, the built chassis package,
joy_linux and teleop_twist_joy must already be installed.

The installer also enables `NetworkManager-wait-online.service`; all three ROS
control services wait for `network-online.target` and load
`config/calibration_dds.xml` for explicit domain-42 DDS discovery, including
loopback peers. The profile currently includes Jetson address `192.0.2.10`.
Reserve that address in the router or update the profile if it changes.
This addresses the September 14 startup ordering gap: joy and teleop launched
before Wi-Fi was configured, and manual service restarts restored command
delivery. After the installer and reboot instructions were provided, the user
confirmed working operation on September 14; this is user-reported verification.

After reboot, turn on the paired PS4 controller using its PS button. The reader
waits for `/dev/input/js0` and retries when it disconnects. This configuration
assumes this is the only joystick. The controller was paired and trusted on
2026-09-10. A changed boot ID and all four services enabled and active verified
startup after reboot that day; the user reported "seems to be ok". Bluetooth
disconnect stopping and reconnect behavior still need a dedicated physical test.

Hold L1 to drive: left stick controls forward/backward and sideways sliding;
right stick controls turning. Release L1 to stop. Keep sticks centered and L1
released when connecting. All wheel commands are capped at 30, including combined
stick inputs. Scales are abstract commands, not calibrated metres per second.
The chassis command timeout is 0.35 seconds after velocity messages stop.

`linear.x: 0.1` becomes raw wheel command 3 and produced no visible motion in
the September 14 check. `linear.x: 1.0` becomes 30 and did produce forward
motion; the user then confirmed gamepad operation. Do not interpret these as
measured physical speeds or increase the wheel-command limit to fix discovery.

After installing and rebooting, wake the controller with centered sticks and
L1 released. Check `systemctl is-active robomaster-chassis robomaster-joy
robomaster-teleop`, then inspect `/robomaster/joy` and `/cmd_vel`. With wheels
lifted, holding L1 should produce commands and releasing it should stop motion.
An active service and a listed topic alone do not verify message delivery.

Inspect and stop control:

```bash
systemctl status robomaster-chassis robomaster-joy robomaster-teleop
journalctl -u robomaster-joy -u robomaster-teleop -u robomaster-chassis -f
sudo systemctl stop robomaster-teleop robomaster-joy robomaster-chassis
```

Stopping services does not disable next-boot startup. To disable it:

```bash
sudo systemctl disable --now robomaster-teleop robomaster-joy robomaster-chassis
```

Before floor use, test L1 release and controller disconnect with the wheels lifted,
then verify reconnect with released controls and no unintended movement.
