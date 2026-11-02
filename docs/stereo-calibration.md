# Real IMX219 stereo calibration preparation

The hardware mapping launcher uses reliable QoS for local rectified images
(`qos_image:=1`), matching the rectifier's reliable publishers. CameraInfo stays
best effort (`qos_camera_info:=2`) to match the Jetson camera publishers. In a
40-second stationary timing check, compressed input and rectification were
steady at 10 Hz, but best-effort odometry image subscriptions yielded 5.83 Hz
with a 1.30 s maximum output gap. With reliable image subscriptions, odometry
reached 10.00 Hz with a 0.179 s maximum gap. This validates stationary delivery;
it does not yet validate a moving map. Evidence is under
`output/mapping/timing/` and `output/mapping/reliable/` (ignored by Git).

Mapping timing investigation (September 14): the long loop had 68 frame-drop
warnings and odometry update gaps as large as 3.44 s. The launcher now sets
`odom_always_process_most_recent_frame:=false`, making odometry process callbacks
synchronously instead of dropping arrivals while its worker is busy. A 35-second
stationary check produced 233 updates, one lost update and no such warnings.
This is not proof of improved moving-map accuracy; excessive processing load can
still cause queueing or dropped frames. The existing calibration is unchanged.

Generated archives, calibration YAML files, reviews, screenshots, and session
data live under `output/calibration/`, which is excluded from Git. Tools and
printable board templates remain under `calibration/`. On a fresh checkout,
restore the output directory from your backup before running calibration review
or starting calibrated cameras. Copy `output/calibration/refined-20260913/`
to the same relative location on the Jetson before deploying the updated camera
launch file, or override `calibration_dir` to the existing deployed directory.
The running Jetson installation was not moved by this repository cleanup.

## First hardware mapping run (prepared; runtime verification pending)

On the Dell laptop, install the missing package:

```bash
sudo apt install ros-jazzy-rtabmap-ros
```

Close stereo preview windows, exit Conda, then from the project directory run:

```bash
bash scripts/run_hardware_stereo_mapping.sh
```

This uses the [upstream RTAB-Map stereo launch](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_launch/launch/rtabmap.launch.py)
with camera-based odometry. Keep the robot stationary for initial verification;
check that odometry and map updates arrive before attempting a slow manual run.
Maps are saved to unique databases under `maps/hardware/`. Stop with Ctrl+C.
The launcher has passed shell syntax checks but awaits package installation
and a live RTAB-Map test. It does not launch navigation.

Both cameras and restored LiDAR scans were verified on the Jetson. No hardware
odometry publisher was found. This initial mapping mode uses stereo only;
LiDAR fusion needs its currently unverified mounting yaw and chassis transform.

## Stationary stereo depth test

Close the rectified preview with Q, then on the Dell laptop outside Conda run:

```bash
bash scripts/run_stereo_rectified_preview.sh --depth
```

Keep the robot still. The left panel is the rectified left camera; the right
panel is estimated depth: red is near (0.3 m), blue far (5 m), black unknown
or outside that display range. Use textured boxes/books at different distances.
The centre box reports median estimated depth when enough pixels are matched.
Compare a stationary object's estimate to a tape measurement from the camera
lenses, then repeat farther away. Watch for large fluctuations without motion.

The overlay shows accepted pair timestamp gap, last result age, and fraction
of pixels with estimates in range. A growing age means results have stopped.
Frames are used once, with a bounded queue and maximum 30 ms header timestamp
gap; this is approximate pairing, not synchronized exposure. Statistics are
printed on exit. Q or Escape closes the test. It publishes diagnostic float
depth in metres on `/stereo/depth/image`, with NaN for unknown pixels.
This topic is not connected to navigation or motor control.

Implementation uses [OpenCV StereoSGBM](https://docs.opencv.org/4.13.0/d2/d85/classcv_1_1StereoSGBM.html)
and the published rectified projection matrices. Matching filters reduce bad
estimates but do not prove accuracy. Passing this stationary test does not
validate depth during motion or authorize autonomous navigation.

## Live rectified preview

The recovered calibration is now installed on the Jetson (192.0.2.10).
Both gscam publishers load the 640x360 YAML files and publish calibrated
`/stereo/{left,right}/camera/camera_info`. Physical left is sensor 1.

On the Dell laptop, outside Conda, run:

```bash
bash scripts/run_stereo_rectified_preview.sh
```

The Jazzy launcher uses this project's Python environment and installed ROS/
OpenCV packages. The window shows both rectified images with horizontal guides;
press Q or Escape to close. Keep the robot still and check that matching scene
features lie at the same height. The views show each camera's latest image,
not synchronized stereo pairs. This preview does not estimate depth or run SLAM.
It also publishes `/stereo/left/camera/image_rect` and
`/stereo/right/camera/image_rect` for RViz Image displays while it is running.

The previous Jetson launch file is backed up beside the active file as
`stereo_camera.launch.xml.before-calibration-20260914`. Calibration paths can
be overridden with the launch argument `calibration_dir` on other hosts.

Latest review (September 14): all 41 new image pairs were checked. The new
model gives a 62.00 mm baseline and 1.019 px stereo vertical RMS; the previous
recovered model gives 0.976 px on those same images. Keep the recovered model
as the rectification candidate for that development robot. The comparison data
is private and not distributed. These results do not calibrate a new camera pair.

## Recovered result from existing captures

The development deployment used private calibration models with a 60.33 mm
baseline, 0.314 px retained-training alignment RMS and 1.793 px reserved-view
RMS. These measurements are historical evidence, not supplied camera models.
The source snapshot excludes those models and all captured images. Calibrate
your own pair and validate reserved views before using rectification or depth.
The launcher's default `output/calibration/refined-20260913/` path is only a
directory convention; it is not populated by cloning this repository.

## Native Jazzy on the Dell laptop

The native launcher now loads `config/calibration_dds.xml` for explicit Jetson
and local discovery, matching the discovery fix used in Docker. Both decoded
640-pixel image inputs were received in a native test on 2026-09-13; the check
also reported dropped frames. Keep the board still while samples are accepted.

```bash
bash scripts/run_stereo_calibration.sh 8x6 0.025
```

Run outside Conda and keep only one calibration session open. Native SAVE uses
`/tmp/calibrationdata.tar.gz`; copy it into the project before another run.

## Working laptop launch (Humble container)

After the camera publisher restart, native Jazzy discovery was unreliable.
Use the existing Humble image on the Dell laptop with direct Jetson discovery:

```bash
bash scripts/run_stereo_calibration.sh --docker 8x6 0.025
```

The container uses host networking (approved by the user), domain 42 and
`config/calibration_dds.xml` to contact Jetson 192.0.2.10 directly. That file
lists domain-42 discovery ports for participant indices 0–63. The small C++
decoder receives JPEG camera feeds using best-effort QoS and publishes decoded
images for calibration. It builds inside the existing Humble container.
Close any old native calibration session before starting this command.

**SAVE writes directly to `output/calibration/session-XXXXXXXX/calibrationdata.tar.gz`**
on the laptop; the exact directory is printed at startup. No copying from host
`/tmp` is needed in Docker mode. Session directories also hold temporary build
files and are excluded from Git; preserve the calibration archive separately
for backup. Old calibration archives are not overwritten.

## Print the checkerboard

Use [the supplied A4 PDF](../calibration/boards/checkerboard-8x6-25mm-a4.pdf)
or its [vector SVG source](../calibration/boards/checkerboard-8x6-25mm-a4.svg).
It contains 9×7 alternating squares, which provide 8×6 internal corner
intersections. Each square is designed to be 25 mm; the pattern is 225×175 mm.
The calibration tool uses these known intersections to estimate lens distortion
and the relative camera geometry. An ordinary scene is not a calibration board.

1. Open the PDF in a PDF viewer and print one page on **A4, landscape**.
2. Select **Actual size / 100%**. Disable Fit, Shrink, scaling and multiple pages
   per sheet. Use clear black printing on matte white paper.
3. Measure the print with a ruler. Four consecutive square widths must span
   **100 mm** in both directions; the full pattern should be **225×175 mm**.
   If it is scaled or stretched, correct the print settings and reprint.
4. Mount the sheet flat on rigid foam board or another flat backing. Avoid
   wrinkles, curved cardboard, glossy lamination and glare. Keep the white
   margin around the pattern.
5. Start calibration with `8x6 0.025` only after checking that printed scale.

## Capture calibration views

Keep the robot stationary and illuminate the board evenly. Begin with the whole
board visible in both camera views; move it closer until its corners are clearly
resolved while keeping the edges inside both images. Coloured corner markers
indicate successful detection. If absent, check the board count, lighting, focus,
glare and whether both cameras see all corners.

Move the board across the shared view (left/right/top/bottom), closer/farther,
and tilt it about both axes. Pause roughly two seconds at each pose so the
independent cameras observe a stationary target. Aim for varied views, not many
copies of the same pose, and watch X, Y, Size and Skew progress bars. Keep hands
off the pattern. Once CALIBRATE enables, click it and wait; computation can make
the window temporarily unresponsive. Inspect rectified views, click SAVE and
copy the reported archive out of `/tmp`. Do not apply it to the cameras before
reviewing alignment, errors and recovered baseline.

## Launch and prerequisites

Use the real robot's camera publishers (domain 42), not Isaac (domain 43).
The current preview configuration provides compressed 640×360 images at a
target 10 Hz. Keep resolution, crop and camera focus unchanged after calibration.
Physical camera ordering was confirmed on 2026-09-13: **left Argus sensor 1,
right sensor 0**. Left/right are defined looking forward from behind the robot.
The first saved archive `stereo-20260913-165101.tar.gz` has reversed ordering;
its calibration YAML files must not be applied to the corrected streams.
Restart the calibration GUI after changing camera assignments so old and new
samples cannot be mixed.

Start the camera publishers first, in a separate SSH terminal on the Jetson:

```bash
ssh ubuntu@192.0.2.10
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch /home/ubuntu/robomaster-s1-jetson/config/stereo_camera.launch.xml
```

Keep that terminal running during calibration. Do not launch a second copy if
the cameras are already streaming. The laptop calibration launcher decodes
existing streams; it does not start the Jetson cameras.

Run on a workstation, using its native ROS Jazzy and graphical
desktop. The Jetson at 192.0.2.10 supplies camera images; the simulation host remains dedicated
to Isaac. Deactivate Conda first (`conda deactivate` if the prompt shows `(base)`).
Install these prerequisites on the laptop:

```bash
sudo apt install ros-jazzy-camera-calibration ros-jazzy-image-transport ros-jazzy-compressed-image-transport
```

The laptop has image_transport but camera_calibration and compressed_image_transport
were missing during the 2026-09-13 check. The launcher selects an already sourced
Jazzy or Humble environment, otherwise prefers installed Jazzy. It handles the
different image republisher arguments. Humble users should substitute `humble`
in the package names. Live cross-distribution image reception and calibration
remain untested; verify both camera views before collecting samples.

Provide a flat checkerboard, the number of **inner corners** across and down,
and the measured side length of one square. Do not count squares as corners.
Mount a printed board on a rigid flat surface and measure the printed squares;
printer scaling can change their dimensions.

Run from the repository in a fresh terminal, substituting actual values:

```bash
bash scripts/run_stereo_calibration.sh 8x6 0.025
```

The numbers above are an example, not an assumption about your board. The
launcher decodes both compressed streams into separate calibration topics.
Keep the robot stationary. Show the entire board in both images at varied
positions, distances and tilts, holding it still at each pose. The 50 ms
approximate pairing tolerance accommodates independent camera streams; it
does not establish hardware synchronization or suitability for moving scenes.

When enough views have been collected, click CALIBRATE, inspect the rectified
pair, then SAVE. Preserve the archive at the path reported by the tool (normally
`/tmp/calibrationdata.tar.gz`) outside `/tmp` before another calibration run.
Review reprojection/alignment errors and compare recovered baseline with the
measured 60 mm separation before configuring the camera publishers. Saving
calibration alone does not configure rectification or validate synchronization.

Reference: [ROS camera calibration](https://docs.ros.org/en/rolling/p/camera_calibration/index.html).
