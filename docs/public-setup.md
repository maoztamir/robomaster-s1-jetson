# Public deployment setup

This repository is a source snapshot of an experimental robot deployment. The
private development history, session handoffs, camera captures, calibration
archives and measured room maps are not included.

## Configure your machines

The systemd templates and remote commands assume the example account `ubuntu`
and checkout `/home/ubuntu/robomaster-s1-jetson`. If your account or path differs,
update the service templates, installer path checks and remote commands before
installation. The Isaac examples also assume `/home/ubuntu/isaacsim` and a
separately installed Isaac ROS workspace.

Every `192.0.2.*` address in tracked files is a documentation example. It is not
an address you should connect to. On each machine using the launchers:

```bash
cp config/deployment.example.env config/deployment.local.env
```

Edit that local file with your Jetson, workstation and optional simulation-host
addresses. Then generate the local DDS profile:

```bash
bash scripts/configure_deployment.sh
```

`config/deployment.local.env` and `config/calibration_dds.local.xml` are ignored
by Git. Do not commit them. The launchers load the local settings through
`scripts/deployment_env.sh`. Existing exported SSH targets and `ROS_STATIC_PEERS`
can override their defaults. For direct Python invocations, source the helper
first so `JETSON_SSH_TARGET` is exported.

On the Jetson, generate the local DDS profile before installing services. The
service templates reference `config/calibration_dds.local.xml`. For manual ROS
commands, after sourcing the matching ROS distribution:

```bash
source scripts/deployment_env.sh
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROBOMASTER_DDS_PROFILE"
```

Other documentation sometimes shows direct ROS or SSH commands with example
addresses/profile paths. Substitute your local settings there. Hardware uses
domain 42 and simulation uses 43; these IDs are separation conventions, not
authentication. Use a trusted network.

## Install and validate hardware

Follow [hardware and safety](hardware-and-safety.md),
[chassis calibration](chassis-calibration.md), and
[Jetson installation](jetson-orin-nano.md) in that order. Only one process may
control the chassis. Do not run service installers or motion tests on your
development workstation.

Install Python dependencies into a virtual environment. Core driver and unit
tests use `requirements-dev.txt`. Stereo preview/calibration tools additionally
use `requirements-vision.txt` and system ROS packages. On the workstation,
create the vision environment with access to ROS system packages:

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements-vision.txt
```

Source the matching ROS installation before running ROS tools. Hardware service
templates target Humble; mapping/simulation examples target Jazzy. Follow the
guides' cross-distribution limitations rather than assuming compatibility.

## Supply your calibration and maps

Calibrate your own stereo pair using [the calibration guide](stereo-calibration.md).
No captured frames or saved camera models are supplied. The sensor installer
currently expects `left.yaml` and `right.yaml` under
`output/calibration/refined-20260913/`; either install your measured models there
or change the calibration path consistently in the installer and camera launch
file. The directory name is an implementation convention, not a calibration
that ships with this project.

Create your own map before localization/navigation. Pass its YAML path to
`scripts/run_hardware_localization.sh`; the default recorded-map path does not
exist in a fresh clone. Save a new home pose for that map. Never copy someone
else's room map, home pose or motion gains and assume they match your robot.

## Optional simulation assets

The source includes an Isaac navigation program and synthetic maze parameters,
but does not distribute the robot USD or original URDF. You must supply those
assets before running the scene. The default USD path in
`config/robomaster_sim.json` and `config/robomaster_inspection.usda` is an example;
adjust it to your generated model.

`scripts/repair_isaac_robomaster.py` expects a separately supplied
`robomaster_s1_base.urdf` under `/home/ubuntu/robomaster_isaac/urdf/`, and meshes
under `vendor/robomaster_description/`. The recorded mesh source is
[`jeguzzi/robomaster_ros`](https://github.com/jeguzzi/robomaster_ros), commit
`c05a39d7f0fa8b3b277aa74826aa92e202efc987`. Obtain those sources separately,
preserve their licenses, and review the import script's paths before running it
through Isaac Sim. This repository does not provide a complete automated asset
build from that upstream repository alone.

Generated `assets/` and downloaded `vendor/` directories are ignored. See
[Isaac navigation](isaac-navigation.md) for scene limitations. Simulation does
not establish hardware motion safety or calibrated dynamics.

## Before sharing changes

Run the README's tests and a secret scanner such as Gitleaks. Inspect every new
image or archive yourself: secret scanners cannot decide whether camera images
or room maps are private. Check `git diff --cached` and `git status` before
committing. Report only results actually checked on your configuration.
