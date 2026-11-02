"""Short, automatically stopping physical motion tests.

Lift the chassis off the ground before first use.
"""

from __future__ import annotations

import argparse
import logging
import time

from .chassis import DEFAULT_HEARTBEAT_PROFILE, HeartbeatProfile, S1Chassis

MOTIONS = {
    "forward": (1.0, 0.0, 0.0),
    "backward": (-1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "right": (0.0, -1.0, 0.0),
    "cw": (0.0, 0.0, -1.0),
    "ccw": (0.0, 0.0, 1.0),
    "stop": (0.0, 0.0, 0.0),
}

COMMAND_PERIOD = 0.05  # 20 Hz; safely faster than the 0.35 s watchdog.
DEFAULT_WARMUP = 1.0
DEFAULT_COOLDOWN = 1.0
ZERO_MOTION = (0.0, 0.0, 0.0)


def drive_for(
    robot: S1Chassis,
    motion: tuple[float, float, float],
    duration: float,
) -> None:
    """Refresh a motion command for the requested duration.

    A single ``drive`` call expires at the chassis command watchdog. Refreshing
    at 20 Hz lets ``--duration`` control the test while preserving watchdog
    protection if this loop stops unexpectedly.
    """
    deadline = time.monotonic() + duration
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        robot.drive(*motion)
        time.sleep(min(COMMAND_PERIOD, remaining))


def run_motion_test(
    robot: S1Chassis,
    motion: tuple[float, float, float],
    duration: float,
    warmup: float = DEFAULT_WARMUP,
    cooldown: float = DEFAULT_COOLDOWN,
) -> None:
    """Run one motion with controller-enable warm-up and explicit cooldown.

    The stable C++ sequence sends the legacy heartbeat and zero wheel speed for
    one second before its first non-zero command. Keeping the Python driver
    alive through warm-up, motion, and cooldown maintains heartbeat and wheel
    command refresh between those phases.
    """
    try:
        robot.start()
        drive_for(robot, ZERO_MOTION, warmup)
        drive_for(robot, motion, duration)
        drive_for(robot, ZERO_MOTION, cooldown)
    finally:
        robot.stop()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion", choices=MOTIONS)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--command", type=int, default=30)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--warmup", type=float, default=DEFAULT_WARMUP)
    parser.add_argument("--cooldown", type=float, default=DEFAULT_COOLDOWN)
    parser.add_argument(
        "--heartbeat-profile",
        choices=[profile.value for profile in HeartbeatProfile],
        default=DEFAULT_HEARTBEAT_PROFILE.value,
    )
    args = parser.parse_args()
    if not 0 < args.command <= 32767:
        parser.error("--command must be in [1, 32767]")
    if not 0 <= args.duration <= 5.0:
        parser.error("--duration must be in [0, 5] seconds")
    if not 0 <= args.warmup <= 5.0:
        parser.error("--warmup must be in [0, 5] seconds")
    if not 0 <= args.cooldown <= 5.0:
        parser.error("--cooldown must be in [0, 5] seconds")

    print("Safety: lift the chassis clear of the ground; Ctrl-C also sends zero.")
    robot = S1Chassis(
        args.interface,
        max_wheel_command=args.command,
        linear_scale=args.command,
        angular_scale=args.command,
        heartbeat_profile=args.heartbeat_profile,
    )
    run_motion_test(
        robot,
        MOTIONS[args.motion],
        args.duration,
        args.warmup,
        args.cooldown,
    )


if __name__ == "__main__":
    main()
