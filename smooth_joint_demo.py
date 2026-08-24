#!/usr/bin/env python3
# coding=utf-8
"""Minimal, hardware-safe smooth-motion demo for the RoArm M3-S.

The demo sweeps a small A -> B -> C -> A motion loop. Safety model:

- Joints are read first and the demo aborts before any motion unless six
  finite values are returned and the first five joints are each within
  0.25 rad of the expected home pose [0, 0, pi/2, 0, 0].
- The measured gripper value is preserved unchanged in every pose.
- Every transition is exactly ONE arm.joints_radian_ctrl(...) call
  (all six joints at once). No per-joint moves, no streamed waypoints.
- joints_radian_get is polled only to confirm arrival (max error
  <= 0.08 rad) and to collect timestamped telemetry, bounded by a
  conservative timeout derived from the commanded delta and speed,
  with a 3 second floor.
- The serial is closed in a finally block. Ctrl+C stops the loop without
  releasing torque (torque_set is never called).

Importing this module opens nothing and commands no hardware; the SDK is
imported lazily inside main().
"""

import argparse
import math
import time

ROARM_TYPE = "roarm_m3"
NUM_JOINTS = 6
GRIPPER_INDEX = 5
HOME_JOINTS = [0.0, 0.0, math.pi / 2.0, 0.0, 0.0]
HOME_TOLERANCE_RAD = 0.25
# The physical M3-S trial settled as far as 0.070 rad from the requested
# shoulder target under load, while all five commanded joints moved together.
ARRIVAL_TOLERANCE_RAD = 0.08
MIN_TIMEOUT_S = 3.0
TIMEOUT_BUFFER_S = 1.0
POLL_INTERVAL_S = 0.1

DEFAULT_PORT = "COM21"
DEFAULT_BAUDRATE = 115200
DEFAULT_SPEED = 180
DEFAULT_ACC = 10
DEFAULT_CYCLES = 2


def build_poses(gripper):
    """Return the safe absolute poses A, B and C as 6-value lists.

    The measured gripper value is appended unchanged to every pose.
    """
    a = [0.0, 0.05, math.pi / 2.0, 0.0, 0.0, gripper]
    b = [0.18, 0.23, math.pi / 2.0 - 0.18, -0.18, 0.18, gripper]
    c = [-0.18, -0.13, math.pi / 2.0 + 0.18, 0.18, -0.18, gripper]
    return {"A": a, "B": b, "C": c}


def valid_six_floats(values):
    """True when values is a list/tuple of exactly six finite numbers."""
    if not isinstance(values, (list, tuple)) or len(values) != NUM_JOINTS:
        return False
    for v in values:
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            return False
        if not math.isfinite(float(v)):
            return False
    return True


def check_home_position(radians):
    """Verify the arm is at the expected home pose before any motion.

    Returns (ok, reason). All six values must be finite and the first five
    joints must each be within HOME_TOLERANCE_RAD of HOME_JOINTS. The
    gripper value is only required to be finite; it is never range-checked
    so it can be preserved unchanged.
    """
    if not valid_six_floats(radians):
        return False, "expected six finite joint values, got {!r}".format(radians)
    for joint, (value, home) in enumerate(zip(radians[:5], HOME_JOINTS), start=1):
        if abs(float(value) - home) > HOME_TOLERANCE_RAD:
            return False, (
                "joint {} at {:.3f} rad is not within {} rad of home {:.3f} rad".format(
                    joint, float(value), HOME_TOLERANCE_RAD, home
                )
            )
    return True, "home position confirmed"


def speed_rad_per_second(speed):
    """Convert SDK speed units to rad/s (SDK speed 2048 equals 180 deg/s)."""
    if isinstance(speed, bool) or not isinstance(speed, (int, float)):
        return 0.0
    if not math.isfinite(float(speed)) or float(speed) <= 0:
        return 0.0
    return float(speed) * math.pi / 2048.0


def command_timeout_s(delta, speed, min_s=MIN_TIMEOUT_S, buffer_s=TIMEOUT_BUFFER_S):
    """Conservative arrival timeout for a commanded delta at the given speed.

    Longest commanded delta divided by the linear speed, plus a buffer,
    with a floor of min_s seconds. The keyword defaults keep the
    historical 3 second floor and 1 second buffer; callers (e.g.
    large_motion_demo) may pass their own floor/buffer.
    """
    max_delta = max((abs(float(d)) for d in delta), default=0.0)
    rad_per_s = speed_rad_per_second(speed)
    if rad_per_s <= 0.0:
        return min_s
    return max(min_s, max_delta / rad_per_s + buffer_s)


def max_joint_error(current, target):
    """Largest absolute per-joint error between two 6-value poses."""
    return max(abs(float(c) - float(t)) for c, t in zip(current, target))


def wait_for_arrival(arm, target, timeout_s, sleep=time.sleep, clock=time.monotonic):
    """Poll arm.joints_radian_get until arrival or timeout.

    Returns (arrived, telemetry). Telemetry is a list of
    (timestamp, radians) tuples collected only from valid six-float
    feedback. Arrival means max_joint_error <= ARRIVAL_TOLERANCE_RAD.
    """
    telemetry = []
    deadline = clock() + timeout_s
    while True:
        values = arm.joints_radian_get()
        if valid_six_floats(values):
            samples = [float(v) for v in values]
            telemetry.append((clock(), samples))
            if max_joint_error(samples, target) <= ARRIVAL_TOLERANCE_RAD:
                return True, telemetry
        if clock() >= deadline:
            return False, telemetry
        sleep(POLL_INTERVAL_S)


class DemoRunner:
    """Runs the A/B/C/A smooth-motion loop against an injected arm object.

    The arm only needs joints_radian_ctrl(radians, speed, acc),
    joints_radian_get() and disconnect(). sleep/clock/print_fn are injected
    so tests can run without hardware or real delays.
    """

    def __init__(self, arm, speed=DEFAULT_SPEED, acc=DEFAULT_ACC,
                 cycles=DEFAULT_CYCLES, sleep=time.sleep,
                 clock=time.monotonic, print_fn=print):
        self.arm = arm
        self.speed = speed
        self.acc = acc
        self.cycles = cycles
        self.sleep = sleep
        self.clock = clock
        self.print_fn = print_fn
        self.transitions_completed = 0
        self.telemetry = []

    def _transition(self, name, target, previous):
        delta = [float(t) - float(p) for t, p in zip(target, previous)]
        timeout_s = command_timeout_s(delta, self.speed)
        self.print_fn(
            "-> pose {}: {} (speed={}, acc={}, timeout={:.2f}s)".format(
                name, [round(v, 4) for v in target],
                self.speed, self.acc, timeout_s,
            )
        )
        # Exactly one all-joints command per transition, never per-joint.
        self.arm.joints_radian_ctrl(radians=list(target), speed=self.speed, acc=self.acc)
        arrived, samples = wait_for_arrival(
            self.arm, target, timeout_s, self.sleep, self.clock
        )
        self.telemetry.extend(samples)
        if not arrived:
            raise TimeoutError("pose {} not reached within {:.2f}s".format(name, timeout_s))
        self.transitions_completed += 1

    def run(self):
        """Execute the demo sequence; returns the number of completed transitions.

        Aborts (raises) before any motion when the home check fails.
        """
        reading = self.arm.joints_radian_get()
        ok, reason = check_home_position(reading)
        if not ok:
            raise RuntimeError("aborting before motion: " + reason)
        gripper = float(reading[GRIPPER_INDEX])
        poses = build_poses(gripper)
        self.print_fn("home confirmed; preserving gripper at {:.4f} rad".format(gripper))
        previous = [float(v) for v in reading]
        self._transition("A", poses["A"], previous)
        previous = poses["A"]
        for _ in range(self.cycles):
            for name in ("B", "C", "A"):
                self._transition(name, poses[name], previous)
                previous = poses[name]
        return self.transitions_completed


def run_demo(arm, speed=DEFAULT_SPEED, acc=DEFAULT_ACC, cycles=DEFAULT_CYCLES,
             sleep=time.sleep, clock=time.monotonic, print_fn=print):
    """Run the full demo on `arm`; always close the connection in finally.

    Returns the number of completed transitions.
    """
    runner = DemoRunner(
        arm, speed=speed, acc=acc, cycles=cycles,
        sleep=sleep, clock=clock, print_fn=print_fn,
    )
    try:
        return runner.run()
    finally:
        arm.disconnect()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Minimal smooth-motion demo for the RoArm M3-S."
    )
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help="serial port (default: %(default)s)")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help="SDK speed units (default: %(default)s)")
    parser.add_argument("--acc", type=int, default=DEFAULT_ACC,
                        help="acceleration (default: %(default)s)")
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES,
                        help="B/C/A cycles after settling at A (default: %(default)s)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        from roarm_sdk.roarm import roarm
    except ImportError as exc:
        print("roarm_sdk is required but unavailable: {}".format(exc))
        return 1
    arm = roarm(roarm_type=ROARM_TYPE, port=args.port, baudrate=args.baudrate)
    try:
        completed = run_demo(
            arm, speed=args.speed, acc=args.acc, cycles=args.cycles
        )
    except KeyboardInterrupt:
        # Deliberately do not torque off: leave the arm in its current state.
        print("Interrupted: leaving torque as-is; closing serial.")
        return 130
    except Exception as exc:
        print("Demo aborted: {}".format(exc))
        return 1
    print("Summary: {} transitions completed.".format(completed))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
