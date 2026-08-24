#!/usr/bin/env python3
# coding=utf-8
"""Minimal, hardware-safe large-motion demo for the RoArm M3-S.

Runs the official-model large poses (absolute, degree space converted to
radians). These poses and the equal-speed paths between them have already
been mesh collision checked with >= 111 mm moving-link floor clearance:

    A_HOME                        [0, 0, 90, 0, 0, 0]
    B_HIGH_RIGHT                  [90, 45, 45, -45, 0, 0]
    C_HIGH_LEFT_OPEN_ROLL_PLUS    [-90, 45, 45, -45, 180, 90]
    D_HIGH_LEFT_OPEN_ROLL_MINUS   [-90, 45, 45, -45, -180, 90]
    H_HIGH_ELBOW_PLUS             [0, -45, 135, -45, 0, 0]
    G_HIGH_NEGATIVE_PITCH         [0, -45, 45, 45, 0, 0]
    F_HOME                        [0, 0, 90, 0, 0, 0]

Safety model:

- Joints are read first and the demo aborts before any motion unless six
  finite values are returned, the first five joints are each within
  0.25 rad of home [0, 0, pi/2, 0, 0] and the gripper is within 0.25 rad
  of closed.
- The normal sequence uses only physically validated legs:
  A -> B -> C -> D -> B -> A -> H -> A -> G -> F. --stage-only runs
  A -> B -> F so the large pitch/base excursion can be tested and
  returned home before a full run. Home separates the H and G pitch
  combinations.
- Every transition is exactly ONE arm.joints_radian_ctrl(...) call
  (all six joints at once). No per-joint moves, no streamed waypoints,
  no torque changes, no custom interpolation.
- joints_radian_get is polled until the max target error is <= 0.12 rad
  and the readings are stable for 3 consecutive samples (max
  sample-to-sample change <= 0.015 rad). The gripper and roll are
  treated like all other joints. The timeout includes the measured
  hardware speed margin: 1.5 times nominal travel plus 5 seconds, with
  an 8 second floor.
- During every move, firmware XYZ feedback must keep the tool at least
  120 mm high. A violation sends one emergency hold at the measured
  joints and aborts; torque is never disabled.
- Each transition prints target degrees, achieved degrees, max error and
  transition duration.
- The serial is closed in a finally block. Ctrl+C stops the run without
  releasing torque (torque_set is never called).

Importing this module opens nothing and commands no hardware; the SDK is
imported lazily inside main(). Pure validation and timeout helpers are
reused from smooth_joint_demo.
"""

import argparse
import math
import time

import smooth_joint_demo as sjd

ROARM_TYPE = "roarm_m3"
NUM_JOINTS = sjd.NUM_JOINTS
GRIPPER_INDEX = sjd.GRIPPER_INDEX

# Official-model poses in degree space (absolute), converted to radians.
POSES_DEG = {
    "A": [0, 0, 90, 0, 0, 0],          # HOME
    "B": [90, 45, 45, -45, 0, 0],      # HIGH RIGHT
    "C": [-90, 45, 45, -45, 180, 90],  # HIGH LEFT OPEN, ROLL +180
    "D": [-90, 45, 45, -45, -180, 90],  # HIGH LEFT OPEN, ROLL -180
    "H": [0, -45, 135, -45, 0, 0],      # HIGH, ELBOW +45 FROM HOME
    "G": [0, -45, 45, 45, 0, 0],        # HIGH, NEGATIVE PITCH PAIR
    "F": [0, 0, 90, 0, 0, 0],          # HOME
}
POSES = {name: [math.radians(v) for v in angles]
         for name, angles in POSES_DEG.items()}

NORMAL_SEQUENCE = ("A", "B", "C", "D", "B", "A", "H", "A", "G", "F")
STAGE_SEQUENCE = ("A", "B", "F")

GRIPPER_CLOSED_RAD = 0.0
GRIPPER_TOLERANCE_RAD = 0.25
ARRIVAL_TOLERANCE_RAD = 0.12
STABILITY_TOLERANCE_RAD = 0.015
STABLE_SAMPLES = 3
MIN_TIMEOUT_S = 8.0
TIMEOUT_SCALE = 1.5
TIMEOUT_BUFFER_S = 5.0
POLL_INTERVAL_S = 0.1
MIN_TOOL_Z_MM = 120.0

DEFAULT_PORT = sjd.DEFAULT_PORT
DEFAULT_BAUDRATE = sjd.DEFAULT_BAUDRATE
DEFAULT_SPEED = sjd.DEFAULT_SPEED
DEFAULT_ACC = sjd.DEFAULT_ACC


def command_timeout_s(delta, speed):
    """Arrival timeout for a commanded delta at the given speed.

    The SDK speed conversion under-predicted the physical 180/360 degree
    trials, so use 1.5 times nominal travel plus a 5 second buffer, with
    an 8 second floor.
    """
    max_delta = max((abs(float(value)) for value in delta), default=0.0)
    rad_per_s = sjd.speed_rad_per_second(speed)
    if rad_per_s <= 0.0:
        return MIN_TIMEOUT_S
    return max(
        MIN_TIMEOUT_S,
        TIMEOUT_SCALE * max_delta / rad_per_s + TIMEOUT_BUFFER_S,
    )


class HeightGuardError(RuntimeError):
    """Raised with the measured joints when live tool height is unsafe."""

    def __init__(self, tool_z_mm, joints):
        super().__init__(
            "tool height {:.1f} mm is below {:.1f} mm".format(
                tool_z_mm, MIN_TOOL_Z_MM
            )
        )
        self.tool_z_mm = tool_z_mm
        self.joints = list(joints)


def check_startup_position(radians):
    """Verify startup feedback before any motion.

    Returns (ok, reason). All six values must be finite, the first five
    joints must each be within sjd.HOME_TOLERANCE_RAD of
    sjd.HOME_JOINTS, and the gripper must be within
    GRIPPER_TOLERANCE_RAD of closed (GRIPPER_CLOSED_RAD).
    """
    if not sjd.valid_six_floats(radians):
        return False, "expected six finite joint values, got {!r}".format(radians)
    for joint, (value, home) in enumerate(zip(radians[:5], sjd.HOME_JOINTS), start=1):
        if abs(float(value) - home) > sjd.HOME_TOLERANCE_RAD:
            return False, (
                "joint {} at {:.3f} rad is not within {} rad of home {:.3f} rad".format(
                    joint, float(value), sjd.HOME_TOLERANCE_RAD, home
                )
            )
    gripper = float(radians[GRIPPER_INDEX])
    if abs(gripper - GRIPPER_CLOSED_RAD) > GRIPPER_TOLERANCE_RAD:
        return False, (
            "gripper at {:.3f} rad is not within {} rad of closed".format(
                gripper, GRIPPER_TOLERANCE_RAD
            )
        )
    return True, "startup position confirmed"


def wait_for_arrival(arm, target, timeout_s,
                     tolerance_rad=ARRIVAL_TOLERANCE_RAD,
                     stability_tol=STABILITY_TOLERANCE_RAD,
                     stable_samples=STABLE_SAMPLES,
                     sleep=time.sleep, clock=time.monotonic):
    """Poll arm.joints_radian_get until settled or timeout.

    Settled means the max target error is <= tolerance_rad and the
    readings are stable for stable_samples consecutive samples (max
    sample-to-sample change per joint <= stability_tol). All six joints,
    including gripper and roll, are treated like any other joint.
    Returns (arrived, telemetry); telemetry is a list of
    (timestamp, radians) tuples collected from valid six-float feedback.
    """
    telemetry = []
    previous = None
    stable_count = 0
    deadline = clock() + timeout_s
    while True:
        if hasattr(arm, "feedback_get"):
            feedback = arm.feedback_get()
            if (isinstance(feedback, (list, tuple)) and len(feedback) >= 10
                    and sjd.valid_six_floats(feedback[4:10])
                    and isinstance(feedback[2], (int, float))
                    and math.isfinite(float(feedback[2]))):
                values = feedback[4:10]
                tool_z_mm = float(feedback[2])
                if tool_z_mm < MIN_TOOL_Z_MM:
                    raise HeightGuardError(tool_z_mm, values)
            else:
                values = None
        else:
            values = arm.joints_radian_get()
        if sjd.valid_six_floats(values):
            samples = [float(v) for v in values]
            telemetry.append((clock(), samples))
            if previous is None:
                stable_count = 1
            else:
                step = max(abs(c - p) for c, p in zip(samples, previous))
                stable_count = stable_count + 1 if step <= stability_tol else 1
            previous = samples
            if (stable_count >= stable_samples
                    and sjd.max_joint_error(samples, target) <= tolerance_rad):
                return True, telemetry
        if clock() >= deadline:
            return False, telemetry
        sleep(POLL_INTERVAL_S)


def _degrees(values):
    """Round a radian pose to 3-decimal degrees for display."""
    return [round(math.degrees(v), 3) for v in values]


class LargeMotionRunner:
    """Runs the large-motion pose sequence against an injected arm object.

    The arm only needs joints_radian_ctrl(radians, speed, acc),
    joints_radian_get() and disconnect(). sleep/clock/print_fn are
    injected so tests can run without hardware or real delays.
    """

    def __init__(self, arm, speed=DEFAULT_SPEED, acc=DEFAULT_ACC,
                 stage_only=False, sleep=time.sleep, clock=time.monotonic,
                 print_fn=print):
        self.arm = arm
        self.speed = speed
        self.acc = acc
        self.stage_only = stage_only
        self.sleep = sleep
        self.clock = clock
        self.print_fn = print_fn
        self.transitions_completed = 0
        self.results = []

    def _transition(self, name, target, previous):
        delta = [float(t) - float(p) for t, p in zip(target, previous)]
        timeout_s = command_timeout_s(delta, self.speed)
        self.print_fn(
            "-> pose {}: {} deg (speed={}, acc={}, timeout={:.2f}s)".format(
                name, _degrees(target), self.speed, self.acc, timeout_s
            )
        )
        started = self.clock()
        # Exactly one all-joints command per transition: never per-joint,
        # never streamed, no torque changes, no custom interpolation.
        self.arm.joints_radian_ctrl(
            radians=list(target), speed=self.speed, acc=self.acc
        )
        try:
            arrived, telemetry = wait_for_arrival(
                self.arm, target, timeout_s, sleep=self.sleep, clock=self.clock
            )
        except HeightGuardError as exc:
            # One emergency hold at measured joints; never release torque.
            self.arm.joints_radian_ctrl(
                radians=exc.joints, speed=self.speed, acc=self.acc
            )
            raise RuntimeError("height guard: {}".format(exc))
        duration = self.clock() - started
        achieved = telemetry[-1][1] if telemetry else None
        max_err = (sjd.max_joint_error(achieved, target)
                   if achieved is not None else float("nan"))
        self.results.append({
            "pose": name,
            "target": list(target),
            "achieved": achieved,
            "max_error": max_err,
            "duration_s": duration,
            "arrived": arrived,
        })
        self.print_fn(
            "pose {}: target {} deg, achieved {} deg, max error {:.4f} rad, "
            "duration {:.2f}s".format(
                name, _degrees(target),
                _degrees(achieved) if achieved is not None else "n/a",
                max_err, duration,
            )
        )
        if not arrived:
            raise TimeoutError(
                "pose {} not settled within {:.2f}s".format(name, timeout_s)
            )
        self.transitions_completed += 1

    def run(self):
        """Execute the pose sequence; returns completed transitions.

        Runs the validated normal sequence (or A, B, F when stage_only). Aborts
        (raises) before any motion when the startup check fails.
        """
        reading = self.arm.joints_radian_get()
        ok, reason = check_startup_position(reading)
        if not ok:
            raise RuntimeError("aborting before motion: " + reason)
        self.print_fn(
            "startup confirmed: {} rad, gripper {:.4f} rad".format(
                [round(float(v), 4) for v in reading],
                float(reading[GRIPPER_INDEX]),
            )
        )
        sequence = STAGE_SEQUENCE if self.stage_only else NORMAL_SEQUENCE
        previous = [float(v) for v in reading]
        for name in sequence:
            target = POSES[name]
            self._transition(name, target, previous)
            previous = target
        return self.transitions_completed


def run_large_motion(arm, speed=DEFAULT_SPEED, acc=DEFAULT_ACC,
                     stage_only=False, sleep=time.sleep, clock=time.monotonic,
                     print_fn=print):
    """Run the large-motion sequence on `arm`; always disconnect in finally.

    Returns the number of completed transitions.
    """
    runner = LargeMotionRunner(
        arm, speed=speed, acc=acc, stage_only=stage_only,
        sleep=sleep, clock=clock, print_fn=print_fn,
    )
    try:
        return runner.run()
    finally:
        arm.disconnect()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Minimal large-motion demo for the RoArm M3-S."
    )
    parser.add_argument("--port", default=DEFAULT_PORT,
                        help="serial port (default: %(default)s)")
    parser.add_argument("--baudrate", type=int, default=DEFAULT_BAUDRATE,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help="SDK speed units (default: %(default)s)")
    parser.add_argument("--acc", type=int, default=DEFAULT_ACC,
                        help="acceleration (default: %(default)s)")
    parser.add_argument("--stage-only", action="store_true",
                        help="run only A, B, F to test the riskiest "
                             "pitch/base excursion and return home")
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
        completed = run_large_motion(
            arm, speed=args.speed, acc=args.acc, stage_only=args.stage_only
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
