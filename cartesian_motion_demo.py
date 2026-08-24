#!/usr/bin/env python3
# coding=utf-8
"""Move one XYZ target through Waveshare IK and the proven T=102 transport.

XYZ is supplied in millimetres. Waveshare's upstream RoArm M3 Cartesian IK
produces the five arm-joint radians; the gripper is appended as joint six.
The resulting target is executed by the already hardware-tested coordinated
transition in ``large_motion_demo``. No firmware pose command, custom IK,
waypoint stream, per-joint command or torque change is used.
"""

import argparse
import math
import time

import large_motion_demo as lmd
import smooth_joint_demo as sjd
from roarm_sdk.waveshare_ik import compute_joint_rad_by_pos


def build_joint_target(x_mm, y_mm, z_mm, roll_rad, pitch_rad, gripper_rad):
    """Convert a Cartesian target to one six-joint coordinated target."""
    try:
        z_value = float(z_mm)
        gripper_value = float(gripper_rad)
    except (TypeError, ValueError):
        raise ValueError("Cartesian target and gripper must be numeric")
    if (not math.isfinite(z_value)
            or z_value < lmd.MIN_TOOL_Z_MM):
        raise ValueError(
            "target z must be finite and at least {:.1f} mm".format(
                lmd.MIN_TOOL_Z_MM
            )
        )
    arm_joints = compute_joint_rad_by_pos(
        x_mm, y_mm, z_value, roll_rad, pitch_rad
    )
    target = arm_joints + [gripper_value]
    if not sjd.valid_six_floats(target):
        raise ValueError(
            "Waveshare IK did not produce six finite joint values"
        )
    return target


def _current_pitch_degrees(arm):
    """Read pitch from the official SDK Cartesian feedback API."""
    pose = arm.pose_get()
    if (not isinstance(pose, (list, tuple)) or len(pose) != 6
            or isinstance(pose[3], bool)
            or not isinstance(pose[3], (int, float))
            or not math.isfinite(float(pose[3]))):
        raise RuntimeError(
            "official SDK pose_get returned invalid pitch: {!r}".format(pose)
        )
    return float(pose[3])


class CartesianMotionRunner(object):
    """Resolve and execute one Cartesian target against an injected arm."""

    def __init__(self, arm, x_mm, y_mm, z_mm, roll_deg=None, pitch_deg=None,
                 gripper_deg=None, speed=lmd.DEFAULT_SPEED,
                 acc=lmd.DEFAULT_ACC, sleep=time.sleep,
                 clock=time.monotonic, print_fn=print):
        self.arm = arm
        self.x_mm = x_mm
        self.y_mm = y_mm
        self.z_mm = z_mm
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.gripper_deg = gripper_deg
        self.speed = speed
        self.acc = acc
        self.sleep = sleep
        self.clock = clock
        self.print_fn = print_fn
        self.target = None

    def run(self):
        reading = self.arm.joints_radian_get()
        ok, reason = lmd.check_startup_position(reading)
        if not ok:
            raise RuntimeError("aborting before motion: " + reason)
        current = [float(value) for value in reading]

        roll_rad = (current[4] if self.roll_deg is None
                    else math.radians(float(self.roll_deg)))
        pitch_deg = (self._resolved_pitch_degrees()
                     if self.pitch_deg is None else float(self.pitch_deg))
        pitch_rad = math.radians(pitch_deg)
        gripper_rad = (current[lmd.GRIPPER_INDEX]
                       if self.gripper_deg is None
                       else math.radians(float(self.gripper_deg)))

        self.target = build_joint_target(
            self.x_mm, self.y_mm, self.z_mm,
            roll_rad, pitch_rad, gripper_rad,
        )
        self.print_fn(
            "Waveshare IK: XYZ [{:.3f}, {:.3f}, {:.3f}] mm, roll {:.3f} deg, "
            "pitch {:.3f} deg -> {} deg".format(
                float(self.x_mm), float(self.y_mm), float(self.z_mm),
                math.degrees(roll_rad), pitch_deg,
                [round(math.degrees(value), 3) for value in self.target],
            )
        )

        coordinated = lmd.LargeMotionRunner(
            self.arm, speed=self.speed, acc=self.acc,
            sleep=self.sleep, clock=self.clock, print_fn=self.print_fn,
        )
        coordinated._transition("XYZ", self.target, current)
        return coordinated.results[-1]

    def _resolved_pitch_degrees(self):
        return _current_pitch_degrees(self.arm)


def run_cartesian_motion(arm, x_mm, y_mm, z_mm, roll_deg=None,
                         pitch_deg=None, gripper_deg=None,
                         speed=lmd.DEFAULT_SPEED, acc=lmd.DEFAULT_ACC,
                         sleep=time.sleep, clock=time.monotonic,
                         print_fn=print):
    """Execute one Cartesian target and always close the serial connection."""
    runner = CartesianMotionRunner(
        arm, x_mm, y_mm, z_mm,
        roll_deg=roll_deg, pitch_deg=pitch_deg,
        gripper_deg=gripper_deg, speed=speed, acc=acc,
        sleep=sleep, clock=clock, print_fn=print_fn,
    )
    try:
        return runner.run()
    finally:
        arm.disconnect()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Move one XYZ target using Waveshare upstream RoArm M3 IK."
    )
    parser.add_argument("--x-mm", type=float, required=True)
    parser.add_argument("--y-mm", type=float, required=True)
    parser.add_argument("--z-mm", type=float, required=True)
    parser.add_argument(
        "--roll-deg", type=float,
        help="wrist roll; omitted preserves current joint feedback",
    )
    parser.add_argument(
        "--pitch-deg", type=float,
        help="tool pitch; omitted preserves official SDK pose feedback",
    )
    parser.add_argument(
        "--gripper-deg", type=float,
        help="gripper angle; omitted preserves current joint feedback",
    )
    parser.add_argument("--port", default=lmd.DEFAULT_PORT,
                        help="serial port (default: %(default)s)")
    parser.add_argument("--baudrate", type=int, default=lmd.DEFAULT_BAUDRATE,
                        help="baud rate (default: %(default)s)")
    parser.add_argument("--speed", type=int, default=lmd.DEFAULT_SPEED,
                        help="SDK speed units (default: %(default)s)")
    parser.add_argument("--acc", type=int, default=lmd.DEFAULT_ACC,
                        help="acceleration (default: %(default)s)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        from roarm_sdk.roarm import roarm
    except ImportError as exc:
        print("roarm_sdk is required but unavailable: {}".format(exc))
        return 1
    arm = roarm(
        roarm_type=lmd.ROARM_TYPE,
        port=args.port,
        baudrate=args.baudrate,
    )
    try:
        result = run_cartesian_motion(
            arm, args.x_mm, args.y_mm, args.z_mm,
            roll_deg=args.roll_deg, pitch_deg=args.pitch_deg,
            gripper_deg=args.gripper_deg,
            speed=args.speed, acc=args.acc,
        )
    except KeyboardInterrupt:
        print("Interrupted: leaving torque as-is; closing serial.")
        return 130
    except Exception as exc:
        print("Cartesian move aborted: {}".format(exc))
        return 1
    print(
        "Summary: Cartesian target reached in {:.2f}s.".format(
            result["duration_s"]
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
