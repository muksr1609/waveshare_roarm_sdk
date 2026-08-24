# coding=utf-8
"""Hardware-free tests for upstream Cartesian IK and coordinated execution."""

import math
import unittest

import cartesian_motion_demo as cmd
from roarm_sdk.waveshare_ik import compute_joint_rad_by_pos


HOME = [0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0]
B_XYZ = [
    2.8425861275868324e-14,
    464.22954431693307,
    257.95255377685248,
]


class FakeClock(object):
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeArm(object):
    def __init__(self, joints=None, pose=None):
        self.current = list(HOME if joints is None else joints)
        self.pose = ([346.16, 0.0, 223.13, 0.0, 0.0, 0.0]
                     if pose is None else list(pose))
        self.joints_radian_ctrl_calls = []
        self.joint_radian_ctrl_calls = []
        self.pose_ctrl_calls = []
        self.torque_set_calls = []
        self.pose_get_calls = 0
        self.disconnect_calls = 0

    def joints_radian_get(self):
        return list(self.current)

    def joints_radian_ctrl(self, radians, speed, acc):
        self.joints_radian_ctrl_calls.append({
            "radians": list(radians), "speed": speed, "acc": acc,
        })
        self.current = list(radians)
        return 1

    def joint_radian_ctrl(self, *args, **kwargs):
        self.joint_radian_ctrl_calls.append((args, kwargs))

    def pose_ctrl(self, pose):
        self.pose_ctrl_calls.append(list(pose))

    def pose_get(self):
        self.pose_get_calls += 1
        return list(self.pose)

    def torque_set(self, cmd_value):
        self.torque_set_calls.append(cmd_value)

    def disconnect(self):
        self.disconnect_calls += 1


def run_fake(arm, **kwargs):
    clock = FakeClock()
    params = {
        "x_mm": B_XYZ[0], "y_mm": B_XYZ[1], "z_mm": B_XYZ[2],
        "roll_deg": 0.0, "pitch_deg": -45.0, "gripper_deg": 0.0,
        "sleep": clock.sleep, "clock": clock,
        "print_fn": lambda *args, **unused: None,
    }
    params.update(kwargs)
    result = cmd.run_cartesian_motion(arm, **params)
    return result, clock


class UpstreamReferenceTest(unittest.TestCase):
    def test_home_matches_upstream_cpp_reference(self):
        target = compute_joint_rad_by_pos(346.16, 0.0, 223.13, 0.0, 0.0)
        expected = [
            0.0,
            -4.2463518690194491e-07,
            1.5708055679408721,
            -8.8165107885451732e-06,
            0.0,
        ]
        for actual, reference in zip(target, expected):
            self.assertAlmostEqual(actual, reference, places=12)

    def test_safe_b_matches_upstream_cpp_reference(self):
        target = compute_joint_rad_by_pos(
            B_XYZ[0], B_XYZ[1], B_XYZ[2], 0.0, -math.pi / 4.0,
        )
        expected = [
            math.pi / 2.0,
            0.78540377501688841,
            0.78539188832782847,
            -0.7853974999472686,
            0.0,
        ]
        for actual, reference in zip(target, expected):
            self.assertAlmostEqual(actual, reference, places=12)
        expected_degrees = [90.0, 45.0, 45.0, -45.0, 0.0]
        for actual, reference in zip(target, expected_degrees):
            self.assertAlmostEqual(math.degrees(actual), reference, places=3)

    def test_unreachable_and_non_finite_are_rejected(self):
        with self.assertRaises(ValueError):
            compute_joint_rad_by_pos(2000.0, 0.0, 200.0, 0.0, 0.0)
        with self.assertRaises(ValueError):
            compute_joint_rad_by_pos(float("nan"), 0.0, 200.0, 0.0, 0.0)


class CoordinatedExecutionTest(unittest.TestCase):
    def test_uses_one_six_joint_command_and_no_other_motion_api(self):
        arm = FakeArm()
        result, _ = run_fake(arm)
        self.assertTrue(result["arrived"])
        self.assertEqual(len(arm.joints_radian_ctrl_calls), 1)
        call = arm.joints_radian_ctrl_calls[0]
        self.assertEqual(len(call["radians"]), 6)
        expected = [
            math.pi / 2.0,
            0.78540377501688841,
            0.78539188832782847,
            -0.7853974999472686,
            0.0,
            0.0,
        ]
        for actual, reference in zip(call["radians"], expected):
            self.assertAlmostEqual(actual, reference, places=12)
        self.assertEqual(arm.joint_radian_ctrl_calls, [])
        self.assertEqual(arm.pose_ctrl_calls, [])
        self.assertEqual(arm.torque_set_calls, [])
        self.assertEqual(arm.disconnect_calls, 1)

    def test_omitted_orientation_and_gripper_retain_feedback(self):
        current = [0.0, 0.0, math.pi / 2.0, 0.0, 0.1, 0.2]
        arm = FakeArm(
            joints=current,
            pose=[B_XYZ[0], B_XYZ[1], B_XYZ[2], -45.0, 5.73, 11.46],
        )
        run_fake(
            arm, roll_deg=None, pitch_deg=None, gripper_deg=None,
        )
        target = arm.joints_radian_ctrl_calls[0]["radians"]
        self.assertAlmostEqual(target[4], current[4])
        self.assertAlmostEqual(target[5], current[5])
        self.assertEqual(arm.pose_get_calls, 1)
        self.assertAlmostEqual(math.degrees(target[3]), -45.0, places=3)

    def test_explicit_pitch_does_not_read_pose_api(self):
        arm = FakeArm(pose=[-1])
        run_fake(arm)
        self.assertEqual(arm.pose_get_calls, 0)

    def test_below_height_and_unreachable_issue_no_motion(self):
        for values in (
            {"x_mm": 300.0, "y_mm": 0.0, "z_mm": 100.0},
            {"x_mm": 2000.0, "y_mm": 0.0, "z_mm": 200.0},
        ):
            arm = FakeArm()
            with self.assertRaises(ValueError):
                run_fake(arm, **values)
            self.assertEqual(arm.joints_radian_ctrl_calls, [])
            self.assertEqual(arm.pose_ctrl_calls, [])
            self.assertEqual(arm.disconnect_calls, 1)

    def test_startup_refusal_disconnects_without_motion(self):
        arm = FakeArm(joints=[0.5, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0])
        with self.assertRaises(RuntimeError):
            run_fake(arm)
        self.assertEqual(arm.joints_radian_ctrl_calls, [])
        self.assertEqual(arm.disconnect_calls, 1)


class CliTest(unittest.TestCase):
    def test_defaults(self):
        args = cmd.parse_args([
            "--x-mm", "346.16", "--y-mm", "0", "--z-mm", "223.13",
        ])
        self.assertEqual(args.port, "COM21")
        self.assertEqual(args.baudrate, 115200)
        self.assertEqual(args.speed, 180)
        self.assertEqual(args.acc, 10)
        self.assertIsNone(args.roll_deg)
        self.assertIsNone(args.pitch_deg)
        self.assertIsNone(args.gripper_deg)


if __name__ == "__main__":
    unittest.main()
