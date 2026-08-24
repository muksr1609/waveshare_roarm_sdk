# coding=utf-8
"""Unit tests for large_motion_demo.

These tests use only unittest and a fake arm. They never open a serial
port, never import the real SDK, and never command hardware. Time is
faked so the suite runs instantly.
"""

import math
import unittest

import large_motion_demo as lmd
import smooth_joint_demo as sjd

HOME = [0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0]


class FakeClock(object):
    """Monotonic clock and sleeper that advance only when told to."""

    def __init__(self):
        self.now = 0.0
        self.slept = []

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.slept.append(seconds)
        self.now += seconds


class FakeArm(object):
    """In-memory stand-in for a RoArm M3. Never touches serial or the SDK.

    Records every motion command it is asked to make and returns scripted
    joint readings. With readings=None it reports its commanded position.
    """

    def __init__(self, home=None, readings=None):
        # Home with a closed gripper (0 rad) as expected by the startup check.
        self.home = home if home is not None else list(HOME)
        self.readings = readings
        self._reading_index = 0
        self._current = list(self.home)
        self.joints_radian_ctrl_calls = []
        self.joint_radian_ctrl_calls = []  # per-joint (must stay empty)
        self.disconnect_calls = 0
        self.torque_set_calls = []

    def joints_radian_ctrl(self, radians, speed, acc):
        self.joints_radian_ctrl_calls.append(
            {"radians": list(radians), "speed": speed, "acc": acc}
        )
        self._current = list(radians)
        return 1

    def joint_radian_ctrl(self, joint, radian, speed, acc):
        self.joint_radian_ctrl_calls.append({"joint": joint, "radian": radian})
        return 1

    def joints_radian_get(self):
        if self.readings is not None:
            value = self.readings[min(self._reading_index, len(self.readings) - 1)]
            self._reading_index += 1
            return value
        return list(self._current)

    def torque_set(self, cmd):
        self.torque_set_calls.append(cmd)

    def disconnect(self):
        self.disconnect_calls += 1


class LowFeedbackArm(FakeArm):
    """Fake arm whose firmware XYZ reports an unsafe live tool height."""

    def __init__(self):
        super().__init__()
        self.feedback_calls = 0

    def feedback_get(self):
        self.feedback_calls += 1
        z_mm = 200.0 if self.feedback_calls == 1 else 100.0
        return [0.0, 0.0, z_mm, 0.0] + list(self._current)


class DroppedBCommandArm(FakeArm):
    """Ignores the first B target exactly like the observed firmware drop."""

    def __init__(self):
        super().__init__()
        self.dropped = False

    def joints_radian_ctrl(self, radians, speed, acc):
        self.joints_radian_ctrl_calls.append(
            {"radians": list(radians), "speed": speed, "acc": acc}
        )
        if list(radians) == lmd.POSES["B"] and not self.dropped:
            self.dropped = True
            return 1
        self._current = list(radians)
        return 1


def make_runner(arm, **kwargs):
    clock = FakeClock()
    params = {"sleep": clock.sleep, "clock": clock,
              "print_fn": lambda *a, **k: None}
    params.update(kwargs)
    runner = lmd.LargeMotionRunner(arm, **params)
    return runner, clock


def radians_of(*degrees):
    return [math.radians(d) for d in degrees]


class PosesTest(unittest.TestCase):
    def test_exact_pose_values_in_radians(self):
        self.assertEqual(lmd.POSES_DEG, {
            "A": [0, 0, 90, 0, 0, 0],
            "B": [90, 45, 45, -45, 0, 0],
            "C": [-90, 45, 45, -45, 180, 90],
            "D": [-90, 45, 45, -45, -180, 90],
            "H": [0, -45, 135, -45, 0, 0],
            "G": [0, -45, 45, 45, 0, 0],
            "F": [0, 0, 90, 0, 0, 0],
        })
        expected = {
            "A": radians_of(0, 0, 90, 0, 0, 0),
            "B": radians_of(90, 45, 45, -45, 0, 0),
            "C": radians_of(-90, 45, 45, -45, 180, 90),
            "D": radians_of(-90, 45, 45, -45, -180, 90),
            "H": radians_of(0, -45, 135, -45, 0, 0),
            "G": radians_of(0, -45, 45, 45, 0, 0),
            "F": radians_of(0, 0, 90, 0, 0, 0),
        }
        self.assertEqual(set(lmd.POSES), set(expected))
        for name, angles in expected.items():
            self.assertEqual(lmd.POSES[name], angles, "pose %s mismatch" % name)

    def test_base_sweeps_plus_minus_90(self):
        self.assertEqual(lmd.POSES["B"][0], math.radians(90))
        self.assertEqual(lmd.POSES["C"][0], math.radians(-90))
        self.assertEqual(lmd.POSES["D"][0], math.radians(-90))
        for name in ("A", "H", "G", "F"):
            self.assertEqual(lmd.POSES[name][0], 0.0)

    def test_shoulder_and_wrist_sweep_plus_minus_45(self):
        for name in ("B", "C", "D"):
            self.assertEqual(lmd.POSES[name][1], math.radians(45))
            self.assertEqual(lmd.POSES[name][3], math.radians(-45))
        self.assertEqual(lmd.POSES["H"][1], math.radians(-45))
        self.assertEqual(lmd.POSES["H"][3], math.radians(-45))
        self.assertEqual(lmd.POSES["G"][1], math.radians(-45))
        self.assertEqual(lmd.POSES["G"][3], math.radians(45))

    def test_elbow_45_and_135(self):
        for name in ("B", "C", "D", "G"):
            self.assertEqual(lmd.POSES[name][2], math.radians(45))
        self.assertEqual(lmd.POSES["H"][2], math.radians(135))

    def test_roll_spans_plus_180_to_minus_180(self):
        self.assertEqual(lmd.POSES["C"][4], math.radians(180))
        self.assertEqual(lmd.POSES["D"][4], math.radians(-180))
        self.assertAlmostEqual(
            abs(lmd.POSES["C"][4] - lmd.POSES["D"][4]), math.radians(360))

    def test_gripper_90_when_open_else_closed(self):
        for name in ("C", "D"):
            self.assertEqual(lmd.POSES[name][lmd.GRIPPER_INDEX], math.radians(90))
        for name in ("A", "B", "H", "G", "F"):
            self.assertEqual(lmd.POSES[name][lmd.GRIPPER_INDEX], 0.0)


class SequenceTest(unittest.TestCase):
    def test_retries_identical_target_once_when_firmware_drops_command(self):
        arm = DroppedBCommandArm()
        runner, _ = make_runner(arm, stage_only=True)
        self.assertEqual(runner.run(), 3)
        b_calls = [
            call for call in arm.joints_radian_ctrl_calls
            if call["radians"] == lmd.POSES["B"]
        ]
        self.assertEqual(len(b_calls), 2)
        self.assertEqual(arm.joint_radian_ctrl_calls, [])
        self.assertEqual(arm.torque_set_calls, [])

    def test_full_sequence_issues_exact_poses_in_order(self):
        arm = FakeArm()
        runner, _ = make_runner(arm)
        completed = runner.run()
        expected_sequence = ("A", "B", "C", "D", "B", "A", "H", "A", "G", "F")
        self.assertEqual(lmd.NORMAL_SEQUENCE, expected_sequence)
        self.assertEqual(completed, len(expected_sequence))
        calls = [c["radians"] for c in arm.joints_radian_ctrl_calls]
        self.assertEqual(calls, [lmd.POSES[n] for n in expected_sequence])
        # A and F are the same home pose.
        self.assertEqual(lmd.POSES["A"], lmd.POSES["F"])

    def test_stage_only_sequence_is_a_b_f(self):
        arm = FakeArm()
        runner, _ = make_runner(arm, stage_only=True)
        completed = runner.run()
        self.assertEqual(lmd.STAGE_SEQUENCE, ("A", "B", "F"))
        self.assertEqual(completed, 3)
        calls = [c["radians"] for c in arm.joints_radian_ctrl_calls]
        self.assertEqual(calls, [lmd.POSES["A"], lmd.POSES["B"], lmd.POSES["F"]])

    def test_one_all_joints_command_per_transition(self):
        arm = FakeArm()
        runner, _ = make_runner(arm)
        runner.run()
        self.assertEqual(len(arm.joints_radian_ctrl_calls), len(lmd.NORMAL_SEQUENCE))
        # Never per-joint moves, never torque calls, no streaming.
        self.assertEqual(arm.joint_radian_ctrl_calls, [])
        self.assertEqual(arm.torque_set_calls, [])

    def test_stage_only_also_single_command_per_transition(self):
        arm = FakeArm()
        runner, _ = make_runner(arm, stage_only=True)
        runner.run()
        self.assertEqual(len(arm.joints_radian_ctrl_calls), 3)
        self.assertEqual(arm.joint_radian_ctrl_calls, [])
        self.assertEqual(arm.torque_set_calls, [])

    def test_speed_and_acc_forwarded(self):
        arm = FakeArm()
        runner, _ = make_runner(arm, speed=300, acc=5, stage_only=True)
        runner.run()
        for call in arm.joints_radian_ctrl_calls:
            self.assertEqual(call["speed"], 300)
            self.assertEqual(call["acc"], 5)


class StartupCheckTest(unittest.TestCase):
    def test_accepts_exact_home_with_closed_gripper(self):
        ok, reason = lmd.check_startup_position(HOME)
        self.assertTrue(ok)
        self.assertIn("startup", reason)

    def test_accepts_deviation_within_tolerance(self):
        values = [0.1, 0.0, math.pi / 2.0 - 0.1, 0.05, 0.0, 0.2]
        ok, _ = lmd.check_startup_position(values)
        self.assertTrue(ok)

    def test_refuses_joint_out_of_home_tolerance(self):
        values = [0.26, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0]
        ok, reason = lmd.check_startup_position(values)
        self.assertFalse(ok)
        self.assertIn("joint 1", reason)

    def test_refuses_open_gripper(self):
        values = [0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.3]
        ok, reason = lmd.check_startup_position(values)
        self.assertFalse(ok)
        self.assertIn("gripper", reason)

    def test_refuses_non_finite_value(self):
        ok, _ = lmd.check_startup_position(
            [0.0, 0.0, math.pi / 2.0, 0.0, 0.0, float("nan")])
        self.assertFalse(ok)

    def test_refuses_error_sentinel(self):
        # The SDK returns -1 on a failed feedback read.
        ok, _ = lmd.check_startup_position(-1)
        self.assertFalse(ok)

    def test_runner_refusal_aborts_before_motion(self):
        arm = FakeArm(home=[0.5, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0])
        runner, _ = make_runner(arm)
        with self.assertRaises(RuntimeError):
            runner.run()
        self.assertEqual(arm.joints_radian_ctrl_calls, [])
        self.assertEqual(arm.disconnect_calls, 0)

    def test_runner_refusal_on_open_gripper(self):
        arm = FakeArm(home=[0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 1.2])
        runner, _ = make_runner(arm)
        with self.assertRaises(RuntimeError):
            runner.run()
        self.assertEqual(arm.joints_radian_ctrl_calls, [])


class ArrivalTest(unittest.TestCase):
    def test_live_height_guard_holds_measured_joints_and_aborts(self):
        arm = LowFeedbackArm()
        clock = FakeClock()
        with self.assertRaisesRegex(RuntimeError, "height guard"):
            lmd.run_large_motion(
                arm, stage_only=True, sleep=clock.sleep, clock=clock,
                print_fn=lambda *a: None,
            )
        # First call targets A; second is the emergency hold at feedback joints.
        self.assertEqual(len(arm.joints_radian_ctrl_calls), 2)
        self.assertEqual(
            arm.joints_radian_ctrl_calls[-1]["radians"], lmd.POSES["A"]
        )
        self.assertEqual(arm.torque_set_calls, [])
        self.assertEqual(arm.disconnect_calls, 1)

    def test_arrives_after_three_consecutive_stable_samples(self):
        target = lmd.POSES["B"]
        arm = FakeArm(readings=[list(target)] * 10)
        clock = FakeClock()
        arrived, telemetry = lmd.wait_for_arrival(
            arm, target, 4.0, sleep=clock.sleep, clock=clock)
        self.assertTrue(arrived)
        self.assertEqual(len(telemetry), 3)

    def test_unstable_readings_never_arrive(self):
        # Jitter of 0.02 rad per sample (> 0.015 stability tolerance)
        # breaks the stable chain forever.
        target = lmd.POSES["B"]
        readings = []
        for i in range(100):
            values = list(target)
            values[0] += 0.02 if i % 2 == 0 else -0.02
            readings.append(values)
        arm = FakeArm(readings=readings)
        clock = FakeClock()
        arrived, _ = lmd.wait_for_arrival(
            arm, target, 4.0, sleep=clock.sleep, clock=clock)
        self.assertFalse(arrived)

    def test_accepts_settling_error_within_tolerance(self):
        target = lmd.POSES["B"]
        settled = list(target)
        settled[1] += 0.11  # within the 0.12 rad arrival tolerance
        arm = FakeArm(readings=[settled] * 10)
        clock = FakeClock()
        arrived, telemetry = lmd.wait_for_arrival(
            arm, target, 4.0, sleep=clock.sleep, clock=clock)
        self.assertTrue(arrived)
        self.assertEqual(len(telemetry), 3)

    def test_refuses_settling_error_beyond_tolerance(self):
        target = lmd.POSES["B"]
        off = list(target)
        off[1] += 0.13  # beyond the 0.12 rad arrival tolerance
        arm = FakeArm(readings=[off] * 10)
        clock = FakeClock()
        arrived, _ = lmd.wait_for_arrival(
            arm, target, 4.0, sleep=clock.sleep, clock=clock)
        self.assertFalse(arrived)

    def test_gripper_and_roll_count_as_joints(self):
        target = lmd.POSES["C"]
        # A 0.11 rad offset on the roll joint still settles...
        roll_off = list(target)
        roll_off[4] -= 0.11
        arm = FakeArm(readings=[roll_off] * 10)
        clock = FakeClock()
        arrived, _ = lmd.wait_for_arrival(
            arm, target, 4.0, sleep=clock.sleep, clock=clock)
        self.assertTrue(arrived)
        # ...and a 0.13 rad offset on the gripper blocks arrival equally.
        grip_off = list(target)
        grip_off[lmd.GRIPPER_INDEX] += 0.13
        arm = FakeArm(readings=[grip_off] * 10)
        clock = FakeClock()
        arrived, _ = lmd.wait_for_arrival(
            arm, target, 4.0, sleep=clock.sleep, clock=clock)
        self.assertFalse(arrived)

    def test_requires_valid_feedback(self):
        # The SDK -1 sentinel must not count as a settled sample.
        target = lmd.POSES["B"]
        arm = FakeArm(readings=[-1] * 100)
        clock = FakeClock()
        arrived, telemetry = lmd.wait_for_arrival(
            arm, target, 4.0, sleep=clock.sleep, clock=clock)
        self.assertFalse(arrived)
        self.assertEqual(telemetry, [])

    def test_timeout_floor_and_delta_scaling(self):
        small = lmd.command_timeout_s(
            [0.01, 0.0, 0.0, 0.0, 0.0, 0.0], lmd.DEFAULT_SPEED)
        self.assertEqual(small, lmd.MIN_TIMEOUT_S)
        delta = [math.pi / 2.0] * 6
        large = lmd.command_timeout_s(delta, lmd.DEFAULT_SPEED)
        self.assertGreater(large, lmd.MIN_TIMEOUT_S)
        expected = lmd.TIMEOUT_SCALE * (math.pi / 2.0) / sjd.speed_rad_per_second(
            lmd.DEFAULT_SPEED) + lmd.TIMEOUT_BUFFER_S
        self.assertAlmostEqual(large, expected)
        self.assertEqual(lmd.MIN_TIMEOUT_S, 12.0)
        self.assertEqual(lmd.TIMEOUT_SCALE, 3.0)
        self.assertEqual(lmd.TIMEOUT_BUFFER_S, 10.0)
        self.assertEqual(lmd.INTER_POSE_DWELL_S, 1.0)
        self.assertEqual(lmd.MOTION_START_PROBE_S, 2.0)
        self.assertEqual(lmd.MOTION_START_TOLERANCE_RAD, 0.02)
        self.assertEqual(lmd.COOLDOWN_AFTER_INDEX_S, {3: 30.0, 5: 10.0})

    def test_runner_times_out_when_arm_never_moves(self):
        # The arm keeps reporting the home pose no matter what is commanded.
        arm = FakeArm(home=HOME, readings=[list(HOME)] * 1000)
        runner, clock = make_runner(arm, stage_only=True)
        with self.assertRaises(TimeoutError):
            runner.run()
        # A equals the home pose (gripper closed), so A settles; B times out.
        self.assertEqual(runner.transitions_completed, 1)
        # A target, B target, one no-motion retry, then a measured hold.
        self.assertEqual(len(arm.joints_radian_ctrl_calls), 4)
        self.assertEqual(arm.joints_radian_ctrl_calls[-1]["radians"], HOME)
        self.assertEqual(arm.torque_set_calls, [])
        self.assertGreaterEqual(clock.now, lmd.MIN_TIMEOUT_S)


class OutputTest(unittest.TestCase):
    def test_prints_target_and_achieved_degrees_max_error_duration(self):
        arm = FakeArm()
        lines = []
        runner, _ = make_runner(arm, stage_only=True, print_fn=lines.append)
        runner.run()
        a_lines = [ln for ln in lines if ln.startswith("pose A:")]
        b_lines = [ln for ln in lines if ln.startswith("pose B:")]
        self.assertEqual(len(a_lines), 1)
        self.assertEqual(len(b_lines), 1)
        self.assertIn("[0.0, 0.0, 90.0, 0.0, 0.0, 0.0] deg", a_lines[0])
        self.assertIn("[90.0, 45.0, 45.0, -45.0, 0.0, 0.0] deg", b_lines[0])
        for line in (a_lines[0], b_lines[0]):
            self.assertIn("target", line)
            self.assertIn("achieved", line)
            self.assertIn("deg", line)
            self.assertIn("max error 0.0000 rad", line)
            self.assertIn("duration", line)


class DisconnectTest(unittest.TestCase):
    def test_disconnect_called_on_success(self):
        arm = FakeArm()
        clock = FakeClock()
        completed = lmd.run_large_motion(
            arm, stage_only=True, sleep=clock.sleep, clock=clock,
            print_fn=lambda *a: None)
        self.assertEqual(completed, 3)
        self.assertEqual(arm.disconnect_calls, 1)

    def test_disconnect_called_on_timeout(self):
        arm = FakeArm(home=HOME, readings=[list(HOME)] * 1000)
        clock = FakeClock()
        with self.assertRaises(TimeoutError):
            lmd.run_large_motion(
                arm, stage_only=True, sleep=clock.sleep, clock=clock,
                print_fn=lambda *a: None)
        self.assertEqual(arm.disconnect_calls, 1)

    def test_disconnect_called_on_startup_refusal(self):
        arm = FakeArm(home=[0.5, 0.0, math.pi / 2.0, 0.0, 0.0, 0.0])
        clock = FakeClock()
        with self.assertRaises(RuntimeError):
            lmd.run_large_motion(
                arm, sleep=clock.sleep, clock=clock,
                print_fn=lambda *a: None)
        self.assertEqual(arm.disconnect_calls, 1)


class CliTest(unittest.TestCase):
    def test_defaults(self):
        args = lmd.parse_args([])
        self.assertEqual(args.port, "COM21")
        self.assertEqual(args.baudrate, 115200)
        self.assertEqual(args.speed, 180)
        self.assertEqual(args.acc, 10)
        self.assertFalse(args.stage_only)

    def test_stage_only_flag(self):
        args = lmd.parse_args(["--stage-only", "--port", "COM5", "--speed", "120"])
        self.assertTrue(args.stage_only)
        self.assertEqual(args.port, "COM5")
        self.assertEqual(args.speed, 120)


if __name__ == "__main__":
    unittest.main()
