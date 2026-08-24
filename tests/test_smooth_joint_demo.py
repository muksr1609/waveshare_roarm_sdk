# coding=utf-8
"""Unit tests for smooth_joint_demo.

These tests use only unittest and a fake arm. They never open a serial
port, never import the real SDK, and never command hardware. Time is
faked so the suite runs instantly.
"""

import math
import unittest

import smooth_joint_demo as sjd


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
    """In-memory stand-in for a RoArm M3.

    Records every motion command it is asked to make and returns scripted
    joint readings. It never touches serial or the SDK.
    """

    def __init__(self, home=None, readings=None):
        # Default to the expected home pose with a non-zero gripper so the
        # gripper-preservation behaviour is observable.
        self.home = home if home is not None else [0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.37]
        # readings: list of values returned by joints_radian_get in order.
        # If None, the arm reports its commanded position (immediate arrival).
        self.readings = readings
        self._reading_index = 0
        # Last commanded all-joint position (used when readings is None).
        self._current = list(self.home)
        self.joints_radian_ctrl_calls = []
        self.joint_radian_ctrl_calls = []  # per-joint (must stay empty)
        self.disconnect_calls = 0
        self.torque_set_calls = []

    def joints_radian_ctrl(self, radians, speed, acc):
        self.joints_radian_ctrl_calls.append({"radians": list(radians), "speed": speed, "acc": acc})
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


def make_runner(arm, **kwargs):
    clock = FakeClock()
    params = {"sleep": clock.sleep, "clock": clock, "print_fn": lambda *a, **k: None}
    params.update(kwargs)
    runner = sjd.DemoRunner(arm, **params)
    return runner, clock


class BuildPosesTest(unittest.TestCase):
    def test_pose_values(self):
        poses = sjd.build_poses(0.37)
        self.assertEqual(poses["A"], [0.0, 0.05, math.pi / 2.0, 0.0, 0.0, 0.37])
        self.assertEqual(poses["B"], [0.18, 0.23, math.pi / 2.0 - 0.18, -0.18, 0.18, 0.37])
        self.assertEqual(poses["C"], [-0.18, -0.13, math.pi / 2.0 + 0.18, 0.18, -0.18, 0.37])

    def test_gripper_preserved_for_each_pose(self):
        for gripper in (0.0, 0.37, math.pi):
            poses = sjd.build_poses(gripper)
            for name in ("A", "B", "C"):
                self.assertEqual(poses[name][sjd.GRIPPER_INDEX], gripper,
                                 "gripper not preserved for pose %s" % name)


class HomeCheckTest(unittest.TestCase):
    def test_accepts_exact_home(self):
        ok, _ = sjd.check_home_position([0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.37])
        self.assertTrue(ok)

    def test_accepts_small_deviation_within_tolerance(self):
        values = [0.1, 0.0, math.pi / 2.0 - 0.1, 0.05, 0.0, 0.37]
        ok, _ = sjd.check_home_position(values)
        self.assertTrue(ok)

    def test_refuses_joint_out_of_tolerance(self):
        values = [0.26, 0.0, math.pi / 2.0, 0.0, 0.0, 0.37]
        ok, reason = sjd.check_home_position(values)
        self.assertFalse(ok)
        self.assertIn("joint 1", reason)

    def test_refuses_non_finite_value(self):
        ok, _ = sjd.check_home_position([0.0, 0.0, math.pi / 2.0, 0.0, 0.0, float("nan")])
        self.assertFalse(ok)

    def test_refuses_too_few_values(self):
        ok, _ = sjd.check_home_position([0.0, 0.0, math.pi / 2.0, 0.0, 0.0])
        self.assertFalse(ok)

    def test_refuses_non_numeric(self):
        ok, _ = sjd.check_home_position([0.0, 0.0, math.pi / 2.0, 0.0, 0.0, "x"])
        self.assertFalse(ok)

    def test_refuses_error_sentinel(self):
        # The SDK returns -1 on a failed feedback read.
        ok, _ = sjd.check_home_position(-1)
        self.assertFalse(ok)


class TransitionCommandTest(unittest.TestCase):
    def _expected_sequence(self, cycles, gripper):
        poses = sjd.build_poses(gripper)
        seq = ["A"]
        for _ in range(cycles):
            seq.extend(["B", "C", "A"])
        return [poses[name] for name in seq]

    def test_exactly_one_all_joints_command_per_transition(self):
        cycles = 2
        arm = FakeArm()
        runner, _ = make_runner(arm, cycles=cycles)
        completed = runner.run()
        expected = self._expected_sequence(cycles, arm.home[sjd.GRIPPER_INDEX])
        self.assertEqual(completed, len(expected))
        self.assertEqual(len(arm.joints_radian_ctrl_calls), len(expected))
        for call, target in zip(arm.joints_radian_ctrl_calls, expected):
            self.assertEqual(call["radians"], target)
        # Never per-joint moves.
        self.assertEqual(arm.joint_radian_ctrl_calls, [])
        # Never torque off (or any torque call at all).
        self.assertEqual(arm.torque_set_calls, [])

    def test_speed_and_acc_forwarded(self):
        arm = FakeArm()
        runner, _ = make_runner(arm, speed=250, acc=7, cycles=1)
        runner.run()
        for call in arm.joints_radian_ctrl_calls:
            self.assertEqual(call["speed"], 250)
            self.assertEqual(call["acc"], 7)

    def test_default_pose_refusal_aborts_before_motion(self):
        arm = FakeArm(home=[0.5, 0.0, math.pi / 2.0, 0.0, 0.0, 0.37])
        runner, _ = make_runner(arm)
        with self.assertRaises(RuntimeError):
            runner.run()
        # No motion and no disconnect from the runner itself.
        self.assertEqual(arm.joints_radian_ctrl_calls, [])
        self.assertEqual(arm.disconnect_calls, 0)


class ArrivalTest(unittest.TestCase):
    def test_accepts_observed_hardware_settling_error(self):
        target = [0.0, 0.05, math.pi / 2.0, 0.0, 0.0, 0.37]
        settled = list(target)
        settled[1] += 0.07
        arm = FakeArm(readings=[settled])
        clock = FakeClock()
        arrived, telemetry = sjd.wait_for_arrival(
            arm, target, 3.0, sleep=clock.sleep, clock=clock
        )
        self.assertTrue(arrived)
        self.assertEqual(len(telemetry), 1)

    def test_completes_when_feedback_reaches_target(self):
        arm = FakeArm()  # reports commanded position -> immediate arrival
        runner, clock = make_runner(arm, cycles=1)
        completed = runner.run()
        self.assertEqual(completed, 4)
        self.assertEqual(len(arm.joints_radian_ctrl_calls), 4)
        # Telemetry collected with timestamps.
        self.assertGreater(len(runner.telemetry), 0)
        for ts, values in runner.telemetry:
            self.assertIsInstance(ts, float)
            self.assertEqual(len(values), 6)

    def test_times_out_when_feedback_never_arrives(self):
        # The arm keeps reporting the home pose no matter what is commanded.
        home = [0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.37]
        arm = FakeArm(home=home, readings=[list(home)] * 1000)
        runner, clock = make_runner(arm, cycles=1)
        with self.assertRaises(TimeoutError):
            runner.run()
        # Home is already within the empirical 0.08-rad tolerance of A,
        # so A completes and the unchanged feedback times out on B.
        self.assertEqual(runner.transitions_completed, 1)
        self.assertEqual(len(arm.joints_radian_ctrl_calls), 2)
        # Waited no less than the 3 second floor (in fake time).
        self.assertGreaterEqual(clock.now, sjd.MIN_TIMEOUT_S)

    def test_timeout_respects_floor_and_delta(self):
        # A larger delta at a slow speed must exceed the 3s floor.
        timeout_small = sjd.command_timeout_s([0.01, 0.0, 0.0, 0.0, 0.0, 0.0], 4096)
        self.assertEqual(timeout_small, sjd.MIN_TIMEOUT_S)
        timeout_large = sjd.command_timeout_s([0.5, 0.5, 0.0, 0.0, 0.0, 0.0], 10)
        self.assertGreater(timeout_large, sjd.MIN_TIMEOUT_S)
        # Speed never drives the timeout below the floor.
        self.assertGreaterEqual(sjd.command_timeout_s([0.0], 1), sjd.MIN_TIMEOUT_S)

    def test_requires_valid_six_float_feedback(self):
        # Invalid feedback (SDK -1 sentinel) during arrival polling must not
        # count as arrival. First reading is valid so the home check passes.
        home = [0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.37]
        readings = [list(home), -1] + [list(home)] * 1000
        arm = FakeArm(home=home, readings=readings)
        runner, clock = make_runner(arm, cycles=1)
        with self.assertRaises(TimeoutError):
            runner.run()


class DisconnectTest(unittest.TestCase):
    def test_disconnect_called_on_success(self):
        arm = FakeArm()
        clock = FakeClock()
        completed = sjd.run_demo(arm, cycles=1, sleep=clock.sleep,
                                 clock=clock, print_fn=lambda *a: None)
        self.assertEqual(completed, 4)
        self.assertEqual(arm.disconnect_calls, 1)

    def test_disconnect_called_on_timeout(self):
        home = [0.0, 0.0, math.pi / 2.0, 0.0, 0.0, 0.37]
        arm = FakeArm(home=home, readings=[list(home)] * 1000)
        clock = FakeClock()
        with self.assertRaises(TimeoutError):
            sjd.run_demo(arm, cycles=1, sleep=clock.sleep,
                         clock=clock, print_fn=lambda *a: None)
        self.assertEqual(arm.disconnect_calls, 1)

    def test_disconnect_called_on_home_refusal(self):
        arm = FakeArm(home=[0.5, 0.0, math.pi / 2.0, 0.0, 0.0, 0.37])
        clock = FakeClock()
        with self.assertRaises(RuntimeError):
            sjd.run_demo(arm, cycles=1, sleep=clock.sleep,
                         clock=clock, print_fn=lambda *a: None)
        self.assertEqual(arm.disconnect_calls, 1)


class CliTest(unittest.TestCase):
    def test_defaults(self):
        args = sjd.parse_args([])
        self.assertEqual(args.port, "COM21")
        self.assertEqual(args.baudrate, 115200)
        self.assertEqual(args.speed, 180)
        self.assertEqual(args.acc, 10)
        self.assertEqual(args.cycles, 2)


if __name__ == "__main__":
    unittest.main()
