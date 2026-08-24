import json
import math
import threading
from unittest.mock import Mock

import pytest

from roarm_sdk.generate import CommandGenerator
from roarm_sdk.roarm import roarm
from roarm_sdk.utils import RoarmDataException


def make_arm(*, thread_lock=False):
    arm = object.__new__(roarm)
    CommandGenerator.__init__(
        arm,
        roarm_type="roarm_m3",
        debug=False,
        gripper_type="angular_direct",
    )
    arm.thread_lock = thread_lock
    if thread_lock:
        arm.lock = threading.Lock()
    arm._res = Mock(side_effect=AssertionError("single-attempt command used _res"))
    return arm


def test_single_attempt_group_motion_validates_encodes_and_writes_once():
    arm = make_arm()
    arm._request_once = Mock(side_effect=lambda command, _genre: command)
    target = [0.1, 0.2, 1.5, -0.1, 0.3, 0.0]

    result = arm.joints_radian_ctrl_once(target, speed=180, acc=10)

    assert arm._request_once.call_count == 1
    arm._res.assert_not_called()
    encoded = json.loads(result.decode("utf-8"))
    assert encoded["T"] == 102
    assert encoded["base"] == pytest.approx(0.1)
    assert encoded["hand"] == pytest.approx(math.pi)
    assert encoded["spd"] == 180
    assert encoded["acc"] == 10


@pytest.mark.parametrize("failed_result", [None, b""])
def test_single_attempt_group_motion_does_not_retry_failure(failed_result):
    arm = make_arm()
    arm._request_once = Mock(return_value=failed_result)

    assert arm.joints_radian_ctrl_once([0, 0, 1.5, 0, 0, 0], 180, 10) == -1
    assert arm._request_once.call_count == 1
    arm._res.assert_not_called()


def test_single_attempt_group_motion_validates_before_writing():
    arm = make_arm()
    arm._request_once = Mock()

    with pytest.raises(RoarmDataException):
        arm.joints_radian_ctrl_once([0, 0, 1.5, 0, 0, 2.0], 180, 10)

    arm._request_once.assert_not_called()


def test_single_attempt_group_motion_uses_sdk_lock():
    arm = make_arm(thread_lock=True)

    def request(command, _genre):
        assert arm.lock.locked()
        return command

    arm._request_once = Mock(side_effect=request)
    arm.joints_radian_ctrl_once([0, 0, 1.5, 0, 0, 0], 180, 10)

    assert arm._request_once.call_count == 1
    assert not arm.lock.locked()
