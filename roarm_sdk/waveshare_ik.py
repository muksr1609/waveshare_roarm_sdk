# coding=utf-8
"""Waveshare's upstream Cartesian IK for RoArm M3.

This is a faithful Python transcription of ``roarm_m3::computeJointRadbyPos``
and the helper functions it calls in Waveshare's ``roarm_ws`` repository:

https://github.com/waveshareteam/roarm_ws/blob/40dbd84b553695212fab713e8465f817ba95454d/src/roarm_main/roarm_moveit_cmd/include/roarm_moveit_cmd/solver.hpp

Inputs are XYZ millimetres plus roll and pitch radians. The returned values
are base, shoulder, elbow, wrist-pitch and wrist-roll radians, in that order.
No alternative IK, interpolation or pose selection is implemented here.
"""

import math


# Names and values match Waveshare's upstream roarm_m3 namespace.
ARM_L2_LENGTH_MM_A = 236.82
ARM_L2_LENGTH_MM_B = 30.00
ARM_L3_LENGTH_MM_A_0 = 144.49
ARM_L3_LENGTH_MM_B_0 = 0.0
ARM_L4_LENGTH_MM_A = 171.67
ARM_L4_LENGTH_MM_B = 13.69

_L2 = math.sqrt(ARM_L2_LENGTH_MM_A ** 2 + ARM_L2_LENGTH_MM_B ** 2)
_T2_RAD = math.atan2(ARM_L2_LENGTH_MM_B, ARM_L2_LENGTH_MM_A)
_L3 = math.sqrt(ARM_L3_LENGTH_MM_A_0 ** 2 + ARM_L3_LENGTH_MM_B_0 ** 2)
_T3_RAD = math.atan2(ARM_L3_LENGTH_MM_B_0, ARM_L3_LENGTH_MM_A_0)
_LE = math.sqrt(ARM_L4_LENGTH_MM_A ** 2 + ARM_L4_LENGTH_MM_B ** 2)
_TE_RAD = math.atan2(ARM_L4_LENGTH_MM_B, ARM_L4_LENGTH_MM_A)


def _simple_linkage_ik_rad(a_in, b_in):
    la = _L2
    lb = _L3
    if abs(b_in) < 1e-6:
        psi = math.acos(
            (la * la + a_in * a_in - lb * lb) / (2 * la * a_in)
        ) + _T2_RAD
        alpha = math.pi / 2.0 - psi
        omega = math.acos(
            (a_in * a_in + lb * lb - la * la) / (2 * a_in * lb)
        )
        beta = psi + omega - _T3_RAD
    else:
        l2c = a_in * a_in + b_in * b_in
        lc = math.sqrt(l2c)
        linkage_lambda = math.atan2(b_in, a_in)
        psi = math.acos(
            (la * la + l2c - lb * lb) / (2 * la * lc)
        ) + _T2_RAD
        alpha = math.pi / 2.0 - linkage_lambda - psi
        omega = math.acos(
            (lb * lb + l2c - la * la) / (2 * lc * lb)
        )
        beta = psi + omega - _T3_RAD

    delta = math.pi / 2.0 - alpha - beta
    return [alpha, beta, delta]


def _rotate_point(theta):
    alpha = _TE_RAD + theta
    return [-_LE * math.cos(alpha), -_LE * math.sin(alpha)]


def _move_point(x_a, y_a, distance_to_move):
    distance = math.sqrt(x_a ** 2 + y_a ** 2)
    if distance - distance_to_move <= 1e-6:
        return [0.0, 0.0]
    ratio = (distance - distance_to_move) / distance
    return [x_a * ratio, y_a * ratio]


def _cartesian_to_polar(x, y):
    return [math.sqrt(x * x + y * y), math.atan2(y, x)]


def compute_joint_rad_by_pos(x_mm, y_mm, z_mm, roll_rad, pitch_rad):
    """Return Waveshare's five-joint RoArm M3 IK result.

    This follows upstream ``computeJointRadbyPos`` exactly. A ValueError is
    raised when the inputs are non-finite or the upstream equations have no
    finite solution, so an invalid target cannot reach the motion command.
    """
    values = [x_mm, y_mm, z_mm, roll_rad, pitch_rad]
    try:
        x_mm, y_mm, z_mm, roll_rad, pitch_rad = [float(v) for v in values]
    except (TypeError, ValueError):
        raise ValueError("Cartesian pose must contain five numeric values")
    if not all(math.isfinite(value) for value in
               (x_mm, y_mm, z_mm, roll_rad, pitch_rad)):
        raise ValueError("Cartesian pose values must be finite")

    try:
        # Upstream intentionally uses the decimal constant 3.1416 here.
        delta = _rotate_point(pitch_rad - 3.1416)
        beta = _move_point(x_mm, y_mm, delta[0])
        bases = _cartesian_to_polar(beta[0], beta[1])
        radians = _simple_linkage_ik_rad(bases[0], z_mm + delta[1])
        wrist_joint_rad = radians[2] + pitch_rad
        result = [
            bases[1],
            radians[0],
            radians[1],
            wrist_joint_rad,
            roll_rad,
        ]
    except (ValueError, ZeroDivisionError, OverflowError):
        raise ValueError("Waveshare IK found no finite solution for this pose")

    if not all(math.isfinite(value) for value in result):
        raise ValueError("Waveshare IK found no finite solution for this pose")
    return result
