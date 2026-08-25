import math
import sys
import time

sys.path.insert(0, r"C:\Users\Mukund\Documents\Ro-Arm")

from roarm_sdk.roarm import roarm
from roarm_sdk.waveshare_ik import compute_joint_rad_by_pos

PORT = "COM21"
SPEED = 180
ACC = 10
TARGETS_MM = [100, 120, 140, 160, 180, 200]

POLL_S = 0.1
STABLE_RAD = 0.01
STABLE_SAMPLES = 3
SETTLE_TIMEOUT_S = 25.0
ARRIVAL_TOL_RAD = 0.12     # "reached" = within this of the commanded joints
MAX_ATTEMPTS = 8
DWELL_S = 1.5
PRE_SEND_PAUSE_S = 0.5


def read_fb(arm):
    fb = arm.feedback_get()
    if not isinstance(fb, list) or len(fb) < 10:
        return None, None, None
    joints = fb[4:10]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)) for v in joints):
        return None, None, None
    z = fb[2]
    if isinstance(z, bool) or not isinstance(z, (int, float)) or not math.isfinite(float(z)):
        return None, None, None
    return [float(v) for v in joints], float(z), (float(fb[0]), float(fb[1]))


def max_err(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def wait_settled(arm, timeout_s=SETTLE_TIMEOUT_S):
    prev = None
    stable = 0
    last = (None, None)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        j, z, _ = read_fb(arm)
        if j is not None:
            last = (j, z)
            if prev is not None:
                step = max(abs(a - b) for a, b in zip(j, prev))
                stable = stable + 1 if step <= STABLE_RAD else 1
            else:
                stable = 1
            prev = j
            if stable >= STABLE_SAMPLES:
                return last
        time.sleep(POLL_S)
    return last


def degs(vals):
    return [round(math.degrees(v), 2) for v in vals]


def attempt_pose(arm, target):
    """Drive one pose to completion. Returns (ach_j, ach_z, attempts, reached)."""
    for attempt in range(1, MAX_ATTEMPTS + 1):
        cur_j, cur_z, _ = read_fb(arm)
        if cur_j is not None and max_err(cur_j, target) <= ARRIVAL_TOL_RAD:
            j, z = wait_settled(arm, timeout_s=6.0)
            if j is not None and max_err(j, target) <= ARRIVAL_TOL_RAD:
                return j, z, attempt, True
        time.sleep(PRE_SEND_PAUSE_S)
        arm.joints_radian_ctrl(radians=[float(v) for v in target], speed=SPEED, acc=ACC)
        time.sleep(1.0)
        j, z = wait_settled(arm)
        if j is not None and max_err(j, target) <= ARRIVAL_TOL_RAD:
            return j, z, attempt, True
        # stopped short / no feedback / timeout -> loop and re-send same pose
    j, z, _ = read_fb(arm)
    reached = j is not None and max_err(j, target) <= ARRIVAL_TOL_RAD
    return j, z, MAX_ATTEMPTS, reached


def main():
    arm = roarm(roarm_type="roarm_m3", port=PORT, baudrate=115200)
    try:
        time.sleep(0.5)
        j0, z0, (x0, y0) = read_fb(arm)
        if j0 is None:
            print("no valid feedback at start")
            return 1
        roll0 = j0[4]
        pitch0 = math.radians(float(arm.pose_get()[3]))
        grip0 = j0[5]
        print("START: tip x=%.2f y=%.2f z=%.2f mm | joints(deg)=%s" % (x0, y0, z0, degs(j0)))
        print("holding x=%.2f y=%.2f roll=%.4f pitch=%.4f gripper=%.4f (rad) constant\n"
              % (x0, y0, roll0, pitch0, grip0))
        print("%-6s %-9s %-9s %-8s %-7s %-8s  %s" % ("cmd_z", "meas_z", "mismatch", "max_jerr", "tries", "reached", "achieved joints (deg)"))
        print("-" * 88)
        all_reached = True
        for tz in TARGETS_MM:
            target = compute_joint_rad_by_pos(x0, y0, float(tz), roll0, pitch0) + [grip0]
            ach_j, ach_z, tries, reached = attempt_pose(arm, target)
            if ach_j is None:
                all_reached = False
                print("%-6d %-9s %-9s %-8s %-7d %-8s  %s" % (tz, "n/a", "n/a", "n/a", tries, "NO", "NO FEEDBACK"))
                continue
            if not reached:
                all_reached = False
            jerr = max_err(ach_j, target)
            mismatch = ach_z - float(tz)
            print("%-6d %-9.2f %+9.2f %-8.4f %-7d %-8s  %s"
                  % (tz, ach_z, mismatch, jerr, tries, "yes" if reached else "NO", degs(ach_j)))
            sys.stdout.flush()
            time.sleep(DWELL_S)

        print("\nALL POSES REACHED: %s" % ("yes" if all_reached else "no"))
        print("(mismatch = measured_firmware_z - commanded_z; max_jerr = achieved vs commanded, rad)")
        return 0
    finally:
        arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
