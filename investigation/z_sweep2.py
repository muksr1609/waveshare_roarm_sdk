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
SETTLE_TIMEOUT_S = 20.0
DWELL_S = 1.5                 # let the stock controller finish before next cmd
PROBE_S = 2.5                 # wait this long to see if motion started
PROBE_MOVE_RAD = 0.02         # motion must exceed this to count as "started"


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


def send_with_retry(arm, target, pre_joints):
    """Send target; if no motion after PROBE_S, resend once. Returns # sends used."""
    arm.joints_radian_ctrl(radians=[float(v) for v in target], speed=SPEED, acc=ACC)
    time.sleep(PROBE_S)
    j, _, _ = read_fb(arm)
    if j is not None:
        moved = max(abs(a - b) for a, b in zip(j, pre_joints))
        if moved <= PROBE_MOVE_RAD:
            print("   (no motion after %.1fs -> resending target once)" % PROBE_S)
            arm.joints_radian_ctrl(radians=[float(v) for v in target], speed=SPEED, acc=ACC)
            return 2
    return 1


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
        print("%-7s %-9s %-10s %-9s %-4s  %s" % ("cmd_z", "meas_z", "mismatch", "max_jerr", "sends", "achieved joints (deg)"))
        print("-" * 80)

        for tz in TARGETS_MM:
            target = compute_joint_rad_by_pos(x0, y0, float(tz), roll0, pitch0) + [grip0]
            pre, _, _ = read_fb(arm)
            if pre is None:
                pre = j0
            sends = send_with_retry(arm, target, pre)
            ach_j, ach_z = wait_settled(arm)
            if ach_j is None:
                print("%-7d %-9s %-10s %-9s %-4d  %s" % (tz, "n/a", "n/a", "n/a", sends, "NO FEEDBACK"))
                continue
            jerr = max(abs(a - b) for a, b in zip(ach_j, target))
            mismatch = ach_z - float(tz)
            print("%-7d %-9.2f %+10.2f %-9.4f %-4d  %s" % (tz, ach_z, mismatch, jerr, sends, degs(ach_j)))
            sys.stdout.flush()
            time.sleep(DWELL_S)

        print("\n(mismatch = measured_firmware_z - commanded_z;  max_jerr = achieved vs commanded joints, rad)")
        return 0
    finally:
        arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
