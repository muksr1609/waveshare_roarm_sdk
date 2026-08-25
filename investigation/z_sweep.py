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


def read_fb(arm):
    """Return (joints[6], tool_z) from one firmware feedback read, or (None, None)."""
    fb = arm.feedback_get()
    if not isinstance(fb, list) or len(fb) < 10:
        return None, None
    joints = fb[4:10]
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)) for v in joints):
        return None, None
    z = fb[2]
    if isinstance(z, bool) or not isinstance(z, (int, float)) or not math.isfinite(float(z)):
        return None, None
    return [float(v) for v in joints], float(z)


def wait_settled(arm, timeout_s=SETTLE_TIMEOUT_S):
    """Poll until joints are stable (3 consecutive small steps). Returns last (joints, z)."""
    prev = None
    stable = 0
    last = (None, None)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        j, z = read_fb(arm)
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


def main():
    arm = roarm(roarm_type="roarm_m3", port=PORT, baudrate=115200)
    try:
        time.sleep(0.3)
        j0, z0 = read_fb(arm)
        if j0 is None:
            print("no valid feedback at start")
            return 1
        fb0 = arm.feedback_get()
        x0, y0 = float(fb0[0]), float(fb0[1])
        roll0 = j0[4]            # wrist roll joint r
        pitch0 = math.radians(float(arm.pose_get()[3]))  # tit
        grip0 = j0[5]            # gripper g
        print("START: tip x=%.2f y=%.2f z=%.2f mm | joints(deg)=%s" % (x0, y0, z0, degs(j0)))
        print("holding x=%.2f y=%.2f roll=%.3f pitch=%.3f gripper=%.3f (rad) constant\n"
              % (x0, y0, roll0, pitch0, grip0))
        print("%-8s %-10s %-10s %-10s  %s" % ("cmd_z", "meas_z", "mismatch", "max_jerr", "achieved joints (deg)"))
        print("-" * 78)

        for tz in TARGETS_MM:
            target5 = compute_joint_rad_by_pos(x0, y0, float(tz), roll0, pitch0)
            target = target5 + [grip0]
            arm.joints_radian_ctrl(radians=[float(v) for v in target], speed=SPEED, acc=ACC)
            time.sleep(1.0)  # let the move start before settling poll
            ach_j, ach_z = wait_settled(arm)
            if ach_j is None:
                print("%-8d %-10s %-10s %-10s  %s" % (tz, "n/a", "n/a", "n/a", "NO FEEDBACK"))
                continue
            jerr = max(abs(a - b) for a, b in zip(ach_j, target))
            mismatch = ach_z - float(tz)
            print("%-8d %-10.2f %+10.2f  %-10.4f  %s" % (tz, ach_z, mismatch, jerr, degs(ach_j)))
            sys.stdout.flush()

        print("\n(mismatch = measured_firmware_z - commanded_z;  max_jerr = achieved vs commanded joints, rad)")
        return 0
    finally:
        arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
