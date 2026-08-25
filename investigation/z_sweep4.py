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
ARRIVAL_TOL_RAD = 0.08      # "reached" = within 4.6 deg of commanded joints
ALREADY_THERE_RAD = 0.03    # skip send only if truly on target
MAX_ATTEMPTS = 5
PRE_SEND_PAUSE_S = 1.0
RETRY_WAIT_S = 8.0          # base recovery wait; escalates per attempt
DWELL_S = 3.0


def read_fb(arm):
    fb = arm.feedback_get()
    if not isinstance(fb, list) or len(fb) < 10:
        return None
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v)) for v in fb[4:10]):
        return None
    if isinstance(fb[2], bool) or not isinstance(fb[2], (int, float)) or not math.isfinite(float(fb[2])):
        return None
    return fb


def joints_of(fb):
    return [float(v) for v in fb[4:10]]


def max_err(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def wait_settled(arm, timeout_s=SETTLE_TIMEOUT_S):
    prev = None
    stable = 0
    last = None
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        fb = read_fb(arm)
        if fb is not None:
            last = fb
            j = joints_of(fb)
            if prev is not None:
                step = max(abs(a - b) for a, b in zip(j, prev))
                stable = stable + 1 if step <= STABLE_RAD else 1
            else:
                stable = 1
            prev = j
            if stable >= STABLE_SAMPLES:
                return fb
        time.sleep(POLL_S)
    return last


def degs(vals):
    return [round(math.degrees(v), 2) for v in vals]


def roundtrip_err(fb):
    """Feed the firmware's own (x,y,z,roll,pitch) back through IK; max joint err (deg)."""
    try:
        x, y, z = float(fb[0]), float(fb[1]), float(fb[2])
        tit, r = float(fb[3]), float(fb[8])
        ik = compute_joint_rad_by_pos(x, y, z, r, tit)
        return max(abs(math.degrees(a) - math.degrees(c)) for a, c in zip(ik, joints_of(fb)[:5]))
    except Exception:
        return float("nan")


def attempt_pose(arm, target):
    """Drive one pose until settled within ARRIVAL_TOL. Returns (fb, attempts, reached)."""
    for attempt in range(1, MAX_ATTEMPTS + 1):
        fb = wait_settled(arm, timeout_s=10.0)   # be settled before commanding
        if fb is not None and max_err(joints_of(fb), target) <= ALREADY_THERE_RAD:
            return fb, attempt, True
        time.sleep(PRE_SEND_PAUSE_S)
        arm.joints_radian_ctrl(radians=[float(v) for v in target], speed=SPEED, acc=ACC)
        time.sleep(1.0)
        fb = wait_settled(arm)
        if fb is not None and max_err(joints_of(fb), target) <= ARRIVAL_TOL_RAD:
            return fb, attempt, True
        if attempt < MAX_ATTEMPTS:
            wait_s = RETRY_WAIT_S * attempt      # escalate: 8,16,24,32
            print("   ...dropped/short on try %d; recovery wait %.0fs" % (attempt, wait_s))
            sys.stdout.flush()
            time.sleep(wait_s)
    fb = wait_settled(arm, timeout_s=5.0)
    reached = fb is not None and max_err(joints_of(fb), target) <= ARRIVAL_TOL_RAD
    return fb, MAX_ATTEMPTS, reached


def main():
    arm = roarm(roarm_type="roarm_m3", port=PORT, baudrate=115200)
    try:
        time.sleep(1.0)
        fb0 = wait_settled(arm, timeout_s=15.0)
        if fb0 is None:
            print("no valid feedback at start")
            return 1
        j0 = joints_of(fb0)
        x0, y0 = float(fb0[0]), float(fb0[1])
        roll0, pitch0, grip0 = j0[4], float(fb0[3]), j0[5]
        print("START settled: x=%.2f y=%.2f z=%.2f mm | joints(deg)=%s" % (x0, y0, float(fb0[2]), degs(j0)))
        print("holding x=%.2f y=%.2f roll=%.4f pitch=%.4f gripper=%.4f (rad) constant\n" % (x0, y0, roll0, pitch0, grip0))
        print("%-6s %-9s %-9s %-8s %-5s %-6s %-6s  %s" % ("cmd_z", "meas_z", "mismatch", "max_jerr", "tries", "reached", "rt_err", "achieved joints (deg)"))
        print("  (rt_err = round-trip IK(firmware x,y,z) vs firmware joints, deg; ~0 means firmware self-consistent)")
        print("-" * 100)
        all_reached = True
        for tz in TARGETS_MM:
            target = compute_joint_rad_by_pos(x0, y0, float(tz), roll0, pitch0) + [grip0]
            fb, tries, reached = attempt_pose(arm, target)
            if fb is None:
                all_reached = False
                print("%-6d %-9s %-9s %-8s %-5d %-6s %-6s  %s" % (tz, "n/a", "n/a", "n/a", tries, "NO", "n/a", "NO FEEDBACK"))
                continue
            j = joints_of(fb)
            if not reached:
                all_reached = False
            jerr = max_err(j, target)
            mz = float(fb[2])
            rt = roundtrip_err(fb)
            print("%-6d %-9.2f %+9.2f %-8.4f %-5d %-6s %-6.2f  %s"
                  % (tz, mz, mz - float(tz), jerr, tries, "yes" if reached else "NO", rt, degs(j)))
            sys.stdout.flush()
            time.sleep(DWELL_S)

        print("\nALL POSES REACHED: %s" % ("yes" if all_reached else "no"))
        print("(mismatch = measured_firmware_z - commanded_z; max_jerr = achieved vs commanded, rad)")
        return 0
    finally:
        arm.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
