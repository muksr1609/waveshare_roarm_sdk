import math
import sys

sys.path.insert(0, r"C:\Users\Mukund\Documents\Ro-Arm")

from roarm_sdk.roarm import roarm
from roarm_sdk.waveshare_ik import compute_joint_rad_by_pos

arm = roarm(roarm_type="roarm_m3", port="COM21", baudrate=115200)
try:
    fb = arm.feedback_get()
    x, y, z = float(fb[0]), float(fb[1]), float(fb[2])
    tit = float(fb[3])
    joints = [float(v) for v in fb[4:10]]
    b, s, e, t, r, g = joints
    print("firmware: x=%.2f y=%.2f z=%.2f" % (x, y, z))
    print("joints (deg): b=%.2f s=%.2f e=%.2f t=%.2f r=%.2f g=%.2f" %
          tuple(math.degrees(v) for v in joints))
    print("tit=%.4f rad (%.2f deg)" % (tit, math.degrees(tit)))
    print()

    candidates = {
        "roll=r, pitch=tit": (r, tit),
        "roll=r, pitch=0": (r, 0.0),
        "roll=r, pitch=t (wrist joint)": (r, t),
        "roll=0, pitch=tit": (0.0, tit),
        "roll=0, pitch=0": (0.0, 0.0),
    }
    print("%-32s %s   max_err(deg)" % ("IK inputs", "-> IK joints (deg)"))
    for name, (roll, pitch) in candidates.items():
        try:
            ik5 = compute_joint_rad_by_pos(x, y, z, roll, pitch)
            err = max(abs(math.degrees(a) - math.degrees(c)) for a, c in zip(ik5, joints[:5]))
            print("%-32s [%s]  %.2f" % (name,
                  ", ".join("%.2f" % math.degrees(v) for v in ik5), err))
        except ValueError as exc:
            print("%-32s FAILED: %s" % (name, exc))
finally:
    arm.disconnect()
