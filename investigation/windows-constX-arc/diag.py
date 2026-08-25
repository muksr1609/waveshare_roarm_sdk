#!/usr/bin/env python3
"""Controlled responsiveness diagnostic for COM21: read the current pose, send ONE
clear base-joint command (+0.3 rad), and watch the base position over ~3 s to see
how quickly the arm starts moving and whether the feedback is fresh. Then move the
base back. No Cartesian motion, no arc."""
import importlib.util, inspect, json, math, os, sys, time

here = os.path.dirname(os.path.abspath(__file__))
ik_path = os.path.join(os.path.dirname(os.path.dirname(here)), "roarm_sdk", "waveshare_ik.py")
import roarm_sdk
sdk_dir = os.path.dirname(os.path.abspath(inspect.getfile(roarm_sdk)))
assert "site-packages/roarm_sdk" in sdk_dir.replace(os.sep, "/"), sdk_dir
from roarm_sdk.roarm import roarm

def valid(fb):
    return fb is not None and len(fb) >= 10 and all(math.isfinite(v) for v in fb[:10])

print("opening COM21")
arm = roarm(roarm_type="roarm_m3", port="COM21", baudrate=115200)

# settle + current pose
cur = None
for _ in range(20):
    fb = arm.feedback_get()
    if valid(fb):
        cur = fb
        break
    time.sleep(0.2)
if cur is None:
    raise SystemExit("no valid feedback")
t0 = cur[4:10]
print("current pose  xyz=[%.2f %.2f %.2f]  base=%.4f shoulder=%.4f elbow=%.4f wrist=%.4f" % (
    cur[0], cur[1], cur[2], t0[0], t0[1], t0[2], t0[3]))

target_base = t0[0] + 0.3
cmd = list(t0[:5])
cmd[0] = target_base
cmd.append(t0[5])
print("sending base command: %.4f -> %.4f (speed=1000 acc=50)" % (t0[0], target_base))
t_start = time.monotonic()
arm.joints_radian_ctrl(cmd, 1000, 50)
print("time  base_cmd  base_meas  d_base(deg)  xyz")
first_move_t = None
for i in range(16):
    time.sleep(0.2)
    fb = arm.feedback_get()
    if not valid(fb):
        print("  %4.1fs  (invalid fb)" % (time.monotonic() - t_start)); continue
    b = fb[4]
    d = abs(b - t0[0])
    if first_move_t is None and d > math.radians(0.5):
        first_move_t = time.monotonic() - t_start
    print("  %4.1fs  %.4f    %.4f    %6.2f    (%.1f, %.1f, %.1f)" % (
        time.monotonic() - t_start, target_base, b, math.degrees(d), fb[0], fb[1], fb[2]))
print("first motion (>0.5 deg) at t = %s s" % ("%.2f" % first_move_t if first_move_t else "NEVER in 3.2s"))

# move base back
back = list(t0[:5]); back[0] = t0[0]; back.append(t0[5])
arm.joints_radian_ctrl(back, 1000, 50)
time.sleep(1.5)
fb = arm.feedback_get()
if valid(fb):
    print("after back-move: base=%.4f (was %.4f)  xyz=[%.1f %.1f %.1f]" % (fb[4], t0[0], fb[0], fb[1], fb[2]))
