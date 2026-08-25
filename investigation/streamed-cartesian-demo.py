#!/usr/bin/env python3
"""Streamed Cartesian toolpath demo for the Waveshare RoArm-M3-S.

    desired constant-Z XY arc
      -> Waveshare upstream Cartesian IK (roarm_sdk/waveshare_ik.py)
      -> continuously streamed changing joint targets (20 Hz, speed=1000, acc=50)
      -> physical arm, with live feedback logged every cycle

No ROS, no MoveIt. Starts from the arm's CURRENT measured pose; keeps Z, tool
pitch, tool roll and gripper constant. The entire path is planned and
validated before the first command is sent. Runs the arc once, then the same
path in reverse once, then stops.

Usage:
    python3 investigation/streamed-cartesian-demo.py [--port /dev/ttyUSB0]
"""

import argparse
import datetime
import importlib.util
import json
import math
import os
import sys
import time

SPEED = 1000
ACC = 50
RATE_HZ = 20.0
MIN_WAYPOINTS = 101
MAX_STEP_MM = 5.0

ARC_CANDIDATES = [(80.0, 75.0), (80.0, 60.0), (60.0, 75.0), (60.0, 60.0)]
JOINT_LIMITS = [(-3.3, 3.3), (-1.9, 1.9), (-1.2, 3.3), (-1.9, 1.9), (-3.3, 3.3), (-0.2, 1.9)]
JOINT_NAMES = ["base", "shoulder", "elbow", "wrist", "roll", "hand"]
LIMIT_MARGIN_RAD = math.radians(2.5)
MAX_ADJ_STEP_RAD = math.radians(5.0)
MAX_EXCURSION_RAD = math.radians(35.0)
SANITY_TOL_RAD = math.radians(3.0)
GROSS_ERR_RAD = math.radians(15.0)
GROSS_ERR_SUSTAIN = 8
NO_MOTION_TOL_RAD = math.radians(0.5)
INVALID_FB_ABORT = 3


def load_ik_module():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "roarm_sdk", "waveshare_ik.py"),
        os.path.join(here, "waveshare_ik.py"),
        os.path.join(os.path.dirname(here), "roarm_sdk", "waveshare_ik.py"),
        os.environ.get("WAVESHARE_IK", ""),
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            spec = importlib.util.spec_from_file_location("waveshare_ik", path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise SystemExit("waveshare_ik.py not found; set WAVESHARE_IK=/path/to/waveshare_ik.py")


def wait_valid_fb(arm, tries=10):
    for _ in range(tries):
        fb = arm.feedback_get()
        if fb and all(math.isfinite(v) for v in fb):
            return fb
        time.sleep(0.25)
    return None


def validate(joints, start_joints, max_excursion_rad):
    n = len(joints)
    for i in range(n):
        for k in range(6):
            v = joints[i][k]
            lo, hi = JOINT_LIMITS[k]
            if not (math.isfinite(v) and lo + LIMIT_MARGIN_RAD <= v <= hi - LIMIT_MARGIN_RAD):
                return None
        for k in range(5):
            if i == 0:
                if abs(joints[0][k] - start_joints[k]) > SANITY_TOL_RAD:
                    return None
            elif abs(joints[i][k] - joints[i - 1][k]) > MAX_ADJ_STEP_RAD:
                return None
    for k in range(6):
        col = [j[k] for j in joints]
        if max(col) - min(col) > max_excursion_rad:
            return None
    return True


def plan_arc(ik_fn, x0, y0, z0, pitch0, roll0, grip0, start_joints, candidates,
             max_excursion_rad):
    for R, S in candidates:
        Srad = math.radians(S)
        n = max(MIN_WAYPOINTS, int(round(R * Srad / MAX_STEP_MM)) + 1)
        for offx, offy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            cx, cy = x0 + R * offx, y0 + R * offy
            a0 = math.atan2(y0 - cy, x0 - cx)
            for sign in (1, -1):
                waypoints, joints = [], []
                ok = True
                for i in range(n):
                    a = a0 + sign * Srad * i / (n - 1)
                    x, y = cx + R * math.cos(a), cy + R * math.sin(a)
                    try:
                        j5 = ik_fn(x, y, z0, roll0, pitch0)
                    except ValueError:
                        ok = False
                        break
                    joints.append(j5 + [grip0])
                    waypoints.append([x, y, z0])
                if not ok or not validate(joints, start_joints, max_excursion_rad):
                    continue
                path_len = sum(
                    math.dist(waypoints[i], waypoints[i + 1]) for i in range(n - 1)
                )
                xs = [w[0] for w in waypoints]
                ys = [w[1] for w in waypoints]
                zs = [w[2] for w in waypoints]
                excursion, max_step = {}, 0.0
                for k in range(5):
                    col = [j[k] for j in joints]
                    excursion[JOINT_NAMES[k]] = max(col) - min(col)
                    for i in range(1, n):
                        max_step = max(max_step, abs(joints[i][k] - joints[i - 1][k]))
                return {
                    "radius_mm": R,
                    "sweep_deg": S,
                    "n_waypoints": n,
                    "center_mm": [cx, cy],
                    "sweep_sign": sign,
                    "waypoints": waypoints,
                    "joints": joints,
                    "path_length_mm": path_len,
                    "cartesian_bbox_mm": {"x": [min(xs), max(xs)], "y": [min(ys), max(ys)], "z": [min(zs), max(zs)]},
                    "max_joint_excursion_rad": excursion,
                    "max_adjacent_step_rad": max_step,
                }
    return None


def run_leg(arm, joints, leg_name, logf, z_record):
    start_fb = wait_valid_fb(arm)
    if start_fb is None:
        return {"aborted": True, "abort_reason": "no valid feedback before leg start"}
    start_meas = start_fb[4:10]
    z_record.append(start_fb[2])
    t0 = time.monotonic()
    fb_ok = fb_bad = 0
    max_track = 0.0
    gross_run = 0
    aborted, reason = False, None
    for i, cmd in enumerate(joints):
        now = time.monotonic()
        arm.joints_radian_ctrl(cmd, SPEED, ACC)
        fb = arm.feedback_get()
        if fb and all(math.isfinite(v) for v in fb):
            fb_ok += 1
            fb_bad = 0
            meas = fb[4:10]
            z_record.append(fb[2])
            track = max(abs(meas[k] - cmd[k]) for k in range(5))
            max_track = max(max_track, track)
            if track > GROSS_ERR_RAD:
                gross_run += 1
                if gross_run >= GROSS_ERR_SUSTAIN:
                    aborted = True
                    reason = "sustained gross tracking error %.1f deg at waypoint %d" % (
                        math.degrees(track), i)
            else:
                gross_run = 0
            if i == int(RATE_HZ):
                moved = max(abs(start_meas[k] - meas[k]) for k in range(5))
                if moved < NO_MOTION_TOL_RAD:
                    aborted = True
                    reason = "no motion start after 1 s (moved %.2f deg)" % math.degrees(moved)
            logf.write("ts=%.3f leg=%s idx=%d cmd=[%s] meas=[%s] xyz=%.2f,%.2f,%.2f\n" % (
                now, leg_name, i,
                " ".join("%.4f" % v for v in cmd),
                " ".join("%.4f" % v for v in meas),
                fb[0], fb[1], fb[2]))
        else:
            fb_bad += 1
            if fb_bad >= INVALID_FB_ABORT:
                aborted = True
                reason = "%d consecutive invalid feedback at waypoint %d" % (fb_bad, i)
            logf.write("ts=%.3f leg=%s idx=%d INVALID_FEEDBACK\n" % (now, leg_name, i))
        if aborted:
            break
        target_t = t0 + i / RATE_HZ
        delay = target_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    duration = time.monotonic() - t0
    time.sleep(1.0)
    final_fb = None
    for _ in range(10):
        fb = arm.feedback_get()
        if fb and all(math.isfinite(v) for v in fb):
            final_fb = fb
            z_record.append(fb[2])
        time.sleep(0.1)
    final_meas = final_fb[4:10] if final_fb else None
    final_err = None
    if final_meas is not None:
        final_err = [abs(final_meas[k] - joints[-1][k]) for k in range(5)]
    return {
        "aborted": aborted,
        "abort_reason": reason,
        "duration_s": round(duration, 3),
        "commands_sent": int(i + 1) if not aborted else int(i + 1),
        "fb_ok": fb_ok,
        "fb_invalid": fb_bad,
        "max_tracking_err_rad": max_track,
        "final_xyz_mm": list(final_fb[:3]) if final_fb else None,
        "final_joints_rad": list(final_meas) if final_meas else None,
        "final_joint_err_rad": final_err,
        "final_joint_err_max_rad": max(final_err) if final_err else None,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--radius-mm", type=float, default=80.0)
    ap.add_argument("--sweep-deg", type=float, default=75.0)
    ap.add_argument("--max-excursion-deg", type=float, default=35.0)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--out", default=os.path.join(script_dir, "streamed-cartesian-result.json"))
    ap.add_argument("--log", default=os.path.join(script_dir, "streamed-cartesian.log"))
    args = ap.parse_args()

    ik_mod = load_ik_module()
    ik_fn = ik_mod.compute_joint_rad_by_pos

    import inspect
    import roarm_sdk
    sdk_dir = os.path.dirname(os.path.abspath(inspect.getfile(roarm_sdk)))
    if "site-packages/roarm_sdk" not in sdk_dir or "/mnt/c/" in sdk_dir:
        raise SystemExit("refusing to run: not the stock user SDK: %s" % sdk_dir)
    from roarm_sdk.roarm import roarm

    print("opening %s" % args.port)
    arm = roarm(roarm_type="roarm_m3", port=args.port, baudrate=115200)
    fb = wait_valid_fb(arm)
    if fb is None:
        raise SystemExit("no valid feedback; aborting before any motion")
    x0, y0, z0 = fb[0], fb[1], fb[2]
    pitch0, roll0 = fb[3], fb[8]
    grip0 = fb[9]
    start_joints = list(fb[4:10])
    print("start pose  xyz=[%.2f %.2f %.2f]  pitch=%.4f  roll=%.4f" % (x0, y0, z0, pitch0, roll0))
    print("start joints [%s]  grip=%.4f" % (
        " ".join("%.4f" % v for v in start_joints), grip0))

    candidates = [(args.radius_mm * f, args.sweep_deg * f) for f in (1.0, 0.75, 0.5, 0.35)]
    for c in ARC_CANDIDATES:
        if c not in candidates:
            candidates.append(c)
    plan = plan_arc(ik_fn, x0, y0, z0, pitch0, roll0, grip0, start_joints, candidates,
                    math.radians(args.max_excursion_deg))
    if plan is None:
        raise SystemExit("no candidate arc passed validation; no motion sent")
    print("planned arc  R=%.0f mm  sweep=%.0f deg  sign=%+d  center=[%.1f %.1f]" % (
        plan["radius_mm"], plan["sweep_deg"], plan["sweep_sign"],
        plan["center_mm"][0], plan["center_mm"][1]))
    print("path length  %.1f mm   waypoints=%d   rate=%.0f Hz   duration=%.2f s" % (
        plan["path_length_mm"], plan["n_waypoints"], RATE_HZ,
        (plan["n_waypoints"] - 1) / RATE_HZ))
    print("cartesian bbox mm  x=[%.2f %.2f]  y=[%.2f %.2f]  z=[%.2f %.2f]" % (
        plan["cartesian_bbox_mm"]["x"][0], plan["cartesian_bbox_mm"]["x"][1],
        plan["cartesian_bbox_mm"]["y"][0], plan["cartesian_bbox_mm"]["y"][1],
        plan["cartesian_bbox_mm"]["z"][0], plan["cartesian_bbox_mm"]["z"][1]))
    print("max joint excursion  " + "  ".join(
        "%s %.2f deg" % (n, math.degrees(v)) for n, v in
        plan["max_joint_excursion_rad"].items()))
    print("max adjacent step  %.3f deg" % math.degrees(plan["max_adjacent_step_rad"]))
    print("constant Z=%.2f mm  pitch=%.4f  roll=%.4f  grip=%.4f (unchanged)" % (
        z0, pitch0, roll0, grip0))
    print("--- validation passed; executing forward leg ---")

    z_record = []
    with open(args.log, "w") as logf:
        logf.write("# %s  port=%s  sdk=%s  start_xyz=[%.2f %.2f %.2f]\n" % (
            datetime.datetime.now(datetime.timezone.utc).isoformat(),
            args.port, sdk_dir, x0, y0, z0))
        forward = run_leg(arm, plan["joints"], "F", logf, z_record)
        print("forward  aborted=%s reason=%s fb_ok=%d fb_invalid=%d max_track=%.2f deg" % (
            forward["aborted"], forward["abort_reason"], forward["fb_ok"],
            forward["fb_invalid"], math.degrees(forward["max_tracking_err_rad"])))
        reverse = None
        if not forward["aborted"]:
            print("--- executing reverse leg ---")
            reverse = run_leg(arm, list(reversed(plan["joints"])), "R", logf, z_record)
            print("reverse  aborted=%s reason=%s fb_ok=%d fb_invalid=%d max_track=%.2f deg" % (
                reverse["aborted"], reverse["abort_reason"], reverse["fb_ok"],
                reverse["fb_invalid"], math.degrees(reverse["max_tracking_err_rad"])))

    result = {
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "port": args.port,
        "sdk": sdk_dir,
        "speed_acc": [SPEED, ACC],
        "rate_hz": RATE_HZ,
        "start_pose": {
            "xyz_mm": [x0, y0, z0],
            "pitch_rad": pitch0,
            "roll_rad": roll0,
            "gripper": grip0,
            "joints_rad": start_joints,
        },
        "planned_arc": {k: v for k, v in plan.items() if k not in ("waypoints", "joints")},
        "planned_waypoints_xy": [[round(w[0], 3), round(w[1], 3)] for w in plan["waypoints"]],
        "planned_joints_rad": [[round(v, 5) for v in j] for j in plan["joints"]],
        "measured_z_range_mm": [min(z_record), max(z_record)] if z_record else None,
        "forward": forward,
        "reverse": reverse,
    }
    with open(args.out, "w") as fh:
        json.dump(result, fh, indent=1)
    print("wrote %s" % args.out)
    print("wrote %s" % args.log)
    if forward["aborted"] or (reverse is not None and reverse["aborted"]):
        print("RESULT: ABORTED")
        return 1
    print("RESULT: COMPLETE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
