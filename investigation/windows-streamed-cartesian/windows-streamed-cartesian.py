#!/usr/bin/env python3
"""Streamed Cartesian toolpath demo for the Waveshare RoArm-M3-S (native Windows).

This is a platform-port of investigation/streamed-cartesian-demo.py (the known-good
WSL/Linux run, commit 3974bdc). The control method is intentionally UNCHANGED:

    desired constant-Z XY arc
      -> Waveshare upstream Cartesian IK (roarm_sdk/waveshare_ik.py, unmodified)
      -> continuously streamed changing joint targets (20 Hz, speed=1000, acc=50)
      -> physical arm, with live feedback logged every cycle

Same trajectory size, same waypoint count, same frequency, same speed/acc, same
forward-once/reverse-once sequence, same validation rules and same safety
thresholds. The ONLY changed variable is the platform: WSL/Linux -> native
Windows, driving COM21 with the STOCK pip roarm-sdk.

What is different from the WSL script (all platform/recording only, none of them
touch the motion algorithm):
  * --port defaults to COM21; --out/--log default to this directory.
  * IK module search also includes the repo-root roarm_sdk/waveshare_ik.py.
  * SDK guard is adapted to the Windows site-packages path (and still refuses the
    repo SDK).
  * Records Python/SDK/pyserial versions, roarm_sdk.__file__, and the physical
    serial-line settings (RTS/DTR/timeout/dsrdtr/rtscts) actually in force.
  * Pre-motion: confirms COM21 opens, then reads 10 consecutive feedback frames
    and aborts before any motion if communication is already unstable.
  * run_leg additionally records command/feedback timestamps, per-step tracking
    error, the achieved command rate, and detects a sustained mid-trajectory
    freeze (the historical Windows failure mode). It does NOT chase endpoints.

Usage:
    python investigation\\windows-streamed-cartesian\\windows-streamed-cartesian.py [--port COM21]
"""

import argparse
import collections
import datetime
import importlib.util
import inspect
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

# Mid-trajectory freeze detection (diagnostic for the historical Windows failure).
# Over a window of FREEZE_WIN waypoints (== 1 s at 20 Hz), if the arm is clearly
# commanded to move (cmd motion > FREEZE_CMD_TOL) but the measured joints barely
# move (meas motion < FREEZE_MEAS_TOL), that window is a "freeze". A freeze
# persisting for FREEZE_SUSTAIN consecutive windows aborts the leg. The check is
# only armed after the first FREEZE_ARM waypoints so startup lag is not counted.
FREEZE_WIN = 20
FREEZE_ARM = 20
FREEZE_MEAS_TOL_RAD = math.radians(0.5)
FREEZE_CMD_TOL_RAD = math.radians(3.0)
FREEZE_SUSTAIN = 2


def load_ik_module():
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "roarm_sdk", "waveshare_ik.py"),
        os.path.join(here, "waveshare_ik.py"),
        os.path.join(os.path.dirname(here), "roarm_sdk", "waveshare_ik.py"),
        # Repo-root copy used by the WSL run (here is one level deeper than the
        # original script, so the Ro-Arm root is two levels up).
        os.path.join(os.path.dirname(os.path.dirname(here)), "roarm_sdk", "waveshare_ik.py"),
        os.environ.get("WAVESHARE_IK", ""),
    ]
    for path in candidates:
        if path and os.path.isfile(path):
            spec = importlib.util.spec_from_file_location("waveshare_ik", path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod, path
    raise SystemExit("waveshare_ik.py not found; set WAVESHARE_IK=C:\\path\\to\\waveshare_ik.py")


def _valid(fb):
    return fb is not None and isinstance(fb, (list, tuple)) and len(fb) >= 10 \
        and all(math.isfinite(v) for v in fb[:10])


def wait_valid_fb(arm, tries=10):
    for _ in range(tries):
        fb = arm.feedback_get()
        if _valid(fb):
            return fb
        time.sleep(0.25)
    return None


def read_stable_state(arm, frames=10):
    """Read `frames` consecutive feedback frames; require all valid/finite/plausible.

    Returns (last_fb, per_frame_records) on success, or (None, records) if any
    frame is invalid/non-finite/out-of-plausible-range.
    """
    records = []
    last = None
    for i in range(frames):
        t = time.monotonic()
        fb = arm.feedback_get()
        ok = _valid(fb)
        plausible = False
        if ok:
            # XYZ plausible for a RoArm M3 workspace; joints inside hard limits.
            plausible = (
                all(math.isfinite(fb[j]) for j in range(3))
                and all(0.0 <= fb[j] <= 400.0 for j in range(3))
                and all(JOINT_LIMITS[k][0] <= fb[4 + k] <= JOINT_LIMITS[k][1] for k in range(6))
            )
        records.append({
            "i": i,
            "t": round(t, 3),
            "valid": ok,
            "plausible": plausible,
            "xyz": list(fb[:3]) if ok else None,
            "joints": list(fb[4:10]) if ok else None,
        })
        if ok and plausible:
            last = fb
        time.sleep(0.1)
    return (last if (last is not None and records[-1]["valid"] and records[-1]["plausible"]) else None), records


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


def _serial_settings(port_obj):
    """Best-effort snapshot of the physical serial-line settings in force."""
    def g(name):
        try:
            v = getattr(port_obj, name)
            try:
                v = v()
            except TypeError:
                pass
            return v
        except Exception as e:
            return "unavailable: %r" % (e,)
    return {
        "port": g("port"),
        "baudrate": g("baudrate"),
        "timeout": g("timeout"),
        "rtscts": g("rtscts"),
        "dsrdtr": g("dsrdtr"),
        "xonxoff": g("xonxoff"),
        "rts": g("rts"),
        "dtr": g("dtr"),
        "is_open": g("is_open"),
    }


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
    cmd_ts_list, fb_ts_list = [], []
    intervals = []
    last_cmd_ts = None
    meas_hist = collections.deque(maxlen=FREEZE_WIN)
    cmd_hist = collections.deque(maxlen=FREEZE_WIN)
    freeze_run = 0
    freeze_windows = 0
    max_meas_motion_win = 0.0
    for i, cmd in enumerate(joints):
        now = time.monotonic()
        cmd_ts = time.monotonic()
        arm.joints_radian_ctrl(cmd, SPEED, ACC)
        if last_cmd_ts is not None:
            intervals.append(cmd_ts - last_cmd_ts)
        last_cmd_ts = cmd_ts
        cmd_ts_list.append(cmd_ts)
        fb = arm.feedback_get()
        fb_ts = time.monotonic()
        fb_ts_list.append(fb_ts)
        if _valid(fb):
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
            meas_hist.append(meas)
            cmd_hist.append(cmd)
            if i >= FREEZE_ARM and len(meas_hist) == FREEZE_WIN and len(cmd_hist) == FREEZE_WIN:
                old_m = meas_hist[0]
                old_c = cmd_hist[0]
                meas_motion = max(abs(meas[k] - old_m[k]) for k in range(5))
                cmd_motion = max(abs(cmd[k] - old_c[k]) for k in range(5))
                max_meas_motion_win = max(max_meas_motion_win, meas_motion)
                if meas_motion < FREEZE_MEAS_TOL_RAD and cmd_motion > FREEZE_CMD_TOL_RAD:
                    freeze_run += 1
                    freeze_windows += 1
                    if freeze_run >= FREEZE_SUSTAIN:
                        aborted = True
                        reason = ("sustained mid-trajectory freeze: measured motion "
                                  "< %.1f deg over %d consecutive 1 s windows at waypoint %d"
                                  % (math.degrees(FREEZE_MEAS_TOL_RAD), FREEZE_SUSTAIN, i))
                else:
                    freeze_run = 0
            logf.write("ts=%.3f cmd_ts=%.3f fb_ts=%.3f leg=%s idx=%d cmd=[%s] meas=[%s] xyz=%.2f,%.2f,%.2f track=%.4f\n" % (
                now, cmd_ts - t0, fb_ts - t0, leg_name, i,
                " ".join("%.4f" % v for v in cmd),
                " ".join("%.4f" % v for v in meas),
                fb[0], fb[1], fb[2], track))
        else:
            fb_bad += 1
            if fb_bad >= INVALID_FB_ABORT:
                aborted = True
                reason = "%d consecutive invalid feedback at waypoint %d" % (fb_bad, i)
            logf.write("ts=%.3f cmd_ts=%.3f fb_ts=%.3f leg=%s idx=%d INVALID_FEEDBACK\n" % (
                now, cmd_ts - t0, fb_ts - t0, leg_name, i))
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
        if _valid(fb):
            final_fb = fb
            z_record.append(fb[2])
        time.sleep(0.1)
    final_meas = final_fb[4:10] if final_fb else None
    final_err = None
    if final_meas is not None:
        final_err = [abs(final_meas[k] - joints[-1][k]) for k in range(5)]
    achieved_hz = None
    if len(intervals) > 1:
        achieved_hz = (len(intervals) - 1) / sum(intervals)
    return {
        "aborted": aborted,
        "abort_reason": reason,
        "duration_s": round(duration, 3),
        "commands_sent": int(i + 1),
        "fb_ok": fb_ok,
        "fb_invalid": fb_bad,
        "max_tracking_err_rad": max_track,
        "achieved_cmd_rate_hz": round(achieved_hz, 3) if achieved_hz is not None else None,
        "median_cmd_interval_s": round(sorted(intervals)[len(intervals) // 2], 5) if intervals else None,
        "max_cmd_interval_s": round(max(intervals), 5) if intervals else None,
        "min_cmd_interval_s": round(min(intervals), 5) if intervals else None,
        "freeze_windows": freeze_windows,
        "max_meas_motion_over_1s_window_rad": round(max_meas_motion_win, 5),
        "final_xyz_mm": list(final_fb[:3]) if final_fb else None,
        "final_joints_rad": list(final_meas) if final_meas else None,
        "final_joint_err_rad": final_err,
        "final_joint_err_max_rad": max(final_err) if final_err else None,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM21")
    ap.add_argument("--radius-mm", type=float, default=80.0)
    ap.add_argument("--sweep-deg", type=float, default=75.0)
    ap.add_argument("--max-excursion-deg", type=float, default=35.0)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--out", default=os.path.join(script_dir, "windows-result.json"))
    ap.add_argument("--log", default=os.path.join(script_dir, "windows.log"))
    args = ap.parse_args()

    ik_mod, ik_path = load_ik_module()
    ik_fn = ik_mod.compute_joint_rad_by_pos

    import roarm_sdk
    sdk_dir = os.path.dirname(os.path.abspath(inspect.getfile(roarm_sdk)))
    sdk_dir_norm = sdk_dir.replace(os.sep, "/")
    repo_root_norm = os.path.dirname(os.path.dirname(script_dir)).replace(os.sep, "/")
    if "site-packages/roarm_sdk" not in sdk_dir_norm:
        raise SystemExit("refusing to run: not the stock site-packages SDK: %s" % sdk_dir)
    if repo_root_norm in sdk_dir_norm:
        raise SystemExit("refusing to run: importing the repo SDK, not site-packages: %s" % sdk_dir)
    from roarm_sdk.roarm import roarm

    try:
        import serial as _pyserial
        pyserial_ver = _pyserial.__version__
    except Exception:
        pyserial_ver = None
    try:
        import importlib.metadata as _imd
        roarm_sdk_ver = _imd.version("roarm-sdk")
    except Exception:
        roarm_sdk_ver = "unknown"

    print("=== environment ===")
    print("python          %s (%s)" % (sys.version.split()[0], sys.executable))
    print("roarm-sdk       %s" % roarm_sdk_ver)
    print("roarm_sdk file  %s" % roarm_sdk.__file__)
    print("pyserial        %s" % pyserial_ver)
    print("ik module       %s" % ik_path)
    print("platform        %s" % sys.platform)

    print("opening %s" % args.port)
    import serial as _serial_mod
    t_open = time.monotonic()
    try:
        arm = roarm(roarm_type="roarm_m3", port=args.port, baudrate=115200)
    except _serial_mod.SerialException as e:
        raise SystemExit("could not open %s (another application may own it): %s" % (args.port, e))
    open_to_first_valid_fb = None
    serial_settings = _serial_settings(arm._serial_port)
    print("serial settings %s" % json.dumps(serial_settings))

    fb = wait_valid_fb(arm)
    open_to_first_valid_fb = time.monotonic() - t_open
    if fb is None:
        raise SystemExit("no valid feedback; aborting before any motion")

    # Pre-motion stability gate: 10 consecutive valid/finite/plausible frames.
    print("reading 10 consecutive feedback frames (stability gate)")
    stable_fb, stable_records = read_stable_state(arm, frames=10)
    for r in stable_records:
        print("  frame %2d valid=%s plausible=%s xyz=%s" % (
            r["i"], r["valid"], r["plausible"],
            ("[%.2f %.2f %.2f]" % tuple(r["xyz"])) if r["xyz"] else None))
    if stable_fb is None:
        raise SystemExit("communication unstable at open (10-frame gate failed); aborting before any motion")

    x0, y0, z0 = stable_fb[0], stable_fb[1], stable_fb[2]
    pitch0, roll0 = stable_fb[3], stable_fb[8]
    grip0 = stable_fb[9]
    start_joints = list(stable_fb[4:10])
    print("start pose  xyz=[%.2f %.2f %.2f]  pitch=%.4f  roll=%.4f" % (x0, y0, z0, pitch0, roll0))
    print("start joints [%s]  grip=%.4f" % (
        " ".join("%.4f" % v for v in start_joints), grip0))

    # Protocol for this comparison: NO silent shrinking. Only the full-size
    # 80 mm / 75 deg arc (with the same center/direction search) is tried. If it
    # cannot validate from the current pose, the physical test is aborted.
    candidates = [(args.radius_mm, args.sweep_deg)]
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
        logf.write("# %s  port=%s  sdk=%s  ik=%s  serial=%s  start_xyz=[%.2f %.2f %.2f]\n" % (
            datetime.datetime.now(datetime.timezone.utc).isoformat(),
            args.port, sdk_dir, ik_path, json.dumps(serial_settings), x0, y0, z0))
        forward = run_leg(arm, plan["joints"], "F", logf, z_record)
        print("forward  aborted=%s reason=%s fb_ok=%d fb_invalid=%d max_track=%.2f deg rate=%s Hz" % (
            forward["aborted"], forward["abort_reason"], forward["fb_ok"],
            forward["fb_invalid"], math.degrees(forward["max_tracking_err_rad"]),
            forward["achieved_cmd_rate_hz"]))
        reverse = None
        if not forward["aborted"]:
            print("--- executing reverse leg ---")
            reverse = run_leg(arm, list(reversed(plan["joints"])), "R", logf, z_record)
            print("reverse  aborted=%s reason=%s fb_ok=%d fb_invalid=%d max_track=%.2f deg rate=%s Hz" % (
                reverse["aborted"], reverse["abort_reason"], reverse["fb_ok"],
                reverse["fb_invalid"], math.degrees(reverse["max_tracking_err_rad"]),
                reverse["achieved_cmd_rate_hz"]))

    result = {
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "platform": sys.platform,
        "environment": {
            "python": sys.version.split()[0],
            "executable": sys.executable,
            "roarm_sdk_version": roarm_sdk_ver,
            "roarm_sdk_file": roarm_sdk.__file__,
            "pyserial_version": pyserial_ver,
            "ik_module": ik_path,
        },
        "port": args.port,
        "sdk": sdk_dir,
        "serial_settings": serial_settings,
        "open_to_first_valid_fb_s": round(open_to_first_valid_fb, 3),
        "first_frame_after_open": {
            "t_open_to_fb_s": round(open_to_first_valid_fb, 3),
            "xyz_mm": list(fb[:3]),
            "joints_rad": list(fb[4:10]),
        },
        "open_stability_frames": stable_records,
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
