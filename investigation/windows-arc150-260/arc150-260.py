#!/usr/bin/env python3
"""150 mm / 260 deg streamed Cartesian arc for the Waveshare RoArm-M3-S (native Windows).

Sequence (per the request): HOME the arm first, then run the requested arc, on
Windows (not WSL), with the STOCK pip roarm-sdk.

The control method is the KNOWN-GOOD streamed method from
investigation/windows-streamed-cartesian (commit 316e5d4): Waveshare upstream
Cartesian IK -> continuously streamed changing joint targets (20 Hz,
speed=1000, acc=50) -> physical arm, live feedback logged every cycle, forward
once + reverse once.

Protocol changes vs the 80/75 run (only the ones this task requires):
  * The arm is HOMED first via the stock move_init() (-> [0,0,1.5708,0,0,0] rad)
    and the arc is planned from the home tip pose.
  * Radius/sweep default to 150 mm / 260 deg.
  * The "total joint excursion <= 35 deg" small-motion heuristic is DROPPED,
    because a 260 deg arc inherently requires a large base sweep. EVERY other
    safety rule is kept and applied to the UNWRAPPED joints:
        - every waypoint within the real joint limits (2.5 deg margin)
        - max adjacent joint step <= 5 deg
        - first waypoint within 3 deg of the current (home) joints
        - all IK finite
    so no command can leave the safety envelope.

Base-joint unwrap: the IK base angle is an atan2 in [-180,180] deg. A 260 deg
sweep makes that value wrap, which would look like a 360 deg jump. The base (and
all drive joints) are therefore unwrapped across waypoints so consecutive values
are continuous; limits/steps/jumps are all checked on the unwrapped (physical)
angles.

Usage:
    python arc150-260.py                # DRY: open, HOME, plan+validate, report (NO arc)
    python arc150-260.py --execute      # open, HOME, plan+validate, then execute F+R
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

JOINT_LIMITS = [(-3.3, 3.3), (-1.9, 1.9), (-1.2, 3.3), (-1.9, 1.9), (-3.3, 3.3), (-0.2, 1.9)]
JOINT_NAMES = ["base", "shoulder", "elbow", "wrist", "roll", "hand"]
LIMIT_MARGIN_RAD = math.radians(2.5)
MAX_ADJ_STEP_RAD = math.radians(5.0)
SANITY_TOL_RAD = math.radians(3.0)
GROSS_ERR_RAD = math.radians(15.0)
GROSS_ERR_SUSTAIN = 8
NO_MOTION_TOL_RAD = math.radians(0.5)
INVALID_FB_ABORT = 3

HOME_JOINTS = [0.0, 0.0, 1.5708, 0.0, 0.0, 0.0]
# The stock home target settles a few degrees off (gravity sag, mostly elbow),
# so accept "at home" within this tolerance and plan from the ACTUAL pose.
HOME_TOL_RAD = math.radians(5.0)

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
    records = []
    last = None
    for i in range(frames):
        t = time.monotonic()
        fb = arm.feedback_get()
        ok = _valid(fb)
        plausible = False
        if ok:
            plausible = (
                all(math.isfinite(fb[j]) for j in range(3))
                and all(0.0 <= fb[j] <= 400.0 for j in range(3))
                and all(JOINT_LIMITS[k][0] <= fb[4 + k] <= JOINT_LIMITS[k][1] for k in range(6))
            )
        records.append({
            "i": i, "t": round(t, 3), "valid": ok, "plausible": plausible,
            "xyz": list(fb[:3]) if ok else None,
            "joints": list(fb[4:10]) if ok else None,
        })
        if ok and plausible:
            last = fb
        time.sleep(0.1)
    return (last if (last is not None and records[-1]["valid"] and records[-1]["plausible"]) else None), records


def home_and_settle(arm, timeout_s=40.0):
    """Send stock move_init() and wait until the pose is within HOME_TOL of home
    and stable. Returns (reached, last_err_rad, polls)."""
    t0 = time.monotonic()
    arm.move_init()
    stable = 0
    last = None
    within = None
    polls = 0
    while time.monotonic() - t0 < timeout_s:
        polls += 1
        fb = arm.feedback_get()
        if _valid(fb):
            mj = list(fb[4:10])
            err = max(abs(mj[k] - HOME_JOINTS[k]) for k in range(5))
            within = err
            if err < HOME_TOL_RAD:
                if last is not None and max(abs(mj[k] - last[k]) for k in range(5)) < math.radians(0.5):
                    stable += 1
                else:
                    stable = 1
                last = mj
                if stable >= 4:
                    return True, err, polls
            else:
                stable = 0
        else:
            stable = 0
        time.sleep(0.5)
    return False, within, polls


def _unwrap(joints):
    """Copy of joints with the 5 drive joints unwrapped so consecutive values
    are continuous (removes the IK atan2 wrap on the base angle)."""
    out = [list(j) for j in joints]
    for k in range(5):
        prev = out[0][k]
        for i in range(1, len(out)):
            v = out[i][k]
            while v - prev > math.pi:
                v -= 2 * math.pi
            while v - prev < -math.pi:
                v += 2 * math.pi
            out[i][k] = v
            prev = v
    return out


def validate_unwrapped(joints, start_joints):
    """Safety gate on UNWRAPPED joints. No total-excursion cap (a 260 deg arc
    needs a large base sweep); the per-waypoint limit check is the guard."""
    n = len(joints)
    for i in range(n):
        for k in range(6):
            v = joints[i][k]
            lo, hi = JOINT_LIMITS[k]
            if not (math.isfinite(v) and lo + LIMIT_MARGIN_RAD <= v <= hi - LIMIT_MARGIN_RAD):
                return False
        for k in range(5):
            if i == 0:
                if abs(joints[0][k] - start_joints[k]) > SANITY_TOL_RAD:
                    return False
            elif abs(joints[i][k] - joints[i - 1][k]) > MAX_ADJ_STEP_RAD:
                return False
    return True


def plan_all(ik_fn, x0, y0, z0, pitch0, roll0, grip0, start_joints, R, S):
    Srad = math.radians(S)
    n = max(MIN_WAYPOINTS, int(round(R * Srad / MAX_STEP_MM)) + 1)
    results = []
    for offx, offy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        cx, cy = x0 + R * offx, y0 + R * offy
        a0 = math.atan2(y0 - cy, x0 - cx)
        for sign in (1, -1):
            waypoints, ikj = [], []
            ok = True
            for i in range(n):
                a = a0 + sign * Srad * i / (n - 1)
                x, y = cx + R * math.cos(a), cy + R * math.sin(a)
                try:
                    j5 = ik_fn(x, y, z0, roll0, pitch0)
                except ValueError:
                    ok = False
                    break
                ikj.append(j5 + [grip0])
                waypoints.append([x, y, z0])
            if not ok:
                results.append({"center_mm": [cx, cy], "sweep_sign": sign,
                                "feasible": False, "reject": "IK no finite solution"})
                continue
            uj = _unwrap(ikj)
            if not validate_unwrapped(uj, start_joints):
                results.append({"center_mm": [cx, cy], "sweep_sign": sign,
                                "feasible": False, "reject": "limit/step/jump/sanity"})
                continue
            base_col = [j[0] for j in uj]
            max_step = max(abs(uj[i][k] - uj[i - 1][k]) for i in range(1, n) for k in range(5))
            results.append({
                "center_mm": [cx, cy],
                "sweep_sign": sign,
                "feasible": True,
                "waypoints": waypoints,
                "joints": uj,
                "n_waypoints": n,
                "path_length_mm": sum(math.dist(waypoints[i], waypoints[i + 1]) for i in range(n - 1)),
                "max_adjacent_step_rad": max_step,
                "base_min_rad": min(base_col),
                "base_max_rad": max(base_col),
                "peak_abs_base_rad": max(abs(v) for v in base_col),
                "excursion_rad": {JOINT_NAMES[k]: max(j[k] for j in uj) - min(j[k] for j in uj) for k in range(5)},
                "cartesian_bbox_mm": {
                    "x": [min(w[0] for w in waypoints), max(w[0] for w in waypoints)],
                    "y": [min(w[1] for w in waypoints), max(w[1] for w in waypoints)],
                    "z": [z0, z0],
                },
            })
    return results


def _serial_settings(port_obj):
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
        "port": g("port"), "baudrate": g("baudrate"), "timeout": g("timeout"),
        "rtscts": g("rtscts"), "dsrdtr": g("dsrdtr"), "xonxoff": g("xonxoff"),
        "rts": g("rts"), "dtr": g("dtr"), "is_open": g("is_open"),
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
    intervals = []
    last_cmd_ts = None
    meas_hist = collections.deque(maxlen=FREEZE_WIN)
    cmd_hist = collections.deque(maxlen=FREEZE_WIN)
    freeze_run = 0
    freeze_windows = 0
    max_meas_motion_win = 0.0
    i = 0
    for i, cmd in enumerate(joints):
        now = time.monotonic()
        cmd_ts = time.monotonic()
        arm.joints_radian_ctrl(cmd, SPEED, ACC)
        if last_cmd_ts is not None:
            intervals.append(cmd_ts - last_cmd_ts)
        last_cmd_ts = cmd_ts
        fb = arm.feedback_get()
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
                now, cmd_ts - t0, time.monotonic() - t0, leg_name, i,
                " ".join("%.4f" % v for v in cmd),
                " ".join("%.4f" % v for v in meas),
                fb[0], fb[1], fb[2], track))
        else:
            fb_bad += 1
            if fb_bad >= INVALID_FB_ABORT:
                aborted = True
                reason = "%d consecutive invalid feedback at waypoint %d" % (fb_bad, i)
            logf.write("ts=%.3f cmd_ts=%.3f fb_ts=%.3f leg=%s idx=%d INVALID_FEEDBACK\n" % (
                now, cmd_ts - t0, time.monotonic() - t0, leg_name, i))
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
    ap.add_argument("--radius-mm", type=float, default=150.0)
    ap.add_argument("--sweep-deg", type=float, default=260.0)
    ap.add_argument("--execute", action="store_true",
                    help="actually run the arc (default is a dry run: home + validate only)")
    script_dir = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--out", default=os.path.join(script_dir, "arc150-260-result.json"))
    ap.add_argument("--log", default=os.path.join(script_dir, "arc150-260.log"))
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
    print("mode            %s" % ("EXECUTE" if args.execute else "DRY (no arc motion)"))

    print("opening %s" % args.port)
    import serial as _serial_mod
    t_open = time.monotonic()
    try:
        arm = roarm(roarm_type="roarm_m3", port=args.port, baudrate=115200)
    except _serial_mod.SerialException as e:
        raise SystemExit("could not open %s (another application may own it): %s" % (args.port, e))
    serial_settings = _serial_settings(arm._serial_port)
    print("serial settings %s" % json.dumps(serial_settings))

    fb = wait_valid_fb(arm)
    open_to_first_valid_fb = time.monotonic() - t_open
    if fb is None:
        raise SystemExit("no valid feedback; aborting before any motion")

    print("pre-home pose xyz=[%.2f %.2f %.2f] joints=[%s]" % (
        fb[0], fb[1], fb[2], " ".join("%.4f" % v for v in fb[4:10])))

    print("homing (move_init) ...")
    reached, home_err, home_polls = home_and_settle(arm)
    print("home reached=%s err=%s polls=%d" % (
        reached, ("%.2f deg" % math.degrees(home_err)) if home_err is not None else None, home_polls))
    if not reached:
        raise SystemExit("arm did not settle at home within timeout; aborting before any arc")

    print("reading 10 consecutive feedback frames at home (stability gate)")
    stable_fb, stable_records = read_stable_state(arm, frames=10)
    for r in stable_records:
        print("  frame %2d valid=%s plausible=%s xyz=%s" % (
            r["i"], r["valid"], r["plausible"],
            ("[%.2f %.2f %.2f]" % tuple(r["xyz"])) if r["xyz"] else None))
    if stable_fb is None:
        raise SystemExit("communication unstable at home (10-frame gate failed); aborting before any arc")

    x0, y0, z0 = stable_fb[0], stable_fb[1], stable_fb[2]
    pitch0, roll0 = stable_fb[3], stable_fb[8]
    grip0 = stable_fb[9]
    start_joints = list(stable_fb[4:10])
    print("home pose   xyz=[%.2f %.2f %.2f]  pitch=%.4f  roll=%.4f  grip=%.4f" % (
        x0, y0, z0, pitch0, roll0, grip0))
    print("home joints [%s]" % " ".join("%.4f" % v for v in start_joints))

    print("planning R=%.0f mm / sweep=%.0f deg (all center/direction combinations)" % (
        args.radius_mm, args.sweep_deg))
    plans = plan_all(ik_fn, x0, y0, z0, pitch0, roll0, grip0, start_joints,
                     args.radius_mm, args.sweep_deg)
    feasible = [p for p in plans if p.get("feasible")]
    for p in plans:
        if p.get("feasible"):
            print("  FEASIBLE center=[%.1f %.1f] sign=%+d base[%.1f..%.1f] deg peak|base|=%.1f deg maxstep=%.2f deg path=%.1f mm" % (
                p["center_mm"][0], p["center_mm"][1], p["sweep_sign"],
                math.degrees(p["base_min_rad"]), math.degrees(p["base_max_rad"]),
                math.degrees(p["peak_abs_base_rad"]), math.degrees(p["max_adjacent_step_rad"]),
                p["path_length_mm"]))
        else:
            print("  reject   center=[%.1f %.1f] sign=%+d reason=%s" % (
                p["center_mm"][0], p["center_mm"][1], p["sweep_sign"], p.get("reject")))

    if not feasible:
        result = {
            "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "platform": sys.platform,
            "mode": "dry" if not args.execute else "execute",
            "verdict": "INFEASIBLE_NO_SAFE_ARC",
            "home_pose_xyz_mm": [x0, y0, z0],
            "home_joints_rad": start_joints,
            "candidate_evaluations": plans,
            "joint_limits_rad": JOINT_LIMITS,
        }
        with open(args.out, "w") as fh:
            json.dump(result, fh, indent=1)
        print("no center/direction keeps the arc inside the joint limits; NO motion sent")
        print("wrote %s" % args.out)
        print("RESULT: INFEASIBLE")
        return 1

    best = min(feasible, key=lambda p: p["peak_abs_base_rad"])
    print("selected     center=[%.1f %.1f] sign=%+d  (smallest peak |base| = %.1f deg)" % (
        best["center_mm"][0], best["center_mm"][1], best["sweep_sign"],
        math.degrees(best["peak_abs_base_rad"])))
    print("path length  %.1f mm   waypoints=%d   rate=%.0f Hz   duration=%.2f s" % (
        best["path_length_mm"], best["n_waypoints"], RATE_HZ,
        (best["n_waypoints"] - 1) / RATE_HZ))
    print("cartesian bbox mm  x=[%.2f %.2f]  y=[%.2f %.2f]  z=[%.2f %.2f]" % (
        best["cartesian_bbox_mm"]["x"][0], best["cartesian_bbox_mm"]["x"][1],
        best["cartesian_bbox_mm"]["y"][0], best["cartesian_bbox_mm"]["y"][1],
        best["cartesian_bbox_mm"]["z"][0], best["cartesian_bbox_mm"]["z"][1]))
    print("base range   [%.2f, %.2f] deg (limit +/-189 deg)" % (
        math.degrees(best["base_min_rad"]), math.degrees(best["base_max_rad"])))
    print("max joint excursion  " + "  ".join(
        "%s %.2f deg" % (n, math.degrees(v)) for n, v in best["excursion_rad"].items()))
    print("max adjacent step  %.3f deg" % math.degrees(best["max_adjacent_step_rad"]))
    print("constant Z=%.2f mm  pitch=%.4f  roll=%.4f  grip=%.4f (unchanged)" % (
        z0, pitch0, roll0, grip0))

    if not args.execute:
        result = {
            "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "platform": sys.platform,
            "mode": "dry",
            "verdict": "FEASIBLE_NOT_EXECUTED",
            "environment": {"python": sys.version.split()[0], "executable": sys.executable,
                            "roarm_sdk_version": roarm_sdk_ver, "roarm_sdk_file": roarm_sdk.__file__,
                            "pyserial_version": pyserial_ver, "ik_module": ik_path},
            "port": args.port,
            "serial_settings": serial_settings,
            "open_to_first_valid_fb_s": round(open_to_first_valid_fb, 3),
            "pre_home_pose": {"xyz_mm": list(fb[:3]), "joints_rad": list(fb[4:10])},
            "home_reached": True,
            "home_err_rad": home_err,
            "home_pose": {"xyz_mm": [x0, y0, z0], "pitch_rad": pitch0, "roll_rad": roll0,
                          "gripper": grip0, "joints_rad": start_joints},
            "open_stability_frames": stable_records,
            "radius_mm": args.radius_mm,
            "sweep_deg": args.sweep_deg,
            "candidate_evaluations": plans,
            "selected": {k: v for k, v in best.items() if k not in ("waypoints", "joints")},
            "joint_limits_rad": JOINT_LIMITS,
        }
        with open(args.out, "w") as fh:
            json.dump(result, fh, indent=1)
        print("DRY RUN: arc is feasible and validated; no arc motion sent (pass --execute to run)")
        print("wrote %s" % args.out)
        print("RESULT: DRY_FEASIBLE")
        return 0

    print("--- validation passed; executing forward leg ---")
    z_record = []
    forward = None
    reverse = None
    with open(args.log, "w") as logf:
        logf.write("# %s  port=%s  sdk=%s  ik=%s  serial=%s  home_xyz=[%.2f %.2f %.2f]\n" % (
            datetime.datetime.now(datetime.timezone.utc).isoformat(),
            args.port, sdk_dir, ik_path, json.dumps(serial_settings), x0, y0, z0))
        forward = run_leg(arm, best["joints"], "F", logf, z_record)
        print("forward  aborted=%s reason=%s fb_ok=%d fb_invalid=%d max_track=%.2f deg rate=%s Hz" % (
            forward["aborted"], forward["abort_reason"], forward["fb_ok"],
            forward["fb_invalid"], math.degrees(forward["max_tracking_err_rad"]),
            forward["achieved_cmd_rate_hz"]))
        if not forward["aborted"]:
            print("--- executing reverse leg ---")
            reverse = run_leg(arm, list(reversed(best["joints"])), "R", logf, z_record)
            print("reverse  aborted=%s reason=%s fb_ok=%d fb_invalid=%d max_track=%.2f deg rate=%s Hz" % (
                reverse["aborted"], reverse["abort_reason"], reverse["fb_ok"],
                reverse["fb_invalid"], math.degrees(reverse["max_tracking_err_rad"]),
                reverse["achieved_cmd_rate_hz"]))

    result = {
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "platform": sys.platform,
        "mode": "execute",
        "environment": {"python": sys.version.split()[0], "executable": sys.executable,
                        "roarm_sdk_version": roarm_sdk_ver, "roarm_sdk_file": roarm_sdk.__file__,
                        "pyserial_version": pyserial_ver, "ik_module": ik_path},
        "port": args.port,
        "sdk": sdk_dir,
        "serial_settings": serial_settings,
        "open_to_first_valid_fb_s": round(open_to_first_valid_fb, 3),
        "speed_acc": [SPEED, ACC],
        "rate_hz": RATE_HZ,
        "pre_home_pose": {"xyz_mm": list(fb[:3]), "joints_rad": list(fb[4:10])},
        "home_reached": True,
        "home_err_rad": home_err,
        "home_pose": {"xyz_mm": [x0, y0, z0], "pitch_rad": pitch0, "roll_rad": roll0,
                      "gripper": grip0, "joints_rad": start_joints},
        "open_stability_frames": stable_records,
        "radius_mm": args.radius_mm,
        "sweep_deg": args.sweep_deg,
        "selected_plan": {k: v for k, v in best.items() if k not in ("waypoints", "joints")},
        "planned_waypoints_xy": [[round(w[0], 3), round(w[1], 3)] for w in best["waypoints"]],
        "planned_joints_rad": [[round(v, 5) for v in j] for j in best["joints"]],
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
