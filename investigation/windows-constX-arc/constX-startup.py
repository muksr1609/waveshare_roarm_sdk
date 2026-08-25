#!/usr/bin/env python3
"""Bounded constant-X STARTUP experiment (single discriminating question):

  "Does the changing constant-X trajectory start tracking if it is allowed to
   continue beyond ~1 s?"

Deliberately BOUNDED: the commanded pose never runs past the already-tested
waypoint 20, so if the robot moves it only moves toward wp20 (~21 deg from
home), not tens of degrees farther along the aggressive arc.

Procedure (all identical to arc-constX.py / constX-experiment.py -- nothing
altered: same home procedure, same exact constant-X trajectory, same IK,
T=102, speed=1000, acc=50, 20 Hz, live feedback, stock site-packages SDK):

  1. Home + settle + 10-frame stable read.
  2. Stream the EXACT constant-X waypoints 0..20 at 20 Hz (21 T=102 packets).
  3. After waypoint 20, KEEP sending T=102 at 20 Hz for 3.0 s, repeating
     waypoint 20 UNCHANGED (60 identical packets). Bounded at wp20.
  4. STOP all T=102 transmission; record feedback for 2.0 s.

NO abort for lack of motion (that is the thing under test). Safety is still
enforced and WILL abort: feedback validity, hard joint limits, gross
single-sample joint jump, physically-unsafe xyz. Every command + feedback
sample is recorded. The run is classified:

  A: motion begins while identical WP20 packets are STILL being sent (phase 2).
     -> changing-target priming occurred; arm moves once target is stationary;
        cessation of serial writes is NOT required.
  B: frozen while repeated WP20 packets are sent; moves only after T=102 stops
     (phase 3). -> repeated T=102 itself suppresses/restarts actuation.
  C: motion begins before waypoint 20 while targets are still changing
     (phase 1). -> the constant-X stream merely has a longer startup/commit
        delay than the previous (overlapping-window) tests allowed.
  D: never moves. -> deeper intermittent command-latching/controller-state issue.
"""
import inspect
import json
import math
import os
import sys
import time
import datetime
import importlib.util

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("arc_constx", os.path.join(here, "arc-constX.py"))
cx = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cx)

SPEED = cx.SPEED          # 1000 (unchanged)
ACC = cx.ACC              # 50    (unchanged)
RATE_HZ = cx.RATE_HZ      # 20.0  (unchanged)
MOVE_TOL_RAD = math.radians(0.5)     # "first >0.5 deg measured movement"
RADIUS_MM = 150.0
SWEEP_DEG = 110.0

# --- safety thresholds (abort conditions; NO no-motion abort) ---
INVALID_FB_ABORT = 3
GROSS_JUMP_RAD = math.radians(20.0)  # single 50 ms sample joint jump > 20 deg
XYZ_LO, XYZ_HI = 0.0, 400.0

PHASE1 = "stream"    # changing targets wp0..wp20
PHASE2 = "repeat"    # identical wp20 repeated
PHASE3 = "stop"      # no T=102 sent


def moved_from_home(start_joints, fb):
    return max(abs(fb[4 + k] - start_joints[k]) for k in range(5))


def track_to_wp20(fb, wp20):
    return max(abs(fb[4 + k] - wp20[k]) for k in range(5))


def main():
    import roarm_sdk
    sdk_dir = os.path.dirname(os.path.abspath(inspect.getfile(roarm_sdk)))
    sdk_dir_norm = sdk_dir.replace(os.sep, "/")
    repo_root_norm = os.path.dirname(os.path.dirname(here)).replace(os.sep, "/")
    if "site-packages/roarm_sdk" not in sdk_dir_norm:
        raise SystemExit("refusing to run: not the stock site-packages SDK: %s" % sdk_dir)
    if repo_root_norm in sdk_dir_norm:
        raise SystemExit("refusing to run: importing the repo SDK: %s" % sdk_dir)
    from roarm_sdk.roarm import roarm

    try:
        import importlib.metadata as _imd
        roarm_sdk_ver = _imd.version("roarm-sdk")
    except Exception:
        roarm_sdk_ver = "unknown"

    ik_mod, ik_path = cx.load_ik_module()
    ik_fn = ik_mod.compute_joint_rad_by_pos

    print("=== environment ===")
    print("python      %s (%s)" % (sys.version.split()[0], sys.executable))
    print("roarm-sdk   %s  (%s)" % (roarm_sdk_ver, roarm_sdk.__file__))
    print("ik          %s" % ik_path)
    print("params      T=102 speed=%d acc=%d rate=%.0f Hz  R=%.0fmm sweep=%.0f deg" % (
        SPEED, ACC, RATE_HZ, RADIUS_MM, SWEEP_DEG))

    print("opening COM21")
    arm = roarm(roarm_type="roarm_m3", port="COM21", baudrate=115200)
    fb = cx.wait_valid_fb(arm)
    if fb is None:
        raise SystemExit("no valid feedback; aborting before any motion")

    # ---- 1. HOME + settle + stable read ----
    reached, home_err, polls = cx.home_and_settle(arm)
    print("home reached=%s err=%s polls=%d" % (
        reached, ("%.2f deg" % math.degrees(home_err)) if home_err is not None else None, polls))
    if not reached:
        raise SystemExit("arm did not settle at home")
    stable_fb, _ = cx.read_stable_state(arm, frames=10)
    if stable_fb is None:
        raise SystemExit("not stable at home")
    x0, y0, z0 = stable_fb[0], stable_fb[1], stable_fb[2]
    pitch0, roll0, grip0 = stable_fb[3], stable_fb[8], stable_fb[9]
    startA = list(stable_fb[4:10])
    print("home pose xyz=[%.2f %.2f %.2f] joints=[%s]" % (
        x0, y0, z0, " ".join("%.4f" % v for v in startA)))

    # ---- regenerate the EXACT constant-X trajectory (same planner/selection) ----
    plans = cx.plan_const_x(ik_fn, x0, y0, z0, pitch0, roll0, grip0, startA, RADIUS_MM, SWEEP_DEG)
    feasible = [p for p in plans if p.get("feasible")]
    if not feasible:
        raise SystemExit("no feasible constant-X arc")
    best = min(feasible, key=lambda p: (-min(w[2] for w in p["waypoints"]), p["peak_abs_base_rad"]))
    joints = best["joints"]
    wp20 = joints[20]
    wp0to20 = joints[:21]
    print("selected center_yz=%s sign=%+d (min Z=%.1f mm)" % (
        best["center_yz_mm"], best["sweep_sign"], min(w[2] for w in best["waypoints"])))
    print("waypoint 20 = [%s]" % " ".join("%.4f" % v for v in wp20))

    log_path = os.path.join(here, "constX-startup.log")
    json_path = os.path.join(here, "constX-startup-result.json")
    logf = open(log_path, "w")
    logf.write("# %s  port=COM21  sdk=%s  ik=%s  speed=%d acc=%d rate=%.0fHz  "
               "home_xyz=[%.2f %.2f %.2f]  constX=%.2f  wp20=[%s]\n" % (
                   datetime.datetime.now(datetime.timezone.utc).isoformat(),
                   sdk_dir, ik_path, SPEED, ACC, RATE_HZ, x0, y0, z0, x0,
                   " ".join("%.4f" % v for v in wp20)))

    results = {
        "home_pose": {"xyz": [x0, y0, z0], "joints": startA},
        "selected": {"center_yz_mm": best["center_yz_mm"], "sweep_sign": best["sweep_sign"]},
        "waypoint20": wp20,
        "params": {"speed": SPEED, "acc": ACC, "rate_hz": RATE_HZ,
                   "radius_mm": RADIUS_MM, "sweep_deg": SWEEP_DEG, "ik": ik_path,
                   "sdk_version": roarm_sdk_ver},
        "safety": {"invalid_fb_abort": INVALID_FB_ABORT,
                   "gross_jump_rad": GROSS_JUMP_RAD,
                   "xyz_range": [XYZ_LO, XYZ_HI]},
    }

    samples = []          # every (cmd, fb) record
    aborted = False
    abort_reason = None
    fb_bad = 0
    prev_meas = None
    first_move = None     # dict set on first >0.5 deg movement

    def record(phase, idx, cmd, fb, t):
        nonlocal fb_bad, prev_meas, first_move, aborted, abort_reason
        if cx._valid(fb):
            fb_bad = 0
            meas = list(fb[4:10])
            moved = moved_from_home(startA, fb)
            # --- safety: hard joint limits ---
            for k in range(6):
                lo, hi = cx.JOINT_LIMITS[k]
                if not (lo <= meas[k] <= hi):
                    aborted = True
                    abort_reason = ("hard joint limit: %s measured %.4f rad outside [%.2f, %.2f]"
                                    " at phase=%s idx=%s" % (cx.JOINT_NAMES[k], meas[k], lo, hi, phase, idx))
            # --- safety: physically unsafe xyz ---
            if not aborted:
                for j in range(3):
                    if not (XYZ_LO <= fb[j] <= XYZ_HI):
                        aborted = True
                        abort_reason = "physically unsafe xyz[%d]=%.2f mm at phase=%s idx=%s" % (j, fb[j], phase, idx)
            # --- safety: gross single-sample joint jump ---
            if not aborted and prev_meas is not None:
                jump = max(abs(meas[k] - prev_meas[k]) for k in range(5))
                if jump > GROSS_JUMP_RAD:
                    aborted = True
                    abort_reason = "gross single-sample joint jump %.1f deg at phase=%s idx=%s" % (
                        math.degrees(jump), phase, idx)
            prev_meas = meas
            # --- first motion ---
            if first_move is None and moved > MOVE_TOL_RAD:
                first_move = {"t_from_start_s": round(t, 3), "phase": phase, "idx": idx,
                              "cmd": [round(v, 4) for v in cmd] if cmd is not None else None,
                              "meas": [round(v, 4) for v in meas],
                              "moved_deg": round(math.degrees(moved), 3)}
            rec = {"t": round(t, 3), "phase": phase, "idx": idx,
                   "cmd": [round(v, 4) for v in cmd] if cmd is not None else None,
                   "meas": [round(v, 4) for v in meas],
                   "xyz": [round(fb[0], 2), round(fb[1], 2), round(fb[2], 2)],
                   "moved_deg": round(math.degrees(moved), 3),
                   "track_wp20_deg": round(math.degrees(track_to_wp20(fb, wp20)), 3)}
            samples.append(rec)
            logf.write("t=%.3f phase=%s idx=%s cmd=[%s] meas=[%s] xyz=%.2f,%.2f,%.2f moved=%.3f track_wp20=%.3f\n" % (
                t, phase, idx,
                " ".join("%.4f" % v for v in cmd) if cmd is not None else "-----",
                " ".join("%.4f" % v for v in meas),
                fb[0], fb[1], fb[2], rec["moved_deg"], rec["track_wp20_deg"]))
        else:
            fb_bad += 1
            rec = {"t": round(t, 3), "phase": phase, "idx": idx, "cmd":
                   [round(v, 4) for v in cmd] if cmd is not None else None,
                   "meas": None, "xyz": None, "moved_deg": None, "track_wp20_deg": None, "invalid": True}
            samples.append(rec)
            logf.write("t=%.3f phase=%s idx=%s INVALID_FEEDBACK\n" % (t, phase, idx))
            if fb_bad >= INVALID_FB_ABORT:
                aborted = True
                abort_reason = "%d consecutive invalid feedback at phase=%s idx=%s" % (fb_bad, phase, idx)

    # ================= PHASE 1: stream exact wp0..wp20 at 20 Hz =================
    print("\n===== PHASE 1: stream constant-X waypoints 0..20 at 20 Hz (21 packets) =====")
    t0 = time.monotonic()
    for i, cmd in enumerate(wp0to20):
        arm.joints_radian_ctrl(cmd, SPEED, ACC)
        fb = arm.feedback_get()
        record(PHASE1, i, cmd, fb, time.monotonic() - t0)
        if aborted:
            break
        delay = (t0 + i / RATE_HZ) - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    t_repeat_start = time.monotonic() - t0
    print("phase 1 done at t=%.3f s (first repeat packet will be at ~t=%.3f s)" % (t_repeat_start, t_repeat_start))

    # ================= PHASE 2: repeat identical wp20 for 3.0 s =================
    print("\n===== PHASE 2: KEEP SENDING identical wp20 (T=102) for 3.0 s (60 packets) =====")
    n_repeat = int(round(3.0 * RATE_HZ))   # 60
    t_rep0 = time.monotonic()
    for j in range(n_repeat):
        arm.joints_radian_ctrl(wp20, SPEED, ACC)
        fb = arm.feedback_get()
        record(PHASE2, 20, wp20, fb, time.monotonic() - t0)
        if aborted:
            break
        delay = (t_rep0 + j / RATE_HZ) - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    t_stop = time.monotonic() - t0
    print("phase 2 done; stopping T=102 at t=%.3f s" % t_stop)

    # ================= PHASE 3: stop T=102, record 2.0 s =================
    print("\n===== PHASE 3: STOP all T=102 transmission; record feedback 2.0 s =====")
    n_post = int(round(2.0 * RATE_HZ))    # 40
    t_post0 = time.monotonic()
    for k in range(n_post):
        fb = arm.feedback_get()
        record(PHASE3, None, None, fb, time.monotonic() - t0)
        if aborted:
            break
        delay = (t_post0 + k / RATE_HZ) - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    logf.close()

    # ---- per-phase stats ----
    def phase_stats(phase):
        ps = [s for s in samples if s["phase"] == phase and s["meas"] is not None]
        if not ps:
            return {"n_valid": 0, "max_moved_deg": None, "first_t_s": None, "last_t_s": None,
                    "max_moved_deg": None}
        return {
            "n_valid": len(ps),
            "max_moved_deg": max(s["moved_deg"] for s in ps),
            "first_t_s": ps[0]["t"],
            "last_t_s": ps[-1]["t"],
            "last_moved_deg": ps[-1]["moved_deg"],
            "last_track_wp20_deg": ps[-1]["track_wp20_deg"],
        }

    p1 = phase_stats(PHASE1)
    p2 = phase_stats(PHASE2)
    p3 = phase_stats(PHASE3)

    # ---- classification A/B/C/D ----
    if aborted and abort_reason and "no motion" not in abort_reason:
        classification = "SAFETY_ABORT: %s" % abort_reason
    elif first_move is None:
        classification = "D: never moves (no >0.5 deg in any phase)"
    elif first_move["phase"] == PHASE1:
        classification = "C: motion begins while targets are still changing (before wp20)"
    elif first_move["phase"] == PHASE2:
        classification = "A: motion begins while identical WP20 packets are still being sent"
    elif first_move["phase"] == PHASE3:
        classification = "B: frozen during repeated WP20; moves only after T=102 stops"
    else:
        classification = "UNCLASSIFIED"

    # ---- first-motion timing relative to repeat start ----
    first_move_from_repeat = None
    if first_move is not None:
        first_move_from_repeat = round(first_move["t_from_start_s"] - t_repeat_start, 3)

    # ---- convergence toward wp20 ----
    conv = None
    if first_move is not None:
        # track_wp20 at end of each phase (lower = closer to wp20)
        conv = {
            "track_wp20_end_phase2_deg": p2.get("last_track_wp20_deg"),
            "track_wp20_end_phase3_deg": p3.get("last_track_wp20_deg"),
            "converging_toward_wp20": (
                (p3.get("last_track_wp20_deg") is not None
                 and p2.get("last_track_wp20_deg") is not None
                 and p3["last_track_wp20_deg"] <= p2["last_track_wp20_deg"])),
        }

    # ---- does stopping T=102 change anything ----
    stop_effect = {
        "phase2_max_moved_deg": p2.get("max_moved_deg"),
        "phase3_max_moved_deg": p3.get("max_moved_deg"),
        "moved_more_after_stop": (
            (p3.get("max_moved_deg") is not None and p2.get("max_moved_deg") is not None
             and p3["max_moved_deg"] > p2["max_moved_deg"] + 1e-6)),
    }

    results["phases"] = {PHASE1: p1, PHASE2: p2, PHASE3: p3}
    results["t_repeat_start_s"] = round(t_repeat_start, 3)
    results["t_stop_s"] = round(t_stop, 3)
    results["first_motion"] = {
        "occurred": first_move is not None,
        "t_from_start_s": first_move["t_from_start_s"] if first_move else None,
        "t_from_first_repeat_packet_s": first_move_from_repeat,
        "phase": first_move["phase"] if first_move else None,
        "idx": first_move["idx"] if first_move else None,
        "cmd": first_move["cmd"] if first_move else None,
        "meas": first_move["meas"] if first_move else None,
        "moved_deg": first_move["moved_deg"] if first_move else None,
    }
    results["convergence_to_wp20"] = conv
    results["stop_effect"] = stop_effect
    results["classification"] = classification
    results["aborted"] = aborted
    results["abort_reason"] = abort_reason

    # ---- return home (leave arm safe) ----
    print("\nreturning home (move_init) ...")
    arm.move_init()
    h_reached, h_err, h_polls = cx.home_and_settle(arm)
    print("home reached=%s err=%s polls=%d" % (
        h_reached, ("%.2f deg" % math.degrees(h_err)) if h_err is not None else None, h_polls))
    results["return_home"] = {"reached": h_reached, "err_rad": h_err, "polls": h_polls}
    results["samples"] = samples

    with open(json_path, "w") as fh:
        json.dump(results, fh, indent=1)

    # ---- report ----
    print("\n===== REPORT =====")
    print("t_repeat_start (first WP20-repeat packet) = %.3f s" % t_repeat_start)
    print("t_stop (last WP20-repeat packet)          = %.3f s" % t_stop)
    print("PHASE 1 (stream  wp0..20): max_moved=%s deg  over %s s" % (
        p1.get("max_moved_deg"),
        ("%.2f->%.2f" % (p1["first_t_s"], p1["last_t_s"])) if p1.get("first_t_s") else "?"))
    print("PHASE 2 (repeat  wp20 x60): max_moved=%s deg  track_wp20_end=%s deg" % (
        p2.get("max_moved_deg"), p2.get("last_track_wp20_deg")))
    print("PHASE 3 (stop    T=102)  : max_moved=%s deg  track_wp20_end=%s deg" % (
        p3.get("max_moved_deg"), p3.get("last_track_wp20_deg")))
    print("first >0.5 deg motion : %s" % (
        "t=%.3f s from start | t=%.3f s from first repeat packet | phase=%s | meas=[%s]" % (
            results["first_motion"]["t_from_start_s"],
            results["first_motion"]["t_from_first_repeat_packet_s"],
            results["first_motion"]["phase"],
            " ".join("%.4f" % v for v in results["first_motion"]["meas"]))
        if first_move else "NEVER in any phase"))
    print("convergence to wp20   : %s" % conv)
    print("stop changes anything : %s" % stop_effect)
    print("\nCLASSIFICATION: %s" % classification)
    if aborted:
        print("SAFETY ABORT: %s" % abort_reason)
    print("\nwrote %s" % json_path)
    print("wrote %s" % log_path)
    print("DONE")


if __name__ == "__main__":
    main()
