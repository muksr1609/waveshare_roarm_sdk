#!/usr/bin/env python3
"""Discriminating experiment (single question):
  Does repeatedly REPLACING the joint target at 20 Hz suppress/delay motion
  compared with HOLDING the exact same final target?

Part A (held)   : home, regenerate the constant-X trajectory, take waypoint 20,
                  send it ONCE (T=102, speed=1000, acc=50), send NOTHING else for
                  3 s, record feedback at ~20 Hz, measure time to first >0.5 deg
                  movement. Then return home and VERIFY it actually returns.
Part B (streamed): separate run from the same home state, send waypoints 0..20 at
                  20 Hz exactly as the arc does, then STOP. Record feedback
                  continuously for the first 1 s of streaming and 2 s after.

Nothing is altered: same IK, speed=1000, acc=50, rate=20 Hz, same targets,
same watchdog thresholds (the watchdog is not used here at all).
"""
import importlib.util
import inspect
import json
import math
import os
import sys
import time

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("arc_constx", os.path.join(here, "arc-constX.py"))
cx = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cx)

SPEED = cx.SPEED          # 1000 (unchanged)
ACC = cx.ACC              # 50    (unchanged)
RATE_HZ = cx.RATE_HZ      # 20.0  (unchanged)
MOVE_TOL_RAD = math.radians(0.5)
RADIUS_MM = 150.0
SWEEP_DEG = 110.0


def moved_rad(start_joints, fb):
    return max(abs(fb[4 + k] - start_joints[k]) for k in range(5))


def sample(fb, start_joints, t, phase, idx):
    return {
        "t": round(t, 3),
        "phase": phase,
        "idx": idx,
        "xyz": [round(fb[0], 2), round(fb[1], 2), round(fb[2], 2)],
        "joints": [round(v, 4) for v in fb[4:10]],
        "moved_deg": round(math.degrees(moved_rad(start_joints, fb)), 3),
    }


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

    ik_mod, ik_path = cx.load_ik_module()
    ik_fn = ik_mod.compute_joint_rad_by_pos

    print("opening COM21")
    arm = roarm(roarm_type="roarm_m3", port="COM21", baudrate=115200)
    fb = cx.wait_valid_fb(arm)
    if fb is None:
        raise SystemExit("no valid feedback; aborting before any motion")

    # ---- HOME + settle + stable read (shared home state for both parts) ----
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

    # ---- regenerate the exact constant-X trajectory (same planner/selection) ----
    plans = cx.plan_const_x(ik_fn, x0, y0, z0, pitch0, roll0, grip0, startA, RADIUS_MM, SWEEP_DEG)
    feasible = [p for p in plans if p.get("feasible")]
    if not feasible:
        raise SystemExit("no feasible constant-X arc")
    best = min(feasible, key=lambda p: (-min(w[2] for w in p["waypoints"]), p["peak_abs_base_rad"]))
    joints = best["joints"]
    wp20 = joints[20]
    wp0to20 = joints[:21]
    print("selected center_yz=%s sign=%+d  (min Z=%.1f mm)" % (
        best["center_yz_mm"], best["sweep_sign"], min(w[2] for w in best["waypoints"])))
    print("waypoint 20 = [%s]" % " ".join("%.4f" % v for v in wp20))
    print("home joints = [%s]" % " ".join("%.4f" % v for v in startA))

    results = {
        "home_pose": {"xyz": [x0, y0, z0], "joints": startA},
        "selected": {"center_yz_mm": best["center_yz_mm"], "sweep_sign": best["sweep_sign"]},
        "waypoint20": wp20,
        "params": {"speed": SPEED, "acc": ACC, "rate_hz": RATE_HZ,
                   "radius_mm": RADIUS_MM, "sweep_deg": SWEEP_DEG, "ik": ik_path},
    }

    # ================= PART A : held target (waypoint 20) =================
    print("\n===== PART A: send waypoint 20 ONCE, hold 3 s (no further command) =====")
    t0 = time.monotonic()
    arm.joints_radian_ctrl(wp20, SPEED, ACC)
    partA, first_move_t = [], None
    i = 0
    while time.monotonic() - t0 < 3.0:
        fb = arm.feedback_get()
        t = time.monotonic() - t0
        if cx._valid(fb):
            m = moved_rad(startA, fb)
            partA.append(sample(fb, startA, t, "held", i))
            if first_move_t is None and m > MOVE_TOL_RAD:
                first_move_t = t
        i += 1
        delay = (t0 + i / RATE_HZ) - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    finalA = partA[-1] if partA else None
    results["partA_held_wp20"] = {
        "first_move_t_s": round(first_move_t, 3) if first_move_t is not None else None,
        "final_moved_deg": finalA["moved_deg"] if finalA else None,
        "final_joints": finalA["joints"] if finalA else None,
        "samples": partA,
    }
    print("PART A: first >0.5 deg movement at t = %s ; final moved = %s deg" % (
        ("%.3f s" % first_move_t) if first_move_t is not None else "NEVER in 3 s",
        finalA["moved_deg"] if finalA else None))

    # ---- return home via joints_radian_ctrl (mirrors the diagnostic return) ----
    print("\nreturning home via joints_radian_ctrl(home joints) ...")
    t_ret = time.monotonic()
    arm.joints_radian_ctrl(startA, SPEED, ACC)
    retA, ret_first_t = [], None
    i = 0
    pos_before_ret = finalA["joints"] if finalA else startA
    while time.monotonic() - t_ret < 3.0:
        fb = arm.feedback_get()
        t = time.monotonic() - t_ret
        if cx._valid(fb):
            m = moved_rad(pos_before_ret, fb)   # movement back toward home
            retA.append(sample(fb, startA, t, "return", i))
            if ret_first_t is None and m > MOVE_TOL_RAD:
                ret_first_t = t
        i += 1
        delay = (t_ret + i / RATE_HZ) - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    ret_final = retA[-1] if retA else None
    ret_err = max(abs(ret_final["joints"][k] - startA[k]) for k in range(5)) if ret_final else None
    results["partA_return_via_jrc"] = {
        "first_move_back_t_s": round(ret_first_t, 3) if ret_first_t is not None else None,
        "final_joints": ret_final["joints"] if ret_final else None,
        "final_err_from_home_rad": round(ret_err, 4) if ret_err is not None else None,
        "final_err_from_home_deg": round(math.degrees(ret_err), 3) if ret_err is not None else None,
        "samples": retA,
    }
    print("PART A return(jrc): first move-back at %s ; final err from home = %s deg" % (
        ("%.3f s" % ret_first_t) if ret_first_t is not None else "NEVER in 3 s",
        ("%.3f" % math.degrees(ret_err)) if ret_err is not None else None))

    # ---- guarantee home via stock move_init + verify ----
    arm.move_init()
    v_reached, v_err, v_polls = cx.home_and_settle(arm)
    print("move_init home reached=%s err=%s polls=%d" % (
        v_reached, ("%.2f deg" % math.degrees(v_err)) if v_err is not None else None, v_polls))
    results["partA_verify_move_init"] = {"reached": v_reached, "err_rad": v_err, "polls": v_polls}
    if not v_reached:
        raise SystemExit("arm did not verify home before Part B; stopping")

    # ================= PART B : streamed target (0..20 at 20 Hz) =================
    print("\n===== PART B: stream waypoints 0..20 at 20 Hz, then STOP =====")
    t0b = time.monotonic()
    partB = []
    for i, cmd in enumerate(wp0to20):
        arm.joints_radian_ctrl(cmd, SPEED, ACC)
        fb = arm.feedback_get()
        t = time.monotonic() - t0b
        if cx._valid(fb):
            partB.append(sample(fb, startA, t, "stream", i))
        delay = (t0b + i / RATE_HZ) - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    t_stop = time.monotonic()
    print("stream stopped at t=%.3f s; recording 2 s of post-stop settling" % (t_stop - t0b))
    j = 0
    while time.monotonic() - t_stop < 2.0:
        fb = arm.feedback_get()
        t = time.monotonic() - t0b
        if cx._valid(fb):
            partB.append(sample(fb, startA, t, "post", None))
        j += 1
        delay = (t_stop + j / RATE_HZ) - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    stream_s = [s for s in partB if s["phase"] == "stream"]
    post_s = [s for s in partB if s["phase"] == "post"]
    max_stream = max((s["moved_deg"] for s in stream_s), default=0.0)
    max_post = max((s["moved_deg"] for s in post_s), default=0.0)
    results["partB_stream_0_20"] = {
        "max_moved_during_stream_deg": max_stream,
        "max_moved_after_stop_deg": max_post,
        "samples": partB,
    }
    print("PART B: max moved during stream = %.3f deg ; max moved after stop = %.3f deg" % (max_stream, max_post))

    # ---- return home after Part B ----
    arm.move_init()
    b_reached, b_err, b_polls = cx.home_and_settle(arm)
    print("move_init home (after Part B) reached=%s err=%s" % (
        b_reached, ("%.2f deg" % math.degrees(b_err)) if b_err is not None else None))
    results["partB_verify_move_init"] = {"reached": b_reached, "err_rad": b_err, "polls": b_polls}

    # ---- comparison ----
    cmp = {
        "partA_first_move_t_s": results["partA_held_wp20"]["first_move_t_s"],
        "partA_final_moved_deg": results["partA_held_wp20"]["final_moved_deg"],
        "partA_return_jrc_first_move_t_s": results["partA_return_via_jrc"]["first_move_back_t_s"],
        "partA_return_jrc_final_err_deg": results["partA_return_via_jrc"]["final_err_from_home_deg"],
        "partB_max_moved_during_stream_deg": max_stream,
        "partB_max_moved_after_stop_deg": max_post,
    }
    results["comparison"] = cmp
    out = os.path.join(here, "constX-experiment-result.json")
    with open(out, "w") as fh:
        json.dump(results, fh, indent=1)
    print("\nwrote %s" % out)
    print("\n===== COMPARISON =====")
    for k, v in cmp.items():
        print("  %-40s %s" % (k, v))
    print("DONE")


if __name__ == "__main__":
    main()
