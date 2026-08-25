#!/usr/bin/env python3
"""Bounded STARTUP / COMMIT HANDSHAKE for the 150 mm / 110 deg constant-X arc.

Purpose: before the real trajectory is allowed to advance materially, PROVE
from feedback that the arm has physically begun responding. This replaces a
blind "startup grace" (which is unsafe given the intermittent command-latching
observed: one run committed at 0.843 s while targets were still changing,
another stayed frozen past 1 s and only moved after updates stopped, and a
single static T=102 from rest can fail completely).

The trajectory is UNCHANGED: same planner, same selection, same IK, same
T=102 / speed=1000 / acc=50 / 20 Hz, same run_leg watchdog -- all imported from
arc-constX.py. Only the startup behaviour is added, factored into
bounded_startup_handshake() so it can later live in the RoArm adapter.

PROTOCOL
  1. Home + settle + stable read; take the ACTUAL measured start state M.
  2. Bounded startup target = the FURTEST point on the planned trajectory whose
     max joint displacement from M is <= ENVELOPE (3 deg). Not an arbitrary
     waypoint number.
  3. Stream waypoints 0..target at 20 Hz (the "ramp"). Commanded state stays
     inside the <=3 deg envelope.
  4. STOP advancing; keep sending the target (the "hold"), staying in envelope.
  5. Monitor feedback: measured motion > 0.5 deg while commands are sent =>
     committed (record the phase: ramp | hold).
  6. If still no motion after the bounded ramp+hold, STOP sending T=102 and
     observe for STOP_S (2 s). Motion toward the target => commit-after-stop.
  7. If no motion: repeat the SAME bounded attempt once from the newly measured
     state. Envelope is NOT expanded.
  8. If two attempts give no physical response: ABORT, do not run the main arc.
  9. Once committed: wait for the small move to settle, take the ACTUAL measured
     state, find the nearest COMPATIBLE point on the planned trajectory, restart
     timed execution from there with trajectory time reset to zero, and only
     then enable the normal progress/freeze watchdog (run_leg).
 10. Main arc = forward (resume..end) + reverse (end..home). Feedback timeout,
     joint limits and gross divergence stay active; there is NO startup grace.

Safety (always active, aborts): feedback validity, hard joint limits, gross
single-sample joint jump, physically-unsafe xyz. No "no-motion" abort in the
handshake (that is what is being proven); the normal watchdog is only enabled
for the main arc.

Usage:
    python constX-handshake.py                  # home+plan+HANDSHAKE+ (if committed) full F+R arc
    python constX-handshake.py --handshake-only # handshake only, skip the main arc
    python constX-handshake.py --dry            # home+plan+validate only, NO motion
"""
import argparse
import datetime
import inspect
import json
import math
import os
import sys
import time
import importlib.util

here = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("arc_constx", os.path.join(here, "arc-constX.py"))
cx = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cx)

SPEED = cx.SPEED            # 1000 (unchanged)
ACC = cx.ACC                # 50    (unchanged)
RATE_HZ = cx.RATE_HZ        # 20.0  (unchanged)
RADIUS_MM = 150.0
SWEEP_DEG = 110.0

# --- handshake parameters ---
ENVELOPE_RAD = math.radians(3.0)     # bounded startup envelope (max joint disp from measured start)
HOLD_S = 3.0                          # hold the bounded target (repeated T=102) after the ramp
STOP_S = 2.0                          # stop T=102 and observe (commit-after-stop window)
MOVE_TOL_RAD = math.radians(0.5)      # "measured motion > 0.5 deg" => committed
MAX_ATTEMPTS = 2
SETTLE_TIMEOUT_S = 5.0
SETTLE_TOL_RAD = math.radians(0.5)
SETTLE_STABLE = 4
# --- safety (abort) thresholds for the handshake ---
INVALID_FB_ABORT = 3
GROSS_JUMP_RAD = math.radians(20.0)   # single 50 ms sample joint jump
XYZ_LO, XYZ_HI = 0.0, 400.0
RESUME_GROSS_RAD = cx.GROSS_ERR_RAD   # 15 deg: max joint err for a "compatible" resume point


def _in_limits(meas):
    return all(cx.JOINT_LIMITS[k][0] <= meas[k] <= cx.JOINT_LIMITS[k][1] for k in range(6))


def _xyz_safe(fb):
    return all(XYZ_LO <= fb[j] <= XYZ_HI for j in range(3))


def measure_stable(arm, frames=8):
    """Return a plausible stable measured joint vector (fb[4:10]) or None."""
    last = None
    for _ in range(frames):
        fb = arm.feedback_get()
        if cx._valid(fb) and _xyz_safe(fb) and _in_limits(list(fb[4:10])):
            last = list(fb[4:10])
        time.sleep(0.1)
    return last


def find_bounded_target(joints, start_j, envelope_rad):
    """Furthest trajectory index whose max joint displacement from start_j is
    <= envelope_rad, taking the contiguous block from waypoint 0 (the ramp).
    Returns (index, max_disp_rad, per_joint_disp_deg). index may be -1."""
    best_i = -1
    best_disp = 0.0
    for i, j in enumerate(joints):
        disp = max(abs(j[k] - start_j[k]) for k in range(5))
        if disp <= envelope_rad:
            best_i = i
            best_disp = disp
        else:
            break  # the ramp leaves the envelope; keep the contiguous block
    per_joint = ([math.degrees(abs(joints[best_i][k] - start_j[k])) for k in range(5)]
                 if best_i >= 0 else None)
    return best_i, best_disp, per_joint


def wait_settled(arm, timeout_s=SETTLE_TIMEOUT_S, tol_rad=SETTLE_TOL_RAD,
                 stable_needed=SETTLE_STABLE):
    """Wait until measured joints are stable; return the settled joint vector
    (best-effort last plausible state, may be None)."""
    t0 = time.monotonic()
    last = None
    stable = 0
    while time.monotonic() - t0 < timeout_s:
        fb = arm.feedback_get()
        if cx._valid(fb) and _xyz_safe(fb) and _in_limits(list(fb[4:10])):
            meas = list(fb[4:10])
            if last is not None and max(abs(meas[k] - last[k]) for k in range(5)) < tol_rad:
                stable += 1
            else:
                stable = 1
            last = meas
            if stable >= stable_needed:
                return last
        else:
            stable = 0
        time.sleep(0.1)
    return last


def bounded_startup_handshake(arm, joints, logf):
    """Bounded startup / commit handshake. Proves the arm is physically
    responding before the main trajectory advances. Returns a result dict."""
    result = {
        "committed": False,
        "attempts": 0,
        "commit_phase": None,
        "first_motion_t_s": None,
        "commit_state": None,
        "settled_state": None,
        "resume_index": None,
        "resume_err_deg": None,
        "startup_target_index": None,
        "startup_max_disp_deg": None,
        "startup_per_joint_deg": None,
        "per_attempt": [],
        "aborted": False,
        "abort_reason": None,
    }

    for attempt in range(1, MAX_ATTEMPTS + 1):
        result["attempts"] = attempt
        att = {
            "attempt": attempt,
            "start_state": None,
            "startup_target_index": None,
            "startup_max_disp_deg": None,
            "startup_per_joint_deg": None,
            "committed": False,
            "commit_phase": None,
            "first_motion_t_s": None,
            "commit_state": None,
            "samples": [],
        }
        M = measure_stable(arm)
        if M is None:
            result["aborted"] = True
            result["abort_reason"] = "no stable measured state at start of attempt %d" % attempt
            result["per_attempt"].append(att)
            break
        att["start_state"] = [round(v, 4) for v in M]

        i_target, max_disp, per_joint = find_bounded_target(joints, M, ENVELOPE_RAD)
        if i_target < 1:
            result["aborted"] = True
            result["abort_reason"] = ("no trajectory point within %.0f deg envelope "
                                      "(i_target=%d); cannot prove a bounded startup" %
                                      (math.degrees(ENVELOPE_RAD), i_target))
            result["per_attempt"].append(att)
            break
        att["startup_target_index"] = i_target
        att["startup_max_disp_deg"] = round(math.degrees(max_disp), 3)
        att["startup_per_joint_deg"] = [round(v, 3) for v in per_joint]
        logf.write("# attempt %d: M=[%s]  i_target=%d  max_disp=%.3f deg  per_joint=[%s] deg\n" % (
            attempt, " ".join("%.4f" % v for v in M), i_target, math.degrees(max_disp),
            " ".join("%.2f" % v for v in per_joint)))

        ramp = joints[:i_target + 1]
        hold_cmd = joints[i_target]
        fb_bad = 0
        prev_meas = None
        t0 = time.monotonic()

        def sample(phase, idx, cmd):
            nonlocal fb_bad, prev_meas
            fb = arm.feedback_get()
            t = time.monotonic() - t0
            if cx._valid(fb):
                fb_bad = 0
                meas = list(fb[4:10])
                moved = max(abs(meas[k] - M[k]) for k in range(5))
                if not _in_limits(meas):
                    result["aborted"] = True
                    result["abort_reason"] = "hard joint limit at attempt %d phase %s" % (attempt, phase)
                elif not _xyz_safe(fb):
                    result["aborted"] = True
                    result["abort_reason"] = "unsafe xyz at attempt %d phase %s" % (attempt, phase)
                elif prev_meas is not None:
                    jump = max(abs(meas[k] - prev_meas[k]) for k in range(5))
                    if jump > GROSS_JUMP_RAD:
                        result["aborted"] = True
                        result["abort_reason"] = ("gross single-sample jump %.1f deg at attempt %d phase %s" %
                                                  (math.degrees(jump), attempt, phase))
                prev_meas = meas
                if not att["committed"] and moved > MOVE_TOL_RAD:
                    att["committed"] = True
                    att["commit_phase"] = phase
                    att["first_motion_t_s"] = round(t, 3)
                    att["commit_state"] = [round(v, 4) for v in meas]
                att["samples"].append({
                    "t": round(t, 3), "phase": phase, "idx": idx,
                    "cmd": [round(v, 4) for v in cmd] if cmd is not None else None,
                    "meas": [round(v, 4) for v in meas],
                    "moved_deg": round(math.degrees(moved), 3)})
                logf.write("t=%.3f att=%d phase=%s idx=%s cmd=[%s] meas=[%s] moved=%.3f\n" % (
                    t, attempt, phase, idx,
                    " ".join("%.4f" % v for v in cmd) if cmd is not None else "-----",
                    " ".join("%.4f" % v for v in meas), math.degrees(moved)))
            else:
                fb_bad += 1
                att["samples"].append({"t": round(t, 3), "phase": phase, "idx": idx, "invalid": True})
                logf.write("t=%.3f att=%d phase=%s idx=%s INVALID_FEEDBACK\n" % (t, attempt, phase, idx))
                if fb_bad >= INVALID_FB_ABORT:
                    result["aborted"] = True
                    result["abort_reason"] = "%d consecutive invalid fb at attempt %d phase %s" % (
                        fb_bad, attempt, phase)

        # PHASE ramp: stream wp0..i_target (changing targets, inside envelope)
        for i, cmd in enumerate(ramp):
            arm.joints_radian_ctrl(cmd, SPEED, ACC)
            sample("ramp", i, cmd)
            if result["aborted"] or att["committed"]:
                break
            delay = (t0 + i / RATE_HZ) - time.monotonic()
            if delay > 0:
                time.sleep(delay)

        # PHASE hold: repeat i_target (held target, inside envelope)
        if not result["aborted"] and not att["committed"]:
            n_hold = int(round(HOLD_S * RATE_HZ))
            t_h0 = time.monotonic()
            for j in range(n_hold):
                arm.joints_radian_ctrl(hold_cmd, SPEED, ACC)
                sample("hold", i_target, hold_cmd)
                if result["aborted"] or att["committed"]:
                    break
                delay = (t_h0 + j / RATE_HZ) - time.monotonic()
                if delay > 0:
                    time.sleep(delay)

        # PHASE stop: no T=102, observe for STOP_S (commit-after-stop)
        if not result["aborted"] and not att["committed"]:
            n_stop = int(round(STOP_S * RATE_HZ))
            t_s0 = time.monotonic()
            for k in range(n_stop):
                sample("stop", None, None)
                if result["aborted"]:
                    break
                delay = (t_s0 + k / RATE_HZ) - time.monotonic()
                if delay > 0:
                    time.sleep(delay)

        result["per_attempt"].append(att)
        if result["aborted"]:
            break
        if not att["committed"]:
            logf.write("# attempt %d: NO physical response; %s\n" % (
                attempt,
                "retrying from newly measured state" if attempt < MAX_ATTEMPTS else "giving up"))
            continue

        # ---- committed: settle, take measured state, find compatible resume point ----
        result["committed"] = True
        result["commit_phase"] = att["commit_phase"]
        result["first_motion_t_s"] = att["first_motion_t_s"]
        result["commit_state"] = att["commit_state"]
        result["startup_target_index"] = i_target
        result["startup_max_disp_deg"] = att["startup_max_disp_deg"]
        result["startup_per_joint_deg"] = att["startup_per_joint_deg"]
        logf.write("# attempt %d COMMITTED in phase=%s at t=%.3f s meas=[%s]\n" % (
            attempt, att["commit_phase"], att["first_motion_t_s"],
            " ".join("%.4f" % v for v in att["commit_state"])))
        time.sleep(0.5)
        settled = wait_settled(arm)
        if settled is None:
            result["aborted"] = True
            result["abort_reason"] = "could not obtain a settled state after commit"
            break
        result["settled_state"] = [round(v, 4) for v in settled]
        best_i, best_err = None, None
        for i, j in enumerate(joints):
            err = max(abs(j[k] - settled[k]) for k in range(5))
            if best_err is None or err < best_err:
                best_i, best_err = i, err
        if best_err > RESUME_GROSS_RAD:
            result["aborted"] = True
            result["abort_reason"] = ("no compatible resume point (nearest traj err %.1f deg > %.1f deg)" %
                                      (math.degrees(best_err), math.degrees(RESUME_GROSS_RAD)))
            break
        result["resume_index"] = best_i
        result["resume_err_deg"] = round(math.degrees(best_err), 3)
        logf.write("# settled=[%s]  resume_index=%d  resume_err=%.3f deg\n" % (
            " ".join("%.4f" % v for v in settled), best_i, math.degrees(best_err)))
        break

    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM21")
    ap.add_argument("--dry", action="store_true", help="home + plan + validate only, NO motion")
    ap.add_argument("--handshake-only", action="store_true",
                    help="run the handshake but skip the main forward/reverse arc")
    ap.add_argument("--out", default=os.path.join(here, "constX-handshake-result.json"))
    ap.add_argument("--log", default=os.path.join(here, "constX-handshake.log"))
    args = ap.parse_args()

    ik_mod, ik_path = cx.load_ik_module()
    ik_fn = ik_mod.compute_joint_rad_by_pos

    import roarm_sdk
    sdk_dir = os.path.dirname(os.path.abspath(inspect.getfile(roarm_sdk)))
    sdk_dir_norm = sdk_dir.replace(os.sep, "/")
    repo_root_norm = os.path.dirname(os.path.dirname(here)).replace(os.sep, "/")
    if "site-packages/roarm_sdk" not in sdk_dir_norm:
        raise SystemExit("refusing to run: not the stock site-packages SDK: %s" % sdk_dir)
    if repo_root_norm in sdk_dir_norm:
        raise SystemExit("refusing to run: importing the repo SDK, not site-packages: %s" % sdk_dir)
    from roarm_sdk.roarm import roarm

    try:
        import importlib.metadata as _imd
        roarm_sdk_ver = _imd.version("roarm-sdk")
    except Exception:
        roarm_sdk_ver = "unknown"
    try:
        import serial as _pyserial
        pyserial_ver = _pyserial.__version__
    except Exception:
        pyserial_ver = None

    print("=== environment ===")
    print("python      %s (%s)" % (sys.version.split()[0], sys.executable))
    print("roarm-sdk   %s  (%s)" % (roarm_sdk_ver, roarm_sdk.__file__))
    print("pyserial    %s" % pyserial_ver)
    print("ik          %s" % ik_path)
    print("params      T=102 speed=%d acc=%d rate=%.0f Hz  R=%.0fmm sweep=%.0f deg" % (
        SPEED, ACC, RATE_HZ, RADIUS_MM, SWEEP_DEG))
    print("handshake   envelope=%.0f deg  hold=%.1f s  stop=%.1f s  move_tol=%.1f deg  attempts<=%d" % (
        math.degrees(ENVELOPE_RAD), HOLD_S, STOP_S, math.degrees(MOVE_TOL_RAD), MAX_ATTEMPTS))
    print("mode        %s" % ("DRY" if args.dry else
                              ("HANDSHAKE-ONLY" if args.handshake_only else "HANDSHAKE + FULL ARC")))

    print("opening %s" % args.port)
    import serial as _serial_mod
    try:
        arm = roarm(roarm_type="roarm_m3", port=args.port, baudrate=115200)
    except _serial_mod.SerialException as e:
        raise SystemExit("could not open %s (another application may own it): %s" % (args.port, e))
    serial_settings = cx._serial_settings(arm._serial_port)

    fb = cx.wait_valid_fb(arm)
    if fb is None:
        raise SystemExit("no valid feedback; aborting before any motion")
    print("pre-home pose xyz=[%.2f %.2f %.2f] joints=[%s]" % (
        fb[0], fb[1], fb[2], " ".join("%.4f" % v for v in fb[4:10])))

    print("homing (move_init) ...")
    reached, home_err, home_polls = cx.home_and_settle(arm)
    print("home reached=%s err=%s polls=%d" % (
        reached, ("%.2f deg" % math.degrees(home_err)) if home_err is not None else None, home_polls))
    if not reached:
        raise SystemExit("arm did not settle at home; aborting before any motion")

    print("reading 10 consecutive feedback frames at home (stability gate)")
    stable_fb, stable_records = cx.read_stable_state(arm, frames=10)
    if stable_fb is None:
        raise SystemExit("communication unstable at home (10-frame gate failed); aborting before any motion")
    x0, y0, z0 = stable_fb[0], stable_fb[1], stable_fb[2]
    pitch0, roll0 = stable_fb[3], stable_fb[8]
    grip0 = stable_fb[9]
    start_joints = list(stable_fb[4:10])
    print("home pose   xyz=[%.2f %.2f %.2f]  pitch=%.4f  roll=%.4f  grip=%.4f" % (
        x0, y0, z0, pitch0, roll0, grip0))
    print("home joints [%s]" % " ".join("%.4f" % v for v in start_joints))

    print("planning CONSTANT-X arc  X=%.2f mm  R=%.0f mm / sweep=%.0f deg" % (
        x0, RADIUS_MM, SWEEP_DEG))
    plans = cx.plan_const_x(ik_fn, x0, y0, z0, pitch0, roll0, grip0, start_joints, RADIUS_MM, SWEEP_DEG)
    feasible = [p for p in plans if p.get("feasible")]
    if not feasible:
        raise SystemExit("no feasible constant-X arc; NO motion sent")
    best = min(feasible, key=lambda p: (-min(w[2] for w in p["waypoints"]), p["peak_abs_base_rad"]))
    joints = best["joints"]
    n_wp = best["n_waypoints"]
    print("selected     center_yz=[%.1f %.1f] sign=%+d  (min Z=%.1f mm)  waypoints=%d" % (
        best["center_yz_mm"][0], best["center_yz_mm"][1], best["sweep_sign"],
        min(w[2] for w in best["waypoints"]), n_wp))
    print("path length  %.1f mm   rate=%.0f Hz   full-arc duration=%.2f s" % (
        best["path_length_mm"], RATE_HZ, (n_wp - 1) / RATE_HZ))

    result_common = {
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "platform": sys.platform,
        "port": args.port,
        "serial_settings": serial_settings,
        "roarm_sdk_version": roarm_sdk_ver,
        "roarm_sdk_file": roarm_sdk.__file__,
        "pyserial_version": pyserial_ver,
        "ik_module": ik_path,
        "params": {"speed": SPEED, "acc": ACC, "rate_hz": RATE_HZ,
                   "radius_mm": RADIUS_MM, "sweep_deg": SWEEP_DEG},
        "handshake_params": {"envelope_rad": ENVELOPE_RAD, "hold_s": HOLD_S, "stop_s": STOP_S,
                             "move_tol_rad": MOVE_TOL_RAD, "max_attempts": MAX_ATTEMPTS},
        "home_pose": {"xyz_mm": [x0, y0, z0], "pitch_rad": pitch0, "roll_rad": roll0,
                      "gripper": grip0, "joints_rad": start_joints},
        "selected_plan": {k: v for k, v in best.items() if k not in ("waypoints", "joints")},
    }

    if args.dry:
        result_common["verdict"] = "DRY_FEASIBLE"
        result_common["candidate_evaluations"] = plans
        with open(args.out, "w") as fh:
            json.dump(result_common, fh, indent=1)
        print("DRY RUN: constant-X arc feasible; handshake NOT run (pass no --dry to run)")
        print("wrote %s" % args.out)
        print("RESULT: DRY_FEASIBLE")
        return 0

    print("--- running BOUNDED STARTUP / COMMIT HANDSHAKE ---")
    logf = open(args.log, "w")
    logf.write("# %s  port=%s  sdk=%s  ik=%s  home_xyz=[%.2f %.2f %.2f]  constX=%.2f  "
               "envelope=%.1fdeg hold=%.1fs stop=%.1fs move_tol=%.1fdeg\n" % (
                   datetime.datetime.now(datetime.timezone.utc).isoformat(),
                   args.port, sdk_dir, ik_path, x0, y0, z0, x0,
                   math.degrees(ENVELOPE_RAD), HOLD_S, STOP_S, math.degrees(MOVE_TOL_RAD)))
    handshake = bounded_startup_handshake(arm, joints, logf)
    logf.flush()
    result_common["handshake"] = handshake

    forward = None
    reverse = None
    arc_ran = False
    if handshake["aborted"]:
        print("HANDSHAKE ABORT: %s" % handshake["abort_reason"])
    elif not handshake["committed"]:
        print("HANDSHAKE: no physical response after %d bounded attempts; NOT running the main arc" %
              handshake["attempts"])
    else:
        print("HANDSHAKE COMMITTED: phase=%s first_motion_t=%.3f s attempts=%d" % (
            handshake["commit_phase"], handshake["first_motion_t_s"], handshake["attempts"]))
        print("  startup target wp=%d (max disp %.2f deg)  resume_index=%d (err %.2f deg)" % (
            handshake["startup_target_index"], handshake["startup_max_disp_deg"],
            handshake["resume_index"], handshake["resume_err_deg"]))
        print("  settled=[%s]" % " ".join("%.4f" % v for v in handshake["settled_state"]))

        if args.handshake_only:
            print("HANDSHAKE-ONLY: skipping the main forward/reverse arc")
        else:
            arc_ran = True
            resume_i = handshake["resume_index"]
            z_record = [z0]
            logf.write("\n===== MAIN ARC: forward leg (resume_index=%d .. %d) =====\n" % (
                resume_i, n_wp - 1))
            forward = cx.run_leg(arm, joints[resume_i:], "F", logf, z_record)
            print("forward  aborted=%s reason=%s cmd=%d fb_ok=%d fb_bad=%d max_track=%.2f deg freeze=%d rate=%s Hz" % (
                forward["aborted"], forward["abort_reason"], forward["commands_sent"],
                forward["fb_ok"], forward["fb_invalid"], math.degrees(forward["max_tracking_err_rad"]),
                forward["freeze_windows"], forward["achieved_cmd_rate_hz"]))
            if not forward["aborted"]:
                logf.write("\n===== MAIN ARC: reverse leg (%d .. 0) =====\n" % (n_wp - 1))
                reverse = cx.run_leg(arm, list(reversed(joints)), "R", logf, z_record)
                print("reverse  aborted=%s reason=%s cmd=%d fb_ok=%d fb_bad=%d max_track=%.2f deg freeze=%d rate=%s Hz" % (
                    reverse["aborted"], reverse["abort_reason"], reverse["commands_sent"],
                    reverse["fb_ok"], reverse["fb_invalid"], math.degrees(reverse["max_tracking_err_rad"]),
                    reverse["freeze_windows"], reverse["achieved_cmd_rate_hz"]))
            result_common["forward"] = forward
            result_common["reverse"] = reverse
            result_common["arc_ran"] = True
            result_common["resume_index"] = resume_i
            result_common["measured_z_range_mm"] = [min(z_record), max(z_record)] if z_record else None
    logf.close()

    # ---- leave the arm safe at home ----
    print("returning home (move_init) ...")
    arm.move_init()
    h_reached, h_err, h_polls = cx.home_and_settle(arm)
    print("home reached=%s err=%s polls=%d" % (
        h_reached, ("%.2f deg" % math.degrees(h_err)) if h_err is not None else None, h_polls))
    result_common["return_home"] = {"reached": h_reached, "err_rad": h_err, "polls": h_polls}

    # ---- verdict ----
    if handshake["aborted"]:
        result_common["verdict"] = "HANDSHAKE_ABORTED"
    elif not handshake["committed"]:
        result_common["verdict"] = "HANDSHAKE_NO_RESPONSE"
    elif not arc_ran:
        result_common["verdict"] = "COMMITTED_HANDSHAKE_ONLY"
    elif (forward and forward["aborted"]) or (reverse is not None and reverse["aborted"]):
        result_common["verdict"] = "ARC_ABORTED"
    else:
        result_common["verdict"] = "COMPLETE"

    with open(args.out, "w") as fh:
        json.dump(result_common, fh, indent=1)

    # ---- report ----
    print("\n===== REPORT =====")
    print("startup waypoint (bounded target) : wp=%d" % (handshake["startup_target_index"]))
    print("max commanded joint disp in env   : %.2f deg (per-joint [%s] deg)" % (
        handshake["startup_max_disp_deg"] or -1,
        " ".join("%.2f" % v for v in (handshake["startup_per_joint_deg"] or []))))
    print("commit phase (ramp|hold|stop)     : %s" % handshake["commit_phase"])
    print("time to first >0.5 deg motion     : %s s (from attempt start)" %
          handshake["first_motion_t_s"])
    print("number of startup attempts        : %d" % handshake["attempts"])
    print("measured state at commit          : %s" % (
        "[%s]" % " ".join("%.4f" % v for v in handshake["commit_state"])
        if handshake["commit_state"] else None))
    print("main trajectory resumed at        : wp=%d (err %.2f deg)  settled=[%s]" % (
        handshake["resume_index"], handshake["resume_err_deg"] or -1,
        " ".join("%.4f" % v for v in (handshake["settled_state"] or []))))
    if arc_ran:
        arc_exec_txt = "yes (forward resume->end + reverse end->home)"
    elif handshake["committed"] and not handshake["aborted"]:
        arc_exec_txt = "no (handshake-only)"
    else:
        arc_exec_txt = "no (handshake did not commit / aborted)"
    print("full constant-X F+R arc executed  : %s" % arc_exec_txt)
    for leg_name, leg in (("forward", forward), ("reverse", reverse)):
        if leg is not None:
            print("  %s: cmd=%d fb_ok=%d fb_bad=%d freeze=%d max_track=%.2f deg final_err=%s" % (
                leg_name, leg["commands_sent"], leg["fb_ok"], leg["fb_invalid"],
                leg["freeze_windows"], math.degrees(leg["max_tracking_err_rad"]),
                ("%.2f deg" % math.degrees(leg["final_joint_err_max_rad"]))
                if leg["final_joint_err_max_rad"] is not None else "n/a"))
    print("VERDICT: %s" % result_common["verdict"])
    if handshake["aborted"]:
        print("ABORT: %s" % handshake["abort_reason"])
    print("wrote %s" % args.out)
    print("wrote %s" % args.log)
    print("DONE")
    return 0 if result_common["verdict"] in ("COMPLETE", "COMMITTED_HANDSHAKE_ONLY", "DRY_FEASIBLE") else 1


if __name__ == "__main__":
    sys.exit(main())
