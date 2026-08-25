#!/usr/bin/env python3
"""CONST-Z regression: apply the SAME continuous commit->execution handoff rule
that eliminated the 14-deg const-X second-startup transient to the known-good
150 mm / 110 deg const-Z arc. Confirms the rule generalizes without regressing
constant-Z tracking.

The rule is imported verbatim from constX-continuous.py (cc):
    find_bounded_target -> bounded_startup (continuous pump, no static hold)
    -> continuous_forward (align + advance, single 20 Hz T102 stream, no break)
    -> run_leg_tracked (continuous reverse)
The const-Z trajectory, homing, IK and feedback helpers are imported from
arc150-260.py (az). No firmware / SDK / IK / speed / acc / rate / trajectory /
watchdog changes.

Usage:
    python constZ-continuous.py --dry                 # home + plan, no arc
    python constZ-continuous.py --tag reg1            # full const-Z arc (F+R)
"""

import argparse
import datetime
import importlib.util
import json
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _close_arm(arm):
    try:
        sp = getattr(arm, "_serial_port", None)
        if sp is not None:
            sp.close()
    except Exception:
        pass


az = _load("az", os.path.join(HERE, "arc150-260.py"))
cc = _load("cc", os.path.join(os.path.dirname(HERE), "windows-constX-arc", "constX-continuous.py"))

RADIUS_MM = 150.0
SWEEP_DEG = 110.0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default="COM21")
    ap.add_argument("--radius-mm", type=float, default=RADIUS_MM)
    ap.add_argument("--sweep-deg", type=float, default=SWEEP_DEG)
    ap.add_argument("--dry", action="store_true", help="home + plan, no arc")
    ap.add_argument("--forward-only", action="store_true", help="skip the reverse leg")
    ap.add_argument("--tag", default="constZ")
    ap.add_argument("--speed", type=int, default=cc.SPEED,
                    help="override the joint speed (default %d = 1x; e.g. 3000 = 3x)" % cc.SPEED)
    ap.add_argument("--acc", type=int, default=cc.ACC,
                    help="override the joint acceleration (default %d = 1x; e.g. 150 = 3x)" % cc.ACC)
    ap.add_argument("--out", default=None)
    ap.add_argument("--log", default=None)
    args = ap.parse_args()
    out = args.out or os.path.join(HERE, "constZ-%s-result.json" % args.tag)
    log = args.log or os.path.join(HERE, "constZ-%s.log" % args.tag)
    cc.SPEED = args.speed
    cc.ACC = args.acc

    ik_mod, ik_path = az.load_ik_module()
    ik_fn = ik_mod.compute_joint_rad_by_pos

    import roarm_sdk
    import serial as _serial_mod
    from roarm_sdk.roarm import roarm
    arm = roarm(roarm_type="roarm_m3", port=args.port, baudrate=115200)
    fb = az.wait_valid_fb(arm)
    if fb is None:
        _close_arm(arm)
        raise SystemExit("no valid feedback; aborting before any motion")

    print("homing (move_init) ...")
    reached, home_err, home_polls = az.home_and_settle(arm)
    print("home reached=%s err=%s polls=%d" % (
        reached, ("%.2f deg" % math.degrees(home_err)) if home_err is not None else None, home_polls))
    if not reached:
        _close_arm(arm)
        raise SystemExit("arm did not settle at home; aborting before any motion")

    print("reading stable state at home (retrying)")
    stable_fb, _ = cc.read_stable_state(arm, frames=10, retries=3)
    if stable_fb is None:
        _close_arm(arm)
        raise SystemExit("communication unstable at home; aborting before any motion")
    x0, y0, z0 = stable_fb[0], stable_fb[1], stable_fb[2]
    pitch0, roll0 = stable_fb[3], stable_fb[8]
    grip0 = stable_fb[9]
    start_joints = list(stable_fb[4:10])
    print("home pose   xyz=[%.2f %.2f %.2f]  z0=%.2f  grip=%.4f" % (x0, y0, z0, z0, grip0))
    print("home joints [%s]" % " ".join("%.4f" % v for v in start_joints))

    print("planning const-Z arc R=%.0f mm / sweep=%.0f deg" % (args.radius_mm, args.sweep_deg))
    plans = az.plan_all(ik_fn, x0, y0, z0, pitch0, roll0, grip0, start_joints,
                        args.radius_mm, args.sweep_deg)
    feasible = [p for p in plans if p.get("feasible")]
    if not feasible:
        _close_arm(arm)
        raise SystemExit("no feasible const-Z arc inside the limits; NO motion")
    best = min(feasible, key=lambda p: p["peak_abs_base_rad"])
    joints = best["joints"]
    max_step_rad = best["max_adjacent_step_rad"]
    RESUME_TOL_RAD = max(math.radians(3.0), 3.0 * max_step_rad)
    print("selected     center=[%.1f %.1f] sign=%+d  (smallest peak |base| = %.1f deg)" % (
        best["center_mm"][0], best["center_mm"][1], best["sweep_sign"],
        math.degrees(best["peak_abs_base_rad"])))
    print("path length  %.1f mm   waypoints=%d   rate=%.0f Hz   z0=%.2f mm" % (
        best["path_length_mm"], best["n_waypoints"], cc.RATE_HZ, z0))
    print("resume gate  %.2f deg (max(3deg, 3 x step %.3f deg))" % (
        math.degrees(RESUME_TOL_RAD), math.degrees(max_step_rad)))

    if args.dry:
        print("DRY RUN: const-Z arc feasible; rule NOT run")
        with open(out, "w") as fh:
            json.dump({"verdict": "DRY_FEASIBLE", "selected": {k: v for k, v in best.items()
                        if k not in ("waypoints", "joints")},
                       "home_pose": {"xyz_mm": [x0, y0, z0], "joints_rad": start_joints},
                       "rule_params": {"envelope_rad": cc.ENVELOPE_RAD,
                                       "move_tol_rad": cc.MOVE_TOL_RAD,
                                       "progress_tol_rad": cc.PROGRESS_TOL_RAD,
                                       "sweep_max_s": cc.STARTUP_MAX_S,
                                       "max_attempts": cc.MAX_ATTEMPTS,
                                       "resume_tol_rad": RESUME_TOL_RAD}}, fh, indent=1)
        print("wrote %s" % out)
        _close_arm(arm)
        return 0

    logf = open(log, "w")
    logf.write("# constZ-continuous tag=%s port=%s ik=%s home_xyz=[%.2f %.2f %.2f] z0=%.2f "
               "R=%.0f sweep=%.0f waypoints=%d envelope=%.1fdeg sweep_max=%.1fs move_tol=%.1fdeg "
               "progress_tol=%.1fdeg resume_tol=%.2fdeg\n" % (
                   args.tag, args.port, ik_path, x0, y0, z0, z0,
                   args.radius_mm, args.sweep_deg, best["n_waypoints"],
                   math.degrees(cc.ENVELOPE_RAD), cc.STARTUP_MAX_S, math.degrees(cc.MOVE_TOL_RAD),
                   math.degrees(cc.PROGRESS_TOL_RAD),
                   math.degrees(RESUME_TOL_RAD)))
    z_record = [z0]
    forward = reverse = None
    arc_ran = False
    handshake = {"committed": False, "attempts": 0, "aborted": False, "abort_reason": None,
                 "commit_phase": None, "first_motion_t_s": None, "t102_continuous": None,
                 "resume_index": None, "resume_err_deg": None, "per_attempt": []}

    for attempt in range(1, cc.MAX_ATTEMPTS + 1):
        handshake["attempts"] = attempt
        M = cc.measure_stable(arm)
        if M is None:
            handshake["aborted"] = True
            handshake["abort_reason"] = "no stable measured state at attempt %d" % attempt
            break
        i_target, max_disp, per_joint = cc.find_bounded_target(joints, M, cc.ENVELOPE_RAD)
        if i_target < 1:
            handshake["aborted"] = True
            handshake["abort_reason"] = "no on-path point within envelope (i_target=%d)" % i_target
            break
        logf.write("# attempt %d: M=[%s] i_target=%d max_disp=%.3f deg per_joint=[%s] deg\n" % (
            attempt, " ".join("%.4f" % v for v in M), i_target, math.degrees(max_disp),
            " ".join("%.2f" % v for v in per_joint)))
        su = cc.bounded_startup(arm, joints, M, i_target, logf, attempt)
        handshake["per_attempt"].append({
            "attempt": attempt, "start_state": [round(v, 4) for v in M],
            "startup_target_index": i_target,
            "startup_max_disp_deg": round(math.degrees(max_disp), 3),
            "committed": su["committed"], "commit_phase": su["commit_phase"],
            "first_motion_t_s": su["first_motion_t"], "commit_state": su["commit_state"],
            "t102_continuous": su["t102_continuous"], "aborted": su["aborted"],
            "abort_reason": su.get("abort_reason")})
        if su["aborted"]:
            handshake["aborted"] = True
            handshake["abort_reason"] = su["abort_reason"]
            break
        if not su["committed"]:
            if attempt < cc.MAX_ATTEMPTS:
                logf.write("# attempt %d: NO commit after pump; retrying\n" % attempt)
                continue
            handshake["aborted"] = True
            handshake["abort_reason"] = "no commit after %d pump attempts" % attempt
            break
        logf.write("# attempt %d COMMITTED phase=%s t=%.3f t102_continuous=%s\n" % (
            attempt, su["commit_phase"], su["first_motion_t"], su["t102_continuous"]))
        forward, resume_i, resume_err_deg = cc.continuous_forward(arm, joints, i_target, M, su["commit_state"], logf, z_record)
        handshake["resume_index"] = resume_i
        handshake["resume_err_deg"] = resume_err_deg
        resume_ok = resume_err_deg is not None and resume_err_deg <= math.degrees(RESUME_TOL_RAD)
        if forward["aborted"] or not resume_ok:
            why = forward["abort_reason"] if forward["aborted"] else "resume err too large"
            if attempt < cc.MAX_ATTEMPTS:
                logf.write("# attempt %d: advance failed (%s); retrying\n" % (attempt, why))
                forward = None
                continue
            handshake["aborted"] = True
            handshake["abort_reason"] = "advance failed after %d attempts: %s" % (attempt, why)
            break
        handshake["committed"] = True
        handshake["commit_phase"] = su["commit_phase"]
        handshake["first_motion_t_s"] = su["first_motion_t"]
        handshake["t102_continuous"] = su["t102_continuous"]
        if not args.forward_only:
            logf.write("\n===== REVERSE LEG (continuous, %d .. 0) =====\n" % (len(joints) - 1))
            reverse = cc.run_leg_tracked(arm, list(reversed(joints)), "R", logf, z_record)
            arc_ran = True
        break

    logf.close()

    # leave the arm safe at home
    arm.move_init()
    h_reached, h_err, h_polls = az.home_and_settle(arm)

    if handshake["aborted"]:
        verdict = "ARC_ABORTED"
    elif args.forward_only:
        verdict = "FORWARD_RULE_COMPLETE" if (forward and not forward["aborted"]) else "FORWARD_ABORTED"
    elif (forward and forward["aborted"]) or (reverse is not None and reverse["aborted"]):
        verdict = "ARC_ABORTED"
    else:
        verdict = "COMPLETE"

    fdeg = math.degrees(forward["max_tracking_err_rad"]) if forward else None
    rdeg = math.degrees(reverse["max_tracking_err_rad"]) if reverse else None
    zmin, zmax = (min(z_record), max(z_record)) if z_record else (None, None)
    result = {
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "tag": args.tag, "port": args.port, "ik": ik_path,
        "verdict": verdict, "arc_ran": arc_ran,
        "mode": "const-Z (same continuous rule as constX-continuous)",
        "radius_mm": args.radius_mm, "sweep_deg": args.sweep_deg,
        "selected": {k: v for k, v in best.items() if k not in ("waypoints", "joints")},
        "rule_params": {"envelope_rad": cc.ENVELOPE_RAD, "move_tol_rad": cc.MOVE_TOL_RAD,
                        "progress_tol_rad": cc.PROGRESS_TOL_RAD,
                        "sweep_max_s": cc.STARTUP_MAX_S, "max_attempts": cc.MAX_ATTEMPTS,
                        "handoff": "direct transition to compatible wp (no align hold, no T102 gap)",
                        "resume_tol_rad": RESUME_TOL_RAD,
                        "resume_tol_formula": "max(3 deg, 3 x max_step=%.3f deg) = %.3f deg" % (
                            math.degrees(max_step_rad), math.degrees(RESUME_TOL_RAD))},
        "handshake": handshake, "forward": forward, "reverse": reverse,
        "forward_max_track_deg": fdeg, "reverse_max_track_deg": rdeg,
        "z0_mm": z0, "measured_z_range_mm": [zmin, zmax],
        "measured_z_max_deviation_mm": (max(abs(zmin - z0), abs(zmax - z0)) if zmin is not None else None),
        "return_home": {"reached": h_reached, "err_rad": h_err, "polls": h_polls},
    }
    with open(out, "w") as fh:
        json.dump(result, fh, indent=1)

    print("--- rule result ---")
    print("handshake  committed=%s attempts=%s abort=%s" % (
        handshake["committed"], handshake["attempts"], handshake["abort_reason"]))
    print("forward    aborted=%s max_track=%s deg  fb_ok=%s fb_bad=%s freeze=%s" % (
        forward["aborted"] if forward else "n/a",
        ("%.2f" % fdeg) if fdeg is not None else "n/a",
        forward.get("fb_ok") if forward else "n/a",
        forward.get("fb_invalid") if forward else "n/a",
        forward.get("freeze_windows", "n/a") if forward else "n/a"))
    print("reverse    aborted=%s max_track=%s deg" % (
        reverse["aborted"] if reverse else "n/a", ("%.2f" % rdeg) if rdeg is not None else "n/a"))
    print("constant Z  z0=%.2f  measured=[%.2f %.2f]  max_dev=%.2f mm" % (
        z0, zmin, zmax, (max(abs(zmin - z0), abs(zmax - z0)) if zmin is not None else float("nan"))))
    print("home reached=%s" % h_reached)
    print("VERDICT: %s" % verdict)
    print("wrote %s" % out)
    print("wrote %s" % log)
    _close_arm(arm)
    return 0 if verdict in ("COMPLETE", "FORWARD_RULE_COMPLETE", "DRY_FEASIBLE") else 1


if __name__ == "__main__":
    sys.exit(main())
