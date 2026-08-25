#!/usr/bin/env python3
"""CONTINUOUS COMMIT -> EXECUTION HANDOFF for the 150 mm / 110 deg constant-X arc.

Diagnosis variant of constX-handshake.py that removes the SECOND startup transient.

constX-handshake.py did:
    commit -> STOP T102 (time.sleep(0.5) + feedback-only wait_settled) -> fresh run_leg
and the arm re-enters the command-deaf latching regime for ~0.6 s, so the forward leg
lags the commanded target by ~12 waypoints and peaks at 14.06 deg at local idx 12.

constX-continuous.py instead does:
    a single continuous forward sweep wp0..wp_i_target at 20 Hz (no return toward the
    start, no settling dwell, no static hold) -> commit as soon as the measured state
    shows real actuation (directional progress toward the bounded target) -> transition
    directly into the original trajectory and continue, all inside ONE continuous 20 Hz
    T102 command stream. There is no sleep, no feedback-only settle, no serial silence,
    no align hold, and no disconnected run_leg startup.

The bounded startup target exists to prove real actuation safely, NOT to demand
positioning accuracy during startup:
  1. commit = directional progress: max-joint displacement from the attempt start > 0.5
     deg AND max-joint dist(measured, target) has decreased by > 0.1 deg, on 2 consecutive
     valid feedback samples. Reaching or settling at the target is NOT required.
  2. trajectory re-entry uses a TIGHT compatibility gate RESUME_TOL =
     max(3 deg, 3 x max adjacent step), NOT the old 15 deg gross-error threshold.

UNCHANGED: planner, selection, IK, T=102 / speed=1000 / acc=50 / 20 Hz, the bounded
<=3 deg startup envelope, and the normal main-trajectory watchdog thresholds (ported
verbatim into the continuous advance). The rule starts from the ACTUAL measured state
M (homing only standardises the experiment; it is not a property of the rule).

The forward leg is executed by the rule (bounded startup -> commit -> continuous
align+advance). The reverse leg uses a behaviourally-identical tracked copy of the
known-good run_leg so the forward A/B comparison isolates the handoff change.

Usage:
    python constX-continuous.py                  # home+plan+RULE+full F+R arc
    python constX-continuous.py --forward-only   # rule + forward leg only (no reverse)
    python constX-continuous.py --dry            # home+plan+validate only, NO motion
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

here = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("arc_constx", os.path.join(here, "arc-constX.py"))
cx = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cx)

SPEED = cx.SPEED            # 1000 (unchanged)
ACC = cx.ACC                # 50    (unchanged)
RATE_HZ = cx.RATE_HZ        # 20.0  (unchanged)
RADIUS_MM = 150.0
SWEEP_DEG = 110.0

# --- bounded startup / commit ---
ENVELOPE_RAD = math.radians(3.0)      # bounded on-path startup envelope (max joint disp from M)
MOVE_TOL_RAD = math.radians(0.5)      # commit part 1: measured displacement from attempt start > 0.5 deg
PROGRESS_TOL_RAD = math.radians(0.1)  # commit part 2: distance to the bounded target decreased by > 0.1 deg
PROGRESS_STABLE = 2                   # commit requires this many consecutive valid samples showing directional progress
MAX_ATTEMPTS = 3                      # bounded-startup -> advance retries (arm is intermittently deaf)
# The startup is a single CONTINUOUS forward sweep (wp0..wp_i_target) at 20 Hz:
# always-changing commands keep the motion controller active. A T102 silence gap
# re-triggers the command-deaf latching, so we never go quiet. We do NOT return toward
# the start and do NOT hold still to "settle" the target: the bounded target only proves
# real actuation safely, it does not demand positioning accuracy during startup.
STARTUP_MAX_S = 10.0                  # backstop: if no directional progress within this, give up the attempt
# The monotonic wp0->bounded-target prefix is TIME-STRETCHED over PROBE_SECONDS at 20 Hz:
# a ~0.15 s raw sweep is too short to reliably detect a robot that takes ~0.8-1+ s to begin
# responding, so we resample the same on-path prefix (staying inside the 3 deg envelope)
# over ~1.5-2.0 s. Same target, same 20 Hz traffic, same directional commit criterion; the
# only change is the temporal execution. No reverse, no static endpoint hold, no dwell, no gap.
PROBE_SECONDS = 1.8                   # time-stretch window for the bounded forward prefix (1.5-2.0 s)

# --- safety (abort) ---
INVALID_FB_ABORT = cx.INVALID_FB_ABORT
GROSS_JUMP_RAD = math.radians(20.0)   # single 50 ms sample joint jump
XYZ_LO, XYZ_HI = 0.0, 400.0

# --- normal main-trajectory watchdog (UNCHANGED thresholds, ported from arc-constX) ---
GROSS_ERR_RAD = cx.GROSS_ERR_RAD            # 15 deg
GROSS_ERR_SUSTAIN = cx.GROSS_ERR_SUSTAIN    # 8
NO_MOTION_TOL_RAD = cx.NO_MOTION_TOL_RAD    # 0.5 deg at 1 s into the advance
FREEZE_WIN = cx.FREEZE_WIN
FREEZE_ARM = cx.FREEZE_ARM
FREEZE_MEAS_TOL_RAD = cx.FREEZE_MEAS_TOL_RAD
FREEZE_CMD_TOL_RAD = cx.FREEZE_CMD_TOL_RAD
FREEZE_SUSTAIN = cx.FREEZE_SUSTAIN
JOINT_NAMES = cx.JOINT_NAMES


def _maxj(a, b):
    return max(abs(a[k] - b[k]) for k in range(5))


def _in_limits(meas):
    return all(cx.JOINT_LIMITS[k][0] <= meas[k] <= cx.JOINT_LIMITS[k][1] for k in range(6))


def _xyz_safe(fb):
    return all(XYZ_LO <= fb[j] <= XYZ_HI for j in range(3))


def read_stable_state(arm, frames=10, retries=3):
    """Robust pre-motion stable read (local to this variant).

    The stock cx.read_stable_state requires the tip xyz to be in [0,400]; the
    RoArm-M3-S home tip can have a small NEGATIVE y (and the arc drives y to
    ~-200 mm), so that gate is miscalibrated here. This version only rejects
    non-finite / garbage xyz (|xyz|<500) plus joint-limit violations, and retries
    a full window a few times to ride out a single transient bad frame. Same
    10-frame, last-frame-must-be-good contract otherwise."""
    for _ in range(retries):
        records = []
        last = None
        for i in range(frames):
            fb = arm.feedback_get()
            ok = cx._valid(fb)
            plausible = False
            if ok:
                plausible = (
                    all(math.isfinite(fb[j]) for j in range(3))
                    and all(abs(fb[j]) < 500.0 for j in range(3))
                    and all(cx.JOINT_LIMITS[k][0] <= fb[4 + k] <= cx.JOINT_LIMITS[k][1] for k in range(6))
                )
            records.append({"i": i, "valid": ok, "plausible": plausible})
            if ok and plausible:
                last = fb
            time.sleep(0.1)
        if last is not None and records[-1]["valid"] and records[-1]["plausible"]:
            return last, records
    return None, None


def measure_stable(arm, frames=8):
    """Return a plausible stable measured joint vector (fb[4:10]) or None."""
    last = None
    for _ in range(frames):
        fb = arm.feedback_get()
        if cx._valid(fb) and _in_limits(list(fb[4:10])):
            last = list(fb[4:10])
        time.sleep(0.1)
    return last


def find_bounded_target(joints, start_j, envelope_rad):
    """Furthest contiguous on-path index whose max joint displacement from start_j
    stays <= envelope_rad. Anchors on the waypoint NEAREST start_j (not wp0), because the
    measured state can drift a few degrees off wp0 (homing tolerance > envelope). Returns
    (index, max_disp_rad, per_joint_deg); index is -1 if no waypoint is within the envelope."""
    # find the waypoint nearest to start_j (the measured state)
    anchor_i, _ = nearest_waypoint(start_j, joints)
    best_i = -1
    best_disp = 0.0
    for i in range(anchor_i, len(joints)):
        disp = max(abs(joints[i][k] - start_j[k]) for k in range(5))
        if disp <= envelope_rad:
            best_i = i
            best_disp = disp
        else:
            break
    per_joint = ([math.degrees(abs(joints[best_i][k] - start_j[k])) for k in range(5)]
                 if best_i >= 0 else None)
    return best_i, best_disp, per_joint


def resample_prefix(joints, i_target, total_time, rate):
    """Time-stretch the monotonic wp0..wp_i_target on-path prefix over total_time at rate Hz.

    Resamples the SAME joint-space prefix (wp0 -> wp1 -> ... -> wp_i_target) into a longer,
    slower sequence of interpolated commands so the target continuously progresses forward
    while remaining inside the bounded envelope (every interpolated point lies on a segment
    of the original on-path prefix, whose endpoints are already within the envelope). No
    reverse, no hold, no dwell: a single forward sweep, just stretched in time."""
    n = max(2, int(round(total_time * rate)))
    segs = max(1, i_target)          # path segments: wp0->wp1 ... wp_{i_target-1}->wp_i_target
    out = []
    for k in range(n):
        prog = k / (n - 1)           # 0..1 across the whole stretched prefix
        dist = prog * segs           # 0..segs in segment units
        si = int(dist)
        if si >= segs:
            si = segs - 1
        frac = dist - si
        a = joints[si]
        b = joints[si + 1]
        out.append([a[j] + frac * (b[j] - a[j]) for j in range(len(a))])
    return out


def nearest_waypoint(meas, joints):
    """(index, max_joint_err_rad) of the trajectory waypoint closest to meas."""
    best_i, best_err = 0, None
    for i, j in enumerate(joints):
        err = max(abs(j[k] - meas[k]) for k in range(5))
        if best_err is None or err < best_err:
            best_i, best_err = i, err
    return best_i, best_err


def bounded_startup(arm, joints, M, i_target, logf, attempt):
    """Single CONTINUOUS forward sweep wp0..i_target at 20 Hz (always-changing commands
    keep the motion controller active -- a T102 silence gap re-triggers the command-deaf
    latching). NO return toward the start, NO settling dwell, NO static hold. The bounded
    target only proves real actuation safely: commit as soon as the measured state makes
    directional progress toward it (displaced > 0.5 deg from M AND distance to the target
    decreased by > 0.1 deg, on PROGRESS_STABLE consecutive valid samples). Never
    sleeps/settles after commit so the caller transitions directly into the continuous
    advance (no T102 break). Returns a dict."""
    target = joints[i_target]
    fb_bad = 0
    prev_meas = None
    prev_dist = None
    progress_run = 0
    t0 = time.monotonic()
    committed = False
    commit_phase = None
    first_motion_t = None
    commit_state = None
    samples = []

    def sample(phase, idx, cmd):
        nonlocal fb_bad, prev_meas, prev_dist, progress_run, committed, commit_phase, first_motion_t, commit_state
        fb = arm.feedback_get()
        t = time.monotonic() - t0
        if cx._valid(fb):
            fb_bad = 0
            meas = list(fb[4:10])
            moved = _maxj(meas, M)
            dist_now = _maxj(meas, target)
            if not _in_limits(meas):
                return ("abort", "hard joint limit at attempt %d phase %s" % (attempt, phase))
            if prev_meas is not None and _maxj(meas, prev_meas) > GROSS_JUMP_RAD:
                return ("abort", "gross single-sample jump %.1f deg at attempt %d phase %s" % (
                    math.degrees(_maxj(meas, prev_meas)), attempt, phase))
            prev_meas = meas
            # commit = REAL ACTUATION proven by directional progress toward the bounded
            # target: the measured state has displaced > 0.5 deg from the attempt start
            # AND its distance to the target has decreased by > 0.1 deg, on PROGRESS_STABLE
            # consecutive valid samples. We do NOT require reaching or settling at the
            # target (no positioning accuracy demanded during startup). Because the sweep
            # feeds always-changing forward commands, a committed arm is actively tracking,
            # not latched, so the following continuous advance does not re-latch.
            progress = (prev_dist - dist_now) if prev_dist is not None else 0.0
            if not committed and moved > MOVE_TOL_RAD and progress > PROGRESS_TOL_RAD:
                progress_run += 1
                if progress_run >= PROGRESS_STABLE:
                    committed, commit_phase = True, phase
                    first_motion_t, commit_state = round(t, 3), [round(v, 4) for v in meas]
            else:
                progress_run = 0
            prev_dist = dist_now
            samples.append({"t": round(t, 3), "phase": phase, "idx": idx,
                            "cmd": [round(v, 4) for v in cmd] if cmd is not None else None,
                            "meas": [round(v, 4) for v in meas],
                            "moved_deg": round(math.degrees(moved), 3),
                            "dist_to_target_deg": round(math.degrees(dist_now), 3),
                            "progress_deg": round(math.degrees(progress), 3)})
            logf.write("t=%.3f att=%d phase=%s idx=%s cmd=[%s] meas=[%s] moved=%.3f dist_tgt=%.3f\n" % (
                t, attempt, phase, idx,
                " ".join("%.4f" % v for v in cmd) if cmd is not None else "-----",
                " ".join("%.4f" % v for v in meas), math.degrees(moved), math.degrees(dist_now)))
            return ("ok", None)
        fb_bad += 1
        samples.append({"t": round(t, 3), "phase": phase, "idx": idx, "invalid": True})
        logf.write("t=%.3f att=%d phase=%s idx=%s INVALID_FEEDBACK\n" % (t, attempt, phase, idx))
        if fb_bad >= INVALID_FB_ABORT:
            return ("abort", "%d consecutive invalid fb at attempt %d phase %s" % (fb_bad, attempt, phase))
        return ("ok", None)

    # PHASE sweep: a single forward sweep of the wp0..wp_i_target on-path prefix,
    # TIME-STRETCHED over PROBE_SECONDS at 20 Hz (no return toward the start, no settling
    # dwell, no static hold, no T102 gap). The raw prefix is only ~0.15 s for const-X, too
    # short to reliably detect a robot that takes ~0.8-1+ s to begin responding, so we
    # resample the SAME prefix into a slower sequence (see resample_prefix). Always-changing
    # forward commands keep the motion controller active. The bounded target only proves real
    # actuation safely: commit as soon as the measured state shows directional progress toward
    # it (see sample()). On commit the caller transitions directly into the continuous
    # trajectory (no T102 break). If no progress is shown by the end of the probe (arm deaf),
    # return not-committed and the caller retries from the newly measured state.
    probe_wps = resample_prefix(joints, i_target, PROBE_SECONDS, RATE_HZ)
    n_pump = len(probe_wps)
    t_p0 = time.monotonic()
    cmd_n = 0

    def pump_send(cmd, idx):
        nonlocal cmd_n
        arm.joints_radian_ctrl(cmd, SPEED, ACC)
        st, msg = sample("pump", idx, cmd)
        delay = (t_p0 + cmd_n / RATE_HZ) - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        cmd_n += 1
        return st, msg

    for i in range(n_pump):
        if committed:
            break
        st, msg = pump_send(probe_wps[i], i)
        if st == "abort":
            return {"aborted": True, "abort_reason": msg, "committed": False, "samples": samples}

    return {
        "aborted": False,
        "committed": committed,
        "commit_phase": commit_phase,
        "first_motion_t": first_motion_t,
        "commit_state": commit_state,
        "t102_continuous": committed,
        "samples": samples,
    }


def continuous_forward(arm, joints, i_target, M, commit_state, logf, z_record):
    """DIRECT handoff into the original trajectory in ONE 20 Hz T102 stream (no break).
    Precondition: the arm is committed (real actuation proven by directional progress in
    the time-stretched bounded sweep). The commit can fire MID-sweep (before the arm reaches
    joints[i_target]), so the compatible trajectory waypoint is the one NEAREST the measured
    commit state (not i_target itself). There is NO align hold / settling dwell: we transition
    directly from the current bounded on-path command into that compatible waypoint and
    continue the original trajectory with the normal watchdog. resume_err is the compatibility
    error measured on the first advance sample. Returns (forward_dict, resume_i, resume_err_deg)."""
    n_wp = len(joints)
    leg_t0 = time.monotonic()
    next_t = time.monotonic()
    phase = "advance"
    align_cmds = 0
    if commit_state is not None:
        resume_i, _ = nearest_waypoint(list(commit_state), joints)
    else:
        resume_i = i_target
    resume_err = None
    start_meas = None
    adv_t0 = time.monotonic()
    # forward-leg (advance) accumulators
    fb_ok = fb_bad = 0
    max_track = 0.0
    max_track_local_idx = None
    max_track_global_idx = None
    max_track_dominant = None
    gross_run = 0
    aborted = False
    reason = None
    intervals = []
    last_cmd_ts = None
    meas_hist = collections.deque(maxlen=FREEZE_WIN)
    cmd_hist = collections.deque(maxlen=FREEZE_WIN)
    freeze_run = 0
    freeze_windows = 0
    max_meas_motion_win = 0.0
    adv_i = 0
    adv_cmds = 0
    first20 = []

    while True:
        if adv_i >= (n_wp - resume_i):
            break
        cmd = joints[resume_i + adv_i]
        cmd_ts = time.monotonic()
        arm.joints_radian_ctrl(cmd, SPEED, ACC)
        if last_cmd_ts is not None:
            intervals.append(cmd_ts - last_cmd_ts)
        last_cmd_ts = cmd_ts
        fb = arm.feedback_get()
        if cx._valid(fb):
            fb_ok += 1
            fb_bad = 0
            meas = list(fb[4:10])
            z_record.append(fb[2])
            if adv_i == 0:
                # direct transition: measure the compatibility error at the resume waypoint
                # on the first advance sample (no align hold / settling dwell). The resume
                # waypoint is the bounded target joints[i_target], on-path and where the arm
                # has tracked to during the committed sweep.
                resume_err = _maxj(meas, joints[resume_i])
                start_meas = list(meas)
                logf.write("===== CONTINUOUS FORWARD: direct transition to wp=%d compat_err=%.3fdeg (NO align hold, NO T102 gap) =====\n" % (
                    resume_i, math.degrees(resume_err)))
            local_i = adv_i
            diffs = [abs(meas[k] - cmd[k]) for k in range(5)]
            track = max(diffs)
            if track > max_track:
                max_track = track
                max_track_local_idx = local_i
                max_track_global_idx = resume_i + local_i
                max_track_dominant = JOINT_NAMES[diffs.index(track)]
            if local_i < 20:
                first20.append({"idx": local_i, "global_wp": resume_i + local_i,
                                "track_deg": round(math.degrees(track), 3),
                                "dominant": JOINT_NAMES[diffs.index(track)]})
            # --- normal main-trajectory watchdog (unchanged thresholds) ---
            if track > GROSS_ERR_RAD:
                gross_run += 1
                if gross_run >= GROSS_ERR_SUSTAIN:
                    aborted = True
                    reason = "sustained gross tracking error %.1f deg at local idx %d" % (
                        math.degrees(track), local_i)
            else:
                gross_run = 0
            if local_i == int(RATE_HZ):
                moved = _maxj(start_meas, meas)
                if moved < NO_MOTION_TOL_RAD:
                    aborted = True
                    reason = "no motion start after 1s in advance (moved %.2f deg)" % math.degrees(moved)
            meas_hist.append(meas)
            cmd_hist.append(cmd)
            if local_i >= FREEZE_ARM and len(meas_hist) == FREEZE_WIN and len(cmd_hist) == FREEZE_WIN:
                old_m = meas_hist[0]
                old_c = cmd_hist[0]
                meas_motion = _maxj(meas, old_m)
                cmd_motion = _maxj(cmd, old_c)
                max_meas_motion_win = max(max_meas_motion_win, meas_motion)
                if meas_motion < FREEZE_MEAS_TOL_RAD and cmd_motion > FREEZE_CMD_TOL_RAD:
                    freeze_run += 1
                    freeze_windows += 1
                    if freeze_run >= FREEZE_SUSTAIN:
                        aborted = True
                        reason = ("sustained mid-trajectory freeze: measured motion "
                                  "< %.1f deg over %d consecutive 1s windows at local idx %d" % (
                                      math.degrees(FREEZE_MEAS_TOL_RAD), FREEZE_SUSTAIN, local_i))
                else:
                    freeze_run = 0
            logf.write("adv t=%.3f leg=F idx=%d cmd=[%s] meas=[%s] xyz=%.2f,%.2f,%.2f track=%.4f\n" % (
                time.monotonic() - adv_t0, resume_i + local_i,
                " ".join("%.4f" % v for v in cmd), " ".join("%.4f" % v for v in meas),
                fb[0], fb[1], fb[2], track))
        else:
            fb_bad += 1
            if fb_bad >= INVALID_FB_ABORT:
                aborted = True
                reason = "%d consecutive invalid feedback in continuous forward" % fb_bad
            logf.write("t=%.3f leg=F idx=%s INVALID_FEEDBACK\n" % (
                time.monotonic() - leg_t0, resume_i + adv_i))
        if aborted:
            break
        adv_i += 1
        adv_cmds += 1
        next_t += 1.0 / RATE_HZ
        delay = next_t - time.monotonic()
        if delay > 0:
            time.sleep(delay)

    # ---- finalize (mirror run_leg: settle read, final err, rate) ----
    time.sleep(1.0)
    final_fb = None
    for _ in range(10):
        fb = arm.feedback_get()
        if cx._valid(fb):
            final_fb = fb
            z_record.append(fb[2])
        time.sleep(0.1)
    final_meas = list(final_fb[4:10]) if final_fb else None
    final_err = None
    if final_meas is not None:
        final_err = [abs(final_meas[k] - joints[-1][k]) for k in range(5)]
    achieved_hz = (len(intervals) - 1) / sum(intervals) if len(intervals) > 1 else None
    forward = {
        "aborted": aborted,
        "abort_reason": reason,
        "align_commands": align_cmds,
        "advance_commands": adv_cmds,
        "commands_sent": adv_cmds,
        "fb_ok": fb_ok,
        "fb_invalid": fb_bad,
        "max_tracking_err_rad": max_track,
        "max_tracking_local_idx": max_track_local_idx,
        "max_tracking_global_wp": max_track_global_idx,
        "max_tracking_dominant_joint": max_track_dominant,
        "first20_tracking": first20,
        "achieved_cmd_rate_hz": round(achieved_hz, 3) if achieved_hz is not None else None,
        "freeze_windows": freeze_windows,
        "max_meas_motion_over_1s_window_rad": round(max_meas_motion_win, 5),
        "final_xyz_mm": list(final_fb[:3]) if final_fb else None,
        "final_joints_rad": final_meas,
        "final_joint_err_rad": final_err,
        "final_joint_err_max_rad": max(final_err) if final_err else None,
        "resume_index": resume_i,
        "resume_err_deg": round(math.degrees(resume_err), 3) if resume_err is not None else None,
    }
    return forward, resume_i, (round(math.degrees(resume_err), 3) if resume_err is not None else None)


def run_leg_tracked(arm, joints, leg_name, logf, z_record):
    """Behaviourally identical to cx.run_leg (same pacing, same unchanged watchdog
    thresholds) but also records the peak-tracking location, dominant joint and the
    first ~20 tracking samples. Used for the reverse leg."""
    start_fb = cx.wait_valid_fb(arm)
    if start_fb is None:
        return {"aborted": True, "abort_reason": "no valid feedback before leg start"}
    start_meas = start_fb[4:10]
    z_record.append(start_fb[2])
    t0 = time.monotonic()
    fb_ok = fb_bad = 0
    max_track = 0.0
    max_track_local_idx = None
    max_track_global_idx = None
    max_track_dominant = None
    gross_run = 0
    aborted, reason = False, None
    intervals = []
    last_cmd_ts = None
    meas_hist = collections.deque(maxlen=FREEZE_WIN)
    cmd_hist = collections.deque(maxlen=FREEZE_WIN)
    freeze_run = 0
    freeze_windows = 0
    max_meas_motion_win = 0.0
    first20 = []
    i = 0
    for i, cmd in enumerate(joints):
        now = time.monotonic()
        cmd_ts = time.monotonic()
        arm.joints_radian_ctrl(cmd, SPEED, ACC)
        if last_cmd_ts is not None:
            intervals.append(cmd_ts - last_cmd_ts)
        last_cmd_ts = cmd_ts
        fb = arm.feedback_get()
        if cx._valid(fb):
            fb_ok += 1
            fb_bad = 0
            meas = fb[4:10]
            z_record.append(fb[2])
            diffs = [abs(meas[k] - cmd[k]) for k in range(5)]
            track = max(diffs)
            if track > max_track:
                max_track = track
                max_track_local_idx = i
                max_track_global_idx = i
                max_track_dominant = JOINT_NAMES[diffs.index(track)]
            if i < 20:
                first20.append({"idx": i, "track_deg": round(math.degrees(track), 3),
                                "dominant": JOINT_NAMES[diffs.index(track)]})
            if track > GROSS_ERR_RAD:
                gross_run += 1
                if gross_run >= GROSS_ERR_SUSTAIN:
                    aborted = True
                    reason = "sustained gross tracking error %.1f deg at waypoint %d" % (
                        math.degrees(track), i)
            else:
                gross_run = 0
            if i == int(RATE_HZ):
                moved = _maxj(start_meas, meas)
                if moved < NO_MOTION_TOL_RAD:
                    aborted = True
                    reason = "no motion start after 1 s (moved %.2f deg)" % math.degrees(moved)
            meas_hist.append(meas)
            cmd_hist.append(cmd)
            if i >= FREEZE_ARM and len(meas_hist) == FREEZE_WIN and len(cmd_hist) == FREEZE_WIN:
                old_m = meas_hist[0]
                old_c = cmd_hist[0]
                meas_motion = _maxj(meas, old_m)
                cmd_motion = _maxj(cmd, old_c)
                max_meas_motion_win = max(max_meas_motion_win, meas_motion)
                if meas_motion < FREEZE_MEAS_TOL_RAD and cmd_motion > FREEZE_CMD_TOL_RAD:
                    freeze_run += 1
                    freeze_windows += 1
                    if freeze_run >= FREEZE_SUSTAIN:
                        aborted = True
                        reason = ("sustained mid-trajectory freeze: measured motion "
                                  "< %.1f deg over %d consecutive 1 s windows at waypoint %d" % (
                                      math.degrees(FREEZE_MEAS_TOL_RAD), FREEZE_SUSTAIN, i))
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
        if cx._valid(fb):
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
        "max_tracking_local_idx": max_track_local_idx,
        "max_tracking_global_wp": max_track_global_idx,
        "max_tracking_dominant_joint": max_track_dominant,
        "first20_tracking": first20,
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
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM21")
    ap.add_argument("--dry", action="store_true", help="home + plan + validate only, NO motion")
    ap.add_argument("--forward-only", action="store_true",
                    help="run the rule + forward leg only, skip the reverse leg")
    ap.add_argument("--tag", default="continuous", help="suffix for output files (per run)")
    ap.add_argument("--out", default=os.path.join(here, "constX-%s-result.json" % "continuous"))
    ap.add_argument("--log", default=os.path.join(here, "constX-%s.log" % "continuous"))
    args = ap.parse_args()
    if args.tag != "continuous":
        args.out = os.path.join(here, "constX-%s-result.json" % args.tag)
        args.log = os.path.join(here, "constX-%s.log" % args.tag)

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
    print("rule        envelope=%.0fdeg sweep_max=%.1fs move_tol=%.1fdeg progress_tol=%.1fdeg "
          "progress_stable=%d attempts<=%d" % (
        math.degrees(ENVELOPE_RAD), STARTUP_MAX_S, math.degrees(MOVE_TOL_RAD),
        math.degrees(PROGRESS_TOL_RAD), PROGRESS_STABLE, MAX_ATTEMPTS))
    print("handoff     direct transition to compatible wp (NO align hold, NO T102 gap)")
    print("mode        %s" % ("DRY" if args.dry else ("FORWARD-ONLY" if args.forward_only else "RULE + FULL ARC")))

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

    print("reading 10 consecutive feedback frames at home (stability gate, retrying)")
    stable_fb, stable_records = read_stable_state(arm, frames=10, retries=3)
    if stable_fb is None:
        raise SystemExit("communication unstable at home (stable-read gate failed after retries); aborting before any motion")
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

    # tight resume compatibility gate (replaces the old 15 deg gross-error threshold)
    max_step_rad = best["max_adjacent_step_rad"]
    RESUME_TOL_RAD = max(math.radians(3.0), 3.0 * max_step_rad)

    result_common = {
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "platform": sys.platform,
        "tag": args.tag,
        "port": args.port,
        "serial_settings": serial_settings,
        "roarm_sdk_version": roarm_sdk_ver,
        "roarm_sdk_file": roarm_sdk.__file__,
        "pyserial_version": pyserial_ver,
        "ik_module": ik_path,
        "params": {"speed": SPEED, "acc": ACC, "rate_hz": RATE_HZ,
                   "radius_mm": RADIUS_MM, "sweep_deg": SWEEP_DEG},
          "rule_params": {
             "envelope_rad": ENVELOPE_RAD, "sweep_max_s": STARTUP_MAX_S,
             "move_tol_rad": MOVE_TOL_RAD, "progress_tol_rad": PROGRESS_TOL_RAD,
             "progress_stable": PROGRESS_STABLE,
             "max_attempts": MAX_ATTEMPTS,
             "handoff": "direct transition to compatible wp (no align hold, no T102 gap)",
             "resume_tol_rad": RESUME_TOL_RAD,
            "resume_tol_formula": "max(3 deg, 3 x max_adjacent_step=%.3f deg) = %.3f deg" % (
                math.degrees(max_step_rad), math.degrees(RESUME_TOL_RAD)),
            "watchdog_unchanged": {"gross_err_rad": GROSS_ERR_RAD, "gross_err_sustain": GROSS_ERR_SUSTAIN,
                                   "no_motion_tol_rad": NO_MOTION_TOL_RAD, "freeze_win": FREEZE_WIN,
                                   "freeze_arm": FREEZE_ARM, "freeze_meas_tol_rad": FREEZE_MEAS_TOL_RAD,
                                   "freeze_cmd_tol_rad": FREEZE_CMD_TOL_RAD, "freeze_sustain": FREEZE_SUSTAIN},
        },
        "home_pose": {"xyz_mm": [x0, y0, z0], "pitch_rad": pitch0, "roll_rad": roll0,
                      "gripper": grip0, "joints_rad": start_joints},
        "selected_plan": {k: v for k, v in best.items() if k not in ("waypoints", "joints")},
    }

    if args.dry:
        result_common["verdict"] = "DRY_FEASIBLE"
        result_common["candidate_evaluations"] = plans
        with open(args.out, "w") as fh:
            json.dump(result_common, fh, indent=1)
        print("DRY RUN: constant-X arc feasible; rule NOT run (pass no --dry to run)")
        print("wrote %s" % args.out)
        print("RESULT: DRY_FEASIBLE")
        return 0

    print("--- running BOUNDED STARTUP -> COMMIT -> CONTINUOUS ALIGN+ADVANCE (rule) ---")
    logf = open(args.log, "w")
    logf.write("# %s  tag=%s  port=%s  sdk=%s  ik=%s  home_xyz=[%.2f %.2f %.2f]  constX=%.2f  "
                 "envelope=%.1fdeg sweep_max=%.1fs move_tol=%.1fdeg progress_tol=%.1fdeg progress_stable=%d resume_tol=%.2fdeg\n" % (
                     datetime.datetime.now(datetime.timezone.utc).isoformat(), args.tag,
                     args.port, sdk_dir, ik_path, x0, y0, z0, x0,
                     math.degrees(ENVELOPE_RAD), STARTUP_MAX_S,
                     math.degrees(MOVE_TOL_RAD), math.degrees(PROGRESS_TOL_RAD), PROGRESS_STABLE,
                     math.degrees(RESUME_TOL_RAD)))

    z_record = [z0]
    handshake = {"committed": False, "attempts": 0, "commit_phase": None,
                 "first_motion_t_s": None, "commit_state": None, "resume_index": None,
                 "resume_err_deg": None, "startup_target_index": None,
                 "startup_max_disp_deg": None, "startup_per_joint_deg": None,
                 "per_attempt": [], "aborted": False, "abort_reason": None,
                 "t102_continuous": None, "start_state_M": None}
    forward = None
    reverse = None
    arc_ran = False

    for attempt in range(1, MAX_ATTEMPTS + 1):
        handshake["attempts"] = attempt
        M = measure_stable(arm)
        if M is None:
            handshake["aborted"] = True
            handshake["abort_reason"] = "no stable measured state at start of attempt %d" % attempt
            break
        handshake["start_state_M"] = [round(v, 4) for v in M]
        i_target, max_disp, per_joint = find_bounded_target(joints, M, ENVELOPE_RAD)
        if i_target < 1:
            handshake["aborted"] = True
            handshake["abort_reason"] = ("no on-path trajectory point within %.0f deg envelope "
                                         "(i_target=%d); cannot prove a bounded startup" %
                                         (math.degrees(ENVELOPE_RAD), i_target))
            break
        handshake["startup_target_index"] = i_target
        handshake["startup_max_disp_deg"] = round(math.degrees(max_disp), 3)
        handshake["startup_per_joint_deg"] = [round(v, 3) for v in per_joint]
        logf.write("# attempt %d: M=[%s]  i_target=%d  max_disp=%.3f deg  per_joint=[%s] deg\n" % (
            attempt, " ".join("%.4f" % v for v in M), i_target, math.degrees(max_disp),
            " ".join("%.2f" % v for v in per_joint)))

        su = bounded_startup(arm, joints, M, i_target, logf, attempt)
        handshake["per_attempt"].append({
            "attempt": attempt,
            "start_state": [round(v, 4) for v in M],
            "startup_target_index": i_target,
            "startup_max_disp_deg": round(math.degrees(max_disp), 3),
            "startup_per_joint_deg": [round(v, 3) for v in per_joint],
            "committed": su["committed"],
            "commit_phase": su["commit_phase"],
            "first_motion_t_s": su["first_motion_t"],
            "commit_state": su["commit_state"],
            "t102_continuous": su["t102_continuous"],
            "samples": su["samples"],
        })
        if su["aborted"]:
            # hard abort (limit / gross jump / invalid fb) -- do NOT retry
            handshake["aborted"] = True
            handshake["abort_reason"] = su["abort_reason"]
            break
        if not su["committed"]:
            if attempt < MAX_ATTEMPTS:
                logf.write("# attempt %d: NO commit after pump (arm command-deaf); retrying from newly measured state\n" % attempt)
                continue
            handshake["aborted"] = True
            handshake["abort_reason"] = "no commit after %d pump attempts (arm command-deaf)" % attempt
            break

        # ---- committed: CONTINUOUS align -> advance (no T102 break) ----
        logf.write("# attempt %d COMMITTED in phase=%s at t=%.3f s meas=[%s]  t102_continuous=%s\n" % (
            attempt, su["commit_phase"], su["first_motion_t"],
            " ".join("%.4f" % v for v in su["commit_state"]), su["t102_continuous"]))

        forward, resume_i, resume_err_deg = continuous_forward(arm, joints, i_target, M, su["commit_state"], logf, z_record)
        handshake["resume_index"] = resume_i
        handshake["resume_err_deg"] = resume_err_deg
        logf.write("# resume_index=%s resume_err=%s deg (gate=%.2f deg)\n" % (
            resume_i, ("%.3f" % resume_err_deg) if resume_err_deg is not None else "n/a",
            math.degrees(RESUME_TOL_RAD)))

        resume_ok = (resume_err_deg is not None and resume_err_deg <= math.degrees(RESUME_TOL_RAD))
        if forward["aborted"] or not resume_ok:
            why = (forward["abort_reason"] if forward["aborted"]
                   else ("resume err %.2f deg > gate %.2f deg" % (resume_err_deg, math.degrees(RESUME_TOL_RAD))))
            if attempt < MAX_ATTEMPTS:
                logf.write("# attempt %d: advance failed (%s); retrying from newly measured state\n" % (attempt, why))
                forward = None
                continue
            handshake["aborted"] = True
            handshake["abort_reason"] = "advance failed after %d attempts: %s" % (attempt, why)
            break

        # ---- success: committed -> continuous advance completed ----
        handshake["committed"] = True
        handshake["commit_phase"] = su["commit_phase"]
        handshake["first_motion_t_s"] = su["first_motion_t"]
        handshake["commit_state"] = su["commit_state"]
        handshake["t102_continuous"] = su["t102_continuous"]
        break

    logf.flush()
    result_common["handshake"] = handshake

    if handshake["aborted"]:
        print("HANDSHAKE ABORT: %s" % handshake["abort_reason"])
    elif not handshake["committed"]:
        print("HANDSHAKE: no physical response after %d bounded attempts; NOT running the main arc" %
              handshake["attempts"])
    else:
        print("HANDSHAKE COMMITTED: phase=%s first_motion_t=%.3f s attempts=%d t102_continuous=%s" % (
            handshake["commit_phase"], handshake["first_motion_t_s"], handshake["attempts"],
            handshake["t102_continuous"]))
        print("  startup target wp=%d (max disp %.2f deg)  resume_index=%d (err %.2f deg, gate %.2f deg)" % (
            handshake["startup_target_index"], handshake["startup_max_disp_deg"],
            handshake["resume_index"], handshake["resume_err_deg"], math.degrees(RESUME_TOL_RAD)))

        if forward is not None:
            print("forward  aborted=%s reason=%s align_cmd=%d adv_cmd=%d fb_ok=%d fb_bad=%d "
                  "max_track=%.2f deg @localidx %s (wp %s, %s) freeze=%d rate=%s Hz" % (
                      forward["aborted"], forward["abort_reason"], forward["align_commands"],
                      forward["advance_commands"], forward["fb_ok"], forward["fb_invalid"],
                      math.degrees(forward["max_tracking_err_rad"]), forward["max_tracking_local_idx"],
                      forward["max_tracking_global_wp"], forward["max_tracking_dominant_joint"],
                      forward["freeze_windows"], forward["achieved_cmd_rate_hz"]))
            if not forward["aborted"] and not args.forward_only:
                arc_ran = True
                logf.write("\n===== REVERSE LEG (known-good run_leg_tracked, %d .. 0) =====\n" % (n_wp - 1))
                reverse = run_leg_tracked(arm, list(reversed(joints)), "R", logf, z_record)
                print("reverse  aborted=%s reason=%s cmd=%s fb_ok=%s fb_bad=%s max_track=%.2f deg "
                      "@idx %s (%s) freeze=%s rate=%s Hz" % (
                          reverse["aborted"], reverse.get("abort_reason"),
                          reverse.get("commands_sent"),
                          reverse.get("fb_ok"), reverse.get("fb_invalid"),
                          math.degrees(reverse["max_tracking_err_rad"]),
                          reverse.get("max_tracking_global_wp"), reverse.get("max_tracking_dominant_joint"),
                          reverse.get("freeze_windows", "n/a"), reverse.get("achieved_cmd_rate_hz")))
        result_common["forward"] = forward
        result_common["reverse"] = reverse
        result_common["arc_ran"] = arc_ran
        result_common["resume_index"] = handshake["resume_index"]
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
    elif args.forward_only:
        result_common["verdict"] = "FORWARD_RULE_COMPLETE" if (forward and not forward["aborted"]) else "FORWARD_ABORTED"
    elif (forward and forward["aborted"]) or (reverse is not None and reverse["aborted"]):
        result_common["verdict"] = "ARC_ABORTED"
    else:
        result_common["verdict"] = "COMPLETE"

    with open(args.out, "w") as fh:
        json.dump(result_common, fh, indent=1)

    print("\n===== REPORT =====")
    print("measured start state M          : %s" % (
        "[%s]" % " ".join("%.4f" % v for v in (handshake["start_state_M"] or []))) or "n/a")
    print("startup waypoint (bounded)      : wp=%d (max disp %.2f deg, per-joint [%s] deg)" % (
        handshake["startup_target_index"], handshake["startup_max_disp_deg"] or -1,
        " ".join("%.2f" % v for v in (handshake["startup_per_joint_deg"] or []))))
    print("commit phase / first motion     : %s at %s s   attempts=%d" % (
        handshake["commit_phase"], handshake["first_motion_t_s"], handshake["attempts"]))
    print("commit state                    : %s" % (
        "[%s]" % " ".join("%.4f" % v for v in handshake["commit_state"])
        if handshake["commit_state"] else None))
    print("T102 continuity across handoff  : %s" % handshake["t102_continuous"])
    _ri, _re = handshake["resume_index"], handshake["resume_err_deg"]
    print("resume waypoint / mismatch      : %s" % (
        ("wp=%d err=%.2f deg (gate %.2f deg)" % (_ri, _re or -1, math.degrees(RESUME_TOL_RAD)))
        if _ri is not None else "n/a (no commit)"))
    for leg_name, leg in (("forward", forward), ("reverse", reverse)):
        if leg is not None:
            ncmd = leg.get("commands_sent", (leg.get("align_commands", 0) + leg.get("advance_commands", 0)))
            print("  %s: cmd=%s fb_ok=%s fb_bad=%s freeze=%s max_track=%.2f deg @idx %s (%s) final_err=%s" % (
                leg_name, ncmd, leg.get("fb_ok"), leg.get("fb_invalid"),
                leg.get("freeze_windows", "n/a"), math.degrees(leg["max_tracking_err_rad"]),
                leg.get("max_tracking_global_wp") or leg.get("max_tracking_global_idx"),
                leg.get("max_tracking_dominant_joint"),
                ("%.2f deg" % math.degrees(leg["final_joint_err_max_rad"]))
                if leg.get("final_joint_err_max_rad") is not None else "n/a"))
    print("VERDICT: %s" % result_common["verdict"])
    if handshake["aborted"]:
        print("ABORT: %s" % handshake["abort_reason"])
    print("wrote %s" % args.out)
    print("wrote %s" % args.log)
    print("DONE")
    return 0 if result_common["verdict"] in ("COMPLETE", "FORWARD_RULE_COMPLETE", "DRY_FEASIBLE") else 1


if __name__ == "__main__":
    sys.exit(main())
