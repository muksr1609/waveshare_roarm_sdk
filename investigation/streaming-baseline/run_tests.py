#!/usr/bin/env python3
"""
T=102 joint-command transport experiment for Waveshare RoArm-M3-S.

Purpose:
    Determine whether sparse one-shot T=102 (joints_radian_ctrl) commands are
    reliable on the known-good WSL2 + stock roarm-sdk==0.1.0 environment, and
    isolate which variable (sparsity, speed/acc, OS/serial env, redundancy)
    explains the gap between the failed Windows Z-sweep and the successful
    stock ROS2 baseline.

Rules honoured:
    - STOCK roarm-sdk==0.1.0 only (path asserted at startup).
    - No firmware/SDK changes, no RobotAgent, no MoveIt, no ROS trajectory.
    - Raw Python, one serial object, sequential access only.
    - A command is ACCEPTED only if physical feedback shows motion toward the
      target; the SDK return value is never treated as evidence of execution.
    - Two nearby safe poses A/B, same for every test, max 7 deg per joint.

Outputs (in this file's directory):
    raw.log      - permanent log: every command, timestamp, feedback sample
    results.json - machine-readable results (written incrementally)

Run:
    cd /home/mukund/streaming_baseline && python3 run_tests.py
"""

import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime

WORKDIR = os.path.dirname(os.path.abspath(__file__))
RAW_LOG = os.path.join(WORKDIR, "raw.log")
RESULTS = os.path.join(WORKDIR, "results.json")
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
ROARM_TYPE = "roarm_m3"

JOINT_NAMES = ["base", "shoulder", "elbow", "wrist", "roll", "hand"]
# From stock roarm_sdk/utils.py robot_limit["roarm_m3"]
LIMIT_MIN = [-3.3, -1.9, -1.2, -1.9, -3.3, -0.2]
LIMIT_MAX = [3.3, 1.9, 3.3, 1.9, 3.3, 1.9]

MOTION_THRESH = 0.01      # rad: threshold for "measured movement"
REACHED_TOL = 0.05        # rad: within target
SETTLE_TOL = 0.03         # rad: settled at target
SETTLE_STABLE_FOR = 1.0   # s
NEAR_LIMIT_MARGIN = 0.25  # rad: "close to a joint limit"
MAX_DELIBERATE_DELTA_DEG = 7.0

BASE_PERTURB_DEG = 5.0
WRIST_PERTURB_DEG = 3.0

TRANSITIONS_PER_TEST = 10
MONITOR_SECONDS = 15.0
SAMPLE_PERIOD = 0.10
DWELL_SECONDS = 3.0

D_HZ = 20.0
D_LEG_DURATION = 1.5
D_CYCLES = 5


# ---------------------------------------------------------------- logging

class Log:
    def __init__(self):
        self.fh = open(RAW_LOG, "a", buffering=1)

    def line(self, msg, echo=True):
        ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        full = f"{ts} {msg}"
        self.fh.write(full + "\n")
        if echo:
            print(full, flush=True)

    def close(self):
        self.fh.close()


log = Log()
STATE = {"stop": False}


def on_sigint(signum, frame):
    log.line("!!! SIGINT received - stopping experiment, no more commands will be sent")
    STATE["stop"] = True


def r4(x):
    return [round(v, 4) for v in x]


def degs(joints):
    return [round(math.degrees(v), 2) for v in joints]


def check_stop():
    if STATE["stop"]:
        raise KeyboardInterrupt("stopped by operator")


# ---------------------------------------------------------------- environment

def collect_env():
    def sh(cmd):
        try:
            return subprocess.run(cmd, shell=True, capture_output=True,
                                  text=True, timeout=15).stdout.strip()
        except Exception as e:
            return f"<error: {e}>"

    import roarm_sdk
    import serial
    try:
        from importlib.metadata import version as pkg_version
        sdk_ver = pkg_version("roarm_sdk")
    except Exception:
        sdk_ver = "unknown"
    env = {
        "timestamp_utc": datetime.utcnow().isoformat() + "Z",
        "python": sys.version,
        "roarm_sdk_version": sdk_ver,
        "roarm_sdk_path": roarm_sdk.__file__,
        "pyserial_version": serial.__version__,
        "kernel": os.uname().release,
        "os": f"{os.uname().sysname} {os.uname().version}",
        "lsusb": sh("lsusb"),
        "ttyUSB0": sh(f"ls -l {PORT}"),
        "cwd": os.getcwd(),
    }
    return env


def assert_stock_sdk():
    import roarm_sdk
    p = roarm_sdk.__file__
    if "/mnt/c/" in p:
        raise SystemExit(f"FATAL: imported REPO-LOCAL sdk from {p}; refusing to run.")
    if "site-packages/roarm_sdk" not in p:
        raise SystemExit(f"FATAL: unexpected sdk path {p}; refusing to run.")
    log.line(f"SDK guard OK: stock roarm_sdk at {p}")


def find_port_holders(port, exclude_pid=None):
    holders = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit() or int(pid) == os.getpid():
            continue
        if exclude_pid and int(pid) == exclude_pid:
            continue
        try:
            for fd in os.listdir(f"/proc/{pid}/fd"):
                try:
                    if os.readlink(f"/proc/{pid}/fd/{fd}") == port:
                        comm = open(f"/proc/{pid}/comm").read().strip()
                        holders.append((pid, comm))
                        break
                except OSError:
                    pass
        except OSError:
            continue
    return holders


# ---------------------------------------------------------------- feedback

def read_feedback(arm):
    """One feedback_get() call. Returns (ok, xyz, joints, raw, elapsed_s)."""
    t0 = time.time()
    try:
        val = arm.feedback_get()
    except Exception as e:
        return False, None, None, f"EXC:{e}", time.time() - t0
    el = time.time() - t0
    ok = (isinstance(val, list) and len(val) == 10
          and all(isinstance(v, (int, float)) and math.isfinite(v) for v in val))
    if ok:
        xyz = [val[0], val[1], val[2]]
        joints = val[4:10]
        return True, xyz, joints, val, el
    return False, None, None, val, el


def read_stable(arm, n=3, period=0.05):
    """Median of n feedback reads -> (joints, xyz) or (None, None)."""
    samples = []
    for _ in range(n):
        ok, xyz, joints, raw, el = read_feedback(arm)
        if ok:
            samples.append(joints)
        time.sleep(period)
    if not samples:
        return None, None
    med = [statistics.median([s[i] for s in samples]) for i in range(6)]
    return med, None


def motion_toward(joints, pre, target, thresh=MOTION_THRESH):
    """Returns (moved, toward, max_dev, joint_index)."""
    max_dev = max(abs(a - b) for a, b in zip(joints, pre))
    if max_dev <= thresh:
        return False, False, max_dev, -1
    for i, (j, p, t) in enumerate(zip(joints, pre, target)):
        if abs(t - p) < 1e-6:
            continue
        if (j - p) * (1 if t > p else -1) > thresh:
            return True, True, max_dev, i
    return True, False, max_dev, -1


def err_to(joints, target):
    return max(abs(a - b) for a, b in zip(joints, target))


def wait_settled(arm, target, timeout=25.0, tol=SETTLE_TOL, stable_for=SETTLE_STABLE_FOR):
    t0 = time.time()
    stable_since = None
    last = None
    while time.time() - t0 < timeout:
        check_stop()
        ok, xyz, joints, raw, el = read_feedback(arm)
        if ok:
            last = joints
            if err_to(joints, target) < tol:
                if stable_since is None:
                    stable_since = time.time()
                if time.time() - stable_since >= stable_for:
                    return True, joints
            else:
                stable_since = None
        else:
            stable_since = None
        time.sleep(SAMPLE_PERIOD)
    return False, last


def sample_monitor(arm, target, pre, duration, rec, tag):
    """Sample feedback for `duration` seconds, filling rec. Returns first_motion ts."""
    t_end = time.time() + duration
    first_motion = None
    min_err = None
    n_ok = n_fail = 0
    while time.time() < t_end:
        check_stop()
        ts = time.time()
        ok, xyz, joints, raw, el = read_feedback(arm)
        if ok:
            n_ok += 1
            moved, toward, dev, idx = motion_toward(joints, pre, target)
            if moved and toward and first_motion is None:
                first_motion = ts
                rec["first_motion_latency_s"] = round(first_motion - rec["send_ts"], 3)
                rec["motion_joint"] = JOINT_NAMES[idx]
                log.line(f"{tag} MOTION toward target (joint={JOINT_NAMES[idx]}) "
                         f"latency={rec['first_motion_latency_s']}s dev={dev:.4f}")
            e = err_to(joints, target)
            min_err = e if min_err is None else min(min_err, e)
            rec["samples"].append([round(ts - rec["send_ts"], 3), r4(joints), round(e, 4)])
            log.line(f"{tag} fb ok t+{ts - rec['send_ts']:.3f} j={r4(joints)} "
                     f"err_to_target={e:.4f} (read {el * 1000:.0f}ms)", echo=False)
        else:
            n_fail += 1
            rec["samples"].append([round(ts - rec["send_ts"], 3), None, None])
            log.line(f"{tag} fb FAIL t+{ts - rec['send_ts']:.3f} raw={raw}", echo=False)
        time.sleep(max(0.0, SAMPLE_PERIOD - (time.time() - ts)))
    rec["n_feedback_ok"] = n_ok
    rec["n_feedback_fail"] = n_fail
    if min_err is not None:
        rec["min_err_to_target"] = round(min_err, 4)
        rec["reached"] = min_err < REACHED_TOL
    return first_motion


# ---------------------------------------------------------------- poses

def pick_poses(baseline_joints):
    """A = current physical state (nudged away from limits if close),
    B = A with small perturbation away from nearest limits.

    Only the joints that will actually MOVE (base, wrist) are checked for
    limit proximity and possibly nudged. The gripper (hand) is never moved.
    """
    A = list(baseline_joints)
    nudged = []
    for i in (0, 3):  # base, wrist only
        m_min = A[i] - LIMIT_MIN[i]
        m_max = LIMIT_MAX[i] - A[i]
        if min(m_min, m_max) < NEAR_LIMIT_MARGIN:
            center = (LIMIT_MIN[i] + LIMIT_MAX[i]) / 2
            shift = NEAR_LIMIT_MARGIN + 0.05 - min(m_min, m_max)
            A[i] = A[i] + math.copysign(shift, center - A[i])
            nudged.append(JOINT_NAMES[i])
    B = list(A)
    # base: 5 deg away from nearest limit
    d = math.radians(BASE_PERTURB_DEG)
    sign = -1 if A[0] >= 0 else 1
    B[0] = A[0] + sign * d
    # wrist (joint index 3): 3 deg away from nearest limit
    d = math.radians(WRIST_PERTURB_DEG)
    sign = -1 if A[3] >= 0 else 1
    B[3] = A[3] + sign * d
    # safety checks
    for i in range(6):
        assert LIMIT_MIN[i] <= A[i] <= LIMIT_MAX[i], f"A[{i}]={A[i]} out of limits"
        assert LIMIT_MIN[i] <= B[i] <= LIMIT_MAX[i], f"B[{i}]={B[i]} out of limits"
        assert abs(B[i] - A[i]) <= math.radians(MAX_DELIBERATE_DELTA_DEG), \
            f"delta on joint {i} exceeds {MAX_DELIBERATE_DELTA_DEG} deg"
    return A, B, nudged


# ---------------------------------------------------------------- test bodies

def ensure_at(arm, pose, tag):
    """Setup move (NOT measured): repeated-target delivery until settled."""
    ok, xyz, joints, raw, el = read_feedback(arm)
    if ok and err_to(joints, pose) < SETTLE_TOL:
        log.line(f"{tag} already at target, no setup move needed")
        return
    pre = joints if ok else pose
    log.line(f"{tag} SETUP move to {r4(pose)} (repeated target, not measured)")
    t0 = time.time()
    sends = 0
    while time.time() - t0 < 10.0:
        arm.joints_radian_ctrl(pose, 180, 10)
        sends += 1
        ok, xyz, joints, raw, el = read_feedback(arm)
        if ok:
            moved, toward, dev, idx = motion_toward(joints, pre, pose)
            if moved and toward:
                break
        time.sleep(0.1)
    settled, final = wait_settled(arm, pose)
    log.line(f"{tag} SETUP done sends={sends} settled={settled} final={r4(final) if final else None}")
    if not settled:
        log.line(f"{tag} WARNING: setup move did not settle; continuing (pose drift possible)")


def test_sparse(arm, A, B, speed, acc, test_name):
    """Tests A/B: one T=102 per transition, no retries, 15 s monitor.

    Requested target alternates strictly by parity (B, A, B, A, ...).
    `cur` tracks where we believe the arm ACTUALLY is (drops leave it put).
    A command sent while already at its target is classified 'no-op', not 'dropped'.
    """
    log.line(f"===== TEST {test_name}: one-shot sparse, speed={speed}, acc={acc} =====")
    rec_test = {"test": test_name, "speed": speed, "acc": acc,
                "transitions": [], "accepted": 0, "sent": 0, "dropped": 0,
                "no_op_already_at_target": 0}
    cur, cur_name = A, "A"
    for i in range(1, TRANSITIONS_PER_TEST + 1):
        check_stop()
        target, tname = (B, "B") if i % 2 == 1 else (A, "A")
        ensure_at(arm, cur, f"[{test_name}/{i}]pre")
        log.line(f"[{test_name}/{i}] dwell {DWELL_SECONDS}s before send")
        time.sleep(DWELL_SECONDS)
        pre, _ = read_stable(arm)
        if pre is None:
            log.line(f"[{test_name}/{i}] FATAL: no feedback before send; aborting test")
            break
        rec = {"i": i, "requested_from": cur_name, "to": tname,
               "target_rad": r4(target), "pre_rad": r4(pre), "samples": []}
        already = err_to(pre, target) < 0.02
        if already:
            rec["already_at_target"] = True
            log.line(f"[{test_name}/{i}] WARNING: pre already at target (prev drop) -> no-op send")
        t_send = time.time()
        ret = arm.joints_radian_ctrl(target, speed, acc)
        rec["send_ts"] = t_send
        rec["return_value"] = str(ret)
        rec_test["sent"] += 1
        log.line(f"[{test_name}/{i}] SENT T=102 {cur_name}->{tname} "
                 f"target={r4(target)} ({degs(target)}) speed={speed} acc={acc} ret={ret}")
        fm = sample_monitor(arm, target, pre, MONITOR_SECONDS, rec, f"[{test_name}/{i}]")
        rec["accepted"] = bool(fm is not None and not already)
        if already:
            rec_test["no_op_already_at_target"] += 1
        elif fm is None:
            rec_test["dropped"] += 1
            log.line(f"[{test_name}/{i}] DROPPED: no motion toward target in {MONITOR_SECONDS:.0f}s")
        else:
            rec_test["accepted"] += 1
        settled, final = wait_settled(arm, target, timeout=20.0)
        rec["settled"] = settled
        if final is not None:
            rec["final_err_to_target"] = round(err_to(final, target), 4)
        rec_test["transitions"].append(rec)
        save_results()
        if rec["accepted"]:
            cur, cur_name = target, tname
        # if dropped/no-op: arm stays where it was; cur unchanged
    n_testable = rec_test["sent"] - rec_test["no_op_already_at_target"]
    rec_test["acceptance_rate"] = (
        round(rec_test["accepted"] / n_testable, 3) if n_testable else None)
    rec_test["acceptance_rate_all_sent"] = (
        round(rec_test["accepted"] / rec_test["sent"], 3) if rec_test["sent"] else None)
    log.line(f"===== TEST {test_name} done: {rec_test['accepted']}/{rec_test['sent']} accepted "
             f"(dropped={rec_test['dropped']}, no_op={rec_test['no_op_already_at_target']}) =====")
    return rec_test


def test_repeated(arm, A, B, test_name, speed=180, acc=10):
    """Test C: same final target resent @10 Hz for <=2 s or until motion starts.

    Requested target alternates strictly by parity; `cur` tracks actual pose.
    """
    log.line(f"===== TEST {test_name}: repeated final target @10Hz, speed={speed}, acc={acc} =====")
    rec_test = {"test": test_name, "speed": speed, "acc": acc, "transitions": [],
                "success": 0, "sent_total": 0, "no_op_already_at_target": 0, "dropped": 0}
    cur, cur_name = A, "A"
    for i in range(1, TRANSITIONS_PER_TEST + 1):
        check_stop()
        target, tname = (B, "B") if i % 2 == 1 else (A, "A")
        pre, _ = read_stable(arm)
        if pre is None:
            log.line(f"[{test_name}/{i}] FATAL: no feedback; aborting test")
            break
        rec = {"i": i, "requested_from": cur_name, "to": tname,
               "target_rad": r4(target), "pre_rad": r4(pre), "samples": []}
        if err_to(pre, target) < 0.02:
            rec["already_at_target"] = True
            rec_test["no_op_already_at_target"] += 1
            log.line(f"[{test_name}/{i}] pre already at target (prev drop); no sends needed")
            rec_test["transitions"].append(rec)
            save_results()
            continue
        t0 = time.time()
        sends = 0
        motion_at = None
        while time.time() - t0 < 2.0 and sends < 20:
            check_stop()
            t_send = time.time()
            arm.joints_radian_ctrl(target, speed, acc)
            sends += 1
            rec_test["sent_total"] += 1
            log.line(f"[{test_name}/{i}] resend #{sends} at t+{time.time() - t0:.3f}s", echo=False)
            ok, xyz, joints, raw, el = read_feedback(arm)
            if ok:
                rec["samples"].append([round(time.time() - t0, 3), r4(joints)])
                moved, toward, dev, idx = motion_toward(joints, pre, target)
                if moved and toward:
                    motion_at = time.time()
                    break
            else:
                rec["samples"].append([round(time.time() - t0, 3), None])
            time.sleep(max(0.0, 0.1 - (time.time() - t_send)))
        rec["packets_sent"] = sends
        if motion_at is None:
            rec["success"] = False
            rec["packets_before_motion"] = None
            rec_test["dropped"] += 1
            log.line(f"[{test_name}/{i}] DROPPED: no motion after {sends} packets in 2s")
        else:
            rec["success"] = True
            rec["packets_before_motion"] = sends
            rec["motion_latency_s"] = round(motion_at - t0, 3)
            rec_test["success"] += 1
            log.line(f"[{test_name}/{i}] motion after {sends} packets "
                     f"latency={rec['motion_latency_s']}s")
            settled, final = wait_settled(arm, target, timeout=20.0)
            rec["settled"] = settled
            if final is not None:
                rec["final_err_to_target"] = round(err_to(final, target), 4)
        rec_test["transitions"].append(rec)
        save_results()
        if rec["success"]:
            cur, cur_name = target, tname
    n_tested = sum(1 for t in rec_test["transitions"] if not t.get("already_at_target"))
    rec_test["acceptance_rate"] = (
        round(rec_test["success"] / n_tested, 3) if n_tested else None)
    log.line(f"===== TEST {test_name} done: {rec_test['success']}/{n_tested} successful "
             f"(dropped={rec_test['dropped']}, no_op={rec_test['no_op_already_at_target']}, "
             f"packets_total={rec_test['sent_total']}) =====")
    return rec_test


def test_streamed(arm, A, B, test_name, hz=D_HZ, duration=D_LEG_DURATION,
                  speed=1000, acc=50, cycles=D_CYCLES):
    """Test D: joint-space linear interpolation streamed @hz, ROS-style settings."""
    log.line(f"===== TEST {test_name}: streamed interpolation @{hz}Hz, speed={speed}, acc={acc} =====")
    n_steps = int(hz * duration)
    moving = [i for i in range(6) if abs(B[i] - A[i]) > 1e-9]
    rec_test = {"test": test_name, "hz": hz, "leg_duration_s": duration,
                "speed": speed, "acc": acc, "cycles": cycles, "legs": [],
                "legs_started": 0, "legs_total": 0}
    for c in range(1, cycles + 1):
        for leg_name, start, end in (("A->B", A, B), ("B->A", B, A)):
            check_stop()
            pre, _ = read_stable(arm)
            leg = {"cycle": c, "leg": leg_name, "pre_rad": r4(pre) if pre else None,
                   "n_sends": 0, "feedback_ok": 0, "feedback_fail": 0,
                   "steps": [], "regressions": 0}
            t0 = time.time()
            prev_prog = None
            started = False
            for k in range(n_steps):
                check_stop()
                alpha = (k + 1) / n_steps
                q = [s + alpha * (e - s) for s, e in zip(start, end)]
                t_send = time.time()
                arm.joints_radian_ctrl(q, speed, acc)
                leg["n_sends"] += 1
                ok, xyz, joints, raw, el = read_feedback(arm)
                ts = time.time()
                if ok:
                    leg["feedback_ok"] += 1
                    e = err_to(joints, q)
                    prog = sum(1.0 - min(1.0, abs(j - s) / abs(e2 - s))
                               for j, s, e2 in zip(joints, start, end)
                               if abs(e2 - s) > 1e-9) / len(moving)
                    if prev_prog is not None and prog < prev_prog - 0.05:
                        leg["regressions"] += 1
                    prev_prog = prog
                    if not started and pre is not None:
                        m, tw, dev, idx = motion_toward(joints, pre, end)
                        if m and tw:
                            started = True
                            leg["start_latency_s"] = round(ts - t0, 3)
                    leg["steps"].append([k, round(alpha, 3), round(e, 4), round(prog, 4)])
                else:
                    leg["feedback_fail"] += 1
                    leg["steps"].append([k, round(alpha, 3), None, None])
                time.sleep(max(0.0, 1.0 / hz - (time.time() - t_send)))
            leg["started"] = started
            rec_test["legs_total"] += 1
            if started:
                rec_test["legs_started"] += 1
            # post-leg settle
            settled, final = wait_settled(arm, end, timeout=10.0)
            leg["settled"] = settled
            if final is not None:
                leg["final_err"] = round(err_to(final, end), 4)
            rec_test["legs"].append(leg)
            log.line(f"[{test_name}] cycle {c} {leg_name}: sends={leg['n_sends']} "
                     f"fb_ok={leg['feedback_ok']} fb_fail={leg['feedback_fail']} "
                     f"started={started} regressions={leg['regressions']} "
                     f"settled={settled} final_err={leg.get('final_err')}")
            save_results()
    n_fb = sum(l["feedback_ok"] + l["feedback_fail"] for l in rec_test["legs"])
    rec_test["feedback_success_rate"] = (
        round(sum(l["feedback_ok"] for l in rec_test["legs"]) / n_fb, 3) if n_fb else None)
    rec_test["max_final_err"] = max((l.get("final_err", 0.0) for l in rec_test["legs"]), default=None)
    rec_test["total_regressions"] = sum(l["regressions"] for l in rec_test["legs"])
    log.line(f"===== TEST {test_name} done: {rec_test['legs_started']}/{rec_test['legs_total']} "
             f"legs started, fb_rate={rec_test['feedback_success_rate']} =====")
    return rec_test


# ---------------------------------------------------------------- state io

RESULTS_STORE = {"meta": None, "baseline": None, "tests": {}}


def save_results():
    RESULTS_STORE["finished_utc"] = datetime.utcnow().isoformat() + "Z"
    try:
        with open(RESULTS, "w") as f:
            json.dump(RESULTS_STORE, f, indent=1)
    except Exception as e:
        print(f"warning: could not save results.json: {e}", flush=True)


# ---------------------------------------------------------------- baseline

def run_baseline(arm):
    log.line("===== BASELINE: 3 s idle feedback sampling =====")
    samples = []
    t0 = time.time()
    n_ok = n_fail = 0
    while time.time() - t0 < 3.0:
        ok, xyz, joints, raw, el = read_feedback(arm)
        if ok:
            n_ok += 1
            samples.append({"t": round(time.time() - t0, 3), "xyz": [round(v, 4) for v in xyz],
                            "joints": r4(joints)})
            log.line(f"[baseline] fb ok t+{time.time() - t0:.3f} xyz={samples[-1]['xyz']} j={r4(joints)}")
        else:
            n_fail += 1
            log.line(f"[baseline] fb FAIL t+{time.time() - t0:.3f} raw={raw}")
        time.sleep(max(0.0, SAMPLE_PERIOD - (time.time() - (t0 + len(samples) * SAMPLE_PERIOD))))
    total = n_ok + n_fail
    rec = {"n_ok": n_ok, "n_fail": n_fail,
           "success_rate": round(n_ok / total, 3) if total else None,
           "samples": samples}
    if n_ok == 0:
        raise SystemExit("FATAL: no feedback at all; aborting.")
    allj = [s["joints"] for s in samples]
    for i in range(6):
        vals = [s[i] for s in allj]
        if not (LIMIT_MIN[i] <= min(vals) and max(vals) <= LIMIT_MAX[i]):
            raise SystemExit(f"FATAL: feedback out of limits on joint {i}: {vals}")
        spread = max(vals) - min(vals)
        if spread > 0.15:
            log.line(f"WARNING: joint {i} not idle (spread {spread:.3f} rad); arm may be in motion")
    rec["median_joints"] = r4([statistics.median([s[i] for s in allj]) for i in range(6)])
    rec["last_xyz"] = allj and samples[-1]["xyz"]
    log.line(f"[baseline] success={n_ok}/{total} median_joints={rec['median_joints']} "
             f"({degs(rec['median_joints'])} deg) xyz={rec['last_xyz']}")
    return rec


# ---------------------------------------------------------------- main

def main():
    log.line("################ T=102 TRANSPORT EXPERIMENT START ################")
    assert_stock_sdk()
    env = collect_env()
    log.line(f"ENV: {json.dumps(env, indent=1)}")

    if not os.path.exists(PORT):
        raise SystemExit(f"FATAL: {PORT} does not exist")
    holders = find_port_holders(PORT)
    if holders:
        raise SystemExit(f"FATAL: {PORT} held by {holders}; stop those processes first")
    log.line(f"PORT check OK: {PORT} exists, no foreign holders")

    from roarm_sdk.roarm import roarm
    arm = roarm(roarm_type=ROARM_TYPE, port=PORT, baudrate=BAUDRATE)
    log.line("CONNECTED to arm via stock SDK")

    baseline = run_baseline(arm)
    if (baseline["success_rate"] or 0) < 0.8:
        raise SystemExit(f"FATAL: idle feedback success rate {baseline['success_rate']} < 0.8")

    A, B, nudged = pick_poses(baseline["median_joints"])
    log.line(f"POSE A rad={r4(A)} deg={degs(A)}" + (f" (nudged away from limits: {nudged})" if nudged else ""))
    log.line(f"POSE B rad={r4(B)} deg={degs(B)}")
    log.line("PHYSICAL MOTION IS ABOUT TO BEGIN: small base/wrist moves only, same A/B in all tests")

    RESULTS_STORE["meta"] = {**env,
                             "pose_A_rad": r4(A), "pose_A_deg": degs(A),
                             "pose_B_rad": r4(B), "pose_B_deg": degs(B),
                             "nudged_joints": nudged}
    RESULTS_STORE["baseline"] = baseline
    save_results()

    # ---- Test A
    ensure_at(arm, A, "[setupA]")
    RESULTS_STORE["tests"]["A"] = test_sparse(arm, A, B, 180, 10, "A")
    save_results()

    # ---- Test B
    ensure_at(arm, A, "[setupB]")
    RESULTS_STORE["tests"]["B"] = test_sparse(arm, A, B, 1000, 50, "B")
    save_results()

    # ---- Test C
    ensure_at(arm, A, "[setupC]")
    RESULTS_STORE["tests"]["C"] = test_repeated(arm, A, B, "C", 180, 10)
    save_results()

    # ---- Test D
    ensure_at(arm, A, "[setupD]")
    RESULTS_STORE["tests"]["D"] = test_streamed(arm, A, B, "D", D_HZ, D_LEG_DURATION, 1000, 50, D_CYCLES)
    save_results()

    ensure_at(arm, A, "[end]")
    RESULTS_STORE["summary"] = {
        t: {"sent": RESULTS_STORE["tests"][t].get("sent") or len(RESULTS_STORE["tests"][t].get("transitions", [])) or RESULTS_STORE["tests"][t].get("legs_total"),
            "accepted": RESULTS_STORE["tests"][t].get("accepted") or RESULTS_STORE["tests"][t].get("success") or RESULTS_STORE["tests"][t].get("legs_started"),
            "rate": RESULTS_STORE["tests"][t].get("acceptance_rate")}
        for t in ("A", "B", "C", "D") if t in RESULTS_STORE["tests"]}
    save_results()
    log.line("################ EXPERIMENT COMPLETE ################")
    try:
        arm.disconnect()
    except Exception:
        pass


if __name__ == "__main__":
    signal.signal(signal.SIGINT, on_sigint)
    try:
        main()
    except KeyboardInterrupt:
        log.line("!!! experiment interrupted by operator !!!")
        save_results()
        sys.exit(130)
    except SystemExit:
        save_results()
        raise
