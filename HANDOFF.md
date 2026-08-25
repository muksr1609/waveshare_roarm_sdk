# HANDOFF — RoArm M3 Cartesian z-sweep: silent command drops

Reconstructed 2026-08-25 for external audit (ChatGPT) and for the next working session.
Read top to bottom; sections 8–10 are the active bug.

## 1. Objective

Command the arm through **6 discrete poses** of a Cartesian z-sweep and record, for each,
the firmware-measured tip z and the mismatch vs commanded z:

    commanded z (firmware frame, mm):  100, 120, 140, 160, 180, 200

All other DOF held constant at their values at run start (x, y, roll, tool pitch, gripper).
The z-sweep exists to explain why the arm appears to "move more downward than commanded".
That question is now **solved** (section 7.3). The **blocking bug** is that the arm executes
at most ~1 command per session and then silently drops every subsequent command for minutes
(sections 8–9).

## 2. Current state (TL;DR)

- Arm is ALIVE, stationary, holding the z=100 pose:
  joints(deg) = [0.35, 13.27, 118.65, -34.01, -0.18, 1.49], firmware tip z = 89.88 mm.
  Desk clearance ~210 mm (desk is at z ≈ -120 in this frame).
- 0 of the 6 poses have ever been executed in one session. Best single-run result:
  run 4 landed z=100 (on the 5th send) and then went completely deaf: 15+ further sends
  over ~5 minutes, all dropped, zero motion.
- The command path is fire-and-forget with NO acknowledgement (section 5), so drops are
  invisible to the SDK; the only detection is watching joints not move.
- A byte-corruption was observed on the serial RX stream (section 9) — a live lead.
- The audit is asked to rank hypotheses H1–H5 (section 8) and propose experiments.

## 3. Hardware & environment

| Item | Value |
|---|---|
| Robot | Waveshare RoArm M3-S, `roarm_type="roarm_m3"`, gripper `angular_direct` |
| Port | COM21, 115200 baud |
| Adapter | Silicon Labs CP210x USB-UART bridge, VID:PID=10C4:EA60, SER=06780091796CEF118811A4ADC169B110, USB location 1-1 |
| Python | C:\Users\Mukund\Documents\Ro-Arm\.venv\Scripts\python.exe (3.12, pyserial) |
| Repo | C:\Users\Mukund\Documents\Ro-Arm, branch `upstream-cartesian-ik` |
| SDK | repo-local `roarm_sdk/` (Waveshare's, with local patches — see section 5) |
| Operator | A human was beside the arm with power-off capability for all runs. Re-confirm before commanding any descent. |

Firmware feedback: the controller streams a JSON frame `{"T":1051, "x":.., "y":.., "z":..,
"tit":.., "b":.., "s":.., "e":.., "t":.., "r":.., "g":.., "tB":.., ...}` roughly every
100 ms over serial. x/y/z are computed by the firmware from joint encoders via forward
kinematics (no external sensor). Field meanings (rad unless noted): `b`=base, `s`=shoulder,
`e`=elbow, `t`=wrist pitch, `r`=wrist roll, `g`=gripper, `tit`=tool pitch, x/y/z in mm.

## 4. The 6 poses (exact, computed offline)

Held constants (from run-4 start): x0=358.27, y0=2.20, roll0=-0.0031 rad, pitch0=0.1012 rad
(= raw `tit`), gripper0=0.0261 rad. Targets from `roarm_sdk/waveshare_ik.py::
compute_joint_rad_by_pos` (Waveshare's upstream M3 IK, verified exact round-trip, section 7.1):

| cmd z | base | shoulder | elbow | wrist | roll | gripper | dist from prev (rad) |
|---|---|---|---|---|---|---|---|
| 100 | 0.352 | 12.183 | 118.502 | -34.887 | -0.178 | 1.495 | — |
| 120 | 0.352 | 9.152 | 113.554 | -26.907 | -0.178 | 1.495 | 0.1393 |
| 140 | 0.352 | 6.775 | 108.050 | -19.027 | -0.178 | 1.495 | 0.1375 |
| 160 | 0.352 | 5.045 | 101.979 | -11.226 | -0.178 | 1.495 | 0.1362 |
| 180 | 0.352 | 3.954 | 95.297 | -3.452 | -0.178 | 1.495 | 0.1357 |
| 200 | 0.352 | 3.505 | 87.915 | 4.378 | -0.178 | 1.495 | 0.1367 |

(all degrees except "dist", which is rad). Minimum inter-pose distance = **0.1357 rad (7.77°)**,
dominated by shoulder+wrist. This number drives the tolerance analysis in 7.4.

## 5. Transport & SDK code paths (exact)

Command frame on the wire for all-6-joints motion (T=102), built by
`roarm_sdk/common.py:handle_m3_joints_radian` (lines 150–154):

    {"T":102,"base":..,"shoulder":..,"elbow":..,"wrist":..,"roll":..,"hand":..,"spd":..,"acc":..}\n

- For `angular_direct` gripper the wire `hand` = π − (user value); the same convention is
  applied to the `g` field when parsing feedback (`handle_m3_feedback`, common.py:255–270).
  So: feedback `g` (as returned by `feedback_get()`) = π − wire_g, and commanding that same
  value back is a no-op for the gripper.
- Speed/acc units: spd 1..4096, acc 1..254. Demos use spd=180, acc=10
  (≈ 15.8 deg/s max joint rate).

Fire-and-forget (THE root of the detection problem) — `roarm_sdk/roarm.py:_request_once`
(lines 114–142, critical part lines 133–137):

    self._write(real_command)
    if genre != JsonCmd.FEEDBACK_GET:
        return real_command      # fires T=102 and returns; NO ack is read, ever
    return self._read(genre)

- `joints_radian_ctrl` (generate.py:116) → `_mesg` → `_res` → `_request_once`.
  `_res` (roarm.py:144–181) retries `_request_once` up to 10 times, but for T=102 every call
  "succeeds" immediately after writing — the retry loop never sees a failure. **A dropped
  command is indistinguishable from an accepted one.**
- `roarm_sdk/common.py:write` (lines 359–380) does `reset_input_buffer()` BEFORE every write,
  discarding anything the firmware had sent (including any potential ACK), then writes + flushes.
- `roarm_sdk/common.py:read` (lines 382–393): writes `{"T": 105}\n` (feedback request), then
  `BaseController.feedback_data()` (lines 88–103) reads ONE `T:1051` frame via `ReadLine`
  (lines 33–68): frames are delimited by `}\r\n`, max 512 bytes; on JSONDecodeError it logs
  and clears the buffer.
- Local patch on this branch (commit 1476b68): `roarm.joints_radian_ctrl_once`
  (roarm.py:183–210) — one validated T=102 send, no auto-retry, returns the encoded bytes or -1.
  Still fire-and-forget (no ack read). The sweep scripts use plain `joints_radian_ctrl`.
- `roarm_sdk/waveshare_ik.py`: faithful transcription of Waveshare's upstream M3
  `computeJointRadbyPos`; link lengths L2=(236.82,30), L3=(144.49,0), L4=(171.67,13.69) mm.
  User confirmed link lengths are physically correct.

## 6. Complete run history (all data)

All runs: port COM21, spd=180, acc=10, one connection per run. "Landed" = arm settled near
the commanded joints. "No motion" = joints unchanged across the whole attempt window.

### Run 0 — z=60 descent test (cartesian_motion_demo.py), pre-handoff
- Guard lowered 120→50 mm (`large_motion_demo.py:MIN_TOOL_Z_MM`, uncommitted change).
- Commanded (362.7, 7.79, 60). First send dropped; 2 s no-motion probe → one retry; live
  height guard tripped at measured z=46.7; emergency hold sent at measured joints; aborted.
- Arm ended at z=16.8, joints [1.23, 23.03, 111.27, -17.93, -0.18, 1.49] — i.e. it kept
  going below the guard and below the hold point after the hold command. (Hold did not stop it.)
- User visually confirmed desk was far below; arm later found back at home (power cycle /
  auto-return between sessions).

### Run 1 — z_sweep.py (no retry, immediate next command)
START: z=186.24, joints [0.35, 3.69, 92.72, -0.26, -0.18, 1.49]. Holding x=357.98 y=2.20.

| cmd z | meas z | result |
|---|---|---|
| 100 | 186.24 | DROPPED (no motion) |
| 120 | 91.94 | LANDED, joints [0.35, 12.83, 113.64, -25.4], max_jerr 0.067 rad |
| 140 | 91.94 | DROPPED |
| 160 | 91.94 | DROPPED |
| 180 | 91.94 | DROPPED |
| 200 | 91.94 | DROPPED |

Also logged during this run (RX corruption, see section 9):
`10:46:17.540 ERRO [BaseController] JSON decode error: Expecting ',' delimiter: line 1 column
190 (char 189) with line: {"T":1051,...,"tB":-24,"tS":184\u000b"tE":136,...}` — a comma
(0x2C) arrived as 0x0B (vertical tab).

### Run 2 — z_sweep2.py (1.5 s inter-pose dwell, 2.5 s no-motion probe, one resend per pose)
START: z=154.29, joints [0.35, 8.44, 90.35, 4.75, -0.18, 1.49].
NOTE: arm had DRIFTED on its own from 91.94 (end of run 1) to 154.29 between runs, with no
commands sent.

| cmd z | meas z | result |
|---|---|---|
| 100 | 154.29 | no motion (2 sends) |
| 120 | 154.29 | no motion (2 sends) |
| 140 | 87.63 | PARTIAL: joints [0.35, 13.97, 107.05, -15.03] vs target [6.775, 108.05, -19.027] → shoulder 7.2° short; arm stopped mid-way |
| 160 | 87.63 | no motion (2 sends) |
| 180 | 87.63 | no motion (2 sends) |
| 200 | 87.63 | no motion (2 sends) |

### Run 3 — z_sweep3.py (8 attempts/pose, 1.5 s dwell, 0.5 s pre-send pause, arrival tol 0.12 rad)
START: z=183.05, joints [0.35, 5.71, 90.35, 0.88, -0.18, 1.49] (arm drifted back up on its own again).

| cmd z | meas z | max_jerr | tries | result |
|---|---|---|---|---|
| 100 | 84.34 | 0.0325 | 4 | LANDED, joints [0.35, 15.12, 115.14, -30.32] |
| 120 | 84.34 | 0.1199 | 1 | FALSE "reached" — arm never moved; z=100 pose was within 0.1199 rad of the z=120 target (tolerance bug, section 7.4) |
| 140 | 84.34 | 0.2547 | 8 | no motion |
| 160 | 84.34 | 0.3890 | 8 | no motion |
| 180 | 84.34 | 0.5239 | 8 | no motion |
| 200 | 88.94 | 0.4108 | 8 | PARTIAL: joints [0.35, 14.33, 106.87, -16.0] (moved ~14° of wrist on its own / from one lucky send) |

### Run 4 — z_sweep4.py (5 attempts/pose, escalating recovery waits 8/16/24/32 s, 3 s dwell,
1 s pre-send pause, arrival tol 0.08 rad, already-there tol 0.03 rad, pitch bug fixed — see 7.5)
START: z=188.16, joints [0.35, 3.69, 92.37, -0.26, -0.18, 1.49]. Holding pitch=0.1012 rad.

| cmd z | meas z | max_jerr | tries | result |
|---|---|---|---|---|
| 100 | 89.88 | 0.0190 | 5 | LANDED, joints [0.35, 13.27, 118.65, -34.01], round-trip err 0.00° |
| 120 | 89.88 | 0.1241 | 5 | NO MOTION (joints identical, 4× escalating waits totaling 80 s) |
| 140 | 89.88 | 0.2617 | 5 | NO MOTION |
| 160 | 89.88 | 0.3978 | 5 | NO MOTION |
| 180 | — | — | — | (user aborted the run here) |
| 200 | — | — | — | (not run) |

Context for run 4: its first (previous, unmodified) attempt was killed by a **computer crash**;
after reboot the arm had auto-returned to home (z≈187.6) and was responding to feedback reads.

### Cross-run pattern (the core anomaly)

1. **Exactly ~1 command executes per session.** Runs 1–4 each executed one (run 2: one
   partial; run 3: one + one partial at the very end). 40+ additional sends across all runs:
   zero effect.
2. **Deafness outlasts every wait tried**: 1.5 s, 8 s, 16 s, 24 s, 32 s per retry; total
   deaf windows observed ≥ 5 minutes (runs 3 and 4).
3. **Even the first command of a session is often dropped several times** (run 4: landed on
   the 5th send, after 80 s of cumulative recovery waits; run 3: 4th send).
4. **The arm drifts on its own between sessions** (91.94 → 154.29 → 87.63 → 183.05 → 188.16)
   and auto-returns to home after a host crash. Something in the firmware is actively
   re-positioning/holding without host commands.
5. Settled poses are **1.1°–3.8° off** the commanded joints (max single-joint), mostly
   shoulder/wrist. Run 2's partial stop was 7.2° short on shoulder.

## 7. Established facts (verified, do not re-litigate)

### 7.1 Model consistency
Round-trip test (investigation/roundtrip.py): feeding the firmware's own (x, y, z, r, tit)
back through `compute_joint_rad_by_pos` reproduces the firmware's own joints to **0.00°**.
Firmware FK and Python IK are the same model; `tit` is the correct "pitch" input, `r` the
"roll" input.

### 7.2 Frame offset
`z=0` is NOT the desk. The desk surface sits ~120 mm below the z=0 reference
(z_desk ≈ -120 mm). All commanded sweep poses (z ≥ 100) keep the tip ≥ ~200 mm above the desk
even with observed undershoot (worst measured: 84.34).

### 7.3 The z "mismatch" is explained (no model bug)
Because 7.1 holds, measured z = FK(achieved joints). The mismatch is just residual joint
error. Check with run 4's z=100: target [12.183, 118.502, -34.887], achieved [13.27, 118.65,
-34.01] → shoulder +1.09°, wrist -0.88°. With the tip ~358 mm from the base axis,
1.09° × 358 mm ≈ 6.8 mm + wrist 0.88° × 172 mm ≈ 2.6 mm ≈ 9.4 mm ≈ observed 10.12 mm. ✓
(The previous handoff's "unexplained 15.7 mm" used a ~200 mm lever arm; the correct lever at
these folded poses is the full horizontal reach ~358 mm, which closes the gap in run 3 too:
1.86° shoulder ≈ 11.6 mm + 0.9° wrist ≈ 2.7 mm ≈ 14.3 mm ≈ observed 15.66 mm.)
**Conclusion: once the command-drop bug is fixed and poses actually land, the sweep's
measured mismatch should shrink to the servo settling residual (~5–15 mm, dominated by
shoulder error), with round-trip error ≈ 0 confirming self-consistency at each pose.**

### 7.4 Tolerance analysis (why the old "already there" check false-triggered)
- Min distance between adjacent pose targets: 0.1357 rad.
- Observed settling error: 0.019–0.067 rad.
- Therefore an "already there" shortcut is only safe below ~0.10 rad, and an arrival
  tolerance below ~0.10 rad. z_sweep3's 0.12 rad arrival tolerance was ABOVE the
  achieved-pose distance to the next target (0.1199) → false "reached" at z=120.
  z_sweep4 uses arrival 0.08 / already-there 0.03 — safe against false triggers.

### 7.5 Bugs found & fixed in the sweep scripts during this handoff cycle
- z_sweep3: 0.12 rad "already there" shortcut (section 7.4) — fixed in z_sweep4.
- z_sweep4 (found 2026-08-25, before its first completed run): line 115 did
  `pitch0 = math.radians(float(fb0[3]))` but `fb0[3]` is raw `tit`, **already radians**
  (v3 correctly used `math.radians(pose_get()[3])` where pose_get converts to degrees).
  The double conversion would have commanded pitch ≈ 0.02 rad instead of ≈ 0.10 rad.
  Fixed to `pitch0 = float(fb0[3])`.

## 8. THE BUG: post-motion command deafness — hypotheses for the auditor

Observed behavior (section 6, "cross-run pattern"): after the firmware executes (or partially
executes) one T=102, it ignores all subsequent T=102 commands for minutes, at any send rate,
with the host reading feedback continuously (so the RX buffer is not full). The SDK never
detects this (section 5). Hypotheses, ranked by current belief:

- **H1 — Firmware command state machine stuck after a move.** The controller enters a long
  "busy/recovery/drift" state after executing a T=102 (consistent with the observed
  self-drift and auto-home behavior) and drops new T=102 until that state clears, which may
  take minutes. The 8–32 s escalating waits never reach it. *Support: pattern 1–4 above;
  the arm demonstrably does firmware-initiated motion (drift, auto-home).*
- **H2 — TX byte corruption (EMI on the CP210x line).** Each packet has an independent
  corruption probability while servo holding currents are active; the firmware silently
  discards malformed JSON. RX corruption was directly observed (section 9). *Against:
  drops persisted through 80 s of waiting with the arm fully settled (holding current only),
  and run 4's first command (arm idle at home for minutes) was also dropped 4 times.*
- **H3 — Host-side serial config.** pyserial opens COM21 with defaults (8N1, no flow
  control, rts=False per roarm.py:50, dtr left default). A CP210x buffer/driver mismatch
  could corrupt or lose TX bytes. *Cheap to test (section 10, E4/E5).*
- **H4 — `reset_input_buffer()` before each write interferes** with a firmware-side
  handshake (e.g. the firmware expects the host to consume its output at a certain rate).
  *Weak: the sweep scripts drain feedback continuously between sends anyway.*
- **H5 — Missing ACK handshake.** The firmware may drop a new T=102 until some per-command
  handshake completes (host reads an ACK it is never given, because the SDK discards it).
  *Testable directly by E1 (does any non-1051 frame exist at all?).*

Key open questions for the auditor:
1. Does the firmware send ANY acknowledgement for T=102 (or T=101, POSE_CTRL)? (E1 answers.)
2. Is the deafness a function of TIME after the last motion (H1) or of per-packet corruption
   probability (H2)? E3 discriminates: if the next command lands only after a long fixed
   delay (e.g. 5–10 min), H1; if burst-sending 10 copies lands it quickly, H2/H3.
3. Why does the arm drift/auto-home without commands? If that is a known firmware "recovery"
   mode, entering it may be exactly what blocks new commands.
4. Is `spd=180` relevant (e.g. the firmware's motion-profile bookkeeping vs actual settle time)?

## 9. Serial corruption evidence

Run 1 log (full line):

    10:46:17.540 ERRO [BaseController] JSON decode error: Expecting ',' delimiter:
    line 1 column 190 (char 189) with line:
    {"T":1051,"x":357.9772007,"y":2.19654816,"z":186.2414631,"tit":0.107378655,
    "b":0.006135923,"s":0.064427193,"e":1.618349731,"t":-0.004601942,"r":-0.003067962,
    "g":3.11551498,"tB":-24,"tS":184\u000b"tE":136,"tT":-20,"tR":24,"tG":-60,
    "torswitchB":1,...,"v":1213}

The comma after `"tS":184` arrived as 0x0B (VT). 0x2C → 0x0B is not a single-bit flip,
so it is not simple bit noise; candidates: dropped byte + re-sync, framing error, EMI burst,
or driver overrun. Frequency unknown — E1 measures it (decode failures per N frames,
arm idle vs arm holding).

## 10. Next experiments (proposed; safe order; none require operator action except E6)

All use raw `serial` (no SDK) so every byte is visible. Scripts to be written into
`investigation/` as executed.

- **E1 — ACK + corruption probe (NO MOTION, safe).** Capture 3 s baseline RX: frames/s,
  JSON decode failure rate, any T != 1051 frames. Then send a NO-OP T=102 (current joints —
  gripper via wire convention hand = π − feedback_g, see section 5) and capture 5 s RX.
  Repeat ×3. Question: does any non-1051 frame appear after the send? (Answers H5, gives
  baseline corruption rate for H2.)
- **E2 — Burst no-op (NO MOTION, safe).** Send the no-op T=102 ×10 at 100 ms spacing;
  capture RX; check whether the firmware reacts at all to repeated identical commands
  (e.g. rate limiting / dedup would show up as silence; an ACK stream would show up as frames).
- **E3 — Deafness recovery timer (small motion, ~7° shoulder).** From the current pose,
  command z=110 (≈ half a step; target joints computable from section 4 interpolation or
  re-run the IK). Verify it lands (motion + settle). Then every 30 s send ONE no-op T=102,
  logging the timestamp of the first one that is accepted (detect via any state change or,
  if E1 found an ACK, via the ACK). Continue up to 15 min. Output: recovery time constant
  → discriminates H1 vs H2/H3.
- **E4 — Baud rate (NO MOTION, safe).** Repeat E1/E2 at 57600 and 9600. If corruption rate
  or acceptance improves, H2/H3.
- **E5 — COM port config audit (host only, safe).** `mode COM21`, Device Manager error
  status, try dtr=True/False, rts toggling; compare acceptance of no-ops.
- **E6 — Fresh power cycle (requires operator).** Power-cycle the arm, then send three small
  REAL moves (z=140, 160, 180) spaced 2 min apart; record which land. Tests whether a fresh
  boot breaks the "one command per session" pattern (H1 in its strongest form).
- **E7 (if E1 finds an ACK) — ACK-gated send loop.** Rewrite the per-pose driver to: send →
  read until ACK or 2 s timeout → if no ACK, resend immediately (no blind waits); if ACK,
  watch joints to settle. This converts the fire-and-forget path into verified delivery.

## 11. Safety model (carry forward unchanged)

- Operator beside arm with power-off capability — MUST be re-confirmed before any descent.
- All sweep poses are z ≥ 100 mm (firmware frame); desk at z ≈ -120 → ≥ ~200 mm clearance
  even with the worst observed undershoot. No pose in the sweep approaches the desk.
- Torque is NEVER released (no `torque_set` calls in any script; Ctrl+C paths leave it on).
- No per-joint moves, no streamed waypoints: exactly one T=102 per attempt, all six joints.
- E1/E2/E4/E5 are no-motion by construction (no-op targets). E3 moves ~7° of shoulder only.
- The arm has a history of moving on its own (drift/auto-home); always re-read state before
  commanding, and keep the live z readout visible (check_pose.py).

## 12. File map

Repo (branch `upstream-cartesian-ik`), C:\Users\Mukund\Documents\Ro-Arm:
- `cartesian_motion_demo.py` — single-XYZ CLI target (Waveshare IK → one T=102 via
  `large_motion_demo.LargeMotionRunner._transition`); not used by the sweep.
- `large_motion_demo.py` — `MIN_TOOL_Z_MM = 50.0` (lowered from 120.0; uncommitted at
  handoff, now committed), pose sequence A..F, dwell/cooldown logic (30 s after index 3,
  10 s after index 5 — from the roll-slowdown observation), arrival polling (0.12 rad /
  3×0.015 stable), 1 s inter-pose dwell.
- `roarm_sdk/roarm.py` — `_request_once` (114–142, fire-and-forget at 134–137),
  `joints_radian_ctrl_once` (183–210, commit 1476b68).
- `roarm_sdk/common.py` — wire handlers, `write` (359–380), `read` (382–393),
  `ReadLine`/`BaseController` (33–103), feedback convention (255–270).
- `roarm_sdk/waveshare_ik.py` — upstream M3 IK (round-trip exact vs firmware).
- `investigation/` — the sweep & probe scripts (copied from temp, see below):
  - `z_sweep.py` (run 1), `z_sweep2.py` (run 2), `z_sweep3.py` (run 3, has the 0.12 rad bug),
  - `z_sweep4.py` (run 4; pitch bug FIXED on line 115: `pitch0 = float(fb0[3])`),
  - `check_pose.py` (6 raw feedback samples), `show_frame.py` (one raw firmware frame,
    decoded), `roundtrip.py` (IK(FK(joints)) consistency test).
- Temp originals (still present): `C:\Users\Mukund\AppData\Local\Temp\opencode\` (same names).

Session data (this machine, opencode's SQLite, table `part`, session
`ses_fc7cae9d2ffeq44MBwN2iVh61t`): full transcript of runs 0–4 including raw outputs —
useful if any number above needs re-verification.

## 13. How to run

    $py = "C:\Users\Mukund\Documents\Ro-Arm\.venv\Scripts\python.exe"
    # state check (read-only, no motion):
    & $py investigation/check_pose.py
    # the sweep (6 poses, per-pose verify/retry; takes minutes due to recovery waits):
    & $py investigation/z_sweep4.py

Do NOT run `cartesian_motion_demo.py --z-mm <50` style descents without re-confirming the
operator is present (section 11).

## 14. What "done" looks like

1. All 6 poses (section 4) executed in one session, each with measured z, mismatch,
   max_jerr, and round-trip err recorded; "ALL POSES REACHED: yes".
2. The drop mechanism identified (H1–H5 or new) with a discriminating experiment's data.
3. A verified-delivery send loop (ACK-gated per E7, or proven-timing retry) in
   `roarm_sdk` or the sweep script, with unit-testable structure.
4. This HANDOFF.md updated with the outcome; investigation scripts kept in-repo.
