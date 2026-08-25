# REPORT: Why sparse one-shot T=102 commands failed while the stock ROS2 stack worked

**Date:** 2026-08-25
**Environment:** WSL2 Ubuntu 22.04 (kernel 6.18.33.2-microsoft-standard-WSL2), Python 3.10.12,
stock `roarm-sdk==0.1.0` at `/home/mukund/.local/lib/python3.10/site-packages/roarm_sdk`
(pyserial 3.5), CP210x on `/dev/ttyUSB0` @ 115200. No ROS, no MoveIt, no repo-local SDK,
no firmware changes. All data in `raw.log` (every command + feedback sample) and `results.json`.

## Setup

- Pre-flight: port free, no foreign holders, stock-SDK guard passed.
- Idle feedback baseline: **29/29 (100 %)** over 3 s; XYZ (204.90, 192.70, 180.71) mm;
  all joints finite and in limits.
- **Pose A** rad = `[0.7547, -0.2531, 2.0371, -0.1442, -0.0031, 0.0261]`
  = deg `[43.24, -14.50, 116.72, -8.26, -0.18, 1.50]` (current physical state)
- **Pose B** rad = `[0.6674, -0.2531, 2.0371, -0.0918, -0.0031, 0.0261]`
  = deg `[38.24, -14.50, 116.72, -5.26, -0.18, 1.50]`
  (base -5.00 deg, wrist +3.00 deg, both away from nearest limit; gripper unchanged —
  hand joint measured constant at 0.0261 rad in every sample)
- The same A/B were used in all four tests.

## Results

| Test | Send pattern | speed/acc | commands | accepted | acceptance rate | notes |
|------|--------------|-----------|----------|----------|-----------------|-------|
| A | one-shot | 180/10 | 10 | 10 | 100 % | all start in 0.324-0.338 s; **every move stops ~1.7-1.9 deg short**; same-target resends ignored |
| B | one-shot | 1000/50 | 10 | 10 | 100 % | all start in 0.221-0.229 s; **identical stop positions and residuals as A** |
| C | repeated final target @10 Hz | 180/10 | 40 (10x4) | 10/10 transitions | 100 % | motion after 4 packets (first packet effective); residual unchanged (1.8-2.0 deg) |
| D | interpolated stream @20 Hz | 1000/50 | 300 (10 legs x 30) | 10/10 legs | 100 % | tracks the moving target (min err 0.009 rad); 0-2 progress regressions/leg; final residual 1.8-2.4 deg |

Totals: **360+ T=102 commands, ~3400 feedback reads, 0 delivery failures, 0 commands that
failed to start motion** (feedback: A 1496/0, B 1535/0, D 300/0 ok/fail).

### The central finding: truncation + same-target no-op

Every commanded static move — at **both** speed settings, under **all three** send patterns —
stopped at the **same position-dependent spot, short of the target**, and then:

- A->B always stopped with base at **0.6995 rad (40.03 deg)** (target 38.24 deg): residual 0.0321 rad.
  Reproduced from starts at both 0.7547 and 0.7240 rad.
- B->A always stopped with base at **0.7240-0.7256 rad (41.4-41.5 deg)** (target 43.24 deg): residual 0.0291-0.0307 rad.
- XYZ confirmed these are true physical stops (stable (214.68, 180.64, 167.65) mm over 3000+ samples), not stale feedback.
- **Resending the identical target does NOT resume the move:** setup re-sends of 56-67 identical
  packets over ~10 s produced zero additional motion, repeatedly.
- **A new (different) target always starts motion immediately** (0.22-0.34 s latency), even from the truncated position.
- 0 of 40 transitions/legs ended within 0.02 rad of target.
- Test D detail: while the interpolated target is *moving*, the arm tracks it (error to commanded
  position drops to 0.0092 rad); as the target stops at the goal, the arm falls behind and ends with the same ~0.03 rad residual.
- Observed firmware state change (recorded, not investigated per protocol): before any T=102
  activity the reported holding torques were tB=86.7, tE=133.1, tR=22.1, tG=-60.3; after T=102
  activity begins (both freeze states and during streaming) tR=0.0, tG=0.0, tE=36.0. Battery 12.10-12.15 V, stable.

## Hypothesis verdicts

- **H1 (sparse one-shot unreliable, streamed works) — partially true, different mechanism than assumed.**
  It is *not* packet loss: on WSL, 100 % of one-shot commands started motion. The failure is that the
  firmware **truncates each static move ~2 deg short and then ignores subsequent identical targets**.
  The ROS-style stream works because it continuously issues *new* targets, keeping the arm in tracking
  mode and never letting it sit in its "move complete" state. Naive redundancy (Test C: same target
  repeated) does **not** fix it.
- **H2 (speed=180/acc=10 is the problem) — false.** 180/10 and 1000/50 produced the *same* stop
  positions and the *same* residuals. Only motion-start latency differs (0.33 s vs 0.22 s).
- **H3 (WSL works, Windows was the problem) — not needed to explain the failure.** Zero delivery
  failures were observed on WSL. The Z-sweep's exact failure signature is reproduced and explained
  without invoking the Windows serial path (see below). If the Z-sweep's *first* commands sometimes
  also required retries, an additional loss factor in the Windows/COM21/repo-local-SDK path remains
  possible but was not measurable in this experiment (Test E skipped per protocol, since the
  mechanism was identified in WSL).

## Connection to the two historical observations

**Z-sweep (failed).** `z_sweep4.py::attempt_pose()` sends one T=102, waits for arrival tolerance, and on
failure **resends the same target** with escalating waits of 8/16/24/32 s. Today's data shows exactly
why that fails: the first command moves the arm partway (truncation ~2 deg short, so arrival tolerance
is never met), and every subsequent *identical* target is a firmware no-op — "one command eventually
executed, then many subsequent commands were ignored; waiting 1.5/8/16/24/32 s did not fix it" is the
predicted observable of truncation + same-target no-op.

**Stock ROS baseline (worked).** `roarm_driver.py` + the 100 Hz ros2_control loop reissues T=102 with
the controller's *continuously changing* target (and a `feedback_get()` after each). Test D (20 Hz
stream of new targets, same settings) reproduces the working behavior: the arm starts every leg and
tracks the progression with a bounded lag. A continuously commanded target stream never triggers the
same-target no-op, so the ~2 deg static residual stays hidden as ordinary tracking lag.

## Answers to the required questions

1. **Does sparse T=102 work reliably in WSL?**
   As command *delivery*: yes, perfectly (0/360+ lost, 100 % feedback). As a way to *reach* a static
   pose: **no** — every move ends 1.7-2.3 deg short of target, and re-sending the same target never
   completes it.

2. **Does changing 180/10 -> 1000/50 materially change acceptance?**
   **No.** Identical acceptance (10/10 vs 10/10), identical stop positions, identical residuals.
   Only motion-start latency improves (0.33 s -> 0.22 s).

3. **Does repeatedly resending the same target fix command loss?**
   There was no command loss to fix. Re-sending the same target (56-67 packets over 10 s) **does not**
   resume a truncated move. Redundancy of *identical* targets is useless here.

4. **Does streamed joint interpolation work reliably?**
   It starts and tracks reliably (10/10 legs, 300/300 feedback OK, 0-2 regressions per leg, monotonic
   tracking of the moving target). But each leg still ends with the same ~2 deg lag once the target
   stops. Reliable for *tracking*, not for exact static goal attainment.

5. **Which variable best explains the discrepancy?**
   The **send pattern**: continuous streaming of *new* targets (ROS 100 Hz loop) vs sparse *static*
   targets with identical-target retries. The ROS architecture keeps the arm in tracking mode; the
   sparse approach lands in the firmware's truncated/"complete" state, where identical targets are
   ignored. Speed/acc: no effect (H2 false). OS/serial: WSL transport was 100 % clean, so it is not
   the primary cause (H3 demoted to possible secondary factor only).

6. **Is the arm itself usable for reliable trajectory experiments?**
   **Yes — as a continuously commanded tracking system.** Stream new targets (20 Hz demonstrated at
   100 % reliability; the stock ROS loop uses 100 Hz), verify state with `feedback_get()`, expect a
   ~2 deg static residual, do not gate on exact static goal attainment, and use the *next different*
   target to continue from the residual position (every Test D leg started from the previous leg's
   residual and moved correctly).

7. **What control strategy should future OhmLab/RobotAgent experiments use?**
   **D: continuously streamed joint trajectories with feedback** (the stock ROS pattern).
   Do not rely on one-shot commands for pose attainment, and do not rely on identical-target
   retransmission to close residuals.

### Final recommendation: **D**

*(Not E — the hardware is usable and the failure is a well-characterized firmware execution
semantics, not random unreliability. Not C — identical-target repetition demonstrably does not close
the residual. Not A/B — one-shots never reach the target under any speed/acc setting tested.)*

## Caveats

- A/B were the arm's current (high-load, elbow 116.7 deg) configuration plus a small perturbation;
  per protocol only this pose pair was characterized. The ~0.03 rad residual magnitude may be
  load/configuration dependent, and the stop positions (40.03 deg / 41.52 deg base) are
  position-dependent.
- All tested patterns included near-immediate `feedback_get()` (~25 ms after each send; every cycle
  in C/D). The "no feedback after send" variant was not isolated; with 100 % acceptance in every
  tested pattern, it is unlikely to be a deciding variable.
- The torque-signature change after T=102 activity is recorded as an observation; firmware internals
  were deliberately not investigated (protocol: do not debug/modify the robot).
- Motion detection threshold 0.01 rad; settled tolerance 0.03 rad. "Accepted" = measured motion
  toward target; it says nothing about whether the move completed.
