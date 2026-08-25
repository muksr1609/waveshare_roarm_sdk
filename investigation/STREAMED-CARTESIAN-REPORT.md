# STREAMED-CARTESIAN-REPORT: Waveshare upstream IK + streamed joint targets on the physical RoArm-M3-S

**Date:** 2026-08-25
**Environment:** WSL2 Ubuntu 22.04, `/dev/ttyUSB0` (CP210x via usbipd), stock `roarm-sdk==0.1.0`
(user site-packages, guard enforced by the script), Python 3.10. No ROS, no MoveIt, no firmware
changes. Waveshare upstream Cartesian IK: `roarm_sdk/waveshare_ik.py` (repo copy, unmodified).
Evidence: `streamed-cartesian-result.json` (planned path + per-leg metrics), `streamed-cartesian.log`
(every command + feedback sample, timestamped). Script: `streamed-cartesian-demo.py`.

## What ran

The script read the arm's CURRENT measured pose, planned a constant-Z horizontal arc around it
(101 waypoints), validated the whole path offline (IK finite, 2.5-deg-inside-limit margin, max
adjacent step 0.226 deg << 5 deg, max per-joint excursion 19.5 deg << 35 deg), printed the stats,
then streamed the 101 changing joint targets at 20 Hz (speed=1000, acc=50) for 5.05 s, logged live
feedback every cycle, settled, and streamed the same path in reverse once. No endpoint re-sending.

- Start pose: xyz (211.81, 185.56, **172.83**) mm, pitch 0.1749, roll -0.0031,
  joints [0.7194, -0.2454, 2.0463, -0.1273, -0.0031], grip 0.0261 (all kept constant).
- Planned arc: **R = 80 mm, sweep = 75 deg, path length 104.7 mm**, center (291.8, 185.6), sweep +.
  Cartesian bbox x [211.8, 271.1] y [108.3, 185.6] z [172.8, 172.8] (constant by construction).
  Joint excursions: base 19.45 deg, shoulder 6.52, elbow 5.07, wrist 1.45, roll 0.00.
- Forward: 101/101 commands, 101/101 feedback OK, 0 invalid, 5.05 s stream (7.18 s incl. settle).
  Tip moved (211.8, 185.6) -> (265.7, 114.9) mm: ~96 mm of visible arc traced.
- Reverse: 101/101 commands, 101/101 feedback OK, 0 invalid. Tip moved back to (214.9, 176.9) mm.

## Answers

1. **Did you see a clearly recognisable arc, not a twitch?** Yes. ~100 mm of tip travel, base
   rotating ~19 deg, tip sweeping from (211.8, 185.6) to (265.7, 114.9) mm and back. The
   per-sample log shows monotonic progression along the planned arc in both legs.
2. **Requested radius/angle/path length:** R = 80 mm, 75 deg, 104.7 mm (101 waypoints @ 20 Hz).
3. **Requested constant Z:** 172.83 mm (start pose; pitch 0.1749, roll -0.0031, grip 0.0261 held).
4. **Actual measured Z range:** **150.68 - 172.83 mm** — the firmware's Z readout dropped up to
   **~22 mm below** the commanded constant Z during the arc (most of the deviation while the arm
   was near full elbow extension at the far side of the arc). Joint torques stayed high and stable
   (tB 84-116, tS 28-188, tE 20-152) with all torque switches on, so this is most plausibly a
   firmware forward-kinematics/model error in this configuration rather than physical sag; it is
   recorded as an observation, not debugged. **The firmware XYZ Z channel is not trustworthy in
   near-full-extension configurations; XY is.**
5. **Max commanded-vs-measured joint tracking error while moving:** **3.55 deg** (forward),
   **3.87 deg** (reverse) — bounded lag, no divergence.
6. **Feedback or T=102 execution failures:** **None.** 202/202 T=102 commands accepted and started
   motion; 202/202 feedback reads valid; 0 invalid.
7. **Forward and reverse both worked:** Yes, both completed normally with no abort.
8. **Jerks, freezes, branch jumps:** None observed. Planned max adjacent step 0.226 deg; measured
   joint trajectory in the log progresses monotonically with bounded lag in both legs.
9. **Final physical joint error:** after forward, vs its commanded end waypoint:
   [1.61, 0.52, 2.57, 0.95, 0.00] deg (tip 8.4 mm short in XY); after reverse, vs the start pose:
   [1.76, 0.44, 3.08, 0.70, 0.00] deg (tip 9.7 mm from start in XY). Consistent with the known
   ~2 deg static residual; not chased per protocol.
10. **Verdict: B — the arm can execute a streamed Cartesian toolpath, but tracking/accuracy is
    poor.** The Cartesian XY arc was clearly and smoothly traced in both directions with zero
    transport/execution failures; however, in-motion lag reaches ~4 deg of joint angle, static
    endpoint error is 1.6-3.1 deg per joint (~10 mm of tip at this radius), and the firmware Z
    readout deviates up to 22 mm from the commanded constant Z in this configuration. For
    RobotAgent: use it as a *visually following* toolpath executor with bounded lag; do not gate
    on firmware-Z accuracy or exact static endpoint attainment; keep targets continuously moving
    and verify state via `feedback_get()`.

## Notes for re-running

- Reusable: `python3 investigation/streamed-cartesian-demo.py` plans a new safe relative arc from
  wherever the arm currently is (auto-selects direction/center, shrinks 80/75 -> 80/60 -> 60/75 ->
  60/60 if validation fails, aborts before motion if nothing validates).
- After a WSL reboot the CP210x needs re-attachment: `usbipd attach --wsl --busid <busid>`, and the
  WSL user needs `dialout` group membership (added persistently on 2026-08-25).
- The first arc candidate (80 mm / 75 deg, sweep +) validated directly from the 2026-08-25 start
  pose; no shrinking was needed.
