# WINDOWS-STREAMED-CARTESIAN-REPORT: same streamed Cartesian control, native Windows

**Date:** 2026-08-25
**Environment:** Native Windows 10/11 (win32), Python 3.12.10, `COM21` @ 115200,
STOCK `roarm-sdk==0.1.0` from `C:\Users\Mukund\AppData\Local\Programs\Python\Python312\Lib\site-packages`
(guard enforced in-script; the repo SDK was NOT imported), pyserial 3.5. Waveshare upstream Cartesian
IK: repo copy `Ro-Arm/roarm_sdk/waveshare_ik.py` (unmodified — the same file the WSL run used; the
stock pip package does not ship `waveshare_ik.py`). No ROS, no MoveIt, no firmware changes, no SDK
changes. Script: `windows-streamed-cartesian.py` (platform port of
`investigation/streamed-cartesian-demo.py` @ 3974bdc; motion algorithm, trajectory, validation
thresholds, speed/acc, rate, and forward/reverse sequence unchanged).
Evidence: `windows-result.json` (environment, serial settings, open observations, planned path,
per-leg metrics), `windows.log` (every command + feedback sample, timestamped),
`windows-console.log` (console capture incl. raw T=102/T=1051 frames).

## Answers

1. **Was native Windows using stock roarm-sdk==0.1.0?** Yes. `roarm_sdk.__file__ =
   ...\Lib\site-packages\roarm_sdk\__init__.py`, version 0.1.0 (downgraded from 0.1.0-previously-0.1.1
   in the same site-packages before the run), Python 3.12.10, pyserial 3.5.
2. **Did opening COM21 cause a controller reset or physical movement?** No reset signature: first
   valid feedback arrived 0.11 s after open, and the first frame after open is byte-identical (XYZ and
   all 6 joints) to all 10 consecutive stability-gate frames — no twitch, no pose change, no feedback
   interruption. (The arm's pose at this run differs from the last recorded arc200 end pose, i.e. the
   arm was moved between experiments, before this run.)
3. **What RTS/DTR states were observed?** RTS=**False** (the stock SDK sets `rts=False` before
   open), DTR=**True** (pyserial default, left untouched by the SDK), rtscts=False, dsrdtr=False,
   xonxoff=False, timeout=0.1 s, 115200 baud. Stock Windows behaviour as-is; no line was forced.
4. **Did the full 80 mm / 75 degree arc validate?** Yes — directly, first candidate, no shrinking.
   Center (323.8, 167.3) mm, sweep +, 101 waypoints, path 104.72 mm, constant Z=80.98 / pitch=0.4357 /
   roll=-0.0031 / grip=0.0261. Bbox x [243.8, 303.1], y [90.0, 167.3], z constant. Max joint
   excursion: base 17.92°, shoulder 7.23°, elbow 8.66°, wrist 1.42°. Max adjacent step 0.259°.
5. **Did the forward ~100 mm physical arc execute?** Yes. 101/101 T=102 commands, 101/101 valid
   feedback, 0 invalid, 5.00 s, achieved rate 19.988 Hz (median interval 47 ms). Tip XY travelled
   (243.8, 167.3) → (296.0, 96.3) mm ≈ 88 mm of visible arc.
6. **Did the reverse arc execute?** Yes. 101/101 commands, 101/101 valid feedback, 0 invalid, 5.00 s,
   19.988 Hz. Tip returned to (244.4, 156.3) mm (11.0 mm from start in XY).
7. **How many T=102 commands were sent?** 202 (101 forward + 101 reverse), all different successive
   targets.
8. **How many valid feedback reads occurred?** 202/202 during the two legs (plus 11 pre-motion
   stability frames, all valid/plausible). 0 invalid in total.
9. **Did any point in the trajectory freeze despite subsequent DIFFERENT targets?** No. 0 freeze
   windows (window test: <0.5° measured over 1 s while >3° commanded). Worst 1-s windows were the
   startup transient at idx 20 (0.79–2.38°/s — still moving). The longest sub-0.3°/step measured
   runs (14/19 steps) occurred near the arc ends where commanded motion itself is slow; over those
   windows measured motion matched commanded (4.02 vs 4.06°/s forward; 3.70 vs 3.92°/s reverse) —
   feedback quantization (0.088°/LSB), not a freeze. Measured base joint vs commanded base
   correlation: 0.995 (F), 0.987 (R) — ordered progression throughout.
10. **Max commanded-vs-measured joint tracking error:** 3.48° forward, 5.05° reverse (bounded lag,
    no divergence).
11. **Final static joint error:** forward max 2.31° (elbow), vs its commanded end waypoint;
    reverse max 2.28° (elbow), vs the start pose. Consistent with the known ~2° static residual.
12. **Measured XYZ range:** X [243.8, 296.0], Y [90.0, 167.3] mm (XY tracked the arc), Z readout
    [53.68, 80.98] mm — the firmware Z channel again dropped up to ~27 mm below the commanded
    constant Z in this near-extended configuration (same firmware forward-kinematics artifact
    documented in the WSL report; recorded, not debugged).
13. **Comparison to 3974bdc WSL result:** identical control method and trajectory size.
    WSL: 202/202 commands, 202/202 feedback, forward 7.18 s / reverse 7.37 s (≈13.9 Hz achieved),
    max tracking 3.55°/3.87°, final static 2.57°/3.08°.
    Windows: 202/202 commands, 202/202 feedback, forward 5.00 s / reverse 5.00 s
    (**19.988 Hz achieved ≈ nominal 20 Hz** — tighter pacing than the WSL run), max tracking
    3.48°/5.05°, final static 2.31°/2.28°. Zero transport/execution failures on both platforms;
    the historical Windows Z-sweep "moves once then ignores all subsequent different targets"
    behaviour was **not** reproduced.

## Verdict

**A. Windows performs equivalently to WSL for streamed control.**

The exact streamed Cartesian pattern that worked in WSL executed the full 80 mm / 75° / 101-waypoint
arc on native Windows with the stock SDK: every one of the 202 different successive T=102 targets was
acted upon, feedback stayed valid throughout, pacing hit the nominal 20 Hz (slightly better than the
WSL run), and tracking/static errors are the same magnitude as WSL. No RTS/DTR secondary test was
needed (per protocol: only if the first stock run fails).
