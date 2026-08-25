# ARC200 run: 200 mm / 270 deg request, auto-shrunk to 150 mm / 202.5 deg

**Date:** 2026-08-25
**Request:** 200 mm radius, 270 deg, constant-Z arc from the arm's current pose.
**Environment:** same known-good setup as the first streamed-Cartesian run (WSL2, `/dev/ttyUSB0`,
stock `roarm-sdk==0.1.0`, Waveshare upstream IK, 20 Hz stream, speed=1000 acc=50).
Evidence: `arc200-result.json`, `arc200.log`. Script: `streamed-cartesian-demo.py`
(now with `--radius-mm`, `--sweep-deg`, `--max-excursion-deg` options; adaptive waypoint count
keeps steps <= 5 mm).

## What happened

1. **200 / 270 failed offline validation at all 8 candidate placements** (4 center offsets x 2
   sweep directions) before any motion was sent — the 3/4 loop of radius 200 mm around the
   current pose (tip at 285 mm from base, low z) necessarily passes either too close to the base
   column (unreachable) or outside the 3D reach envelope at that height. Per the agreed
   auto-shrink rule the script stepped down.
2. **Executed arc (first validating candidate, 0.75x): R = 150 mm, sweep = 202.5 deg,
   path length 530.1 mm**, center (224.8, 24.9), sweep -, 107 waypoints @ 20 Hz (5.30 s stream).
   Constant Z = 127.12 mm (start pose); pitch/roll/gripper unchanged.
   Bbox x [167.4, 374.8] y [-125.1, 174.9] z [127.12, 127.12].
   Joint excursions: base 72.1 deg, shoulder 43.4, elbow 39.3, wrist 9.3.
   Max adjacent step 1.43 deg (no branch jumps).
3. **Forward leg:** 107/107 commands, 107/107 feedback OK, 0 invalid. Tip swept a 202.5 deg arc
   to (175.3, -113.6) mm. Max tracking error 5.91 deg.
4. **Reverse leg:** 107/107 commands, 107/107 feedback OK, 0 invalid. Tip returned to
   (235.1, 172.1) mm, 11.6 mm from the start pose. Max tracking error 5.64 deg.
5. **Measured Z range: 93.55 - 127.12 mm** — the firmware Z readout again deviated from the
   commanded constant Z, this time by up to ~34 mm in the extreme part of the loop
   (shoulder -29 deg / elbow 135 deg region). Reconfirms: firmware Z is unreliable in these
   configurations; XY is what tracks the arc.
6. Final joint error after reverse vs start pose: [1.67, 3.86, 0.35, 0.44, 0.00] deg — the usual
   static residual, not chased. Arm left parked near its pre-run position.

## Takeaways

- The planner's offline gate did its job: the impossible 200/270 request never reached the robot;
  the largest safe arc from that pose (530 mm, 202.5 deg) was executed instead, both directions,
  with zero transport/execution failures.
- In-motion lag grows with path size (5.9 deg vs 3.9 deg in the 105 mm run) — still bounded.
- For future larger arcs: from this family of low/near-extended poses, 150/202 is near the
  validated ceiling; other (higher) base configurations would allow different loops.
