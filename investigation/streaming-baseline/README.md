# streaming-baseline: T=102 joint-command transport experiment

Controlled experiment to determine why sparse one-shot `T=102`
(`joints_radian_ctrl`) commands behaved unreliably in the earlier Windows
Z-sweep while Waveshare's stock ROS2 stack drove the same physical
RoArm-M3-S successfully.

Run **2026-08-25** in the known-good environment: WSL2 Ubuntu 22.04,
stock `roarm-sdk==0.1.0` (user site-packages), `/dev/ttyUSB0` (CP210x),
115200 baud, raw Python — no ROS, no MoveIt, no repo-local SDK, no firmware
changes.

## Files

| File | Purpose |
|------|---------|
| `run_tests.py` | The experiment. Repeatable: baseline + pose selection + Tests A-D, safety checks, permanent logging. |
| `launch.sh` | Detached launcher for WSL (`setsid nohup`). |
| `probe_feedback.py` | Safe pre-flight: connect + 5 feedback reads, no motion commands. |
| `results.json` | Machine-readable results (written incrementally by `run_tests.py`). |
| `raw.log` | Permanent log: every command, timestamp, and feedback sample. |
| `REPORT.md` | Full analysis, hypothesis verdicts, and recommendation. |
| `analyze.py`, `analyze2.py`, `analyze3.py`, `summarize_xyz.sh` | Read-only analysis helpers used for REPORT.md. |

## What it does

1. Guards that the **stock** SDK is imported (refuses repo-local path).
2. Verifies `/dev/ttyUSB0` exists and no other process holds it.
3. Samples idle feedback for 3 s (must be >=80 % success, finite, in limits).
4. Picks pose A = current physical state; pose B = A with base -5 deg,
   wrist +3 deg (sign chosen away from nearest limit, gripper untouched).
   The **same** A/B are used in every test.
5. Runs four tests, all on the same A/B:
   - **A** one-shot T=102, speed=180 acc=10, 10 transitions, 15 s monitor
   - **B** one-shot T=102, speed=1000 acc=50, identical procedure
   - **C** same final target repeated @10 Hz for <=2 s or until motion, 10 transitions
   - **D** joint-space linear interpolation streamed @20 Hz (1.5 s legs,
     speed=1000 acc=50), 5 cycles of A->B->A
6. A command counts as **ACCEPTED only if physical feedback shows measured
   motion toward the target** (>0.01 rad). SDK return values are never used
   as evidence. Every raw sample is logged.

## Re-running

```bash
# 1. make sure no ROS driver / other process holds the port
wsl -d Ubuntu-22.04 -u mukund -- bash -c "fuser /dev/ttyUSB0"

# 2. copy the script into WSL and launch detached
wsl -d Ubuntu-22.04 -u mukund -- bash -c "
  mkdir -p /home/mukund/streaming_baseline
  cp /mnt/c/<repo>/investigation/streaming-baseline/{run_tests.py,launch.sh} /home/mukund/streaming_baseline/
  bash /home/mukund/streaming_baseline/launch.sh"

# 3. watch progress
wsl -d Ubuntu-22.04 -u mukund -- tail -f /home/mukund/streaming_baseline/raw.log
```

Expected duration ~25 min. The arm only makes the small base/wrist moves
between poses A and B (max 7 deg deliberate change on any joint, gripper
never moved).
