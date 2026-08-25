# ROS 2 baseline evidence — Waveshare stock stack

Recovered from an OpenCode session that completed the work but ran out of context
before committing it. Nothing here is a modification to Waveshare's stack; it is
the record of running the **stock** stack against the physical arm.

| file | what it is |
|---|---|
| `REPORT.md` | the baseline test report: environment, build, 5 circle runs, findings |
| `logs/roarm_circle_run1.log`, `logs/roarm_circle_runs2to5.log` | the 5 circle-test runs — the primary evidence |
| `logs/roarm_moveit_cmd.log` | move_group / MoveIt output, including the execution timeouts |
| `logs/roarm_driver.log` | raw serial driver stream (joint progression during motion) |
| `logs/roarm_build.log`, `logs/roarm_setup.log`, `logs/roarm_pip.log` | build and install provenance |
| `logs/roarm_comms.log`, `logs/roarm_servo_retry.log` | comms health checks |
| `transcript-ros-session.json` | full 135-message session transcript |
| `transcript-zsweep-session.json` | the separate z-sweep / command-drop session (see `HANDOFF.md`) |

## Headline findings

- Stock stack **builds and runs** (12/12 packages after sourcing the install space) and the
  arm traces a **recognisable, highly repeatable arc** — 5 runs within ~1 mm.
- **`success=True` is unreliable.** The stock service reports success from `plan()` alone and
  **ignores `execute()`'s return code**. 4 of 50 segments timed out in the execution monitor
  while still reporting success.
- **The "fixed-height" circle does not hold Z constant** — by design of the example; only the
  via and end points are at z=0.2.
- **~20 mm systematic Z bias** between Waveshare's FK/IK geometry (`solver.hpp` link lengths)
  and this arm's actual calibration.

## Reproducing

The workspace lives at `/home/mukund/roarm_ws` inside a **WSL Ubuntu-22.04** distro created
for this test (ROS 2 Humble, Waveshare's documented target). The pre-existing Ubuntu-24.04
distro and this repository were not modified. USB reached WSL via usbipd-win 5.3.0
(CP210x `10C4:EA60`, COM21 on Windows → `/dev/ttyUSB0`).

Note: that distro grants `mukund` passwordless sudo (`/etc/sudoers.d/mukund`), created during
automated setup.
