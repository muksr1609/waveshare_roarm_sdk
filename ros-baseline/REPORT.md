<!-- Recovered 2026-08-25 from the OpenCode session transcript
     (ses_fc76724ffffeadKBSp5zL6uMSY, 135 messages). The session ran out of
     context before it could commit this itself. Raw transcript and all run
     logs are alongside this file. -->

# RoArm-M3 Baseline Test Report — Waveshare Stock Stack

## Environment

| Item | Value |
|---|---|
| OS | Windows 11 Pro (build 26200), WSL2 |
| ROS | **ROS2 Humble** on a freshly installed **Ubuntu 22.04.5 LTS** WSL distro (Waveshare's documented target; your existing Ubuntu-24.04 distro and Ro-Arm project were not touched) |
| `roarm_ws` commit | `40dbd84b553695212fab713e8465f817ba95454d` (official HEAD, `ros2-humble` branch, 2025-07-25) |
| Connection | Physical USB CP210x (VID `10C4:EA60`, was COM21 on Windows) → **usbipd-win 5.3.0** → `/dev/ttyUSB0` in WSL, 115200 baud |
| `ROARM_MODEL` | `roarm_m3` (exported in every launch shell and in `.bashrc`) |

Infra notes (not stack modifications): `python3-pip` was missing on the fresh distro (installed); the launch file reads `os.environ["DISPLAY"]` — WSLg supplied it and rviz2 ran for real; `wsl.exe` uses the Windows cwd, so all commands were run from `/home/mukund` to guarantee only Waveshare's pip `roarm_sdk` (v0.1.0) was imported (one early probe accidentally imported your project's `roarm_sdk` from the cwd — caught immediately, it only attempted to *open* a nonexistent port, no motion involved).

## Build / startup

- **Untouched official source; `build_first.sh` commands verbatim** (only the hardcoded `/home/ws` path adapted to `/home/mukund`).
- **11/12 packages first pass.** `roarm_moveit_servo` (keyboard-servo package; **not** in the `command_control.launch.py` path) failed:

```
CMake Error at CMakeLists.txt:17 (find_package):
  By not providing "Findroarm_msgs.cmake" in CMAKE_MODULE_PATH this project
  has asked CMake to find a package configuration file provided by
  "roarm_msgs", but CMake did not find one.
...
Failed   <<< roarm_moveit_servo [8.67s, exited with code 1]
```

  Cause: `build_first.sh` runs separate `colcon build` invocations without sourcing the workspace install space. After sourcing `install/setup.bash` (standard ROS workflow, zero code change) it built cleanly in 12 s. **Final: 12/12.**
- pip requirements installed cleanly (`roarm-sdk==0.1.0`, `simplejson`, pyserial 3.5 preinstalled).
- `command_control.launch.py` started cleanly: move_group ("You can start planning now!"), ros2_control + spawners, `roarmserver`, `setgrippercmd`, rviz2. Services present: `/get_pose_cmd`, `/move_joint_cmd`, `/move_line_cmd`, `/move_circle_cmd`.
- **At launch the arm moved** to the documented initial pose (README §7: "forearm extending forward and parallel to the horizontal plane") — commanded `{"base":0,"shoulder":0,"elbow":2.618,"wrist":-1.0472,"roll":0,"hand":π,"spd":1000,"acc":50}`; the arm's own servo feedback tracked it (e=2.620 vs 2.618, wrist −1.032 vs −1.0472).
- Comms health: `roarm_sdk.feedback_get()` returned clean JSON, no NaNs, no reconnect loops, torque all ON, battery 12.13 V. (JSON-corruption errors seen once in the driver log at 12:51:48 were caused by **my** concurrent probe sharing the port — "multiple access on port?" — not by the stack.)
- Transient `"A message was lost!!!"` DDS warnings appeared a few times on `ros2 topic echo`; no functional impact observed.

## Circle test (stock command, 5 runs)

`ros2 service call /move_circle_cmd roarm_msgs/srv/MoveCircleCmd "{x0: 0.2, y0: 0.1, z0: 0.2, x1: 0.2, y1: 0.2, z1: 0.2}"`

| Run | Service response | Final pose (mock FK, m) |
|---|---|---|
| 1 | `success=True, 'MoveCircleCmd executed successfully'` | (0.19999, 0.19999, 0.19999) |
| 2 | `success=True` | (0.20000, 0.20002, 0.19999) |
| 3 | `success=True` | (0.20000, 0.20002, 0.19997) |
| 4 | `success=True` | (0.20001, 0.19997, 0.20004) |
| 5 | `success=True` | (0.19999, 0.20001, 0.20002) |

- **Planning: 50/50 waypoint plans succeeded, 0 "Planning failed".**
- **Execution: 4 segments timed out** — the *first* segment of runs 1, 2, and 4. The other 46/50 completed. Critically, the stock server **ignores `execute()`'s return code** and reports success from `plan()` alone, so `success=True` overstates what happened:

```
[WARN] [moveit.simple_controller_manager.follow_joint_trajectory_controller_handle]: waitForExecution timed out
[ERROR] [moveit_ros.trajectory_execution_manager]: Controller is taking too long to execute trajectory (the expected upper bound for the trajectory execution was 0.500000 seconds). Stopping trajectory.
[ERROR] [move_group_interface]: MoveGroupInterface::execute() failed or timeout reached
```

- **Physical motion: real and recognizable as an arc.** The driver streamed a smooth, continuous joint progression (base 0 → 0.7854 rad = 45° = atan2(0.2,0.2); elbow 2.618 → 1.9886; shoulder 0 → −0.2584; wrist −1.0472 → −0.1595). No violent discontinuities, no IK jumps between waypoints, no oscillation.
- **Speed/character:** each arc took ~6–9 s, executed as 10 chained point-to-point segments (stock design: 10-waypoint arc, per-waypoint closed-form IK, per-waypoint OMPL plan+execute) — fast and visibly segmented, not a smooth continuous Cartesian path.
- **Z was NOT maintained constant** — by design of this example: the 3-point circle passes through the start pose (z≈0.098 m), the via point (z=0.2) and the end (z=0.2), so the tool rises ~10 cm along the arc. Only via/end are at 0.2 m.
- **Endpoint accuracy (real arm, from its own servo feedback):** after run 1 the arm was at (205.8, 193.5, **179.5**) mm vs commanded (200, 200, 200) — ~26 mm off, mostly **z ≈ 19–20 mm low**. After run 5: (204.9, 192.7, 180.7) mm. The bias is systematic: at the home pose Waveshare's FK reports z=98.0 mm while the arm's own feedback says 75.4 mm (~23 mm high) — their FK/IK geometry (solver.hpp link lengths) doesn't exactly match this arm's calibration.
- **Repeatability: excellent.** All 5 runs landed within 0.5 mm of each other (mock) and within ~1 mm of the run-1 endpoint (real arm); joint states agreed to <0.0003 rad. No drift accumulation. Returns between runs used stock `/move_joint_cmd "{x: 0.2739, y: 0.0, z: 0.098, roll: 0.0, pitch: 0.0, yaw: 0}"` (all `success=True`).

## Conclusion

**2. Waveshare stock stack works, but physical motion quality is poor.**

The reference implementation, as delivered, does communicate reliably with the arm, plans the example 50/50, and the physical arm traces a recognizable, highly repeatable arc. But: the service's `success` flag is unreliable (execution failures ignored), the first segment of a run intermittently times out in the execution monitor, the "fixed-height" arc does not hold Z constant, motion is fast and segmented rather than smooth, and endpoint accuracy is ~2–3 cm with a systematic ~20 mm z bias between their FK/IK model and the arm's actual calibration. None of this was fixed or worked around — that is the delivered baseline.

## Handoff state

- **Running now:** Ubuntu-22.04 WSL (default distro) with the full `command_control` launch (move_group, rviz2 — a window may be visible on your desktop via WSLg, ros2_control, roarmserver, setgrippercmd) and `roarm_driver` (pid 1774, as mukund). The arm sits at the arc endpoint (torque on, holding).
- **Emergency stop** at any time: the arm's power switch, or unplug its USB. (The stock stack has no software e-stop.)
- **Stop everything:** `wsl -d Ubuntu-22.04 -u root -e "pkill -f command_control; pkill -f move_group; pkill -f ros2_control_node; pkill -f lib/roarm_driver; pkill -f rviz2"` (run from WSL; patterns avoid matching your own shell since they target the node binaries).
- **Give COM21 back to Windows:** elevated `& "C:\Program Files\usbipd-win\usbipd.exe" detach --busid 5-1`.
- **Logs** (in WSL `/home/mukund/`): `roarm_setup.log`, `roarm_build.log`, `roarm_servo_retry.log`, `roarm_comms.log`, `roarm_driver.log`, `roarm_moveit_cmd.log`, `roarm_circle_run1.log`, `roarm_circle_runs2to5.log`.
- Your `C:\Users\Mukund\Documents\Ro-Arm` project was not modified; `roarm_ws` lives at `/home/mukund/roarm_ws` in the WSL distro.
