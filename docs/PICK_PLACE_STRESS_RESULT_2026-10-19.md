# Pick & Place Validation Result — 2026-10-19

## Scope and command

Ran the simulator's real mission stress mode with a fixed seed:

```powershell
.\grasplink_simulator.exe --pick-place-stress 50 --seed 1
```

The executable was `build-ninja-debug/grasplink_simulator.exe`, a prebuilt native Debug binary dated 2026-10-15. This run used the simulator mission, robot controller, Jolt physics, collision checks, and gripper path. It is a Debug-build result and should not be compared as a release performance benchmark.

## Result

| Seed | Attempts | Successes | Failures | Timeouts | Success rate | Elapsed |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 50 | 50 | 0 | 0 | 100% | 210.76 s |

The simulator exited with code `0`. Every attempted mission completed successfully; there were no failure or timeout cases to analyze in this run.

## Focused test executables

The following prebuilt Debug test executables also passed:

| Test executable | Passed |
| --- | ---: |
| `grasplink_inverse_kinematics_tests.exe` | 8 |
| `grasplink_linear_path_planner_tests.exe` | 8 |
| `grasplink_joint_path_planner_tests.exe` | 7 |
| `grasplink_robot_motion_tests.exe` | 32 |
| `grasplink_pick_place_mission_tests.exe` | 18 |
| `grasplink_physics_tests.exe` | 7 |

These were run with GoogleTest's `--gtest_brief=1` option from `build-ninja-debug`. They are evidence for those binaries; a fresh build of the current checkout was not performed for this report.
