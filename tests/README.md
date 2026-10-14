# Test suites by module

The original regression tests have been **moved without deleting assertions**. Test files are grouped by subsystem instead of being kept in one flat directory.

| Directory | Scope | CTest label |
| --- | --- | --- |
| `tests/robotics/` | controllers, FK/IK, motion, path planners | `robotics` |
| `tests/physics/` | standalone Jolt world | `physics` |
| `tests/scene/` | transforms and quaternions | `scene` |
| `tests/model/` | GLB parsing | `model` |
| `tests/application/` | pick-and-place mission and scenario sampling | `application` |
| `tests/integration/` | ECS–Jolt, robot / gripper / collision adapters | `integration` |
| `tests/viewer/` | graphics resources | `viewer` |
| `tests/gui/` | GUI composition | `gui` |
| `tests/TestSupport.h` | shared integration fixtures | (shared) |

Run focused regressions with `ctest --test-dir build-ninja-debug -L robotics --output-on-failure` or `ctest --test-dir build-ninja-debug -L integration --output-on-failure`.

The `e2e` label contains the native Viewer smoke check. The `benchmark` label is **not a deadline test**: it verifies benchmark execution and numerical invariants.

## Testing policy

- Keep tests for **contracts and failures that matter**: FK/IK correctness, joint limits, collision rejection, ECS↔Jolt authority and lifetime, grasp/release, MoveJ/MoveL acceptance, and end-to-end pick-and-place.
- Merge redundant setup into small fixtures and parameterized tests. Do **not** remove meaningful safety regression coverage to reduce the test count.
- A deterministic algorithm unit test does not prove runtime responsiveness. Keep profiling, stress, and UI/browser integration tests distinct.
- Test source relocation is structural only; compile/CTest must be re-run before merging this branch.
