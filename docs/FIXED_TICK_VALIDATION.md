# Fixed 4 ms simulation tick: execution map and deadline validation

**Status:** the simulator uses a **4 ms simulation time step**, not a guaranteed real-time 250 Hz scheduler.

## Current execution path

`ViewerApp::MainLoopFrame` is called by the desktop frame loop or Emscripten browser callback. When planning is active it calls `AdvancePlanningBudget` (nominal per-frame deadline 1.5 ms) and **does not advance the control loop**. Otherwise it calls `FixedControlLoop::Advance(frameDeltaSeconds, callback)`. That accumulator can execute zero or multiple 4 ms ticks per render frame. It drops accumulated time beyond 100 ms per frame; there is no 4 ms OS timer or guaranteed wall-clock period.

`ViewerApp::AdvanceSimulationTick(0.004)` does, in order:

1. Grasp adapter `BeforePhysicsStep`: check release and body/constraint validity.
2. Collision guard `CaptureSafeJointPose`: save last accepted robot joint positions.
3. `RobotUpdate` zone: robot controller `Update`, gripper controller `Update`, then `ApplyControllerPoses` (FK, robot visual and physics adapter, gripper mimic and transform).
4. `TransformSystemModule::UpdateWorldTransforms`: recompute Local→World hierarchy.
5. `RestoreSafePoseIfOverlapping`: Jolt-backed environment and self-collision checks, restore safe joints if overlapping.
6. `PhysicsStep` zone / `PhysicsSystemModule::Step`:
   - PrePhysicsSync: deferred collider-body recreation, Static/Kinematic scene-to-Jolt sync.
   - `PhysicsWorld::Step`: Jolt solver and parallel worker jobs where enabled.
   - PostPhysicsSync: write Dynamic Jolt pose to Scene Local.
7. Grasp adapter `AfterPhysicsStep`: contacts, finger stop, grasp constraint.
8. Recompute World transforms for rendering.

Outside the fixed tick, the frame loop updates mission logic, cameras, input, world transforms, OpenGL and ImGui. The mission is **not fully a fixed-tick system**.

## What must be measured

1. **Tick work duration:** monotonic start/end of `AdvanceSimulationTick`, including all eight steps, p50/p95/p99/max and count above **4.000 ms**. This detects execution-budget misses, not periodic timing guarantees.
2. **Wall-clock tick start interval:** adjacent tick start timestamps and actual wall/simulation-time ratios. Consecutive catch-up ticks can start less than 4 ms apart despite a nominal 4 ms simulation step.
3. **Frame and accumulator:** frame delta, ticks executed per frame, dropped/clamped time, maximum accumulator depth, and all periods with `IsMotionPlanning` true. Report simulation pauses separately.
4. **Stage breakdown:** Tracy `Frame`, `FixedTick`, `RobotUpdate`, `CollisionCheck`, `SelfCollisionCheck`, `PhysicsStep` and `Render`, plus finer segments for grasp and ECS↔Jolt sync if needed.
5. **Scenario and platform:** Release native build with Tracy capture; desktop idle, MoveJ, MoveL, Pick-and-Place, 10–100 Dynamic objects, and load-induced frame drops. Repeat with browser Pthreads and use browser timing/profiling, since Tracy is disabled for Emscripten by CMake.

Useful ratios: `budget_miss_ratio = ticks_where_duration_gt_4ms / observed_ticks`, `sim_speed = advanced_simulated_seconds / elapsed_wall_seconds`, `planning_pause_seconds`, and `max_consecutive_budget_misses`.

A Tracy screenshot can demonstrate how much CPU time was spent in each measured zone, but cannot by itself prove a hard real-time guarantee or that every 4 ms period was met. A deterministic headless step benchmark proves throughput, not wall-clock periodicity.

**Evidence policy:** do not write “250 Hz guaranteed” in the portfolio without a separately scheduled control loop, deadline instrumentation, and representative long-running measurements.
