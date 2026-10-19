#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/planning/JointPathPlanner.h"
#include "physics/PhysicsWorld.h"

#include <benchmark/benchmark.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <random>
#include <sstream>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;
using namespace grasplink::robotics::models;
using namespace grasplink::robotics::models::hanwha;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;

const JointVector kSeed(6, 0.0);
const JointVector kTargetJoints{0.04, -0.05, 0.03, 0.02, -0.03, 0.04};

CartesianPose MakeTargetPose()
{
    DampedLeastSquaresIk solver(kHcr12a);
    return solver.EvaluateTcp(kTargetJoints);
}

void Fk(benchmark::State& state)
{
    RobotKinematics forward(kHcr12a);
    for (auto _ : state)
    {
        const auto& result = forward.Update(kSeed);
        benchmark::DoNotOptimize(result.toolFrameInBaseFrame.positionMeters);
        benchmark::DoNotOptimize(result.toolFrameInBaseFrame.rotation);
    }
}

void BenchmarkDlsIk(benchmark::State& state)
{
    DampedLeastSquaresIk solver(kHcr12a);
    const auto target = MakeTargetPose();
    const auto validation = solver.Solve(target, kSeed);
    if (!validation)
    {
        state.SkipWithError("DLS IK benchmark target did not converge during validation");
        return;
    }

    for (auto _ : state)
    {
        const auto result = solver.Solve(target, kSeed);
        if (!result)
        {
            state.SkipWithError("DLS IK benchmark failed to converge");
            break;
        }
        benchmark::DoNotOptimize(result.jointPositionRadians.data());
        benchmark::DoNotOptimize(result.positionErrorMeters);
    }
}

void LinearPathPlanning(benchmark::State& state)
{
    DampedLeastSquaresIk solver(kHcr12a);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(MakeTargetPose());
    const CartesianPose startTcp = solver.EvaluateTcp(kSeed);

    LinearPathPlan plan;
    const auto validation = BuildLinearPath(
        command, kHcr12a, kSeed, startTcp, solver, {}, plan);
    if (!validation)
    {
        state.SkipWithError("LinearPathPlanner benchmark path failed validation");
        return;
    }

    std::size_t ikSolveCount = 0;
    std::size_t ikIterationCount = 0;
    std::size_t refinementCount = 0;
    std::size_t validityStateChecks = 0;
    std::size_t straightnessChecks = 0;
    for (auto _ : state)
    {
        const auto result = BuildLinearPath(
            command, kHcr12a, kSeed, startTcp, solver, {}, plan);
        if (!result)
        {
            state.SkipWithError("LinearPathPlanner failed to build the benchmark path");
            break;
        }
        benchmark::DoNotOptimize(plan.points.data());
        benchmark::DoNotOptimize(plan.points.size());
        ikSolveCount += plan.ikSolveCount;
        ikIterationCount += plan.ikIterationCount;
        refinementCount += plan.tcpRefinementCount;
        validityStateChecks += plan.validityStateChecks;
        straightnessChecks += plan.tcpStraightnessChecks;
    }
    state.SetItemsProcessed(state.iterations() * static_cast<std::int64_t>(plan.points.size()));
    state.counters["ik_solves_per_path"] = static_cast<double>(ikSolveCount) / state.iterations();
    state.counters["ik_iterations_per_path"] = static_cast<double>(ikIterationCount) / state.iterations();
    state.counters["tcp_refinements_per_path"] = static_cast<double>(refinementCount) / state.iterations();
    state.counters["planner_state_checks_per_path"] = static_cast<double>(validityStateChecks) / state.iterations();
    state.counters["tcp_straightness_checks_per_path"] = static_cast<double>(straightnessChecks) / state.iterations();
}

void IncrementalLinearPathPlanning(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;
    using namespace grasplink::robotics::kinematics::detail;

    DampedLeastSquaresIk solver(kHcr12a);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(MakeTargetPose());
    const CartesianPose startTcp = solver.EvaluateTcp(kSeed);
    std::vector<double> sliceMicroseconds;
    std::vector<double> workFramesPerPath;
    std::size_t succeeded = 0;
    std::size_t plannerStateChecks = 0;
    double maximumPositionErrorMeters = 0.0;
    double maximumOrientationErrorRadians = 0.0;
    std::size_t totalWorkFrames = 0;

    for (auto _ : state)
    {
        LinearPathPlanningJob job;
        if (!job.Begin(command, kHcr12a, kSeed, startTcp, solver, {},
            grasplink::robotics::planning::kInteractivePlanningPolicy))
        {
            state.SkipWithError("incremental planner could not begin the benchmark path");
            break;
        }

        std::size_t workFrames = 0;
        while (job.GetState() == LinearPathPlanningState::Running && workFrames < 4096)
        {
            const auto frameStart = Clock::now();
            const auto frameDeadline = frameStart + std::chrono::microseconds(1500);
            do
            {
                job.Advance(1);
            } while (job.GetState() == LinearPathPlanningState::Running && Clock::now() < frameDeadline);
            sliceMicroseconds.push_back(std::chrono::duration<double, std::micro>(
                Clock::now() - frameStart).count());
            ++workFrames;
        }
        if (job.GetState() != LinearPathPlanningState::Completed)
        {
            state.SkipWithError("incremental planner did not complete a valid benchmark path");
            break;
        }

        const auto& plan = job.GetPlan();
        for (std::size_t point = 1; point < plan.points.size(); ++point)
        {
            const auto& start = plan.points[point - 1];
            const auto& end = plan.points[point];
            const Pose3 desiredStart = FromCartesian(start.tcpPose);
            const Pose3 desiredEnd = FromCartesian(end.tcpPose);
            JointVector joints(start.joints.size());
            for (const double fraction : {0.25, 0.5, 0.75})
            {
                for (std::size_t joint = 0; joint < joints.size(); ++joint)
                    joints[joint] = start.joints[joint] +
                        (end.joints[joint] - start.joints[joint]) * fraction;
                const Pose3 actual = FromCartesian(solver.EvaluateTcp(joints));
                const Pose3 desired = Interpolate(desiredStart, desiredEnd, fraction);
                maximumPositionErrorMeters = std::max(maximumPositionErrorMeters,
                    Length(Subtract(actual.positionMeters, desired.positionMeters)));
                maximumOrientationErrorRadians = std::max(maximumOrientationErrorRadians,
                    Length(RotationError(desired.rotation, actual.rotation)));
            }
        }

        plannerStateChecks += plan.validityStateChecks;
        totalWorkFrames += workFrames;
        workFramesPerPath.push_back(static_cast<double>(workFrames));
        ++succeeded;
        benchmark::DoNotOptimize(plan.points.data());
    }

    const auto percentile = [](std::vector<double>& values, double fraction)
    {
        if (values.empty())
            return 0.0;
        std::sort(values.begin(), values.end());
        const std::size_t index = static_cast<std::size_t>(std::ceil(
            fraction * static_cast<double>(values.size()))) - 1;
        return values[std::min(index, values.size() - 1)];
    };
    if (!sliceMicroseconds.empty())
    {
        state.counters["slice_p50_us"] = percentile(sliceMicroseconds, 0.50);
        state.counters["slice_p95_us"] = percentile(sliceMicroseconds, 0.95);
        state.counters["slice_p99_us"] = percentile(sliceMicroseconds, 0.99);
        state.counters["slice_max_us"] = *std::max_element(sliceMicroseconds.begin(), sliceMicroseconds.end());
    }
    if (succeeded != 0)
    {
        const double averageWorkFrames = static_cast<double>(totalWorkFrames) / succeeded;
        const double workFramesP50 = percentile(workFramesPerPath, 0.50);
        const double workFramesP95 = percentile(workFramesPerPath, 0.95);
        const double workFramesP99 = percentile(workFramesPerPath, 0.99);
        const double workFramesMax = *std::max_element(workFramesPerPath.begin(), workFramesPerPath.end());
        constexpr double millisecondsPerFrameAt60Hz = 1000.0 / 60.0;
        state.counters["success_rate"] = static_cast<double>(succeeded) / state.iterations();
        state.counters["work_frames"] = static_cast<double>(totalWorkFrames);
        state.counters["work_frames_per_path"] = averageWorkFrames;
        state.counters["work_frames_p50"] = workFramesP50;
        state.counters["work_frames_p95"] = workFramesP95;
        state.counters["work_frames_p99"] = workFramesP99;
        state.counters["work_frames_max"] = workFramesMax;
        state.counters["estimated_latency_ms_at_60hz"] = averageWorkFrames * millisecondsPerFrameAt60Hz;
        state.counters["estimated_latency_ms_at_60hz_p50"] = workFramesP50 * millisecondsPerFrameAt60Hz;
        state.counters["estimated_latency_ms_at_60hz_p95"] = workFramesP95 * millisecondsPerFrameAt60Hz;
        state.counters["estimated_latency_ms_at_60hz_p99"] = workFramesP99 * millisecondsPerFrameAt60Hz;
        state.counters["estimated_latency_ms_at_60hz_max"] = workFramesMax * millisecondsPerFrameAt60Hz;
        state.counters["planner_state_checks_per_path"] = static_cast<double>(plannerStateChecks) / succeeded;
        state.counters["max_fk_tcp_position_error_mm"] = maximumPositionErrorMeters * 1000.0;
        state.counters["max_fk_tcp_orientation_error_mrad"] = maximumOrientationErrorRadians * 1000.0;
    }
}

struct RuntimePathCase
{
    CartesianPose targetPose;
    JointVector reachableTargetJoints;
    std::size_t endpointIkIterations = 0;
    bool endpointIkConvergedFromStart = false;
    bool directJointPathValid = false;
};

std::vector<RuntimePathCase> RuntimePathCases(std::size_t caseCount)
{
    std::mt19937_64 random(0x47524153504C494EULL);
    DampedLeastSquaresIk inverse(kHcr12a);
    std::vector<RuntimePathCase> generated;
    generated.reserve(caseCount);
    for (std::size_t sample = 0; sample < caseCount; ++sample)
    {
        JointVector targetJoints(kHcr12a.jointCount);
        for (std::size_t joint = 0; joint < kHcr12a.jointCount; ++joint)
        {
            const auto& limits = kHcr12a.joints[joint];
            std::uniform_real_distribution<double> position(
                limits.minPositionRadians * 0.8, limits.maxPositionRadians * 0.8);
            targetJoints[joint] = position(random);
        }
        RuntimePathCase testCase;
        testCase.targetPose = inverse.EvaluateTcp(targetJoints);
        testCase.reachableTargetJoints = std::move(targetJoints);
        const IkResult endpoint = inverse.SolveSingleSeed(testCase.targetPose, kSeed);
        testCase.endpointIkIterations = endpoint.iterations;
        testCase.endpointIkConvergedFromStart = endpoint.Ok();
        testCase.directJointPathValid = ValidateJointPath(
            kSeed, testCase.reachableTargetJoints, kHcr12a, {}) == JointStateInvalidity::None;
        generated.push_back(std::move(testCase));
    }
    return generated;
}

void InteractiveLinearPathStress(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;
    using namespace grasplink::robotics::kinematics::detail;

    constexpr auto kFramePlanningBudget = std::chrono::microseconds(1500);
    constexpr std::size_t kMaximumPlanningFrames = 100000;
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    DampedLeastSquaresIk inverse(kHcr12a);
    const CartesianPose startTcp = inverse.EvaluateTcp(kSeed);
    std::vector<double> frameSliceMicroseconds;
    std::uint64_t plannerRuns = 0;
    std::uint64_t completed = 0;
    std::uint64_t rejected = 0;
    std::uint64_t timedOut = 0;
    std::uint64_t stateValidityCallbackCalls = 0;
    std::uint64_t totalWorkFrames = 0;
    std::uint64_t maximumWorkFrames = 0;
    std::uint64_t ikSolves = 0;
    std::uint64_t ikIterations = 0;
    std::uint64_t tcpRefinements = 0;
    std::uint64_t tcpStraightnessChecks = 0;
    std::uint64_t jointPathValidityChecks = 0;
    std::uint64_t validityStateChecks = 0;
    double maximumTcpPositionErrorMeters = 0.0;
    double maximumTcpOrientationErrorRadians = 0.0;

    for (auto _ : state)
    {
        for (const auto& testCase : cases)
        {
            LinearPathMoveCommand command;
            command.targetPoses.push_back(testCase.targetPose);
            command.maxLinearVelocityMetersPerSecond = 2.0;
            command.maxAngularVelocityRadiansPerSecond = 8.0;
            command.maxLinearAccelerationMetersPerSecondSquared = 30.0;
            command.maxAngularAccelerationRadiansPerSecondSquared = 120.0;

            const StateValidityChecker dummyAlwaysValid = [&stateValidityCallbackCalls](const JointVector&)
            {
                ++stateValidityCallbackCalls;
                return JointStateInvalidity::None;
            };
            LinearPathPlanningJob job;
            const Result begin = job.Begin(command, kHcr12a, kSeed, startTcp, inverse,
                dummyAlwaysValid, kInteractivePlanningPolicy);
            if (!begin)
            {
                ++rejected;
                ++plannerRuns;
                continue;
            }

            std::size_t workFrames = 0;
            while (job.GetState() == LinearPathPlanningState::Running &&
                workFrames < kMaximumPlanningFrames)
            {
                const auto frameStart = Clock::now();
                const auto frameDeadline = frameStart + kFramePlanningBudget;
                do
                {
                    job.Advance(1);
                } while (job.GetState() == LinearPathPlanningState::Running &&
                    Clock::now() < frameDeadline);
                frameSliceMicroseconds.push_back(std::chrono::duration<double, std::micro>(
                    Clock::now() - frameStart).count());
                ++workFrames;
            }

            ++plannerRuns;
            totalWorkFrames += workFrames;
            maximumWorkFrames = std::max(maximumWorkFrames,
                static_cast<std::uint64_t>(workFrames));
            const auto& plan = job.GetPlan();
            ikSolves += plan.ikSolveCount;
            ikIterations += plan.ikIterationCount;
            tcpRefinements += plan.tcpRefinementCount;
            tcpStraightnessChecks += plan.tcpStraightnessChecks;
            jointPathValidityChecks += plan.jointPathValidityChecks;
            validityStateChecks += plan.validityStateChecks;

            if (job.GetState() == LinearPathPlanningState::Running)
            {
                ++timedOut;
                ++rejected;
                job.Cancel();
                continue;
            }
            if (job.GetState() != LinearPathPlanningState::Completed)
            {
                ++rejected;
                continue;
            }

            ++completed;
            for (std::size_t point = 1; point < plan.points.size(); ++point)
            {
                const auto& pathStart = plan.points[point - 1];
                const auto& pathEnd = plan.points[point];
                const Pose3 desiredStart = FromCartesian(pathStart.tcpPose);
                const Pose3 desiredEnd = FromCartesian(pathEnd.tcpPose);
                JointVector joints(pathStart.joints.size());
                for (const double fraction : {0.25, 0.5, 0.75})
                {
                    for (std::size_t joint = 0; joint < joints.size(); ++joint)
                        joints[joint] = pathStart.joints[joint] +
                            (pathEnd.joints[joint] - pathStart.joints[joint]) * fraction;
                    const Pose3 actual = FromCartesian(inverse.EvaluateTcp(joints));
                    const Pose3 desired = Interpolate(desiredStart, desiredEnd, fraction);
                    maximumTcpPositionErrorMeters = std::max(maximumTcpPositionErrorMeters,
                        Length(Subtract(actual.positionMeters, desired.positionMeters)));
                    maximumTcpOrientationErrorRadians = std::max(maximumTcpOrientationErrorRadians,
                        Length(RotationError(desired.rotation, actual.rotation)));
                }
            }
            benchmark::DoNotOptimize(plan.points.data());
        }
    }

    const double runCount = static_cast<double>(plannerRuns);
    state.counters["interactive_planner_runs"] = runCount;
    state.counters["interactive_completed"] = static_cast<double>(completed);
    state.counters["interactive_rejected"] = static_cast<double>(rejected);
    state.counters["interactive_timeouts"] = static_cast<double>(timedOut);
    state.counters["interactive_success_rate"] = runCount > 0.0
        ? static_cast<double>(completed) / runCount : 0.0;
    state.counters["average_work_frames"] = runCount > 0.0
        ? static_cast<double>(totalWorkFrames) / runCount : 0.0;
    state.counters["worst_work_frames"] = static_cast<double>(maximumWorkFrames);
    state.counters["frame_planning_budget_us"] =
        static_cast<double>(kFramePlanningBudget.count());
    state.counters["ik_solves_per_request"] = runCount > 0.0
        ? static_cast<double>(ikSolves) / runCount : 0.0;
    state.counters["ik_iterations_per_request"] = runCount > 0.0
        ? static_cast<double>(ikIterations) / runCount : 0.0;
    state.counters["tcp_refinements_per_request"] = runCount > 0.0
        ? static_cast<double>(tcpRefinements) / runCount : 0.0;
    state.counters["tcp_straightness_checks_per_request"] = runCount > 0.0
        ? static_cast<double>(tcpStraightnessChecks) / runCount : 0.0;
    state.counters["joint_path_validity_checks_per_request"] = runCount > 0.0
        ? static_cast<double>(jointPathValidityChecks) / runCount : 0.0;
    state.counters["validity_state_checks_per_request"] = runCount > 0.0
        ? static_cast<double>(validityStateChecks) / runCount : 0.0;
    state.counters["dummy_always_valid_callback_calls"] = static_cast<double>(stateValidityCallbackCalls);
    state.counters["dummy_always_valid_callback_calls_per_request"] = runCount > 0.0
        ? static_cast<double>(stateValidityCallbackCalls) / runCount : 0.0;
    state.counters["max_tcp_position_error_mm"] = maximumTcpPositionErrorMeters * 1000.0;
    state.counters["max_tcp_orientation_error_mrad"] = maximumTcpOrientationErrorRadians * 1000.0;
    state.counters["collision_benchmark_mode_dummy_always_valid"] = 1;
    const auto percentile = [](std::vector<double>& values, double fraction)
    {
        if (values.empty())
            return 0.0;
        std::sort(values.begin(), values.end());
        const std::size_t index = static_cast<std::size_t>(std::ceil(
            fraction * static_cast<double>(values.size()))) - 1;
        return values[std::min(index, values.size() - 1)];
    };
    state.counters["frame_slice_p50_us"] = percentile(frameSliceMicroseconds, 0.50);
    state.counters["frame_slice_p95_us"] = percentile(frameSliceMicroseconds, 0.95);
    state.counters["frame_slice_p99_us"] = percentile(frameSliceMicroseconds, 0.99);
    state.counters["frame_slice_max_us"] = frameSliceMicroseconds.empty() ? 0.0 :
        *std::max_element(frameSliceMicroseconds.begin(), frameSliceMicroseconds.end());
    state.SetLabel("interactive policy; 1.5 ms frame planning budget; dummy always-valid callback");
    state.SetItemsProcessed(static_cast<std::int64_t>(plannerRuns));
}

void PosePlanEndpointStress(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;

    constexpr auto kFramePlanningBudget = std::chrono::microseconds(1500);
    constexpr std::size_t kMaximumPlanningFrames = 10000;
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    std::vector<double> frameSliceMicroseconds;
    std::uint64_t requests = 0;
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::uint64_t timedOut = 0;
    std::uint64_t beginRejected = 0;
    std::uint64_t unreachable = 0;
    std::uint64_t jointLimitRejected = 0;
    std::uint64_t ikDidNotConverge = 0;
    std::uint64_t stateValidityCallbackCalls = 0;
    std::uint64_t totalWorkFrames = 0;
    std::uint64_t maximumWorkFrames = 0;
    std::uint64_t poseIkSeedAttempts = 0;
    std::uint64_t maximumPoseIkSeedAttempts = 0;
    std::string firstFailure;

    for (auto _ : state)
    {
        for (std::size_t caseIndex = 0; caseIndex < cases.size(); ++caseIndex)
        {
            SimRobotController controller(kHcr12a);
            if (!controller.Connect())
            {
                state.SkipWithError("SimRobotController could not connect for pose planning benchmark");
                return;
            }
            controller.SetJointStateValidityChecker([&stateValidityCallbackCalls](const JointVector&)
            {
                ++stateValidityCallbackCalls;
                return JointStateInvalidity::None;
            });

            ++requests;
            const Result begin = controller.BeginPosePlanning(cases[caseIndex].targetPose);
            if (!begin)
            {
                ++beginRejected;
                ++rejected;
                if (firstFailure.empty())
                    firstFailure = "BeginPosePlanning case " + std::to_string(caseIndex) + ": " + begin.message;
                continue;
            }

            std::size_t workFrames = 0;
            while (controller.IsMotionPlanning() && workFrames < kMaximumPlanningFrames)
            {
                const auto frameStart = Clock::now();
                const auto frameDeadline = frameStart + kFramePlanningBudget;
                do
                {
                    controller.AdvanceMotionPlanning(1);
                } while (controller.IsMotionPlanning() && Clock::now() < frameDeadline);
                frameSliceMicroseconds.push_back(std::chrono::duration<double, std::micro>(
                    Clock::now() - frameStart).count());
                ++workFrames;
            }

            totalWorkFrames += workFrames;
            maximumWorkFrames = std::max(maximumWorkFrames,
                static_cast<std::uint64_t>(workFrames));
            if (controller.IsMotionPlanning())
            {
                ++timedOut;
                controller.Stop();
                if (firstFailure.empty())
                    firstFailure = "Pose plan exceeded the 10,000-frame benchmark bound at case " +
                        std::to_string(caseIndex);
                continue;
            }

            const std::size_t seedAttempts = controller.GetLastPoseIkSeedAttemptCount();
            poseIkSeedAttempts += seedAttempts;
            maximumPoseIkSeedAttempts = std::max(maximumPoseIkSeedAttempts,
                static_cast<std::uint64_t>(seedAttempts));

            const auto result = controller.TakeMotionPlanningResult();
            if (result && *result)
            {
                ++accepted;
                continue;
            }

            ++rejected;
            if (result)
            {
                unreachable += result->code == ErrorCode::Unreachable;
                jointLimitRejected += result->code == ErrorCode::JointLimitReached;
                ikDidNotConverge += result->code == ErrorCode::IkDidNotConverge;
                if (firstFailure.empty())
                {
                    std::ostringstream details;
                    details << "Pose plan case " << caseIndex << " FK target joints deg=["
                        << std::fixed << std::setprecision(6);
                    for (std::size_t joint = 0;
                        joint < cases[caseIndex].reachableTargetJoints.size(); ++joint)
                        details << (joint == 0 ? "" : ",") <<
                            cases[caseIndex].reachableTargetJoints[joint] * 180.0 /
                                3.14159265358979323846;
                    details << "]: " << result->message;
                    firstFailure = details.str();
                }
            }
            else if (firstFailure.empty())
            {
                firstFailure = "Pose plan case " + std::to_string(caseIndex) +
                    ": planner exited without a result";
            }
        }
    }

    const auto percentile = [](std::vector<double>& values, double fraction)
    {
        if (values.empty())
            return 0.0;
        std::sort(values.begin(), values.end());
        const std::size_t index = static_cast<std::size_t>(std::ceil(
            fraction * static_cast<double>(values.size()))) - 1;
        return values[std::min(index, values.size() - 1)];
    };
    const double totalRequests = static_cast<double>(requests);
    state.counters["fk_generated_reachable_endpoints"] = totalRequests;
    state.counters["pose_plan_accepted"] = static_cast<double>(accepted);
    state.counters["pose_plan_rejected"] = static_cast<double>(rejected);
    state.counters["pose_plan_begin_rejected"] = static_cast<double>(beginRejected);
    state.counters["pose_plan_timeouts"] = static_cast<double>(timedOut);
    state.counters["pose_plan_unreachable"] = static_cast<double>(unreachable);
    state.counters["pose_plan_joint_limit_rejected"] = static_cast<double>(jointLimitRejected);
    state.counters["pose_plan_ik_did_not_converge"] = static_cast<double>(ikDidNotConverge);
    state.counters["pose_plan_acceptance_rate"] = totalRequests > 0.0
        ? static_cast<double>(accepted) / totalRequests : 0.0;
    state.counters["pose_plan_average_work_frames"] = totalRequests > 0.0
        ? static_cast<double>(totalWorkFrames) / totalRequests : 0.0;
    state.counters["pose_plan_worst_work_frames"] = static_cast<double>(maximumWorkFrames);
    state.counters["pose_plan_average_ik_seed_attempts"] = totalRequests > 0.0
        ? static_cast<double>(poseIkSeedAttempts) / totalRequests : 0.0;
    state.counters["pose_plan_maximum_ik_seed_attempts"] =
        static_cast<double>(maximumPoseIkSeedAttempts);
    state.counters["frame_slice_p50_us"] = percentile(frameSliceMicroseconds, 0.50);
    state.counters["frame_slice_p95_us"] = percentile(frameSliceMicroseconds, 0.95);
    state.counters["frame_slice_p99_us"] = percentile(frameSliceMicroseconds, 0.99);
    state.counters["frame_slice_max_us"] = frameSliceMicroseconds.empty() ? 0.0 :
        *std::max_element(frameSliceMicroseconds.begin(), frameSliceMicroseconds.end());
    state.counters["dummy_always_valid_callback_calls"] = static_cast<double>(stateValidityCallbackCalls);
    state.counters["dummy_always_valid_callback_calls_per_request"] = totalRequests > 0.0
        ? static_cast<double>(stateValidityCallbackCalls) / totalRequests : 0.0;
    state.counters["collision_benchmark_mode_dummy_always_valid"] = 1;
    state.SetLabel("PTP endpoint planning; 1.5 ms frame budget; dummy always-valid callback");
    state.SetItemsProcessed(static_cast<std::int64_t>(requests));
    if (!firstFailure.empty())
        state.SetLabel(firstFailure);
}

void EndpointIkGeneratedReachable(benchmark::State& state)
{
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    DampedLeastSquaresIk inverse(kHcr12a);
    std::uint64_t solveCount = 0;
    std::uint64_t iterationCount = 0;
    std::uint64_t converged = 0;
    std::uint64_t unreachable = 0;
    std::uint64_t jointLimitFailures = 0;
    std::uint64_t nonConverged = 0;
    double maximumPositionErrorMeters = 0.0;
    double maximumOrientationErrorRadians = 0.0;

    for (auto _ : state)
    {
        for (const auto& testCase : cases)
        {
            const IkResult result = inverse.SolveSingleSeed(testCase.targetPose, kSeed);
            ++solveCount;
            iterationCount += result.iterations;
            if (!result)
            {
                unreachable += result.status == IkStatus::Unreachable;
                jointLimitFailures += result.status == IkStatus::JointLimitReached;
                nonConverged += result.status == IkStatus::DidNotConverge;
                continue;
            }
            ++converged;
            maximumPositionErrorMeters = std::max(maximumPositionErrorMeters, result.positionErrorMeters);
            maximumOrientationErrorRadians = std::max(maximumOrientationErrorRadians, result.orientationErrorRadians);
            benchmark::DoNotOptimize(result.jointPositionRadians.data());
        }
    }

    const double totalCases = static_cast<double>(state.iterations() * cases.size());
    state.counters["fk_generated_reachable_endpoints"] = totalCases;
    state.counters["endpoint_ik_solves"] = static_cast<double>(solveCount);
    state.counters["endpoint_ik_successes"] = static_cast<double>(converged);
    state.counters["endpoint_ik_unreachable"] = static_cast<double>(unreachable);
    state.counters["endpoint_ik_joint_limit_failures"] = static_cast<double>(jointLimitFailures);
    state.counters["endpoint_ik_nonconvergence"] = static_cast<double>(nonConverged);
    state.counters["endpoint_ik_success_rate"] = totalCases > 0.0 ? static_cast<double>(converged) / totalCases : 0.0;
    state.counters["endpoint_ik_iterations_per_solve"] = solveCount > 0 ?
        static_cast<double>(iterationCount) / static_cast<double>(solveCount) : 0.0;
    state.counters["maximum_endpoint_ik_position_error_meters"] = maximumPositionErrorMeters;
    state.counters["maximum_endpoint_ik_orientation_error_radians"] = maximumOrientationErrorRadians;
    state.SetItemsProcessed(static_cast<std::int64_t>(totalCases));
}

void MoveJJointPathValidation(benchmark::State& state)
{
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    std::uint64_t validPaths = 0;
    std::uint64_t endpointIkConverged = 0;
    std::uint64_t endpointIkIterations = 0;

    for (auto _ : state)
    {
        for (const auto& testCase : cases)
        {
            const auto validity = ValidateJointPath(
                kSeed, testCase.reachableTargetJoints, kHcr12a, {});
            validPaths += validity == JointStateInvalidity::None;
            endpointIkConverged += testCase.endpointIkConvergedFromStart;
            endpointIkIterations += testCase.endpointIkIterations;
            benchmark::DoNotOptimize(validity);
        }
    }

    const double totalCases = static_cast<double>(state.iterations() * cases.size());
    state.counters["fk_generated_reachable_endpoints"] = totalCases;
    state.counters["endpoint_ik_successes_from_start_seed"] = static_cast<double>(endpointIkConverged);
    state.counters["endpoint_ik_iterations_per_endpoint"] = totalCases > 0.0 ?
        static_cast<double>(endpointIkIterations) / totalCases : 0.0;
    state.counters["joint_limit_valid_movej_segments"] = static_cast<double>(validPaths);
    state.counters["joint_limit_valid_movej_segment_rate"] = totalCases > 0.0 ? static_cast<double>(validPaths) / totalCases : 0.0;
    state.counters["environment_collision_checks"] = 0;
    state.SetItemsProcessed(static_cast<std::int64_t>(totalCases));
}

void MoveJCompletion(benchmark::State& state)
{
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    std::uint64_t completed = 0;
    std::uint64_t rejected = 0;
    std::uint64_t runtimeFaults = 0;
    std::uint64_t timeouts = 0;

    for (auto _ : state)
    {
        for (const auto& testCase : cases)
        {
            SimRobotController controller(kHcr12a);
            if (!controller.Connect())
            {
                state.SkipWithError("SimRobotController could not connect for MoveJ benchmark");
                return;
            }
            const Result accepted = controller.MoveJoint({testCase.reachableTargetJoints, 1.0, 1.0});
            if (!accepted)
            {
                ++rejected;
                continue;
            }

            constexpr double fixedDeltaSeconds = 0.004;
            constexpr std::size_t maxTicks = 5000;
            std::size_t tick = 0;
            for (; tick < maxTicks && controller.GetStateView().mode == RobotMode::Moving; ++tick)
                controller.Update(fixedDeltaSeconds);
            if (controller.GetStateView().mode == RobotMode::Idle)
                ++completed;
            else if (controller.GetStateView().mode == RobotMode::Fault)
                ++runtimeFaults;
            else
                ++timeouts;
            benchmark::DoNotOptimize(controller.GetStateView().jointPositionRadians.data());
        }
    }

    const double totalCases = static_cast<double>(state.iterations() * cases.size());
    state.counters["fk_generated_reachable_endpoints"] = totalCases;
    state.counters["movej_completed"] = static_cast<double>(completed);
    state.counters["movej_rejected"] = static_cast<double>(rejected);
    state.counters["movej_runtime_faults"] = static_cast<double>(runtimeFaults);
    state.counters["movej_timeouts"] = static_cast<double>(timeouts);
    state.counters["movej_completion_rate"] = totalCases > 0.0 ? static_cast<double>(completed) / totalCases : 0.0;
    state.counters["environment_collision_checks"] = 0;
    state.SetItemsProcessed(static_cast<std::int64_t>(totalCases));
}

void JoltMoveJDirectPath(benchmark::State& state)
{
    using grasplink::physics::BodyDescription;
    using grasplink::physics::BodyMotionType;
    using grasplink::physics::BoxBodyDescription;
    using grasplink::physics::CollisionLayer;
    using grasplink::physics::PhysicsWorld;
    using grasplink::physics::Transform;

    PhysicsWorld world;
    BoxBodyDescription obstacle;
    obstacle.halfExtentsMeters = {0.10F, 0.15F, 0.10F};
    obstacle.transform.position = {0.0F, 0.5F, 0.0F};
    obstacle.motionType = BodyMotionType::Static;
    obstacle.collisionLayer = CollisionLayer::Environment;
    world.CreateBox(obstacle);

    BodyDescription proxyDescription;
    grasplink::physics::CollisionShapeDescription proxyShape;
    proxyShape.halfExtentsMeters = {0.035F, 0.06F, 0.035F};
    proxyDescription.shapes.push_back(proxyShape);
    proxyDescription.transform.position = {-0.20F, 0.5F, -0.20F};
    proxyDescription.motionType = BodyMotionType::Kinematic;
    proxyDescription.collisionLayer = CollisionLayer::Robot;
    const auto proxy = world.CreateBody(proxyDescription);

    JointVector start(kHcr12a.jointCount, 0.0);
    JointVector goal = start;
    start[0] = -1.0;
    start[1] = -1.0;
    goal[0] = 1.0;
    goal[1] = 1.0;

    std::uint64_t plannerRuns = 0;
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::uint64_t joltQueries = 0;
    std::uint64_t plannerValidityChecks = 0;
    JointPathPlannerOptions options;

    for (auto _ : state)
    {
        StateValidityChecker checker = [&](const JointVector& joints)
        {
            ++joltQueries;
            Transform candidate;
            candidate.position = {
                static_cast<float>(joints[0] * 0.20),
                0.5F,
                static_cast<float>(joints[1] * 0.20)};
            return world.OverlapsEnvironmentAt(proxy, candidate)
                ? JointStateInvalidity::EnvironmentCollision
                : JointStateInvalidity::None;
        };

        JointPathPlanningJob job;
        const Result begin = job.Begin(kHcr12a, start, goal, checker, options);
        if (!begin)
        {
            state.SkipWithError("Jolt-backed MoveJ planner rejected valid benchmark endpoints");
            return;
        }
        for (std::size_t work = 0; work < 10000 && job.GetState() == JointPathPlanningState::Running; ++work)
            job.Advance(1);

        ++plannerRuns;
        if (job.GetState() == JointPathPlanningState::Completed)
        {
            ++accepted;
            plannerValidityChecks += job.GetPlan().validityChecks;
        }
        else
            ++rejected;
        benchmark::DoNotOptimize(job.GetResult().code);
    }

    state.counters["jolt_direct_movej_runs"] = static_cast<double>(plannerRuns);
    state.counters["jolt_direct_movej_accepted"] = static_cast<double>(accepted);
    state.counters["jolt_direct_movej_rejected"] = static_cast<double>(rejected);
    state.counters["jolt_direct_movej_rejection_rate"] = plannerRuns > 0
        ? static_cast<double>(rejected) / static_cast<double>(plannerRuns) : 0.0;
    state.counters["jolt_environment_overlap_queries"] = static_cast<double>(joltQueries);
    state.counters["jolt_overlap_queries_per_run"] = plannerRuns > 0
        ? static_cast<double>(joltQueries) / static_cast<double>(plannerRuns) : 0.0;
    state.counters["validity_checks_per_accepted_run"] = accepted > 0
        ? static_cast<double>(plannerValidityChecks) / static_cast<double>(accepted) : 0.0;
    state.SetItemsProcessed(static_cast<std::int64_t>(plannerRuns));
}

void RuntimeLinearPathStress(benchmark::State& state)
{
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    std::uint64_t plannerRejections = 0;
    std::uint64_t plannerIkRejections = 0;
    std::uint64_t plannerJointLimitStatuses = 0;
    std::uint64_t plannerIkLimitStalls = 0;
    std::uint64_t plannerVerifiedJointLimitViolations = 0;
    std::uint64_t rejectionsWithFkGeneratedReachableEndpoints = 0;
    std::uint64_t rejectionsWithEndpointIkSuccessFromStart = 0;
    std::uint64_t rejectionsWithJointLimitValidMovejSegments = 0;
    std::uint64_t runtimeFaults = 0;
    std::uint64_t runtimeIkFaults = 0;
    std::uint64_t timeouts = 0;
    std::uint64_t completed = 0;
    std::uint64_t stateValidityCallbackCalls = 0;
    std::string lastPlannerRejection;
    double maximumTcpPositionErrorMeters = 0.0;
    double maximumTcpOrientationErrorRadians = 0.0;

    for (auto _ : state)
    {
        for (std::size_t caseIndex = 0; caseIndex < cases.size(); ++caseIndex)
        {
            const auto& testCase = cases[caseIndex];
            SimRobotController controller(kHcr12a);
            if (!controller.Connect())
            {
                state.SkipWithError("SimRobotController could not connect");
                return;
            }
            controller.SetJointStateValidityChecker([&stateValidityCallbackCalls](const JointVector&)
            {
                ++stateValidityCallbackCalls;
                return JointStateInvalidity::None;
            });

            LinearPathMoveCommand command;
            command.targetPoses.push_back(testCase.targetPose);
            command.maxLinearVelocityMetersPerSecond = 2.0;
            command.maxAngularVelocityRadiansPerSecond = 8.0;
            command.maxLinearAccelerationMetersPerSecondSquared = 30.0;
            command.maxAngularAccelerationRadiansPerSecondSquared = 120.0;
            const JointVector startJoints = controller.GetStateView().jointPositionRadians;
            const auto accepted = controller.MoveLinearPath(command);
            if (!accepted)
            {
                std::ostringstream details;
                details << "case " << caseIndex << " start_deg=[" << std::fixed << std::setprecision(6);
                for (std::size_t joint = 0; joint < startJoints.size(); ++joint)
                    details << (joint == 0 ? "" : ",") << startJoints[joint] * 180.0 / 3.14159265358979323846;
                details << "] endpoint_seed_deg=[";
                for (std::size_t joint = 0; joint < testCase.reachableTargetJoints.size(); ++joint)
                    details << (joint == 0 ? "" : ",") <<
                        testCase.reachableTargetJoints[joint] * 180.0 / 3.14159265358979323846;
                details << "]; " << accepted.message;
                lastPlannerRejection = details.str();
                ++plannerRejections;
                if (accepted.code == ErrorCode::IkDidNotConverge)
                    ++plannerIkRejections;
                else if (accepted.code == ErrorCode::JointLimitReached)
                {
                    ++plannerJointLimitStatuses;
                    if (accepted.message.find("IK: joint limits block local improvement") != std::string::npos ||
                        accepted.message.find("IK: iteration limit while constrained by joint limits") != std::string::npos)
                        ++plannerIkLimitStalls;
                    else if (accepted.message.find("SimRobotController: planned state exceeds a joint limit") != std::string::npos)
                        ++plannerVerifiedJointLimitViolations;
                }
                ++rejectionsWithFkGeneratedReachableEndpoints;
                rejectionsWithEndpointIkSuccessFromStart += testCase.endpointIkConvergedFromStart;
                rejectionsWithJointLimitValidMovejSegments += testCase.directJointPathValid;
                continue;
            }

            constexpr double fixedDeltaSeconds = 0.004;
            constexpr std::size_t maxTicks = 5000;
            std::size_t tick = 0;
            for (; tick < maxTicks && controller.GetStateView().mode == RobotMode::Moving; ++tick)
                controller.Update(fixedDeltaSeconds);

            const auto& result = controller.GetStateView();
            if (result.mode == RobotMode::Fault)
            {
                ++runtimeFaults;
                if (result.errorCode == ErrorCode::IkDidNotConverge)
                    ++runtimeIkFaults;
            }
            else if (result.mode == RobotMode::Idle)
            {
                ++completed;
                if (result.tcpPoseValid)
                {
                    const auto& actualPosition = result.tcpPose.positionMeters;
                    const auto& targetPosition = testCase.targetPose.positionMeters;
                    const double dx = actualPosition[0] - targetPosition[0];
                    const double dy = actualPosition[1] - targetPosition[1];
                    const double dz = actualPosition[2] - targetPosition[2];
                    maximumTcpPositionErrorMeters = std::max(maximumTcpPositionErrorMeters,
                        std::sqrt(dx * dx + dy * dy + dz * dz));

                    const auto& actualOrientation = result.tcpPose.orientationXyzw;
                    const auto& targetOrientation = testCase.targetPose.orientationXyzw;
                    double quaternionDot = 0.0;
                    for (std::size_t axis = 0; axis < actualOrientation.size(); ++axis)
                        quaternionDot += actualOrientation[axis] * targetOrientation[axis];
                    quaternionDot = std::clamp(std::abs(quaternionDot), 0.0, 1.0);
                    maximumTcpOrientationErrorRadians = std::max(maximumTcpOrientationErrorRadians,
                        2.0 * std::acos(quaternionDot));
                }
            }
            else
            {
                ++timeouts;
            }
        }
    }

    const double totalCases = static_cast<double>(state.iterations() * cases.size());
    state.counters["cases"] = totalCases;
    state.counters["planner_rejections"] = static_cast<double>(plannerRejections);
    state.counters["planner_ik_rejections"] = static_cast<double>(plannerIkRejections);
    state.counters["planner_joint_limit_statuses"] = static_cast<double>(plannerJointLimitStatuses);
    state.counters["planner_ik_limit_stalls"] = static_cast<double>(plannerIkLimitStalls);
    state.counters["planner_verified_joint_limit_violations"] = static_cast<double>(plannerVerifiedJointLimitViolations);
    state.counters["moveL_rejections_with_fk_generated_reachable_endpoints"] =
        static_cast<double>(rejectionsWithFkGeneratedReachableEndpoints);
    state.counters["moveL_rejections_with_endpoint_ik_success_from_start_seed"] =
        static_cast<double>(rejectionsWithEndpointIkSuccessFromStart);
    state.counters["moveL_rejections_with_joint_limit_valid_movej_segments"] =
        static_cast<double>(rejectionsWithJointLimitValidMovejSegments);
    state.counters["runtime_faults"] = static_cast<double>(runtimeFaults);
    state.counters["runtime_ik_faults"] = static_cast<double>(runtimeIkFaults);
    state.counters["timeouts"] = static_cast<double>(timeouts);
    state.counters["completed"] = static_cast<double>(completed);
    state.counters["moveL_completed"] = static_cast<double>(completed);
    state.counters["moveL_planning_accepted"] = static_cast<double>(totalCases) - static_cast<double>(plannerRejections);
    state.counters["moveL_completion_rate"] = totalCases > 0.0 ? static_cast<double>(completed) / totalCases : 0.0;
    state.counters["moveL_max_tcp_position_error_meters"] = maximumTcpPositionErrorMeters;
    state.counters["moveL_max_tcp_orientation_error_radians"] = maximumTcpOrientationErrorRadians;
    state.counters["dummy_state_validity_callback_calls"] = static_cast<double>(stateValidityCallbackCalls);
    state.counters["dummy_state_validity_callback_calls_per_case"] = totalCases > 0.0 ?
        static_cast<double>(stateValidityCallbackCalls) / totalCases : 0.0;
    state.counters["environment_collision_checks"] = 0;
    state.counters["collision_benchmark_mode_dummy_always_valid"] = 1;
    if (!lastPlannerRejection.empty())
        state.SetLabel(lastPlannerRejection);
    state.SetItemsProcessed(static_cast<std::int64_t>(totalCases));
}

bool ParseIterations(std::string_view value, std::int64_t& iterations)
{
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), iterations);
    return error == std::errc{} && end == value.data() + value.size() && iterations > 0;
}

}

int main(int argc, char** argv)
{
    std::int64_t iterations = 0;
    int output = 1;
    for (int input = 1; input < argc; ++input)
    {
        constexpr std::string_view option = "--iterations=";
        const std::string_view argument(argv[input]);
        if (argument.compare(0, option.size(), option) == 0)
        {
            if (!ParseIterations(argument.substr(option.size()), iterations))
            {
                std::cerr << "--iterations must be a positive integer\n";
                return 2;
            }
            continue;
        }
        argv[output++] = argv[input];
    }
    argc = output;
    argv[argc] = nullptr;

    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv))
        return 2;

    auto* fk = benchmark::RegisterBenchmark("FK", Fk);
    auto* ik = benchmark::RegisterBenchmark("DLS_IK", BenchmarkDlsIk);
    auto* planner = benchmark::RegisterBenchmark("LinearPathPlanner", LinearPathPlanning);
    auto* incrementalPlanner = benchmark::RegisterBenchmark(
        "IncrementalLinearPathPlanner", IncrementalLinearPathPlanning);
    benchmark::RegisterBenchmark("EndpointIkGeneratedReachable", EndpointIkGeneratedReachable)
        ->Arg(64)
        ->Arg(256);
    benchmark::RegisterBenchmark("MoveJJointPathValidation", MoveJJointPathValidation)
        ->Arg(64)
        ->Arg(256);
    benchmark::RegisterBenchmark("MoveJCompletion", MoveJCompletion)
        ->Arg(64)
        ->Arg(256);
    benchmark::RegisterBenchmark("JoltMoveJDirectPath", JoltMoveJDirectPath)
        ->Unit(benchmark::kNanosecond);
    benchmark::RegisterBenchmark("RuntimeLinearPathStress", RuntimeLinearPathStress)
        ->Arg(64)
        ->Arg(256);
    benchmark::RegisterBenchmark("InteractiveLinearPathStress", InteractiveLinearPathStress)
        ->Arg(64)
        ->Iterations(1);
    benchmark::RegisterBenchmark("PosePlanEndpointStress", PosePlanEndpointStress)
        ->Arg(64)
        ->Arg(256)
        ->Iterations(1)
        ->UseRealTime();
    if (iterations > 0)
    {
        fk->Iterations(iterations);
        ik->Iterations(iterations);
        planner->Iterations(iterations);
        incrementalPlanner->Iterations(iterations);
    }

    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
