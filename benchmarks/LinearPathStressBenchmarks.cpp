#include "BenchmarkSupport.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/planning/LinearPathPlanner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace grasplink::benchmarks
{
using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;
using namespace grasplink::robotics::models;
using namespace grasplink::robotics::models::hanwha;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;

namespace
{
double Percentile(std::vector<double>& samples, double fraction)
{
    if (samples.empty())
        return 0.0;
    std::sort(samples.begin(), samples.end());
    const std::size_t index = static_cast<std::size_t>(std::ceil(
        fraction * static_cast<double>(samples.size()))) - 1;
    return samples[std::min(index, samples.size() - 1)];
}
}

// 생성한 여러 목표로 대화형 경로 계획의 처리량과 지연 분포를 측정합니다. 항상 유효하다고 답하는 콜백을 쓰므로 실제 충돌은 검사하지 않습니다.
void InteractiveLinearPathStress(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;
    using namespace grasplink::robotics::kinematics::detail;

    constexpr auto kFramePlanningBudget = std::chrono::microseconds(1500);
    constexpr std::size_t kMaximumPlanningFrames = 100000;
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    LevenbergMarquardtIk inverse(kHcr12a);
    const CartesianPose startTcp = inverse.EvaluateTcp(kSeed);
    std::vector<double> frameSliceMicroseconds;
    std::vector<double> completedPlanMilliseconds;
    std::vector<double> completedPlanFrames;
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
            const auto requestStart = Clock::now();
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
                dummyAlwaysValid, kDefaultPlanningPolicy);
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
            completedPlanMilliseconds.push_back(std::chrono::duration<double, std::milli>(
                Clock::now() - requestStart).count());
            completedPlanFrames.push_back(static_cast<double>(workFrames));
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
    state.counters["frame_slice_p50_us"] = Percentile(frameSliceMicroseconds, 0.50);
    state.counters["frame_slice_p95_us"] = Percentile(frameSliceMicroseconds, 0.95);
    state.counters["frame_slice_p99_us"] = Percentile(frameSliceMicroseconds, 0.99);
    state.counters["frame_slice_max_us"] = frameSliceMicroseconds.empty() ? 0.0 :
        *std::max_element(frameSliceMicroseconds.begin(), frameSliceMicroseconds.end());
    state.counters["completed_plan_p50_ms"] = Percentile(completedPlanMilliseconds, 0.50);
    state.counters["completed_plan_p95_ms"] = Percentile(completedPlanMilliseconds, 0.95);
    state.counters["completed_plan_p99_ms"] = Percentile(completedPlanMilliseconds, 0.99);
    state.counters["completed_plan_frames_p50"] = Percentile(completedPlanFrames, 0.50);
    state.counters["completed_plan_frames_p95"] = Percentile(completedPlanFrames, 0.95);
    state.counters["completed_plan_frames_p99"] = Percentile(completedPlanFrames, 0.99);
    state.SetLabel("대화형 정책, 프레임당 계획 예산 1.5 ms, 충돌 검사는 항상 통과");
    state.SetItemsProcessed(static_cast<std::int64_t>(plannerRuns));
}

// SimRobotController에서 생성한 목표 경로를 실행해 성공과 실패를 셉니다. 상태 검사 콜백이 항상 유효하다고 답하므로 환경 충돌 검사는 포함하지 않습니다.
void RuntimeLinearPathStress(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    const std::size_t expectedCaseCount = static_cast<std::size_t>(state.iterations()) * cases.size();
    std::vector<double> planAttemptMilliseconds;
    std::vector<double> executedPathMilliseconds;
    std::vector<double> executedPathTicks;
    planAttemptMilliseconds.reserve(expectedCaseCount);
    executedPathMilliseconds.reserve(expectedCaseCount);
    executedPathTicks.reserve(expectedCaseCount);
    double totalPlanAttemptMilliseconds = 0.0;
    double totalExecutedPathMilliseconds = 0.0;
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
                state.SkipWithError("SimRobotController에 연결하지 못했습니다.");
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
            const JointVector startJoints = controller.GetStateView().jointPositions;
            const auto planningStarted = Clock::now();
            const auto accepted = controller.MoveLinearPath(command);
            const double planAttemptMillisecondsForCase = std::chrono::duration<double, std::milli>(
                Clock::now() - planningStarted).count();
            planAttemptMilliseconds.push_back(planAttemptMillisecondsForCase);
            totalPlanAttemptMilliseconds += planAttemptMillisecondsForCase;
            if (!accepted)
            {
                std::ostringstream details;
                details << "사례 " << caseIndex << ": 시작 관절각(도)=[" << std::fixed << std::setprecision(6);
                for (std::size_t joint = 0; joint < startJoints.size(); ++joint)
                    details << (joint == 0 ? "" : ",") << startJoints[joint] * 180.0 / 3.14159265358979323846;
                details << "] 목표 관절각(도)=[";
                for (std::size_t joint = 0; joint < testCase.reachableTargetJoints.size(); ++joint)
                    details << (joint == 0 ? "" : ",") <<
                        testCase.reachableTargetJoints[joint] * 180.0 / 3.14159265358979323846;
                details << "]; 거부 원인: " << ErrorCodeMessageKorean(accepted.code);
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
                    else if (accepted.message.find("StateValidity: planned state exceeds a joint limit") != std::string::npos)
                        ++plannerVerifiedJointLimitViolations;
                }
                ++rejectionsWithFkGeneratedReachableEndpoints;
                rejectionsWithEndpointIkSuccessFromStart += testCase.endpointIkConvergedFromStart;
                rejectionsWithJointLimitValidMovejSegments += testCase.directJointPathValid;
                continue;
            }

            constexpr double fixedDeltaSeconds = 0.004;
            constexpr std::size_t maxTicks = 5000;
            const auto executionStarted = Clock::now();
            std::size_t tick = 0;
            for (; tick < maxTicks && controller.GetStateView().mode == RobotMode::Moving; ++tick)
                controller.Update(fixedDeltaSeconds);
            const double executedPathMillisecondsForCase = std::chrono::duration<double, std::milli>(
                Clock::now() - executionStarted).count();
            executedPathMilliseconds.push_back(executedPathMillisecondsForCase);
            executedPathTicks.push_back(static_cast<double>(tick));
            totalExecutedPathMilliseconds += executedPathMillisecondsForCase;

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
    const double iterationCount = static_cast<double>(state.iterations());
    state.counters["planning_total_ms_per_batch"] = totalPlanAttemptMilliseconds / iterationCount;
    state.counters["execution_total_ms_per_batch"] = totalExecutedPathMilliseconds / iterationCount;
    state.counters["plan_attempt_p50_ms"] = Percentile(planAttemptMilliseconds, 0.50);
    state.counters["plan_attempt_p95_ms"] = Percentile(planAttemptMilliseconds, 0.95);
    state.counters["plan_attempt_p99_ms"] = Percentile(planAttemptMilliseconds, 0.99);
    state.counters["executed_path_p50_ms"] = Percentile(executedPathMilliseconds, 0.50);
    state.counters["executed_path_p95_ms"] = Percentile(executedPathMilliseconds, 0.95);
    state.counters["executed_path_p99_ms"] = Percentile(executedPathMilliseconds, 0.99);
    state.counters["executed_path_ticks_p50"] = Percentile(executedPathTicks, 0.50);
    state.counters["executed_path_ticks_p95"] = Percentile(executedPathTicks, 0.95);
    if (!lastPlannerRejection.empty())
        state.SetLabel(lastPlannerRejection);
    state.SetItemsProcessed(static_cast<std::int64_t>(totalCases));
}
}
