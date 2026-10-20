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
// 이 측정은 직선 경로 전체를 한 번의 호출로 만드는 시간을 재고, 경로 하나를 만들 때 필요한 계산량도 기록합니다.
void LinearPathPlanning(benchmark::State& state)
{
    LevenbergMarquardtIk solver(kHcr12a);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(MakeTargetPose());
    const CartesianPose startTcp = solver.EvaluateTcp(kSeed);

    LinearPathPlan plan;
    const auto validation = BuildLinearPath(
        command, kHcr12a, kSeed, startTcp, solver, {}, plan);
    if (!validation)
    {
        state.SkipWithError("선형 경로 벤치마크의 사전 경로 검증이 실패했습니다.");
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
            state.SkipWithError("선형 경로 벤치마크 경로를 만들지 못했습니다.");
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

// 증분 계획은 전체 경로를 짧은 작업 단위로 나눠 만듭니다. 60 Hz 지연 시간은 실제 프레임 측정이 아니라 작업 단위 수로 계산한 추정치입니다.
void IncrementalLinearPathPlanning(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;
    using namespace grasplink::robotics::kinematics::detail;

    LevenbergMarquardtIk solver(kHcr12a);
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
            grasplink::robotics::planning::kDefaultPlanningPolicy))
        {
            state.SkipWithError("증분 경로 계획기가 벤치마크 경로 생성을 시작하지 못했습니다.");
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
            state.SkipWithError("증분 경로 계획기가 유효한 벤치마크 경로를 끝까지 만들지 못했습니다.");
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
}
