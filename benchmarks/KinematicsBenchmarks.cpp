#include "BenchmarkSupport.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/planning/JointPathPlanner.h"
#include "robotics/planning/LinearPathPlanner.h"

#include <algorithm>
#include <cstdint>

namespace grasplink::benchmarks
{
using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;
using namespace grasplink::robotics::models;
using namespace grasplink::robotics::models::hanwha;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;
// 순기구학은 HCR12A의 관절 각도에서 도구 끝의 위치와 방향을 계산합니다. 이 측정은 그 결과를 로봇 베이스 기준으로 구하는 시간을 잽니다.
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

// 같은 초기 관절 각도에서 고정된 도구 끝 목표 자세를 푸는 시간을 잽니다. 측정 전에 먼저 풀어 목표가 유효한지 확인합니다.
void BenchmarkLmIk(benchmark::State& state)
{
    LevenbergMarquardtIk solver(kHcr12a);
    const auto target = MakeTargetPose();
    const auto validation = solver.Solve(target, kSeed);
    if (!validation)
    {
        state.SkipWithError("LM 역기구학 목표가 사전 검증에서 수렴하지 않았습니다.");
        return;
    }

    for (auto _ : state)
    {
        const auto result = solver.Solve(target, kSeed);
        if (!result)
        {
            state.SkipWithError("LM 역기구학 계산이 수렴하지 않았습니다.");
            break;
        }
        benchmark::DoNotOptimize(result.jointPositions.data());
        benchmark::DoNotOptimize(result.positionErrorMeters);
    }
}

// 순기구학으로 실제 도달 가능한 목표 자세를 만든 뒤, 역기구학이 유효한 관절 각도를 찾는 비율과 계산량을 측정합니다.
void EndpointIkGeneratedReachable(benchmark::State& state)
{
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    LevenbergMarquardtIk inverse(kHcr12a);
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
            benchmark::DoNotOptimize(result.jointPositions.data());
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
}
