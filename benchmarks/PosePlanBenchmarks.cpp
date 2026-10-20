#include "BenchmarkSupport.h"

#include "robotics/backends/simulation/SimRobotController.h"

#include <algorithm>
#include <chrono>
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
// 생성한 목표 자세들로 증분 포즈 계획을 반복해 요청 수락 여부, 처리 시간, 실패 원인을 기록합니다.
void PosePlanEndpointStress(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;

    constexpr auto kFramePlanningBudget = std::chrono::microseconds(1500);
    constexpr std::size_t kMaximumPlanningFrames = 10000;
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    std::vector<double> frameSliceMicroseconds;
    std::vector<double> completedPlanMilliseconds;
    std::vector<double> completedPlanFrames;
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
                state.SkipWithError("포즈 계획 벤치마크에서 SimRobotController에 연결하지 못했습니다.");
                return;
            }
            controller.SetJointStateValidityChecker([&stateValidityCallbackCalls](const JointVector&)
            {
                ++stateValidityCallbackCalls;
                return JointStateInvalidity::None;
            });

            ++requests;
            const auto requestStart = Clock::now();
            const Result begin = controller.BeginPosePlanning(cases[caseIndex].targetPose);
            if (!begin)
            {
                ++beginRejected;
                ++rejected;
                if (firstFailure.empty())
                    firstFailure = "BeginPosePlanning 시작 실패. 사례 " + std::to_string(caseIndex) + ": " + std::string(ErrorCodeMessageKorean(begin.code));
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
                    firstFailure = "포즈 계획이 10,000 프레임 제한을 넘었습니다. 사례 " +
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
                completedPlanMilliseconds.push_back(std::chrono::duration<double, std::milli>(
                    Clock::now() - requestStart).count());
                completedPlanFrames.push_back(static_cast<double>(workFrames));
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
                    details << "포즈 계획 사례 " << caseIndex << ", 정기구학 목표 관절각(도)=["
                        << std::fixed << std::setprecision(6);
                    for (std::size_t joint = 0;
                        joint < cases[caseIndex].reachableTargetJoints.size(); ++joint)
                        details << (joint == 0 ? "" : ",") <<
                            cases[caseIndex].reachableTargetJoints[joint] * 180.0 /
                                3.14159265358979323846;
                    details << "]: " << ErrorCodeMessageKorean(result->code);
                    firstFailure = details.str();
                }
            }
            else if (firstFailure.empty())
            {
                firstFailure = "포즈 계획 사례 " + std::to_string(caseIndex) +
                    ": 계획기가 결과 없이 종료했습니다.";
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
    state.counters["completed_plan_p50_ms"] = percentile(completedPlanMilliseconds, 0.50);
    state.counters["completed_plan_p95_ms"] = percentile(completedPlanMilliseconds, 0.95);
    state.counters["completed_plan_p99_ms"] = percentile(completedPlanMilliseconds, 0.99);
    state.counters["completed_plan_frames_p50"] = percentile(completedPlanFrames, 0.50);
    state.counters["completed_plan_frames_p95"] = percentile(completedPlanFrames, 0.95);
    state.counters["completed_plan_frames_p99"] = percentile(completedPlanFrames, 0.99);
    state.counters["dummy_always_valid_callback_calls"] = static_cast<double>(stateValidityCallbackCalls);
    state.counters["dummy_always_valid_callback_calls_per_request"] = totalRequests > 0.0
        ? static_cast<double>(stateValidityCallbackCalls) / totalRequests : 0.0;
    state.counters["collision_benchmark_mode_dummy_always_valid"] = 1;
    state.SetLabel("PTP 끝점 계획, 프레임당 예산 1.5 ms, 충돌 검사는 항상 통과");
    state.SetItemsProcessed(static_cast<std::int64_t>(requests));
    if (!firstFailure.empty())
        state.SetLabel(firstFailure);
}
}
