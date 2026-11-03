#include "scenario/ScenarioResultEvaluator.h"

#include "scenario/ScenarioDefinition.h"
#include "scenario/ScenarioRunner.h"

#include <glm/geometric.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>
#include <unordered_map>

namespace dronesim::scenario::detail
{
namespace
{
using namespace dronesim::simulation;
using DroneCommunicationStatus = ScenarioResult::Drone::CommunicationStatus;

/** @brief 쿼터니언 부호가 반대여도 같은 회전으로 비교한다. */
double OrientationErrorRadians(
    const glm::dquat& actual, const glm::dquat& expected) noexcept
{
    // q와 -q는 같은 회전이므로 내적의 절댓값으로 두 자세 사이의 짧은 각을 구한다.
    const double dot = std::abs(
        actual.w * expected.w + actual.x * expected.x
        + actual.y * expected.y + actual.z * expected.z);
    return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

std::string FlightModeMismatchMessage(const ExpectedDrone& expectedDrone)
{
    return expectedDrone.name + " 드론의 최종 비행 상태가 예상과 다릅니다.";
}

/** @brief 통신이 연결된 드론의 최종 상태를 시나리오 기대값과 비교한다. */
void CheckExpectedDroneResults(
    const ScenarioDefinition& definition, ScenarioResult& result)
{
    std::unordered_map<std::string_view, std::size_t> resultIndexByName;
    resultIndexByName.reserve(result.drones.size());
    for (std::size_t index = 0; index < result.drones.size(); ++index)
        resultIndexByName.emplace(result.drones[index].name, index);

    for (const ExpectedDrone& expectedDrone : definition.expectedDrones)
    {
        const auto found = resultIndexByName.find(expectedDrone.name);
        if (found == resultIndexByName.end())
        {
            result.issues.push_back("예상 결과에 지정한 드론을 찾을 수 없습니다: "
                + expectedDrone.name);
            continue;
        }

        const ScenarioResult::Drone& drone = result.drones[found->second];
        if (drone.communicationStatus != DroneCommunicationStatus::Connected)
        {
            // 끊긴 동안의 마지막 수신 위치는 현재 위치가 아니므로 기대 위치와 비교하지 않는다.
            continue;
        }

        const std::string dronePrefix = expectedDrone.name + " 드론의 ";
        if (drone.finalState.mode != expectedDrone.mode)
            result.issues.push_back(FlightModeMismatchMessage(expectedDrone));
        if (glm::length(drone.finalState.positionMeters - expectedDrone.positionMeters)
            > definition.toleranceMeters)
        {
            result.issues.push_back(dronePrefix + "최종 위치가 허용 오차를 벗어났습니다.");
        }
        if (expectedDrone.orientation
            && OrientationErrorRadians(drone.finalState.orientation, *expectedDrone.orientation)
                > definition.orientationToleranceRadians)
        {
            result.issues.push_back(dronePrefix + "최종 자세가 허용 각도 오차를 벗어났습니다.");
        }
    }

    if (!definition.expectedDrones.empty()
        && definition.expectedDrones.size() != result.drones.size())
    {
        result.issues.push_back("expected.drones 개수가 시나리오 드론 수와 다릅니다.");
    }
}

/** @brief 실행 중 측정한 드론 간 최소 거리가 설정값을 만족하는지 확인한다. */
void CheckExpectedSeparation(
    const ScenarioDefinition& definition, ScenarioResult& result)
{
    if (definition.minimumDroneSeparationMeters
        && result.minimumDroneSeparationMeters
        && *result.minimumDroneSeparationMeters
            < *definition.minimumDroneSeparationMeters)
    {
        result.issues.push_back("드론 사이 최소 거리가 설정한 안전 간격보다 짧았습니다.");
    }
}

/** @brief 기본 드론 카메라의 상태와 생성 프레임 수를 기대값과 비교한다. */
void CheckExpectedCamera(
    const ScenarioDefinition& definition, ScenarioResult& result)
{
    const CameraState& primaryCamera = result.drones.front().finalCamera;
    if (definition.expectedCameraStatus
        && primaryCamera.status != *definition.expectedCameraStatus)
    {
        result.issues.push_back("최종 카메라 상태가 예상값과 다릅니다.");
    }
    if (definition.expectedFrameCount
        && primaryCamera.frameCount != *definition.expectedFrameCount)
    {
        result.issues.push_back("카메라 프레임 수가 예상값과 다릅니다.");
    }
}
}

}

namespace dronesim::scenario
{
ScenarioResult::Outcome ScenarioResult::Mission::GetOutcome() const noexcept
{
    bool allTasksCompleted = true;
    for (const MissionTask& task : tasks)
    {
        if (task.failed)
            return Outcome::Failed;
        if (!task.IsCompleted())
            allTasksCompleted = false;
    }
    return allTasksCompleted ? Outcome::Passed : Outcome::Inconclusive;
}

std::string_view ToString(ScenarioResult::Outcome outcome) noexcept
{
    switch (outcome)
    {
    case ScenarioResult::Outcome::Passed: return "passed";
    case ScenarioResult::Outcome::Failed: return "failed";
    case ScenarioResult::Outcome::Inconclusive: return "inconclusive";
    }
    return "inconclusive";
}
}

namespace dronesim::scenario::detail
{
void EvaluateExpectedResults(
    const ScenarioDefinition& definition, ScenarioResult& result)
{
    // 개별 조건을 모두 확인해 실패 원인을 모은 뒤, 임무 진행 결과와 합쳐 최종 판정을 정한다.
    CheckExpectedDroneResults(definition, result);
    CheckExpectedSeparation(definition, result);
    CheckExpectedCamera(definition, result);

    if (!result.issues.empty())
        result.outcome = ScenarioResult::Outcome::Failed;
    else if (result.mission)
        result.outcome = result.mission->GetOutcome();
    else
        result.outcome = ScenarioResult::Outcome::Passed;
}
}
