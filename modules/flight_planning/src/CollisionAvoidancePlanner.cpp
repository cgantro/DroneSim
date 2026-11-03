#include "flight_planning/CollisionAvoidancePlanner.h"

#include <glm/geometric.hpp>
#include <glm/common.hpp>

namespace dronesim::flight_planning
{
namespace
{
struct PredictedApproach
{
    double distanceMeters = 0.0;
    double timeSeconds = 0.0;
};

/** @brief 두 직선 이동을 예측 시간 안에서 비교해 가장 가까운 순간을 찾는다. */
PredictedApproach PredictClosestApproach(
    const glm::dvec3& firstPositionMeters,
    const glm::dvec3& firstVelocityMetersPerSecond,
    const glm::dvec3& secondPositionMeters,
    const glm::dvec3& secondVelocityMetersPerSecond,
    double predictionHorizonSeconds)
{
    const glm::dvec3 relativePosition = secondPositionMeters - firstPositionMeters;
    const glm::dvec3 relativeVelocity = secondVelocityMetersPerSecond
        - firstVelocityMetersPerSecond;
    const double relativeSpeedSquared = glm::dot(relativeVelocity, relativeVelocity);

    // 상대 위치를 상대 속도에 투영하면 두 경로가 가장 가까워지는 시간을 구할 수 있다.
    // 계산 결과는 현재부터 예측 범위 안으로 잘라낸다.
    const double closestTimeSeconds = relativeSpeedSquared > 1.0e-9
        ? glm::clamp(-glm::dot(relativePosition, relativeVelocity)
            / relativeSpeedSquared, 0.0, predictionHorizonSeconds)
        : 0.0;
    const glm::dvec3 closestOffset = relativePosition
        + relativeVelocity * closestTimeSeconds;
    return {glm::length(closestOffset), closestTimeSeconds};
}

/** @brief 예측 충돌 지점을 지나도록 진행 방향으로 여유 거리를 더한다. */
glm::dvec3 PositionBeyondConflict(
    const glm::dvec3& conflictPositionMeters,
    const glm::dvec3& currentPositionMeters,
    const glm::dvec3& velocityIntentMetersPerSecond,
    double clearanceMeters)
{
    glm::dvec3 forward{velocityIntentMetersPerSecond.x,
        velocityIntentMetersPerSecond.y, 0.0};
    if (glm::length(forward) < 1.0e-6)
    {
        forward = {conflictPositionMeters.x - currentPositionMeters.x,
            conflictPositionMeters.y - currentPositionMeters.y, 0.0};
    }
    if (glm::length(forward) < 1.0e-6)
        return conflictPositionMeters;
    return conflictPositionMeters + glm::normalize(forward) * clearanceMeters;
}

/**
 * @brief 충돌 회피를 위해 속도를 바꿀 드론을 고른다.
 * @details 둘 다 연결되어 임무 중이면 두 대가 함께 비킨다. 그 외에는 연결이 살아 있고
 * 임무를 진행하는 드론만 회피 대상으로 삼는다. 연결이 끊긴 드론은 움직이지 않는 장애물로 본다.
 */
const DroneMotionIntent* SelectYieldingDrone(
    const DroneMotionIntent& first, const DroneMotionIntent& second) noexcept
{
    const bool firstCanYield = first.connected && first.hasActiveTask;
    const bool secondCanYield = second.connected && second.hasActiveTask;

    if (first.connected && second.connected)
    {
        if (firstCanYield == secondCanYield)
            return nullptr;
        return firstCanYield ? &first : &second;
    }
    if (firstCanYield)
        return &first;
    if (secondCanYield)
        return &second;
    return nullptr;
}
}

CollisionAvoidancePlan PlanCollisionAvoidance(
    std::span<const DroneMotionIntent> drones,
    const CollisionAvoidanceSettings& settings,
    const std::unordered_map<std::string, glm::dvec3>& currentTargetsByDrone)
{
    CollisionAvoidancePlan plan;
    for (std::size_t firstIndex = 0; firstIndex < drones.size(); ++firstIndex)
    {
        const DroneMotionIntent& first = drones[firstIndex];
        for (std::size_t secondIndex = firstIndex + 1U;
            secondIndex < drones.size(); ++secondIndex)
        {
            const DroneMotionIntent& second = drones[secondIndex];
            const bool bothHaveTasks = first.connected && second.connected
                && first.hasActiveTask && second.hasActiveTask;
            const DroneMotionIntent* yieldingDrone = SelectYieldingDrone(first, second);
            if (!bothHaveTasks && yieldingDrone == nullptr)
                continue;

            const PredictedApproach intendedPath = PredictClosestApproach(
                first.positionMeters, first.intendedVelocityMetersPerSecond,
                second.positionMeters, second.intendedVelocityMetersPerSecond,
                settings.predictionHorizonSeconds);
            const PredictedApproach routeContinuation = PredictClosestApproach(
                first.positionMeters, first.routeVelocityMetersPerSecond,
                second.positionMeters, second.routeVelocityMetersPerSecond,
                settings.predictionHorizonSeconds);
            if (!plan.minimumPredictedSeparationMeters
                || intendedPath.distanceMeters < *plan.minimumPredictedSeparationMeters)
            {
                plan.minimumPredictedSeparationMeters = intendedPath.distanceMeters;
            }

            const bool alreadyAvoiding = bothHaveTasks
                ? currentTargetsByDrone.contains(first.name)
                    || currentTargetsByDrone.contains(second.name)
                : currentTargetsByDrone.contains(yieldingDrone->name);
            const double releaseDistanceMeters = settings.safetyDistanceMeters
                * (bothHaveTasks ? 1.5 : 1.0);
            const double lateralClearanceMeters = settings.lateralOffsetMeters
                + settings.safetyDistanceMeters;
            const bool shouldAvoid = alreadyAvoiding
                ? routeContinuation.distanceMeters <= releaseDistanceMeters
                : intendedPath.distanceMeters <= settings.safetyDistanceMeters;
            if (!shouldAvoid)
                continue;

            const glm::dvec3 relativePosition = second.positionMeters - first.positionMeters;
            const glm::dvec3 threatVelocity = second.intendedVelocityMetersPerSecond
                - first.intendedVelocityMetersPerSecond;
            glm::dvec3 side{-threatVelocity.y, threatVelocity.x, 0.0};
            if (glm::length(side) < 1.0e-6)
                side = {-relativePosition.y, relativePosition.x, 0.0};
            if (glm::length(side) < 1.0e-6)
                side = {0.0, 1.0, 0.0};
            side = glm::normalize(side);

            const auto makeTarget = [&](const DroneMotionIntent& drone,
                                        const glm::dvec3& lateralDirection)
            {
                const glm::dvec3 conflictPosition = drone.positionMeters
                    + drone.intendedVelocityMetersPerSecond * intendedPath.timeSeconds;
                return PositionBeyondConflict(conflictPosition,
                    drone.positionMeters, drone.intendedVelocityMetersPerSecond,
                    releaseDistanceMeters)
                    + lateralDirection * lateralClearanceMeters;
            };

            if (bothHaveTasks)
            {
                plan.targetsByDrone.try_emplace(first.name, makeTarget(first, side));
                plan.targetsByDrone.try_emplace(second.name, makeTarget(second, -side));
                plan.activeDroneNames.insert(first.name);
                plan.activeDroneNames.insert(second.name);
            }
            else if (yieldingDrone == &first)
            {
                plan.targetsByDrone.try_emplace(first.name, makeTarget(first, -side));
                plan.activeDroneNames.insert(first.name);
            }
            else
            {
                plan.targetsByDrone.try_emplace(second.name, makeTarget(second, side));
                plan.activeDroneNames.insert(second.name);
            }
        }
    }
    return plan;
}
}
