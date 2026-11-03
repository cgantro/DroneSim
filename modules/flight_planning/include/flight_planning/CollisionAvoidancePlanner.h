#pragma once

#include <glm/vec3.hpp>

#include <optional>
#include <span>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace dronesim::flight_planning
{
/** @brief 경로를 함께 피할 때 적용하는 거리와 예측 시간이다. */
struct CollisionAvoidanceSettings
{
    /** @brief 예측한 가장 가까운 거리가 이 값 이하면 회피를 시작한다. 단위는 미터다. */
    double safetyDistanceMeters = 3.0;
    /** @brief 직선 이동을 앞쪽으로 내다볼 시간이다. 단위는 초다. */
    double predictionHorizonSeconds = 2.0;
    /** @brief 충돌 지점을 지난 뒤 옆으로 확보할 최소 거리다. 단위는 미터다. */
    double lateralOffsetMeters = 5.0;
};

/** @brief 한 드론이 현재 위치에서 어느 방향으로 움직이려는지 나타낸다. */
struct DroneMotionIntent
{
    /** @brief 실행 중인 시나리오에서 이 기체를 가리키는 이름이다. */
    std::string name;
    /** @brief 예측 시작 위치다. 월드 ENU 좌표, 단위는 미터다. */
    glm::dvec3 positionMeters{0.0};
    /** @brief 현재 목표를 그대로 따를 때의 수평·수직 속도다. 단위는 m/s다. */
    glm::dvec3 intendedVelocityMetersPerSecond{0.0};
    /** @brief 기존 회피 목표를 제외하고 원래 경로만 따를 때의 속도다. 단위는 m/s다. */
    glm::dvec3 routeVelocityMetersPerSecond{0.0};
    /** @brief 현재 통신이 연결되어 실제 상태를 사용할 수 있는지 나타낸다. */
    bool connected = true;
    /** @brief 이 기체에 진행 중인 경유점 임무가 있는지 나타낸다. */
    bool hasActiveTask = false;
};

/** @brief 이번 Tick에 적용할 회피 목표와 예측 최소 간격이다. */
struct CollisionAvoidancePlan
{
    /** @brief 각 드론의 이름과 이번 회피 경로의 목표 위치다. */
    std::unordered_map<std::string, glm::dvec3> targetsByDrone;
    /** @brief 현재도 충돌 위험이 남아 회피를 계속할 드론 이름이다. */
    std::unordered_set<std::string> activeDroneNames;
    /** @brief 이번 계획에서 예측한 가장 가까운 드론 간 거리다. 단위는 미터다. */
    std::optional<double> minimumPredictedSeparationMeters;
};

/**
 * @brief 드론들의 속도 의도를 짧은 시간 앞까지 외삽해 서로 비켜 갈 목표를 계산한다.
 * @details 기체 가속도나 경유점 도착 뒤의 감속은 계산하지 않는 직선 예측이다.
 * 기존 회피 목표가 있으면 원래 경로와 회피 경로를 모두 비교해 위험이 끝났는지 판단한다.
 */
[[nodiscard]] CollisionAvoidancePlan PlanCollisionAvoidance(
    std::span<const DroneMotionIntent> drones,
    const CollisionAvoidanceSettings& settings,
    const std::unordered_map<std::string, glm::dvec3>& currentTargetsByDrone);
}
