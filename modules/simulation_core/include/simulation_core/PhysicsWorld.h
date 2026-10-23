#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "terrain/TerrainGrid.h"

#include <cstdint>
#include <array>
#include <memory>

namespace aerosimlab::simulation
{
struct QuadrotorParameters;

/** @brief 물리엔진에서 만든 강체를 가리킨다. */
struct RigidBodyHandle
{
    std::uint32_t value = 0;
};

/** @brief 강체 기준 위치·자세에 놓을 상자 충돌 형상이다. */
struct RigidBodyBox
{
    glm::vec3 halfExtentsMeters{0.1F};
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
};

/** @brief 드론 강체를 만들 때 필요한 시작 위치와 자세다. */
struct RigidBodyDefinition
{
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
};

/** @brief 한 Tick 뒤 강체의 월드 자세와 속도다. */
struct RigidBodyState
{
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 linearVelocityMetersPerSecond{0.0};
    glm::dvec3 angularVelocityRadiansPerSecond{0.0};
};

/** @brief 물리 Tick 전체와 접촉 생성에 걸린 시간을 마이크로초로 기록한다. */
struct PhysicsStepMeasurements
{
    double stepDurationMicroseconds = 0.0;
    double contactGenerationMicroseconds = 0.0;
};

/**
 * @brief AirSim 하나로 드론 비행과 충돌 반응을 계산한다.
 * @details AirSim이 힘, 이동, 회전 적분과 충돌 후 반응을 맡는다. Unreal 없이 실행하므로 이 클래스가 지형·장애물 접촉 정보만 만들어 AirSim에 전달한다. 물리 좌표의 위쪽은 Z축이다.
 */
class PhysicsWorld
{
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    /** @brief 장면의 고정 상자 장애물을 추가해 접촉 여부를 검사한다. */
    void AddStaticObstacleBox(
        const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters);

    /** @brief 기체 설정의 질량·관성·로터·충돌 상자로 쿼드콥터를 만든다. */
    [[nodiscard]] RigidBodyHandle CreateQuadrotor(
        const RigidBodyDefinition& definition,
        const QuadrotorParameters& quadrotor);

    /** @brief AirSim이 다음 단계에서 적용할 로터 출력을 받는다. */
    void SetMotorTargets(
        RigidBodyHandle body,
        const std::array<double, 4>& speedRadiansPerSecond);

    /** @brief 물리 월드에 적용할 바람을 지정한다. */
    void SetWindVelocityMetersPerSecond(const glm::dvec3& velocity);

    /** @brief 평평한 바닥을 지형 높이 자료로 바꿔 정적 충돌 지형을 만든다. */
    void SetTerrain(terrain::TerrainGrid terrain);

    /** @brief 기존 강체의 위치·자세·속도를 지정한 상태로 되돌린다. */
    void SetState(RigidBodyHandle body, const RigidBodyState& state);

    /** @brief AirSim이 힘, 이동, 회전과 접촉 반응을 고정 시간만큼 계산한다. */
    void Step(double seconds, PhysicsStepMeasurements* measurements = nullptr);

    /** @brief 물리 세계에서 강체를 제거한다. 이미 제거했거나 잘못된 핸들이면 false를 반환한다. */
    [[nodiscard]] bool DestroyBody(RigidBodyHandle body);

    /** @brief 현재 물리엔진이 계산한 위치·자세·속도를 읽는다. */
    [[nodiscard]] RigidBodyState State(RigidBodyHandle body) const;

    /** @brief AirSim이 계산한 로터별 회전 속도를 라디안/초 단위로 읽는다. */
    [[nodiscard]] std::array<double, 4> MotorSpeedsRadiansPerSecond(
        RigidBodyHandle body) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
