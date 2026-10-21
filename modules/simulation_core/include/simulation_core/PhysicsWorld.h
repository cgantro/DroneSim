#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include "terrain/TerrainGrid.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <span>

namespace aerosimlab::simulation
{
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

/** @brief 상자 모양 강체를 만들 때 필요한 크기와 초기 상태다. */
struct RigidBodyDefinition
{
    glm::vec3 halfExtentsMeters{0.1F};
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    double massKilograms = 1.0;
    /** @brief 기체 축 기준 주관성 모멘트다. 비우면 형상과 질량으로 계산한다. */
    std::optional<glm::dvec3> principalInertiaKilogramMetersSquared;
    bool isStatic = false;
};

/** @brief 한 Tick 뒤 강체의 월드 자세와 속도다. */
struct RigidBodyState
{
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 linearVelocityMetersPerSecond{0.0};
    glm::dvec3 angularVelocityRadiansPerSecond{0.0};
};

/**
 * @brief Jolt 강체와 평평한 바닥을 보유하고 고정 시간만큼 계산한다.
 * @details 물리 좌표는 Z축을 위로 사용한다. 힘과 토크의 단위는 각각 뉴턴과 뉴턴·미터다.
 */
class PhysicsWorld
{
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    /** @brief 상자 모양의 움직이는 물체나 고정된 장애물을 만든다. */
    [[nodiscard]] RigidBodyHandle CreateBox(const RigidBodyDefinition& definition);

    /** @brief 여러 상자를 하나의 강체로 묶어 움직이는 물체나 장애물을 만든다. */
    [[nodiscard]] RigidBodyHandle CreateCompoundBoxes(
        const RigidBodyDefinition& definition,
        std::span<const RigidBodyBox> boxes);

    /** @brief 평평한 바닥을 지형 높이 자료로 바꿔 정적 충돌 지형을 만든다. */
    void SetTerrain(const terrain::TerrainGrid& terrain);

    /** @brief 기존 강체의 위치·자세·속도를 지정한 상태로 되돌린다. */
    void SetState(RigidBodyHandle body, const RigidBodyState& state);

    /** @brief 물체 중심에 힘을 가한다. 한 번의 다음 물리 단계에 적용된다. */
    void ApplyForce(RigidBodyHandle body, const glm::dvec3& forceNewtons);

    /** @brief 물체의 중심에서 떨어진 지점에 힘을 가해 회전도 만든다. */
    void ApplyForceAtPoint(
        RigidBodyHandle body,
        const glm::dvec3& forceNewtons,
        const glm::dvec3& worldPositionMeters);

    /** @brief 물체 중심에 회전력을 가한다. 한 번의 다음 물리 단계에 적용된다. */
    void ApplyTorque(RigidBodyHandle body, const glm::dvec3& torqueNewtonMeters);

    /** @brief 고정 시간만큼 계산하고, 요청되면 Jolt 계산 시간을 마이크로초로 기록한다. */
    void Step(double seconds, int collisionSteps = 1, double* durationMicroseconds = nullptr);

    /** @brief 물리 세계에서 강체를 제거한다. 이미 제거했거나 잘못된 핸들이면 false를 반환한다. */
    [[nodiscard]] bool DestroyBody(RigidBodyHandle body);

    /** @brief 현재 물리엔진이 계산한 위치·자세·속도를 읽는다. */
    [[nodiscard]] RigidBodyState State(RigidBodyHandle body) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
