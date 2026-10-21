#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
#include <memory>
#include <optional>

namespace aerosimlab::simulation
{
/** @brief 물리엔진에서 만든 강체를 가리킨다. */
struct RigidBodyHandle
{
    std::uint32_t value = 0;
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

    /** @brief 정해진 시간만큼 중력과 접촉을 계산한다. */
    void Step(double seconds, int collisionSteps = 1);

    /** @brief 현재 물리엔진이 계산한 위치·자세·속도를 읽는다. */
    [[nodiscard]] RigidBodyState State(RigidBodyHandle body) const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
