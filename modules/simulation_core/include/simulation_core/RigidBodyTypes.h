#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <cstdint>

namespace dronesim::simulation
{
/** @brief 한 물리 월드 안의 강체를 가리킨다. */
struct RigidBodyHandle
{
    std::uint32_t value = 0;
};

/** @brief 기체 중심 기준 위치와 자세를 가진 상자 충돌 모양이다. */
struct RigidBodyBox
{
    glm::vec3 halfExtentsMeters{0.1F};
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
};

/** @brief 강체를 만들 때 필요한 시작 위치와 자세다. */
struct RigidBodyDefinition
{
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
};

/** @brief 강체의 위치·자세·속도다. 위치와 선속도는 ENU, 각속도는 기체 기준이다. */
struct RigidBodyState
{
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 linearVelocityMetersPerSecond{0.0};
    glm::dvec3 angularVelocityRadiansPerSecond{0.0};
};
}
