#pragma once

#include "physics/PhysicsTypes.h"
#include "robotics/core/ControlTypes.h"
#include "robotics/planning/StateValidity.h"
#include "simulation/robotics/RobotSelfCollisionPolicy.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"

#include <flecs.h>
#include <glm/glm.hpp>

#include <optional>
#include <vector>
#include <string>

namespace grasplink::scene { class Entity; }

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::robotics
{
class IGripperController;
namespace backends::simulation { class SimRobotController; }
namespace kinematics { class RobotKinematics; class GripperKinematics; }
}

namespace grasplink::simulation
{
class PhysicsSystemModule;
class RobotPhysicsAdapter;
namespace robotics { class RobotTransformAdapter; class GripperTransformAdapter; }
}

namespace grasplink::simulation::robotics
{
/**
 * @brief 로봇과 그리퍼의 환경 충돌 및 자기 충돌을 검사하고 안전 자세를 복원한다.
 * @details 고정된 환경 물체와의 충돌은 Jolt 환경 쿼리로 검사하고, 로봇 부품끼리의 충돌은 허용 충돌 행렬에 따라 쌍별로 검사한다. 후보 자세는 FK로 각 충돌 프록시의 월드 변환을 직접 계산하므로 Scene을 변경하지 않는다.
 * 제어기, 계산기, 어댑터, Flecs World와 물리 시스템은 이 감시기가 빌려 쓰므로 감시기보다 오래 살아야 한다. Scene이 소유한 충돌 프록시도 감시기가 쓰는 동안 유지해야 한다. 자세 복원은 Controller 명령과 같은 스레드에서 순서대로 실행해야 한다.
 */
class RobotEnvironmentCollisionGuard final
{
public:
    /**
     * @brief 그리퍼 충돌 검사에 필요한 기준 Entity와 계산 객체를 묶는다.
     * @details `root`는 Scene이 소유하는 그리퍼 기준 Entity다. 나머지 세 객체는 Guard가 빌려 쓰므로 Guard보다 오래 살아야 한다.
     */
    struct GripperSupport
    {
        /// Scene이 소유하는 그리퍼 기준 Entity다.
        grasplink::scene::Entity root;
        grasplink::robotics::IGripperController& controller;
        grasplink::robotics::kinematics::GripperKinematics& kinematics;
        GripperTransformAdapter& transformAdapter;
    };

    RobotEnvironmentCollisionGuard(
        grasplink::robotics::backends::simulation::SimRobotController& robotController,
        grasplink::robotics::kinematics::RobotKinematics& robotKinematics,
        RobotTransformAdapter& robotTransformAdapter,
        grasplink::simulation::RobotPhysicsAdapter& robotPhysicsAdapter,
        flecs::world& world,
        grasplink::physics::PhysicsWorld& physicsWorld,
        grasplink::simulation::PhysicsSystemModule& physicsSystem,
        const grasplink::scene::Entity& robotRoot,
        std::optional<GripperSupport> gripperSupport = std::nullopt);
    ~RobotEnvironmentCollisionGuard();

    RobotEnvironmentCollisionGuard(const RobotEnvironmentCollisionGuard&) = delete;
    RobotEnvironmentCollisionGuard& operator=(const RobotEnvironmentCollisionGuard&) = delete;

    /** @brief 현재 Controller 관절각을 다음 고정 tick 충돌 검사에서 쓸 안전 자세로 저장한다. */
    void CaptureSafeJointPose();

    /** @brief 이동 중 현재 자세가 Environment 또는 비허용 로봇 링크 쌍과 겹치면 직전 안전 자세로 복원한다. */
    void RestoreSafePoseIfOverlapping();

private:
    struct SelfCollisionProxy
    {
        flecs::entity entity;
        SelfCollisionIdentity identity;
    };

    struct EndLinkProxy
    {
        std::size_t jointIndex;
        std::size_t linkIndex;
    };

    [[nodiscard]] grasplink::robotics::planning::JointStateInvalidity ValidateCandidatePose(
        const grasplink::robotics::JointVector& candidateJoints);
    void ApplyJointPose(const grasplink::robotics::RobotState& state);
    void CollectCollisionProxies();
    [[nodiscard]] bool RobotAssemblyOverlapsEnvironment();
    [[nodiscard]] bool RobotAssemblyOverlapsEnvironment(
        const std::vector<grasplink::physics::Transform>& candidateTransforms) const;
    [[nodiscard]] bool RobotAssemblyHasSelfCollision();
    [[nodiscard]] bool RobotAssemblyHasSelfCollision(
        const std::vector<grasplink::physics::Transform>& candidateTransforms) const;
    [[nodiscard]] std::string DescribeSelfCollision() const;
    void BuildCandidateProxyTransforms(
        const grasplink::robotics::kinematics::RobotKinematicState& robotState);

    grasplink::robotics::backends::simulation::SimRobotController& m_RobotController;
    grasplink::robotics::kinematics::RobotKinematics& m_RobotKinematics;
    RobotTransformAdapter& m_RobotTransformAdapter;
    grasplink::simulation::RobotPhysicsAdapter& m_RobotPhysicsAdapter;
    std::optional<GripperSupport> m_GripperSupport;
    flecs::world& m_World;
    grasplink::physics::PhysicsWorld& m_PhysicsWorld;
    grasplink::robotics::RobotState m_CandidateState;
    grasplink::robotics::JointVector m_SafeJointPositions;
    std::vector<flecs::entity> m_CollisionEntities;
    std::vector<SelfCollisionProxy> m_SelfCollisionProxies;
    std::vector<grasplink::scene::Entity> m_GripperCollisionEntities;
    std::vector<grasplink::simulation::RobotPhysicsAdapter::CandidateWorldTransform> m_CandidateProxyWorldTransforms;
    std::vector<glm::mat4> m_CandidateGripperWorldTransforms;
    std::vector<grasplink::physics::Transform> m_CandidateCollisionTransforms;
    std::vector<grasplink::physics::Transform> m_CandidateSelfCollisionTransforms;
    grasplink::scene::Entity m_RobotRoot;
    mutable std::string m_LastSelfCollisionDescription;
    std::optional<EndLinkProxy> m_EndLinkProxy;
    flecs::entity m_BaseEnvironmentEntity;
    grasplink::simulation::PhysicsSystemModule& m_PhysicsSystem;
    /// 마지막 관절 자식 링크 기준 그리퍼 root의 고정 위치와 방향이다.
    glm::mat4 m_GripperRootInEndLink{1.0F};
};
}
