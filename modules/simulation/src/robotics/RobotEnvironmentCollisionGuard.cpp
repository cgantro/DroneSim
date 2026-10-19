#include "simulation/robotics/RobotEnvironmentCollisionGuard.h"

#include "scene/Entity.h"
#include "scene/TransformSystemModule.h"

#include "physics/PhysicsWorld.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/core/IGripperController.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/planning/PlanningPolicy.h"

#include "simulation/robotics/RobotTransformAdapter.h"
#include "simulation/robotics/GripperTransformAdapter.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#if GRASPLINK_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) ((void)0)
#endif

namespace grasplink::simulation::robotics
{
using grasplink::scene::Entity;
using grasplink::scene::TransformSystemModule;

namespace
{
glm::mat4 PoseMatrix(const grasplink::robotics::models::Pose3& pose)
{
    const glm::vec3 position{
        static_cast<float>(pose.positionMeters.x),
        static_cast<float>(pose.positionMeters.y),
        static_cast<float>(pose.positionMeters.z)};
    const glm::quat rotation = glm::normalize(glm::quat{
        static_cast<float>(pose.rotation.w),
        static_cast<float>(pose.rotation.x),
        static_cast<float>(pose.rotation.y),
        static_cast<float>(pose.rotation.z)});
    return glm::translate(glm::mat4{1.0F}, position) * glm::mat4_cast(rotation);
}

grasplink::physics::Transform ToPhysicsTransform(const glm::mat4& matrix)
{
    glm::mat3 axes(matrix);
    for (int axis = 0; axis < 3; ++axis)
        axes[axis] = glm::normalize(axes[axis]);
    return {glm::vec3(matrix[3]), glm::normalize(glm::quat_cast(axes))};
}

const glm::mat4* FindWorldTransform(
    const std::vector<grasplink::simulation::RobotPhysicsAdapter::CandidateWorldTransform>& transforms,
    flecs::entity entity)
{
    for (const auto& candidate : transforms)
        if (candidate.entity.GetHandle() == entity)
            return &candidate.worldTransform;
    return nullptr;
}

std::string DescribeIdentity(SelfCollisionIdentity identity)
{
    switch (identity.part)
    {
    case SelfCollisionPart::Base: return "Base";
    case SelfCollisionPart::Link: return "Link" + std::to_string(identity.linkIndex + 1);
    case SelfCollisionPart::GripperBody: return "GripperBody";
    }
    return "Unknown";
}
}

RobotEnvironmentCollisionGuard::RobotEnvironmentCollisionGuard(
    grasplink::robotics::backends::simulation::SimRobotController& robotController,
    grasplink::robotics::IGripperController& gripperController,
    grasplink::robotics::kinematics::RobotKinematics& robotKinematics,
    RobotTransformAdapter& robotTransformAdapter,
    grasplink::simulation::RobotPhysicsAdapter& robotPhysicsAdapter,
    grasplink::robotics::kinematics::GripperKinematics& gripperKinematics,
    GripperTransformAdapter& gripperTransformAdapter,
    flecs::world& world,
    grasplink::physics::PhysicsWorld& physicsWorld,
    grasplink::simulation::PhysicsSystemModule& physicsSystem,
    const Entity& robotRoot)
    : m_RobotController(robotController),
      m_GripperController(gripperController),
      m_RobotKinematics(robotKinematics),
      m_RobotTransformAdapter(robotTransformAdapter),
      m_RobotPhysicsAdapter(robotPhysicsAdapter),
      m_GripperKinematics(gripperKinematics),
      m_GripperTransformAdapter(gripperTransformAdapter),
      m_World(world),
      m_PhysicsWorld(physicsWorld),
      m_CandidateState(robotController.GetStateView()),
      m_SafeJointPositions(robotController.GetStateView().jointPositionRadians),
      m_PhysicsSystem(physicsSystem)
{
    m_RobotRoot = robotRoot;
    m_GripperRoot = robotRoot.FindChildByNameRecursive("Gripper");
    std::unordered_set<std::size_t> registeredLinkIndices;
    std::size_t baseProxyCount = 0;
    std::size_t gripperBodyCount = 0;
    std::vector<Entity> pending{robotRoot};
    while (!pending.empty())
    {
        Entity entity = pending.back();
        pending.pop_back();
        if (entity.Has<RigidBody>())
        {
            const auto layer = entity.Get<RigidBody>().collisionLayer;
            if (entity.Has<RobotBaseEnvironmentProxy>())
            {
                ++baseProxyCount;
                m_BaseEnvironmentEntity = entity.GetHandle();
                m_SelfCollisionProxies.push_back({
                    entity.GetHandle(), {SelfCollisionPart::Base, 0}});
            }
            else if (layer == grasplink::physics::CollisionLayer::Robot &&
                entity.Has<RobotCollisionProxy>())
            {
                const std::size_t linkIndex = entity.Get<RobotCollisionProxy>().linkIndex;
                const std::size_t jointIndex = entity.Get<RobotCollisionProxy>().jointIndex;
                if (linkIndex == RobotCollisionProxy::InvalidLinkIndex ||
                    jointIndex == RobotCollisionProxy::InvalidLinkIndex)
                    throw std::invalid_argument("Robot collision proxy has no link identity.");
                if (linkIndex >= grasplink::robotics::models::hanwha::kHcr12aLinks.size())
                    throw std::invalid_argument("Robot collision proxy link identity is out of range.");
                if (!registeredLinkIndices.insert(linkIndex).second)
                    throw std::invalid_argument("Robot collision proxy link identity is duplicated.");
                if (linkIndex == 5)
                    m_Link6JointIndex = jointIndex;
            }
            else if (layer == grasplink::physics::CollisionLayer::Gripper &&
                entity.Has<GripperCollisionProxy>() &&
                entity.Get<GripperCollisionProxy>().part == GripperCollisionPart::Body)
            {
                ++gripperBodyCount;
            }
            if (layer == grasplink::physics::CollisionLayer::Robot ||
                layer == grasplink::physics::CollisionLayer::Gripper)
            {
                m_CollisionEntities.push_back(entity.GetHandle());
                if (layer == grasplink::physics::CollisionLayer::Robot &&
                    entity.Has<RobotCollisionProxy>())
                {
                    const auto& proxy = entity.Get<RobotCollisionProxy>();
                    m_SelfCollisionProxies.push_back({
                        entity.GetHandle(), {SelfCollisionPart::Link, proxy.linkIndex}});
                }
                else if (layer == grasplink::physics::CollisionLayer::Gripper &&
                    entity.Has<GripperCollisionProxy>() &&
                    entity.Get<GripperCollisionProxy>().part == GripperCollisionPart::Body)
                {
                    m_SelfCollisionProxies.push_back({
                        entity.GetHandle(), {SelfCollisionPart::GripperBody, 0}});
                }
            }
            if (layer == grasplink::physics::CollisionLayer::Gripper)
                m_GripperCollisionEntities.push_back(entity);
        }
        const auto children = entity.GetChildren();
        pending.insert(pending.end(), children.begin(), children.end());
    }

    if (baseProxyCount != 1 || registeredLinkIndices.size() != 6 || gripperBodyCount != 1 ||
        !m_GripperRoot || m_Link6JointIndex == RobotCollisionProxy::InvalidLinkIndex)
        throw std::invalid_argument("Robot self-collision requires one Base, six indexed HCR-12A links, and one gripper body.");
    for (std::size_t index = 0; index < grasplink::robotics::models::hanwha::kHcr12aLinks.size(); ++index)
        if (registeredLinkIndices.count(index) == 0)
            throw std::invalid_argument("Robot collision proxy link identities are incomplete.");

    const auto& initialKinematics = m_RobotKinematics.Update(robotController.GetStateView());
    const glm::mat4 initialLink6World = robotRoot.GetWorldMatrix() *
        PoseMatrix(initialKinematics.linkPosesInBaseFrame.at(m_Link6JointIndex));
    m_GripperRootInLink6 = glm::inverse(initialLink6World) * m_GripperRoot.GetWorldMatrix();
    m_CandidateCollisionTransforms.resize(m_CollisionEntities.size());
    m_CandidateSelfCollisionTransforms.resize(m_SelfCollisionProxies.size());

    m_RobotController.SetJointStateValidityChecker([this](const grasplink::robotics::JointVector& candidateJoints)
    {
        return ValidateCandidatePose(candidateJoints);
    });
    m_RobotController.SetJointStateValidityDiagnosticProvider([this]
    {
        return DescribeSelfCollision();
    });
}

RobotEnvironmentCollisionGuard::~RobotEnvironmentCollisionGuard()
{
    m_RobotController.SetJointStateValidityChecker({});
    m_RobotController.SetJointStateValidityDiagnosticProvider({});
}

void RobotEnvironmentCollisionGuard::CaptureSafeJointPose()
{
    const auto& currentJoints = m_RobotController.GetStateView().jointPositionRadians;
    std::copy(currentJoints.begin(), currentJoints.end(), m_SafeJointPositions.begin());
}

void RobotEnvironmentCollisionGuard::RestoreSafePoseIfOverlapping()
{
    const auto& currentJoints = m_RobotController.GetStateView().jointPositionRadians;
    if (m_RobotController.GetStateView().mode != grasplink::robotics::RobotMode::Moving &&
        currentJoints == m_SafeJointPositions)
        return;

    auto validityChecker = [this](const grasplink::robotics::JointVector& joints)
    {
        return ValidateCandidatePose(joints);
    };
    auto validationPolicy = grasplink::robotics::planning::kDefaultPlanningPolicy;
    validationPolicy.jointCollisionSampleSpacingRadians = 0.002;
    const auto pathInvalidity = grasplink::robotics::planning::ValidateJointPath(
        m_SafeJointPositions, currentJoints, m_RobotController.GetSpecification(),
        validityChecker, validationPolicy);
    if (pathInvalidity == grasplink::robotics::planning::JointStateInvalidity::None)
    {
        CaptureSafeJointPose();
        return;
    }

    const auto collisionReason = pathInvalidity ==
        grasplink::robotics::planning::JointStateInvalidity::SelfCollision
        ? grasplink::robotics::ErrorCode::SelfCollision
        : grasplink::robotics::ErrorCode::EnvironmentContact;

    // 각 고정 tick의 끝 자세뿐 아니라 직전 안전 자세부터 이번 자세까지도 검사한다. 중간 표본 하나라도 충돌하면 물리 계산 전에 되돌린다.
    if (!m_RobotController.RestoreCollisionSafeState(m_SafeJointPositions, collisionReason))
        return;

    ApplyJointPose(m_RobotController.GetStateView());
}

grasplink::robotics::planning::JointStateInvalidity RobotEnvironmentCollisionGuard::ValidateCandidatePose(
    const grasplink::robotics::JointVector& candidateJoints)
{
    m_LastSelfCollisionDescription.clear();
    m_CandidateState.jointPositionRadians = candidateJoints;
    const auto& robotState = m_RobotKinematics.Update(m_CandidateState);
    BuildCandidateProxyTransforms(robotState);
    if (RobotAssemblyOverlapsEnvironment(m_CandidateCollisionTransforms))
        return grasplink::robotics::planning::JointStateInvalidity::EnvironmentCollision;
    if (RobotAssemblyHasSelfCollision(m_CandidateSelfCollisionTransforms))
    {
        m_LastSelfCollisionDescription += " at joints [deg]";
        for (const double angle : candidateJoints)
            m_LastSelfCollisionDescription += " " + std::to_string(angle * 180.0 / 3.14159265358979323846);
        return grasplink::robotics::planning::JointStateInvalidity::SelfCollision;
    }
    m_LastSelfCollisionDescription.clear();
    return grasplink::robotics::planning::JointStateInvalidity::None;
}

void RobotEnvironmentCollisionGuard::ApplyJointPose(const grasplink::robotics::RobotState& state)
{
    const auto& kinematicState = m_RobotKinematics.Update(state);
    m_RobotTransformAdapter.Apply(kinematicState);
    m_RobotPhysicsAdapter.Apply(kinematicState);
    const auto& gripperState = m_GripperKinematics.Update(m_GripperController.GetState());
    m_GripperTransformAdapter.Apply(gripperState);
    TransformSystemModule::UpdateWorldTransforms(m_World);
}

bool RobotEnvironmentCollisionGuard::RobotAssemblyOverlapsEnvironment()
{
    for (std::size_t i = 0; i < m_CollisionEntities.size(); ++i)
        m_CandidateCollisionTransforms[i] = ToPhysicsTransform(
            Entity(m_CollisionEntities[i]).GetWorldMatrix());
    return RobotAssemblyOverlapsEnvironment(m_CandidateCollisionTransforms);
}

bool RobotEnvironmentCollisionGuard::RobotAssemblyOverlapsEnvironment(
    const std::vector<grasplink::physics::Transform>& candidateTransforms) const
{
    ZoneScopedN("CollisionCheck");
    const auto baseEnvironmentBodyHandle = m_PhysicsSystem.GetBodyHandle(m_BaseEnvironmentEntity);
    for (std::size_t i = 0; i < m_CollisionEntities.size(); ++i)
    {
        const flecs::entity collisionEntity = m_CollisionEntities[i];
        const auto collisionBodyHandle = m_PhysicsSystem.GetBodyHandle(collisionEntity);
        if (!collisionBodyHandle.IsValid())
            return true;

        const bool isLink1 = collisionEntity.has<RobotCollisionProxy>() &&
            collisionEntity.get<RobotCollisionProxy>().linkIndex == 0;
        const bool overlapsEnvironment = isLink1 && baseEnvironmentBodyHandle.IsValid()
            ? m_PhysicsWorld.OverlapsEnvironmentAt(
                collisionBodyHandle, candidateTransforms[i], baseEnvironmentBodyHandle)
            : m_PhysicsWorld.OverlapsEnvironmentAt(collisionBodyHandle, candidateTransforms[i]);
        if (overlapsEnvironment)
            return true;
    }
    return false;
}

bool RobotEnvironmentCollisionGuard::RobotAssemblyHasSelfCollision()
{
    for (std::size_t i = 0; i < m_SelfCollisionProxies.size(); ++i)
        m_CandidateSelfCollisionTransforms[i] = ToPhysicsTransform(
            Entity(m_SelfCollisionProxies[i].entity).GetWorldMatrix());
    return RobotAssemblyHasSelfCollision(m_CandidateSelfCollisionTransforms);
}

bool RobotEnvironmentCollisionGuard::RobotAssemblyHasSelfCollision(
    const std::vector<grasplink::physics::Transform>& candidateTransforms) const
{
    ZoneScopedN("SelfCollisionCheck");
    for (std::size_t firstIndex = 0; firstIndex < m_SelfCollisionProxies.size(); ++firstIndex)
    {
        const SelfCollisionProxy& first = m_SelfCollisionProxies[firstIndex];
        const auto firstBody = m_PhysicsSystem.GetBodyHandle(first.entity);
        if (!firstBody.IsValid())
        {
            m_LastSelfCollisionDescription = DescribeIdentity(first.identity) + " has no physics body";
            return true;
        }

        for (std::size_t secondIndex = firstIndex + 1;
            secondIndex < m_SelfCollisionProxies.size();
            ++secondIndex)
        {
            const SelfCollisionProxy& second = m_SelfCollisionProxies[secondIndex];
            if (IsAllowedSelfCollision(first.identity, second.identity))
                continue;

            const auto secondBody = m_PhysicsSystem.GetBodyHandle(second.entity);
            if (!secondBody.IsValid())
            {
                m_LastSelfCollisionDescription = DescribeIdentity(first.identity) + " / " +
                    DescribeIdentity(second.identity) + " has no physics body";
                return true;
            }
            if (m_PhysicsWorld.OverlapsBodiesAt(
                    firstBody, candidateTransforms[firstIndex],
                    secondBody, candidateTransforms[secondIndex]))
            {
                m_LastSelfCollisionDescription = DescribeIdentity(first.identity) + " / " +
                    DescribeIdentity(second.identity);
                return true;
            }
        }
    }
    return false;
}

std::string RobotEnvironmentCollisionGuard::DescribeSelfCollision() const
{
    return m_LastSelfCollisionDescription;
}

void RobotEnvironmentCollisionGuard::BuildCandidateProxyTransforms(
    const grasplink::robotics::kinematics::RobotKinematicState& robotState)
{
    ZoneScopedN("CollisionCandidateFK");
    m_RobotPhysicsAdapter.BuildCandidateWorldTransforms(
        robotState, m_RobotRoot.GetWorldMatrix(), m_CandidateProxyWorldTransforms);
    const glm::mat4* link6World = nullptr;
    for (const SelfCollisionProxy& proxy : m_SelfCollisionProxies)
    {
        if (proxy.identity.part == SelfCollisionPart::Link && proxy.identity.linkIndex == 5)
        {
            link6World = FindWorldTransform(m_CandidateProxyWorldTransforms, proxy.entity);
            break;
        }
    }
    if (link6World == nullptr)
        throw std::logic_error("Robot self-collision candidate is missing Link6.");

    const glm::mat4 candidateGripperRootWorld = *link6World * m_GripperRootInLink6;
    const auto& gripperState = m_GripperKinematics.Update(m_GripperController.GetState());
    m_GripperTransformAdapter.BuildCandidateWorldTransforms(
        gripperState, candidateGripperRootWorld,
        m_GripperCollisionEntities, m_CandidateGripperWorldTransforms);
    for (std::size_t i = 0; i < m_GripperCollisionEntities.size(); ++i)
        m_CandidateProxyWorldTransforms.push_back({
            m_GripperCollisionEntities[i], m_CandidateGripperWorldTransforms[i]});

    for (std::size_t i = 0; i < m_CollisionEntities.size(); ++i)
    {
        const glm::mat4* transform = FindWorldTransform(m_CandidateProxyWorldTransforms, m_CollisionEntities[i]);
        if (transform == nullptr)
            throw std::logic_error("Robot collision candidate is missing a collision proxy transform.");
        m_CandidateCollisionTransforms[i] = ToPhysicsTransform(*transform);
    }
    for (std::size_t i = 0; i < m_SelfCollisionProxies.size(); ++i)
    {
        const glm::mat4* transform = FindWorldTransform(
            m_CandidateProxyWorldTransforms, m_SelfCollisionProxies[i].entity);
        if (transform == nullptr)
            throw std::logic_error("Robot self-collision candidate is missing a proxy transform.");
        m_CandidateSelfCollisionTransforms[i] = ToPhysicsTransform(*transform);
    }
}

}
