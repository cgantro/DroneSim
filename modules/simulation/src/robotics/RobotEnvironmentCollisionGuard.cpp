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

#include "simulation/robotics/RobotTransformAdapter.h"
#include "simulation/robotics/GripperTransformAdapter.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/robotics/RobotModelTransforms.h"

#include <algorithm>
#include <stdexcept>
#include <unordered_set>
#include <utility>
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
using grasplink::physics::Transform;
using grasplink::physics::PhysicsBodyHandle;
using grasplink::robotics::ErrorCode;
using grasplink::robotics::JointVector;
using grasplink::robotics::RobotMode;
using grasplink::robotics::RobotState;
using grasplink::robotics::backends::simulation::SimRobotController;
using grasplink::robotics::kinematics::RobotKinematicState;
using grasplink::robotics::kinematics::RobotKinematics;
using grasplink::robotics::planning::JointStateInvalidity;
using grasplink::scene::Entity;
using grasplink::scene::TransformSystemModule;
using grasplink::simulation::PhysicsSystemModule;
using grasplink::simulation::RobotPhysicsAdapter;

namespace
{
namespace limits
{
inline constexpr std::size_t maximumBaseProxyCount = 1;
inline constexpr std::size_t expectedGripperBodyProxyCount = 1;
inline constexpr std::size_t noGripperBodyProxyCount = 0;
inline constexpr std::size_t firstJointIndex = 0;
inline constexpr std::size_t oneBasedIndexOffset = 1;
inline constexpr double pi = 3.14159265358979323846;
inline constexpr double degreesPerHalfTurn = 180.0;
}

namespace messages
{
inline constexpr char missingRobotProxyIdentity[] =
    "Robot collision proxy has no link identity.";
inline constexpr char duplicateRobotLinkIdentity[] =
    "Robot collision proxy link identity is duplicated.";
inline constexpr char missingCollisionGeometry[] =
    "로봇 충돌 형상을 확인할 수 없습니다.";
inline constexpr char missingGripperRoot[] =
    "그리퍼 기준 Entity를 찾을 수 없습니다.";
inline constexpr char jointAnglesLabel[] = " at joints [deg]";
inline constexpr char missingEndJointTransform[] =
    "로봇 끝 관절의 위치를 찾지 못했습니다.";
inline constexpr char missingCollisionProxyTransform[] =
    "Robot collision candidate is missing a collision proxy transform.";
inline constexpr char missingSelfCollisionProxyTransform[] =
    "Robot self-collision candidate is missing a proxy transform.";
inline constexpr char missingPhysicsBody[] = " has no physics body";
}

namespace part_names
{
inline constexpr char base[] = "Base";
inline constexpr char link[] = "Link";
inline constexpr char gripperBody[] = "GripperBody";
inline constexpr char unknown[] = "Unknown";
}

Transform ToPhysicsTransform(const glm::mat4& matrix)
{
    glm::mat3 axes(matrix);
    for (int axis = 0; axis < 3; ++axis)
        axes[axis] = glm::normalize(axes[axis]);
    return {glm::vec3(matrix[3]), glm::normalize(glm::quat_cast(axes))};
}

const glm::mat4* FindWorldTransform(
    const std::vector<RobotPhysicsAdapter::CandidateWorldTransform>& transforms,
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
    case SelfCollisionPart::Base: return part_names::base;
    case SelfCollisionPart::Link:
        return part_names::link +
            std::to_string(identity.linkIndex + limits::oneBasedIndexOffset);
    case SelfCollisionPart::GripperBody: return part_names::gripperBody;
    }
    return part_names::unknown;
}
}

RobotEnvironmentCollisionGuard::RobotEnvironmentCollisionGuard(
    SimRobotController& robotController,
    RobotKinematics& robotKinematics,
    RobotTransformAdapter& robotTransformAdapter,
    RobotPhysicsAdapter& robotPhysicsAdapter,
    flecs::world& world,
    grasplink::physics::PhysicsWorld& physicsWorld,
    PhysicsSystemModule& physicsSystem,
    const Entity& robotRoot,
    std::optional<GripperSupport> gripperSupport)
    : m_RobotController(robotController),
      m_RobotKinematics(robotKinematics),
      m_RobotTransformAdapter(robotTransformAdapter),
      m_RobotPhysicsAdapter(robotPhysicsAdapter),
      m_GripperSupport(std::move(gripperSupport)),
      m_World(world),
      m_PhysicsWorld(physicsWorld),
      m_CandidateState(robotController.GetStateView()),
      m_SafeJointPositions(robotController.GetStateView().jointPositions),
      m_RobotRoot(robotRoot),
      m_PhysicsSystem(physicsSystem)
{
    if (m_GripperSupport && !m_GripperSupport->root)
        throw std::invalid_argument(messages::missingGripperRoot);

    CollectCollisionProxies();
    m_SelfCollisionBodyHandleCache.resize(m_SelfCollisionProxies.size());

    const auto& initialKinematics = m_RobotKinematics.Update(robotController.GetStateView());
    if (m_GripperSupport)
    {
        const glm::mat4 initialEndJointWorld = robotRoot.GetWorldMatrix() *
            PoseMatrix(initialKinematics.linkPosesInBaseFrame.at(m_EndLinkProxy->jointIndex));
        m_GripperRootInEndLink = glm::inverse(initialEndJointWorld) *
            m_GripperSupport->root.GetWorldMatrix();
    }
    m_CandidateCollisionTransforms.resize(m_CollisionEntities.size());
    m_CandidateSelfCollisionTransforms.resize(m_SelfCollisionProxies.size());

    m_RobotController.SetJointStateValidityChecker([this](const JointVector& candidateJoints)
    {
        return ValidateCandidatePose(candidateJoints);
    });
    m_RobotController.SetJointStateValidityDiagnosticProvider([this]
    {
        return DescribeSelfCollision();
    });
}

void RobotEnvironmentCollisionGuard::CollectCollisionProxies()
{
    std::unordered_set<std::size_t> registeredLinkIndices;
    std::size_t baseProxyCount = 0;
    std::size_t gripperBodyCount = 0;
    std::vector<Entity> pending{m_RobotRoot};
    while (!pending.empty())
    {
        Entity entity = pending.back();
        pending.pop_back();
        const auto children = entity.GetChildren();
        pending.insert(pending.end(), children.begin(), children.end());
        if (!entity.Has<RigidBody>())
            continue;

        const auto layer = entity.Get<RigidBody>().collisionLayer;
        if (entity.Has<RobotBaseEnvironmentProxy>())
        {
            ++baseProxyCount;
            m_BaseEnvironmentEntity = entity.GetHandle();
            m_SelfCollisionProxies.push_back({entity.GetHandle(), {
                SelfCollisionPart::Base, 0, SelfCollisionIdentity::InvalidIndex}});
        }
        switch (layer)
        {
        case grasplink::physics::CollisionLayer::Robot:
        {
            if (!entity.Has<RobotCollisionProxy>())
                throw std::invalid_argument(messages::missingRobotProxyIdentity);

            const auto& proxy = entity.Get<RobotCollisionProxy>();
            if (proxy.linkIndex == RobotCollisionProxy::InvalidLinkIndex ||
                proxy.jointIndex == RobotCollisionProxy::InvalidLinkIndex)
                throw std::invalid_argument(messages::missingRobotProxyIdentity);
            if (!registeredLinkIndices.insert(proxy.linkIndex).second)
                throw std::invalid_argument(messages::duplicateRobotLinkIdentity);
            const bool isBaseMountedLink = proxy.jointIndex ==
                limits::firstJointIndex;
            m_CollisionEntities.push_back({entity.GetHandle(), isBaseMountedLink});
            if (proxy.jointIndex + limits::oneBasedIndexOffset ==
                m_RobotController.GetSpecification().jointCount)
            {
                m_EndLinkProxy = EndLinkProxy{proxy.jointIndex, proxy.linkIndex};
            }
            m_SelfCollisionProxies.push_back({entity.GetHandle(), {
                SelfCollisionPart::Link, proxy.linkIndex, proxy.jointIndex}});
            break;
        }
        case grasplink::physics::CollisionLayer::Gripper:
        {
            m_CollisionEntities.push_back({entity.GetHandle(), false});
            m_GripperCollisionEntities.push_back(entity);
            if (!entity.Has<GripperCollisionProxy>() ||
                entity.Get<GripperCollisionProxy>().part != GripperCollisionPart::Body)
                break;

            ++gripperBodyCount;
            m_SelfCollisionProxies.push_back({
                entity.GetHandle(), {SelfCollisionPart::GripperBody,
                    SelfCollisionIdentity::InvalidIndex,
                    SelfCollisionIdentity::InvalidIndex}});
            break;
        }
        case grasplink::physics::CollisionLayer::Environment:
        case grasplink::physics::CollisionLayer::DynamicObject:
        case grasplink::physics::CollisionLayer::Count:
            break;
        }
    }

    const bool hasGripper = m_GripperSupport.has_value();
    const std::size_t expectedGripperBodyCount = hasGripper
        ? limits::expectedGripperBodyProxyCount
        : limits::noGripperBodyProxyCount;
    if (baseProxyCount > limits::maximumBaseProxyCount ||
        registeredLinkIndices.empty() ||
        gripperBodyCount != expectedGripperBodyCount ||
        (hasGripper && !m_EndLinkProxy))
        throw std::invalid_argument(messages::missingCollisionGeometry);
    for (SelfCollisionProxy& proxy : m_SelfCollisionProxies)
        if (proxy.identity.part == SelfCollisionPart::GripperBody)
        {
            proxy.identity.linkIndex = m_EndLinkProxy->linkIndex;
            proxy.identity.jointIndex = m_EndLinkProxy->jointIndex;
        }
}

RobotEnvironmentCollisionGuard::~RobotEnvironmentCollisionGuard()
{
    m_RobotController.SetJointStateValidityChecker({});
    m_RobotController.SetJointStateValidityDiagnosticProvider({});
}

void RobotEnvironmentCollisionGuard::CaptureSafeJointPose()
{
    const auto& currentJoints = m_RobotController.GetStateView().jointPositions;
    std::copy(currentJoints.begin(), currentJoints.end(), m_SafeJointPositions.begin());
}

void RobotEnvironmentCollisionGuard::RestoreSafePoseIfOverlapping()
{
    if (m_RobotController.GetStateView().mode != RobotMode::Moving)
        return;

    const bool environmentContact = RobotAssemblyOverlapsEnvironment();
    const bool selfCollision = !environmentContact && RobotAssemblyHasSelfCollision();
    if (!environmentContact && !selfCollision)
        return;

    const auto collisionReason = environmentContact
        ? ErrorCode::EnvironmentContact
        : ErrorCode::SelfCollision;

    // Jolt 접촉 callback은 Kinematic 로봇 부품과 Static 바닥의 겹침을 항상 알리지 않으므로 물리 계산 전에 직전 안전 자세를 복원한다.
    if (!m_RobotController.RestoreCollisionSafeState(m_SafeJointPositions, collisionReason))
        return;

    ApplyJointPose(m_RobotController.GetStateView());
}

JointStateInvalidity RobotEnvironmentCollisionGuard::ValidateCandidatePose(
    const JointVector& candidateJoints)
{
    m_LastSelfCollisionDescription.clear();
    m_CandidateState.jointPositions = candidateJoints;
    const auto& robotState = m_RobotKinematics.Update(m_CandidateState);
    BuildCandidateProxyTransforms(robotState);
    if (RobotAssemblyOverlapsEnvironment(m_CandidateCollisionTransforms))
        return JointStateInvalidity::EnvironmentCollision;
    if (RobotAssemblyHasSelfCollision(m_CandidateSelfCollisionTransforms))
    {
        m_LastSelfCollisionDescription += messages::jointAnglesLabel;
        for (const double angle : candidateJoints)
            m_LastSelfCollisionDescription += " " + std::to_string(
                angle * limits::degreesPerHalfTurn / limits::pi);
        return JointStateInvalidity::SelfCollision;
    }
    return JointStateInvalidity::None;
}

void RobotEnvironmentCollisionGuard::ApplyJointPose(const RobotState& state)
{
    const auto& kinematicState = m_RobotKinematics.Update(state);
    m_RobotTransformAdapter.Apply(kinematicState);
    m_RobotPhysicsAdapter.Apply(kinematicState);
    if (m_GripperSupport)
    {
        const auto& gripperState = m_GripperSupport->kinematics.Update(
            m_GripperSupport->controller.GetState());
        m_GripperSupport->transformAdapter.Apply(gripperState);
    }
    TransformSystemModule::UpdateWorldTransforms(m_World);
}

bool RobotEnvironmentCollisionGuard::RobotAssemblyOverlapsEnvironment()
{
    for (std::size_t i = 0; i < m_CollisionEntities.size(); ++i)
        m_CandidateCollisionTransforms[i] = ToPhysicsTransform(
            Entity(m_CollisionEntities[i].entity).GetWorldMatrix());
    return RobotAssemblyOverlapsEnvironment(m_CandidateCollisionTransforms);
}

bool RobotEnvironmentCollisionGuard::RobotAssemblyOverlapsEnvironment(
    const std::vector<Transform>& candidateTransforms) const
{
    ZoneScopedN("CollisionCheck");
    const auto baseEnvironmentBodyHandle = m_PhysicsSystem.GetBodyHandle(m_BaseEnvironmentEntity);
    for (std::size_t i = 0; i < m_CollisionEntities.size(); ++i)
    {
        const CollisionProxy& collisionProxy = m_CollisionEntities[i];
        const flecs::entity collisionEntity = collisionProxy.entity;
        const auto collisionBodyHandle = m_PhysicsSystem.GetBodyHandle(collisionEntity);
        if (!collisionBodyHandle.IsValid())
            return true;

        // jointIndex는 base_link부터 번호를 매긴 움직이는 관절이다. 첫 관절에 붙은 링크만 Base 연결부 겹침을 허용한다.
        const bool overlapsEnvironment = collisionProxy.isBaseMountedLink &&
            baseEnvironmentBodyHandle.IsValid()
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
    const std::vector<Transform>& candidateTransforms)
{
    ZoneScopedN("SelfCollisionCheck");
    std::fill(m_SelfCollisionBodyHandleCache.begin(),
        m_SelfCollisionBodyHandleCache.end(), std::nullopt);
    const auto bodyHandleAt = [this](std::size_t index) -> PhysicsBodyHandle
    {
        auto& cachedHandle = m_SelfCollisionBodyHandleCache[index];
        if (!cachedHandle)
            cachedHandle = m_PhysicsSystem.GetBodyHandle(m_SelfCollisionProxies[index].entity);
        return *cachedHandle;
    };

    for (std::size_t firstIndex = 0; firstIndex < m_SelfCollisionProxies.size(); ++firstIndex)
    {
        const SelfCollisionProxy& first = m_SelfCollisionProxies[firstIndex];
        const auto firstBody = bodyHandleAt(firstIndex);
        if (!firstBody.IsValid())
        {
            m_LastSelfCollisionDescription = DescribeIdentity(first.identity) +
                messages::missingPhysicsBody;
            return true;
        }

        for (std::size_t secondIndex = firstIndex + 1;
            secondIndex < m_SelfCollisionProxies.size();
            ++secondIndex)
        {
            const SelfCollisionProxy& second = m_SelfCollisionProxies[secondIndex];
            if (IsAllowedSelfCollision(first.identity, second.identity))
                continue;

            const auto secondBody = bodyHandleAt(secondIndex);
            if (!secondBody.IsValid())
            {
                m_LastSelfCollisionDescription = DescribeIdentity(first.identity) + " / " +
                    DescribeIdentity(second.identity) + messages::missingPhysicsBody;
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
    const RobotKinematicState& robotState)
{
    ZoneScopedN("CollisionCandidateFK");
    m_RobotPhysicsAdapter.BuildCandidateWorldTransforms(
        robotState, m_RobotRoot.GetWorldMatrix(), m_CandidateProxyWorldTransforms);
    if (m_GripperSupport)
    {
        if (!m_EndLinkProxy || m_EndLinkProxy->jointIndex >= robotState.linkPosesInBaseFrame.size())
            throw std::logic_error(messages::missingEndJointTransform);
        // 장착 오프셋은 마지막 움직이는 관절의 자식 링크 기준으로 저장하므로 후보 자세에서도 같은 기준을 사용한다.
        const glm::mat4 candidateEndJointWorld = m_RobotRoot.GetWorldMatrix() *
            PoseMatrix(robotState.linkPosesInBaseFrame[m_EndLinkProxy->jointIndex]);
        const glm::mat4 candidateGripperRootWorld = candidateEndJointWorld * m_GripperRootInEndLink;
        const auto& gripperState = m_GripperSupport->kinematics.Update(
            m_GripperSupport->controller.GetState());
        m_GripperSupport->transformAdapter.BuildCandidateWorldTransforms(
            gripperState, candidateGripperRootWorld,
            m_GripperCollisionEntities, m_CandidateGripperWorldTransforms);
        for (std::size_t i = 0; i < m_GripperCollisionEntities.size(); ++i)
            m_CandidateProxyWorldTransforms.push_back({
                m_GripperCollisionEntities[i], m_CandidateGripperWorldTransforms[i]});
    }

    for (std::size_t i = 0; i < m_CollisionEntities.size(); ++i)
    {
        const glm::mat4* transform = FindWorldTransform(
            m_CandidateProxyWorldTransforms, m_CollisionEntities[i].entity);
        if (transform == nullptr)
            throw std::logic_error(messages::missingCollisionProxyTransform);
        m_CandidateCollisionTransforms[i] = ToPhysicsTransform(*transform);
    }
    for (std::size_t i = 0; i < m_SelfCollisionProxies.size(); ++i)
    {
        const glm::mat4* transform = FindWorldTransform(
            m_CandidateProxyWorldTransforms, m_SelfCollisionProxies[i].entity);
        if (transform == nullptr)
            throw std::logic_error(messages::missingSelfCollisionProxyTransform);
        m_CandidateSelfCollisionTransforms[i] = ToPhysicsTransform(*transform);
    }
}

}
