#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/robotics/RobotCollisionGeometryBuilder.h"
#include "simulation/robotics/RobotModelTransforms.h"

#include "robotics/kinematics/RobotKinematics.h"
#include "scene/Scene.h"
#include "scene/TransformSystemModule.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <cstddef>
#include <stdexcept>
#include <string>
#include <utility>

namespace grasplink::simulation
{
using grasplink::model::RobotModel;
using grasplink::scene::Entity;
using grasplink::scene::Scene;

namespace
{
namespace messages
{
inline constexpr char invalidRobotRoot[] = "RobotPhysicsAdapter: invalid robot root";
inline constexpr char collisionProxyRemoved[] =
    "RobotPhysicsAdapter: a required collision proxy has been removed from its Scene";
inline constexpr char linkPoseCountMismatch[] =
    "RobotPhysicsAdapter: LinkPose count mismatch";
}

namespace proxy_names
{
inline constexpr char base[] = "Base_CollisionProxy";
inline constexpr char suffix[] = "_CollisionProxy";
}

glm::mat4 LinkPoseMatrix(const grasplink::robotics::models::Pose3& jointPose,
    const grasplink::robotics::models::Pose3& linkOffset)
{
    return robotics::PoseMatrix(jointPose) * robotics::PoseMatrix(linkOffset);
}
}

RobotPhysicsAdapter::RobotPhysicsAdapter(
    Scene& scene,
    const Entity& robotRoot,
    const RobotModel& model)
{
    if (!robotRoot)
        throw std::runtime_error(messages::invalidRobotRoot);
    auto geometry = robotics::collision_geometry::Build(model);
    links_.reserve(geometry.links.size());
    std::vector<Entity> createdEntities;
    createdEntities.reserve(geometry.links.size() + 1U);

    try
    {
        base_ = scene.CreateEntity(proxy_names::base);
        createdEntities.push_back(base_);
        base_.SetParent(robotRoot);
        base_.Add<RobotCollisionProxy>();
        base_.Add<RobotBaseEnvironmentProxy>();
        base_.set<RigidBody>(RigidBody{
                grasplink::physics::BodyMotionType::Static,
                grasplink::physics::CollisionLayer::Environment})
            .set<Colliders>(Colliders{std::move(geometry.baseShapes)});

        for (std::size_t i = 0; i < geometry.links.size(); ++i)
        {
            auto& linkGeometry = geometry.links[i];
            Entity entity = scene.CreateEntity(linkGeometry.name + proxy_names::suffix);
            createdEntities.push_back(entity);
            // FK 자세는 robotRoot 기준이므로 proxy를 그 자식으로 두어 로봇의 장면 배치가 함께 적용되게 한다.
            entity.SetParent(robotRoot);
            entity.set<RobotCollisionProxy>(RobotCollisionProxy{i, linkGeometry.jointIndex})
                .set<RigidBody>(RigidBody{
                grasplink::physics::BodyMotionType::Kinematic,
                grasplink::physics::CollisionLayer::Robot})
                .set<Colliders>(Colliders{std::move(linkGeometry.shapes)});
            links_.push_back({entity, linkGeometry.jointIndex, linkGeometry.poseInJointFrame});
        }
    }
    catch (...)
    {
        // 생성 도중 실패하면 Scene과 물리 시스템에 남은 일부 proxy를 함께 정리한다.
        for (auto entity = createdEntities.rbegin(); entity != createdEntities.rend(); ++entity)
            entity->Destroy();
        throw;
    }
}

void RobotPhysicsAdapter::Apply(
    const grasplink::robotics::kinematics::RobotKinematicState& state)
{
    // 한 링크라도 FK 결과나 Scene에서 빠졌으면 아무 proxy도 움직이지 않는다.
    ValidateState(state);
    for (LinkBinding& link : links_)
    {
        const glm::mat4 transform = GetLinkLocalTransform(link, state);
        link.entity.SetLocalPosition(glm::vec3{transform[3]});
        link.entity.SetLocalRotation(glm::normalize(glm::quat_cast(glm::mat3{transform})));
    }
}

glm::mat4 RobotPhysicsAdapter::GetLinkLocalTransform(
    const LinkBinding& link,
    const grasplink::robotics::kinematics::RobotKinematicState& state) const
{
    return LinkPoseMatrix(state.linkPosesInBaseFrame[link.jointIndex], link.poseInJointFrame);
}

void RobotPhysicsAdapter::ValidateState(
    const grasplink::robotics::kinematics::RobotKinematicState& state) const
{
    if (!base_)
        throw std::runtime_error(messages::collisionProxyRemoved);

    for (const LinkBinding& link : links_)
    {
        if (!link.entity)
            throw std::runtime_error(messages::collisionProxyRemoved);
        if (link.jointIndex >= state.linkPosesInBaseFrame.size())
            throw std::invalid_argument(messages::linkPoseCountMismatch);
    }
}

void RobotPhysicsAdapter::BuildCandidateWorldTransforms(
    const grasplink::robotics::kinematics::RobotKinematicState& state,
    const glm::mat4& robotRootWorldTransform,
    std::vector<CandidateWorldTransform>& output) const
{
    output.clear();
    ValidateState(state);
    output.reserve(links_.size() + 1U);
    const glm::mat4 baseLocal = grasplink::scene::TransformSystemModule::ComposeLocalMatrix(
        base_.GetLocalPosition(), base_.GetLocalRotation(), base_.GetLocalScale());
    output.push_back({base_, robotRootWorldTransform * baseLocal});

    for (const LinkBinding& link : links_)
    {
        const glm::mat4 local = GetLinkLocalTransform(link, state);
        output.push_back({link.entity, robotRootWorldTransform * local});
    }
}
}
