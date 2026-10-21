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
struct RobotPhysicsAdapterMessages final
{
    inline static constexpr char invalidRobotRoot[] = "RobotPhysicsAdapter: invalid robot root";
    inline static constexpr char collisionProxyRemoved[] =
        "RobotPhysicsAdapter: a required collision proxy has been removed from its Scene";
    inline static constexpr char linkPoseCountMismatch[] =
        "RobotPhysicsAdapter: LinkPose count mismatch";
};

struct RobotPhysicsProxyNames final
{
    inline static constexpr char base[] = "Base_CollisionProxy";
    inline static constexpr char suffix[] = "_CollisionProxy";
};

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
        throw std::runtime_error(RobotPhysicsAdapterMessages::invalidRobotRoot);
    auto geometry = robotics::collision_geometry::Build(model);
    if (!geometry.baseShapes.empty())
    {
        base_ = scene.CreateEntity(RobotPhysicsProxyNames::base);
        base_.SetParent(robotRoot);
        base_.Add<RobotCollisionProxy>();
        base_.Add<RobotBaseEnvironmentProxy>();
        base_.set<RigidBody>(RigidBody{
                grasplink::physics::BodyMotionType::Static,
                grasplink::physics::CollisionLayer::Environment})
            .set<Colliders>(Colliders{std::move(geometry.baseShapes)});
    }

    links_.reserve(geometry.links.size());
    for (std::size_t i = 0; i < geometry.links.size(); ++i)
    {
        auto& linkGeometry = geometry.links[i];
        Entity entity = scene.CreateEntity(
            linkGeometry.name + RobotPhysicsProxyNames::suffix);
        // FK에서 나온 위치와 회전은 Robot base 기준이다. proxy를 robotRoot의 자식으로 두어 Scene에서 로봇에 설정한 배치 변환이 부모 계층을 통해 추가된다.
        entity.SetParent(robotRoot);
        entity.set<RobotCollisionProxy>(RobotCollisionProxy{i, linkGeometry.jointIndex})
            .set<RigidBody>(RigidBody{
                grasplink::physics::BodyMotionType::Kinematic,
                grasplink::physics::CollisionLayer::Robot})
            .set<Colliders>(Colliders{std::move(linkGeometry.shapes)});
        links_.push_back({entity, linkGeometry.jointIndex, linkGeometry.poseInJointFrame});
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
        throw std::runtime_error(RobotPhysicsAdapterMessages::collisionProxyRemoved);

    for (const LinkBinding& link : links_)
    {
        if (!link.entity)
            throw std::runtime_error(RobotPhysicsAdapterMessages::collisionProxyRemoved);
        if (link.jointIndex >= state.linkPosesInBaseFrame.size())
            throw std::invalid_argument(RobotPhysicsAdapterMessages::linkPoseCountMismatch);
    }
}

void RobotPhysicsAdapter::BuildCandidateWorldTransforms(
    const grasplink::robotics::kinematics::RobotKinematicState& state,
    const glm::mat4& robotRootWorldTransform,
    std::vector<CandidateWorldTransform>& output) const
{
    output.clear();
    ValidateState(state);
    output.reserve(links_.size() + (base_ ? 1U : 0U));
    if (base_)
    {
        const glm::mat4 local = grasplink::scene::TransformSystemModule::ComposeLocalMatrix(
            base_.GetLocalPosition(), base_.GetLocalRotation(), base_.GetLocalScale());
        output.push_back({base_, robotRootWorldTransform * local});
    }

    for (const LinkBinding& link : links_)
    {
        const glm::mat4 local = GetLinkLocalTransform(link, state);
        output.push_back({link.entity, robotRootWorldTransform * local});
    }
}
}
