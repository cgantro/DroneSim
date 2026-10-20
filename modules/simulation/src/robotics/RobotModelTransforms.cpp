#include "simulation/robotics/RobotModelTransforms.h"

#include <glm/gtx/quaternion.hpp>

#include <stdexcept>

namespace grasplink::simulation::robotics
{
namespace
{
struct RobotModelTransformErrorMessages final
{
    inline static constexpr char invalidBaseLinkIndex[] = "URDF 모델의 base_link 위치가 잘못되었습니다.";
    inline static constexpr char jointReferencesMissingLink[] = "URDF 관절이 없는 링크를 가리킵니다.";
    inline static constexpr char multipleParentJoints[] = "URDF 링크에는 부모 관절을 하나만 둘 수 있습니다.";
    inline static constexpr char linkDisconnectedFromBase[] = "URDF 링크가 base_link와 이어져 있지 않습니다.";
};
}

grasplink::robotics::models::Pose3 ComposeRobotPose(
    const grasplink::robotics::models::Pose3& parent,
    const glm::dvec3& childPosition,
    const glm::dquat& childRotation)
{
    const glm::dquat parentRotation{parent.rotation.w, parent.rotation.x,
        parent.rotation.y, parent.rotation.z};
    const glm::dvec3 parentPosition{parent.positionMeters.x,
        parent.positionMeters.y, parent.positionMeters.z};
    const glm::dvec3 position = parentPosition + parentRotation * childPosition;
    const glm::dquat rotation = glm::normalize(parentRotation * childRotation);
    return {{position.x, position.y, position.z},
        {rotation.w, rotation.x, rotation.y, rotation.z}};
}

grasplink::robotics::models::Pose3 RelativeRobotPose(
    const grasplink::robotics::models::Pose3& parent,
    const grasplink::robotics::models::Pose3& target)
{
    const glm::dquat parentRotation{parent.rotation.w, parent.rotation.x,
        parent.rotation.y, parent.rotation.z};
    const glm::dquat inverseParentRotation = glm::inverse(parentRotation);
    const glm::dvec3 parentPosition{parent.positionMeters.x,
        parent.positionMeters.y, parent.positionMeters.z};
    const glm::dvec3 targetPosition{target.positionMeters.x,
        target.positionMeters.y, target.positionMeters.z};
    const glm::dvec3 position = inverseParentRotation * (targetPosition - parentPosition);
    const glm::dquat targetRotation{target.rotation.w, target.rotation.x,
        target.rotation.y, target.rotation.z};
    const glm::dquat rotation = glm::normalize(inverseParentRotation * targetRotation);
    return {{position.x, position.y, position.z},
        {rotation.w, rotation.x, rotation.y, rotation.z}};
}

std::vector<grasplink::robotics::models::Pose3> BuildRobotLinkBindPoses(
    const grasplink::model::RobotModel& model)
{
    using grasplink::robotics::models::Pose3;

    if (model.baseLink >= model.links.size())
        throw std::invalid_argument(RobotModelTransformErrorMessages::invalidBaseLinkIndex);

    std::vector<std::vector<std::size_t>> childJoints(model.links.size());
    std::vector<bool> hasParentJoint(model.links.size(), false);
    for (std::size_t jointIndex = 0; jointIndex < model.joints.size(); ++jointIndex)
    {
        const auto& joint = model.joints[jointIndex];
        if (joint.parentLink >= model.links.size() || joint.childLink >= model.links.size())
            throw std::invalid_argument(RobotModelTransformErrorMessages::jointReferencesMissingLink);
        if (joint.childLink == model.baseLink || hasParentJoint[joint.childLink])
            throw std::invalid_argument(RobotModelTransformErrorMessages::multipleParentJoints);
        hasParentJoint[joint.childLink] = true;
        childJoints[joint.parentLink].push_back(jointIndex);
    }

    std::vector<Pose3> poses(model.links.size());
    std::vector<bool> visited(model.links.size(), false);
    std::vector<std::size_t> pending{model.baseLink};
    visited[model.baseLink] = true;

    // 부모 링크를 먼저 꺼내면 관절 원점 변환을 base_link 기준으로 한 번씩 누적할 수 있다.
    for (std::size_t next = 0; next < pending.size(); ++next)
    {
        const std::size_t parentLink = pending[next];
        const Pose3& parentPose = poses[parentLink];
        const glm::dquat parentRotation{parentPose.rotation.w, parentPose.rotation.x,
            parentPose.rotation.y, parentPose.rotation.z};
        const glm::dvec3 parentPosition{parentPose.positionMeters.x,
            parentPose.positionMeters.y, parentPose.positionMeters.z};

        for (const std::size_t jointIndex : childJoints[parentLink])
        {
            const auto& joint = model.joints[jointIndex];
            const glm::dquat childRotation = glm::normalize(parentRotation * joint.originRotation);
            const glm::dvec3 childPosition = parentPosition + parentRotation * joint.originPositionMeters;
            poses[joint.childLink] = {
                {childPosition.x, childPosition.y, childPosition.z},
                {childRotation.w, childRotation.x, childRotation.y, childRotation.z}};
            visited[joint.childLink] = true;
            pending.push_back(joint.childLink);
        }
    }

    for (bool linkWasReached : visited)
        if (!linkWasReached)
            throw std::invalid_argument(RobotModelTransformErrorMessages::linkDisconnectedFromBase);
    return poses;
}

} // namespace grasplink::simulation::robotics
