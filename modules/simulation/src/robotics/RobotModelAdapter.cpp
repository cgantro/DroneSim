#include "simulation/robotics/RobotModelAdapter.h"
#include "simulation/robotics/RobotModelTransforms.h"

#include <stdexcept>

namespace grasplink::simulation::robotics
{
namespace
{
struct RobotModelAdapterErrorMessages final
{
    inline static constexpr char unsupportedFixedJoint[] = "URDF의 활성 관절 목록에 고정 관절이 들어 있습니다.";
    inline static constexpr char invalidBaseOrToolLink[] = "URDF의 시작 링크 또는 tool0 위치가 잘못되었습니다.";
    inline static constexpr char noMovingJoint[] = "URDF의 base_link와 tool0 사이에 움직이는 관절이 없습니다.";
    inline static constexpr char invalidActiveJointIndex[] = "URDF의 움직이는 관절 위치가 잘못되었습니다.";
};
}

using grasplink::model::RobotJoint;
using grasplink::model::RobotJointType;
using grasplink::robotics::models::JointSpecification;
using grasplink::robotics::models::LinkSpecification;
using grasplink::robotics::models::RobotSpecification;

namespace
{
using grasplink::robotics::models::Pose3;

grasplink::robotics::models::JointMotionType MotionType(RobotJointType type)
{
    switch (type)
    {
    case RobotJointType::Revolute: return grasplink::robotics::models::JointMotionType::Revolute;
    case RobotJointType::Continuous: return grasplink::robotics::models::JointMotionType::Continuous;
    case RobotJointType::Prismatic: return grasplink::robotics::models::JointMotionType::Prismatic;
    case RobotJointType::Fixed: break;
    }
    throw std::runtime_error(RobotModelAdapterErrorMessages::unsupportedFixedJoint);
}
}

RobotModelAdapter::RobotModelAdapter(const grasplink::model::RobotModel& model)
    : modelName_(model.name)
{
    if (model.toolLink >= model.links.size() || model.baseLink >= model.links.size())
        throw std::runtime_error(RobotModelAdapterErrorMessages::invalidBaseOrToolLink);
    if (model.activeJoints.empty())
        throw std::runtime_error(RobotModelAdapterErrorMessages::noMovingJoint);

    const auto linkBindPoses = BuildRobotLinkBindPoses(model);
    jointNames_.reserve(model.activeJoints.size());
    linkNames_.reserve(model.activeJoints.size());
    joints_.reserve(model.activeJoints.size());
    links_.reserve(model.activeJoints.size());

    std::size_t previousChildLink = model.baseLink;
    for (const std::size_t index : model.activeJoints)
    {
        if (index >= model.joints.size())
            throw std::runtime_error(RobotModelAdapterErrorMessages::invalidActiveJointIndex);
        const RobotJoint& source = model.joints[index];

        // 관절값이 0일 때 자식 링크의 원점은 URDF 관절 원점과 같은 위치와 방향이다.
        const Pose3& jointPoseInBase = linkBindPoses[source.childLink];
        const Pose3 jointOriginInPreviousLink = RelativeRobotPose(
            linkBindPoses[previousChildLink], jointPoseInBase);
        jointNames_.push_back(source.name);
        // 연속 회전 관절의 무한 범위도 유지한다. IK와 경로 계획이 이 관절을 유한 한계가 없는 관절로 처리한다.
        joints_.push_back({jointNames_.back(),
            jointPoseInBase.positionMeters,
            {source.axis.x, source.axis.y, source.axis.z},
            source.lowerLimit,
            source.upperLimit,
            source.maximumVelocity,
            jointOriginInPreviousLink,
            MotionType(source.type)});
        linkNames_.push_back(model.links[source.childLink].name);
        links_.push_back({linkNames_.back(), joints_.size() - 1});
        previousChildLink = source.childLink;
    }

    const Pose3 toolFrame = RelativeRobotPose(
        linkBindPoses[previousChildLink], linkBindPoses[model.toolLink]);
    specification_ = {{}, modelName_, joints_.data(), joints_.size(), links_.data(), links_.size(),
        toolFrame, true};
}

const RobotSpecification& RobotModelAdapter::Specification() const noexcept
{
    return specification_;
}

} // namespace grasplink::simulation::robotics
