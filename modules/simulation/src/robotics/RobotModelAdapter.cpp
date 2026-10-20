#include "simulation/robotics/RobotModelAdapter.h"
#include "simulation/robotics/RobotModelTransforms.h"

#include <glm/gtc/constants.hpp>

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

struct JointPositionLimits final
{
    double minimum;
    double maximum;
};

JointPositionLimits PositionLimits(const RobotJoint& joint)
{
    if (joint.type == RobotJointType::Continuous)
        return {-glm::two_pi<double>(), glm::two_pi<double>()};
    return {joint.lowerLimit, joint.upperLimit};
}

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
    : model_(model)
{
    if (model_.toolLink >= model_.links.size() || model_.baseLink >= model_.links.size())
        throw std::runtime_error(RobotModelAdapterErrorMessages::invalidBaseOrToolLink);
    if (model_.activeJoints.empty())
        throw std::runtime_error(RobotModelAdapterErrorMessages::noMovingJoint);

    const auto linkBindPoses = BuildRobotLinkBindPoses(model_);
    jointNames_.reserve(model_.activeJoints.size());
    joints_.reserve(model_.activeJoints.size());
    links_.reserve(model_.activeJoints.size());

    std::size_t previousChildLink = model_.baseLink;
    for (const std::size_t index : model_.activeJoints)
    {
        if (index >= model_.joints.size())
            throw std::runtime_error(RobotModelAdapterErrorMessages::invalidActiveJointIndex);
        const RobotJoint& source = model_.joints[index];

        const Pose3 jointPoseInBase = ComposeRobotPose(
            linkBindPoses[source.parentLink],
            source.originPositionMeters,
            source.originRotation);
        const Pose3 jointOriginInPreviousLink = RelativeRobotPose(
            linkBindPoses[previousChildLink], jointPoseInBase);
        const JointPositionLimits limits = PositionLimits(source);

        jointNames_.push_back(source.name);
        // URDF의 연속 회전 관절에는 끝 각도가 없다. 수치 계산에는 한 바퀴 범위를 주고, 경로 계획에서 같은 자세의 회전 표현을 맞춘다.
        joints_.push_back({jointNames_.back(),
            jointPoseInBase.positionMeters,
            {source.axis.x, source.axis.y, source.axis.z},
            limits.minimum,
            limits.maximum,
            source.maximumVelocity,
            jointOriginInPreviousLink,
            MotionType(source.type)});
        links_.push_back({model_.links[source.childLink].name, joints_.size() - 1});
        previousChildLink = source.childLink;
    }

    const Pose3 toolFrame = RelativeRobotPose(
        linkBindPoses[previousChildLink], linkBindPoses[model_.toolLink]);
    specification_ = {{}, model_.name, joints_.data(), joints_.size(), links_.data(), links_.size(),
        toolFrame, true};
}

const RobotSpecification& RobotModelAdapter::Specification() const noexcept
{
    return specification_;
}

} // namespace grasplink::simulation::robotics
