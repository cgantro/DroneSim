#include "simulation/robotics/RobotTransformAdapter.h"
#include "simulation/robotics/RobotModelTransforms.h"
#include "scene/TransformSystemModule.h"
#include "scene/TransformComponents.h"

#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <stdexcept>
#include <string>

namespace grasplink::simulation::robotics
{
namespace
{
struct RobotTransformAdapterConstants final
{
    inline static constexpr double bindJointPosition = 0.0;
    inline static constexpr float bindPoseTolerance = 1.0e-5F;
};

struct RobotTransformAdapterMessages final
{
    inline static constexpr char invalidRobotRoot[] = "RobotTransformAdapter: invalid robot root";
    inline static constexpr char emptySpecification[] =
        "RobotTransformAdapter: empty robot specification";
    inline static constexpr char missingJointPrefix[] = "RobotTransformAdapter: joint not found: ";
    inline static constexpr char bindPoseMismatch[] =
        "RobotTransformAdapter: GLB hierarchy or bind pivot differs from robot specification";
    inline static constexpr char jointStateCountMismatch[] =
        "RobotTransformAdapter: joint state count mismatch";
    inline static constexpr char sceneRemoved[] =
        "RobotTransformAdapter: robot Scene has been removed";
};
}

using grasplink::scene::Rotation;
using grasplink::scene::Entity;
using grasplink::scene::TransformSystemModule;

RobotTransformAdapter::RobotTransformAdapter(
    const Entity& robotRoot,
    const ::grasplink::robotics::models::RobotSpecification& specification)
{
    if (!robotRoot)
        throw std::runtime_error(RobotTransformAdapterMessages::invalidRobotRoot);

    if (specification.joints == nullptr || specification.jointCount == 0)
        throw std::invalid_argument(RobotTransformAdapterMessages::emptySpecification);

    ::grasplink::robotics::kinematics::RobotKinematics bindKinematics(specification);
    const auto& bindState = bindKinematics.Update(
        ::grasplink::robotics::JointVector(
            specification.jointCount, RobotTransformAdapterConstants::bindJointPosition));
    joints_.reserve(specification.jointCount);

    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const auto& jointSpec = specification.joints[i];
        const Entity joint = robotRoot.FindChildByNameRecursive(std::string{jointSpec.name});
        if (!joint)
            throw std::runtime_error(
                std::string{RobotTransformAdapterMessages::missingJointPrefix} +
                std::string{jointSpec.name});

        // 관절의 Local 변환과 중간 부모 변환을 누적해 모델 base 기준 pivot을 구한다. Scene 안에서 로봇을 배치하는 robotRoot 변환은 이 bind 검증에 포함하지 않는다.
        glm::mat4 bindInBase = TransformSystemModule::ComposeLocalMatrix(
            joint.GetLocalPosition(), joint.GetLocalRotation(), joint.GetLocalScale());
        Entity parent = joint.GetParent();
        bool followsPreviousJoint = i == 0;
        while (parent && parent != robotRoot)
        {
            // 첫 관절은 robotRoot 아래에 있어야 하고 이후 관절은 직전 관절의 자손이어야 한다. GLB의 중간 link node는 경로에 있어도 된다.
            if (i > 0 && parent == joints_[i - 1].entity)
                followsPreviousJoint = true;
            bindInBase = TransformSystemModule::ComposeLocalMatrix(
                parent.GetLocalPosition(), parent.GetLocalRotation(), parent.GetLocalScale()) * bindInBase;
            parent = parent.GetParent();
        }
        const glm::mat4 expectedBind = PoseMatrix(bindState.linkPosesInBaseFrame[i]);
        // GLB 기준 자세는 URDF 원점이 정한 위치와 방향을 모두 따라야 한다. 회전이 있는 모델도 같은 초기 변환으로 검증한다.
        bool matchesBind = parent == robotRoot && followsPreviousJoint;
        for (int column = 0; column < 4; ++column)
            matchesBind = matchesBind && glm::all(glm::lessThanEqual(
                glm::abs(bindInBase[column] - expectedBind[column]),
                glm::vec4{RobotTransformAdapterConstants::bindPoseTolerance}));
        if (!matchesBind)
            throw std::invalid_argument(RobotTransformAdapterMessages::bindPoseMismatch);

        const glm::quat bindRotation = Rotation{joint.GetLocalRotation()};
        glm::vec3 prismaticAxisInParentFrame{0.0F};
        if (jointSpec.motionType == ::grasplink::robotics::models::JointMotionType::Prismatic)
        {
            const glm::vec3 axis{static_cast<float>(jointSpec.axis.x),
                static_cast<float>(jointSpec.axis.y), static_cast<float>(jointSpec.axis.z)};
            prismaticAxisInParentFrame = bindRotation * glm::normalize(axis);
        }
        joints_.push_back({joint, joint.GetLocalPosition(), bindRotation,
            prismaticAxisInParentFrame, jointSpec.motionType});
    }
}

void RobotTransformAdapter::Apply(const ::grasplink::robotics::kinematics::RobotKinematicState& state)
{
    // FK 결과는 사양 순서로 나오므로 연결한 GLB 관절 수와 회전 결과 수가 같아야 한다.
    if (state.jointLocalRotations.size() != joints_.size() || state.jointPositions.size() != joints_.size())
        throw std::invalid_argument(RobotTransformAdapterMessages::jointStateCountMismatch);

    // 뒤쪽 관절이 사라졌을 때 앞쪽 관절만 바뀌지 않도록 먼저 모두 확인한다.
    for (const JointBinding& binding : joints_)
        if (!binding.entity)
            throw std::runtime_error(RobotTransformAdapterMessages::sceneRemoved);

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        // 모델 원점 회전을 유지하고, 그 뒤에 FK가 계산한 관절 움직임을 곱한다.
        const auto& pose = state.jointLocalRotations[i];
        // 모델 quaternion의 성분 순서는 [w,x,y,z]이며 GLM 생성자도 같은 순서를 받는다. 계산은 double 정밀도로 전달되고 Entity 저장 때 float로 바뀐다.
        const glm::quat motion{static_cast<float>(pose.w), static_cast<float>(pose.x),
            static_cast<float>(pose.y), static_cast<float>(pose.z)};
        auto& binding = joints_[i];
        if (binding.motionType == ::grasplink::robotics::models::JointMotionType::Prismatic)
        {
            binding.entity.SetLocalPosition(binding.bindPosition +
                binding.prismaticAxisInParentFrame * static_cast<float>(state.jointPositions[i]));
        }
        else
        {
            binding.entity.SetLocalRotation(glm::normalize(binding.bindRotation * motion));
        }
    }
}

} // namespace grasplink::simulation::robotics
