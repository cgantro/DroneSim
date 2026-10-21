#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/detail/PoseMath.h"

#include <cmath>
#include <stdexcept>

namespace grasplink::robotics::kinematics
{
namespace
{
namespace tolerances
{
    // 축이 이보다 짧으면 방향을 계산할 수 없는 값으로 본다.
    inline constexpr double minimumAxisLength = 1.0e-12;
}

namespace messages
{
inline constexpr char invalidJointAxis[] = "RobotKinematics: invalid joint axis";
inline constexpr char emptySpecification[] = "RobotKinematics: empty robot specification";
inline constexpr char nonFiniteBindPivot[] = "RobotKinematics: non-finite bind pivot";
inline constexpr char nonFiniteJointOrigin[] = "RobotKinematics: non-finite joint origin";
inline constexpr char nonFiniteToolOffset[] = "RobotKinematics: non-finite tool offset";
inline constexpr char invalidRobotState[] = "RobotKinematics: invalid RobotState";
inline constexpr char jointCountMismatch[] = "RobotKinematics: joint count mismatch";
inline constexpr char nonFiniteJointPosition[] = "RobotKinematics: non-finite joint position";
}

using models::Axis3;
using models::Pose3;
using models::QuaternionWxyz;
using models::Vec3;
using detail::Add;
using detail::Length;
using detail::Normalize;
using detail::Rotate;
using detail::Subtract;

Axis3 NormalizeAxis(const Axis3& axis)
{
    const double length = Length(axis);
    if (!std::isfinite(length) || length <= tolerances::minimumAxisLength)
        throw std::invalid_argument(messages::invalidJointAxis);

    return {axis.x / length, axis.y / length, axis.z / length};
}

QuaternionWxyz AxisRotation(const Axis3& normalizedAxis, double angle)
{
    const double halfAngle = angle * 0.5;
    const double scale = std::sin(halfAngle);
    return Normalize({std::cos(halfAngle), normalizedAxis.x * scale,
        normalizedAxis.y * scale, normalizedAxis.z * scale});
}
}

RobotKinematics::RobotKinematics(const models::RobotSpecification& specification)
    : specification_(specification)
{
    // FK(정방향 기구학)는 관절각에서 각 링크 자세를 계산한다. 계산에 앞서 모델에 관절 pivot(회전 중심), 축, 선택적 ToolFrame이 유효하게 정의되어 있는지 확인한다.
    if (specification_.joints == nullptr || specification_.jointCount == 0)
        throw std::invalid_argument(messages::emptySpecification);

    normalizedJointAxes_.reserve(specification_.jointCount);
    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const auto& joint = specification_.joints[i];
        if (!std::isfinite(joint.bindPivotMeters.x) || !std::isfinite(joint.bindPivotMeters.y) ||
            !std::isfinite(joint.bindPivotMeters.z))
            throw std::invalid_argument(messages::nonFiniteBindPivot);
        normalizedJointAxes_.push_back(NormalizeAxis(joint.axis));
        if (joint.parentToJointOrigin)
        {
            const auto& origin = *joint.parentToJointOrigin;
            if (!std::isfinite(origin.positionMeters.x) || !std::isfinite(origin.positionMeters.y) ||
                !std::isfinite(origin.positionMeters.z))
                throw std::invalid_argument(messages::nonFiniteJointOrigin);
            Normalize(origin.rotation);
        }
    }
    if (specification_.hasToolFrame)
    {
        const auto& tool = specification_.toolFrameInLastJoint;
        if (!std::isfinite(tool.positionMeters.x) || !std::isfinite(tool.positionMeters.y) ||
            !std::isfinite(tool.positionMeters.z))
            throw std::invalid_argument(messages::nonFiniteToolOffset);
        Normalize(tool.rotation);
    }

    state_.jointLocalRotations.resize(specification_.jointCount);
    state_.jointPositions.resize(specification_.jointCount);
    state_.linkPosesInBaseFrame.resize(specification_.jointCount);
    state_.jointAxesInBaseFrame.resize(specification_.jointCount);
}

const RobotKinematicState& RobotKinematics::Update(const RobotState& state)
{
    // 결과 위치와 회전은 Robot base 기준이다. Scene에서 robotRoot에 적용한 배치 위치와 회전은 아직 포함하지 않는다.
    if (!state.valid)
        throw std::invalid_argument(messages::invalidRobotState);
    return Update(state.jointPositions);
}

const RobotKinematicState& RobotKinematics::Update(const JointVector& jointPositions)
{
    if (jointPositions.size() != specification_.jointCount)
        throw std::invalid_argument(messages::jointCountMismatch);
    for (double position : jointPositions)
        if (!std::isfinite(position))
            throw std::invalid_argument(messages::nonFiniteJointPosition);

    state_.jointPositions = jointPositions;

    QuaternionWxyz parentRotation{};
    Vec3 parentPosition{};
    Vec3 previousBindPivot{};

    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const models::JointSpecification& joint = specification_.joints[i];
        const double position = jointPositions[i];
        const Axis3& normalizedAxis = normalizedJointAxes_[i];

        // 모델 초기 상태(bind pose)에서 이웃한 관절 중심 사이의 차이를 구한다. 부모 관절이 회전하면 이 간격도 함께 회전하므로, 누적된 부모 회전을 적용해 현재 관절 중심을 계산한다.
        Vec3 originPosition;
        QuaternionWxyz originRotation;
        if (joint.parentToJointOrigin)
        {
            // 고정 원점 변환은 이전 관절이 움직인 뒤의 자식 링크 좌표계에서 적용한다.
            originPosition = Add(parentPosition,
                Rotate(parentRotation, joint.parentToJointOrigin->positionMeters));
            originRotation = Normalize(detail::Multiply(parentRotation,
                Normalize(joint.parentToJointOrigin->rotation)));
        }
        else
        {
            // 기존 정적 모델은 각 원점을 base 기준으로 보관하므로 이웃한 원점의 차이로 변환을 만든다.
            const Vec3 bindOffset = i == 0
                ? joint.bindPivotMeters
                : Subtract(joint.bindPivotMeters, previousBindPivot);
            originPosition = Add(parentPosition, Rotate(parentRotation, bindOffset));
            originRotation = parentRotation;
        }

        // 회전축은 관절 원점의 Local 좌표로 정의되어 있으므로 원점 방향까지 적용해 base 좌표로 바꾼다.
        state_.jointAxesInBaseFrame[i] = Rotate(originRotation, normalizedAxis);
        // URDF의 고정 원점 회전 뒤에 관절 자체 회전을 적용한다.
        QuaternionWxyz localRotation{};
        Vec3 childPosition = originPosition;
        if (joint.motionType == models::JointMotionType::Prismatic)
        {
            childPosition = Add(originPosition,
                Rotate(originRotation, {normalizedAxis.x * position,
                    normalizedAxis.y * position, normalizedAxis.z * position}));
        }
        else
        {
            localRotation = AxisRotation(normalizedAxis, position);
        }
        const QuaternionWxyz worldRotation = Normalize(detail::Multiply(originRotation, localRotation));

        state_.jointLocalRotations[i] = localRotation;
        state_.linkPosesInBaseFrame[i] = {childPosition, worldRotation};
        parentPosition = childPosition;
        parentRotation = worldRotation;
        previousBindPivot = joint.bindPivotMeters;
    }

    state_.toolFrameValid = specification_.hasToolFrame;
    if (state_.toolFrameValid)
    {
        // ToolFrame은 모델에 고정된 공구 장착 기준점이며 공구 끝 TCP와 다를 수 있다.
        // 여기서 계산한 모델 자세는 Controller가 실제로 보고한 tcpPose feedback이 아니다.
        const auto& tool = specification_.toolFrameInLastJoint;
        state_.toolFrameInBaseFrame = {
            Add(parentPosition, Rotate(parentRotation, tool.positionMeters)),
            Normalize(detail::Multiply(parentRotation, Normalize(tool.rotation)))};
    }
    return state_;
}

}
