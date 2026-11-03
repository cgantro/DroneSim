#include "graphics/X500GimbalAssembly.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <stdexcept>
#include <string_view>

namespace dronesim::graphics
{
X500GimbalAssembly::X500GimbalAssembly(const GlbModel& model)
{
    const auto parts = model.PartBounds();
    baseTransforms_.reserve(parts.size());
    for (const auto& part : parts)
        baseTransforms_.push_back(part.nodeTransform);

    const auto findPart = [parts](std::string_view name)
    {
        for (std::size_t index = 0; index < parts.size(); ++index)
        {
            if (parts[index].nodeName == name)
                return index;
        }
        throw std::logic_error("기체 카메라 모델에 짐벌 부품이 없습니다.");
    };

    yawArmIndex_ = findPart("PX4 CGO3 yaw_arm");
    rollArmIndex_ = findPart("PX4 CGO3 roll_arm");
    cameraIndex_ = findPart("PX4 CGO3 camera");

    // 각 GLB 노드 원점이 짐벌 축점이므로 별도 위치 보정값을 두지 않는다.
    yawPivot_ = glm::vec3(parts[yawArmIndex_].nodeTransform[3]);
    rollPivot_ = glm::vec3(parts[rollArmIndex_].nodeTransform[3]);
    pitchPivot_ = glm::vec3(parts[cameraIndex_].nodeTransform[3]);
}

std::vector<glm::mat4> X500GimbalAssembly::PartTransforms(
    const GimbalPose& pose) const
{
    std::vector<glm::mat4> transforms = baseTransforms_;

    // 자식 부품에는 부모 관절 회전을 앞에서부터 곱해 짐벌의 연결 순서를 유지한다.
    const glm::mat4 yawRotation = RotationAroundPivot(
        yawPivot_, {0.0F, -1.0F, 0.0F}, pose.yawDegrees);
    const glm::mat4 rollRotation = RotationAroundPivot(
        rollPivot_, {-1.0F, 0.0F, 0.0F}, pose.rollDegrees);
    const glm::mat4 pitchRotation = RotationAroundPivot(
        pitchPivot_, {0.0F, 0.0F, 1.0F}, pose.pitchUpDegrees);

    transforms[yawArmIndex_] = yawRotation * transforms[yawArmIndex_];
    transforms[rollArmIndex_] =
        yawRotation * rollRotation * transforms[rollArmIndex_];
    transforms[cameraIndex_] =
        yawRotation * rollRotation * pitchRotation * transforms[cameraIndex_];
    return transforms;
}

glm::mat4 X500GimbalAssembly::RotationAroundPivot(
    const glm::vec3& pivot, const glm::vec3& axis, double degrees)
{
    const glm::mat4 toPivot = glm::translate(glm::mat4(1.0F), pivot);
    const glm::mat4 fromPivot = glm::translate(glm::mat4(1.0F), -pivot);
    const glm::quat rotation = glm::angleAxis(
        glm::radians(static_cast<float>(degrees)), axis);
    return toPivot * glm::mat4_cast(rotation) * fromPivot;
}
} // namespace dronesim::graphics
