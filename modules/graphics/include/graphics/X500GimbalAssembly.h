#pragma once

#include "graphics/GlbModel.h"

#include <glm/glm.hpp>

#include <cstddef>
#include <vector>

namespace dronesim::graphics
{
/** @brief X500 GLB에 연결된 CGO3 짐벌의 세 축 각도다. */
struct GimbalPose
{
    double yawDegrees = 0.0;
    double rollDegrees = 0.0;
    double pitchUpDegrees = 0.0;
};

/**
 * @brief X500 GLB의 본체와 회전 짐벌 사이의 부품 변환을 계산한다.
 * @details GLB의 부모 관계는 모델에 저장되어 있으므로, 각 관절 축점과 원본 변환을
 * 한 번 읽고 렌더링 때에는 현재 짐벌 각도만 적용한다.
 */
class X500GimbalAssembly final
{
public:
    /** @brief 짐벌 회전 부품을 찾아 원본 변환과 축점을 보관한다. */
    explicit X500GimbalAssembly(const GlbModel& model);

    /** @brief GLB 메시 순서에 맞는 현재 부품별 변환을 만든다. */
    [[nodiscard]] std::vector<glm::mat4> PartTransforms(
        const GimbalPose& pose) const;

private:
    [[nodiscard]] static glm::mat4 RotationAroundPivot(
        const glm::vec3& pivot, const glm::vec3& axis, double degrees);

    std::vector<glm::mat4> baseTransforms_;
    std::size_t yawArmIndex_ = 0;
    std::size_t rollArmIndex_ = 0;
    std::size_t cameraIndex_ = 0;
    glm::vec3 yawPivot_{0.0F};
    glm::vec3 rollPivot_{0.0F};
    glm::vec3 pitchPivot_{0.0F};
};
} // namespace dronesim::graphics
