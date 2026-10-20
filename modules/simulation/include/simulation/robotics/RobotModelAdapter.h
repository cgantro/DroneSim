#pragma once

#include "model/RobotModel.h"
#include "robotics/models/RobotSpecification.h"

#include <string>
#include <vector>

namespace grasplink::simulation::robotics
{

/**
 * @brief URDF 로봇을 FK·IK와 제어기가 사용하는 사양으로 바꿔 보관한다.
 * @details 변환 결과에는 관절 이름과 관절·링크 배열이 들어 있다. 이 클래스가 그 배열을 소유하므로 반환하는 사양은 이 객체가 살아 있는 동안 사용할 수 있다.
 * 링크 이름과 모델 이름은 원본 RobotModel에서 빌려 쓰므로 원본 모델은 이 객체보다 오래 살아 있어야 한다.
 * 회전 관절은 [rad], 직선 관절은 [m] 단위로 저장한다.
 */
class RobotModelAdapter final
{
public:
    /**
     * @brief URDF에서 읽은 모델을 제어 사양으로 변환한다.
     * @param model 링크와 관절 자료를 보관한 원본 모델이다. 이 객체보다 먼저 파괴하면 안 된다.
     * @throws std::runtime_error 관절 경로가 잘못되었거나 제어 사양으로 바꿀 수 없는 경우 발생한다.
     */
    explicit RobotModelAdapter(const grasplink::model::RobotModel& model);

    RobotModelAdapter(const RobotModelAdapter&) = delete;
    RobotModelAdapter& operator=(const RobotModelAdapter&) = delete;
    RobotModelAdapter(RobotModelAdapter&&) = delete;
    RobotModelAdapter& operator=(RobotModelAdapter&&) = delete;

    /**
     * @brief Controller와 운동학 계산기가 함께 사용할 변환 사양을 돌려준다.
     * @return 이 어댑터와 원본 모델이 살아 있는 동안 유효한 사양이다.
     */
    [[nodiscard]] const grasplink::robotics::models::RobotSpecification& Specification() const noexcept;

private:
    const grasplink::model::RobotModel& model_;
    std::vector<std::string> jointNames_;
    std::vector<grasplink::robotics::models::JointSpecification> joints_;
    std::vector<grasplink::robotics::models::LinkSpecification> links_;
    grasplink::robotics::models::RobotSpecification specification_;
};

} // namespace grasplink::simulation::robotics
