#pragma once

#include "model/RobotModel.h"
#include "robotics/models/RobotSpecification.h"

#include <string>
#include <vector>

namespace grasplink::simulation::robotics
{

/**
 * @brief URDF 로봇을 FK·IK와 제어기가 사용하는 사양으로 바꿔 보관한다.
 * @details 변환 결과의 이름과 관절·링크 배열을 이 클래스가 소유한다. 원본 RobotModel을 해제하거나 바꿔도 반환한 사양은 어댑터가 살아 있는 동안 유효하다.
 * 회전 관절은 [rad], 직선 관절은 [m] 단위로 저장한다.
 */
class RobotModelAdapter final
{
public:
    /**
     * @brief URDF에서 읽은 모델을 제어 사양으로 변환한다.
     * @param model 링크와 관절 자료를 보관한 원본 모델이다. 필요한 이름과 수치는 생성 중 어댑터가 복사한다.
     * @throws std::invalid_argument 모델의 관절 경로가 잘못된 경우 발생한다.
     * @throws std::runtime_error 관절 종류를 제어 사양으로 바꿀 수 없는 경우 발생한다.
     */
    explicit RobotModelAdapter(const grasplink::model::RobotModel& model);

    // RobotSpecification은 아래 벡터와 문자열을 복사하지 않고 가리키므로, 어댑터의 복사와 이동을 막아 수명을 보장한다.
    RobotModelAdapter(const RobotModelAdapter&) = delete;
    RobotModelAdapter& operator=(const RobotModelAdapter&) = delete;
    RobotModelAdapter(RobotModelAdapter&&) = delete;
    RobotModelAdapter& operator=(RobotModelAdapter&&) = delete;

    /**
     * @brief Controller와 운동학 계산기가 함께 사용할 변환 사양을 돌려준다.
     * @return 이 어댑터가 살아 있는 동안 유효한 사양이다.
     */
    [[nodiscard]] const grasplink::robotics::models::RobotSpecification& Specification() const noexcept;

private:
    std::string modelName_;
    std::vector<std::string> jointNames_;
    std::vector<std::string> linkNames_;
    std::vector<grasplink::robotics::models::JointSpecification> joints_;
    std::vector<grasplink::robotics::models::LinkSpecification> links_;
    grasplink::robotics::models::RobotSpecification specification_;
};

} // namespace grasplink::simulation::robotics
