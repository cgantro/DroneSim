#pragma once

#include "scene/Entity.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::simulation::robotics
{

/**
 * @brief 관절 각도에서 계산한 로봇 링크 방향을 GLB 장면 물체에 적용한다.
 * @details Forward Kinematics(FK)는 관절 값에서 각 팔 링크와 도구 끝의 위치·방향을 계산한다. 이 Adapter는 계산한 관절 움직임을 GLB 관절에 적용한다.
 * 생성 시 모델 사양의 이름으로 GLB node를 찾고, 초기 위치·방향과 부모 관계가 로봇 구조 자료와 맞는지 확인한다.
 * GLB 장면 물체는 Scene이 삭제하므로 Scene 수명이 끝난 뒤 Apply()를 호출하면 실패한다.
 */
class RobotTransformAdapter
{
public:
    /**
     * @brief 로봇 사양에 적힌 관절 이름을 GLB 장면 물체에 연결하고 기준 자세를 검사한다.
     * @param robotRoot Scene 안에서 모델을 배치하는 기준 물체다.
     * @param specification GLB 관절 이름과 순서, 초기 위치·방향을 제공하는 모델 사양이다.
     * @throws std::runtime_error 기준 Entity가 없거나 필요한 관절 이름을 GLB에서 찾지 못한 경우.
     * @throws std::invalid_argument 관절 부모 관계 또는 초기 변환이 정기구학 모델과 맞지 않는 경우.
     */
    RobotTransformAdapter(
        const grasplink::scene::Entity& robotRoot,
        const ::grasplink::robotics::models::RobotSpecification& specification);

    /**
     * @brief FK가 계산한 부모 기준 회전을 이름과 순서가 맞는 GLB 관절에 적용한다.
     * @param state RobotSpecification 순서로 정렬한 링크 회전 결과다.
     * @throws std::invalid_argument 관절 회전 결과 수가 연결한 Entity 수와 다른 경우.
     * @throws std::runtime_error Scene이 제거되어 연결한 Entity를 더 사용할 수 없는 경우.
     */
    void Apply(const ::grasplink::robotics::kinematics::RobotKinematicState& state);

private:
    struct JointBinding
    {
        grasplink::scene::Entity entity;
        glm::vec3 bindPosition{0.0F};
        glm::quat bindRotation{1.0F, 0.0F, 0.0F, 0.0F};
        glm::vec3 prismaticAxisInParentFrame{0.0F};
        ::grasplink::robotics::models::JointMotionType motionType =
            ::grasplink::robotics::models::JointMotionType::Revolute;
    };

    /// 모델 사양에 적힌 순서대로 Scene이 소유하는 관절 Entity를 보관한다. 각 Entity는 계산된 관절 회전을 화면에 반영하는 데 사용한다.
    std::vector<JointBinding> joints_;
};

} // namespace grasplink::simulation::robotics
