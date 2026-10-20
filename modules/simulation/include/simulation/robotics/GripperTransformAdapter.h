#pragma once

#include "scene/Entity.h"
#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/models/GripperSpecification.h"

#include <vector>
#include <glm/glm.hpp>

namespace grasplink::simulation::robotics
{

/**
 * @brief 그리퍼 개폐 계산에서 나온 손가락 방향을 GLB 관절 물체들에 적용한다.
 * @details 손가락은 서로 다른 여러 관절이 함께 움직인다. 이 Adapter는 모델 사양의 이름과 순서로 각 GLB 관절을 찾고 기준 회전과 root까지의 부모 경로를 기억한다.
 * Apply()는 저장한 기준 방향에 새 개폐 회전을 더해 부모 기준 방향만 바꾼다. 그리퍼가 팔에 장착된 위치와 각 관절의 GLB 기준 위치·크기는 유지한다.
 * Scene 전체 위치는 직접 계산하지 않는다. TransformSystemModule이 팔과 그리퍼 부모에서 자식 방향으로 변환을 합친다.
 * Scene이 물체를 삭제하거나 중간 부모 연결이 바뀌면 저장한 참조를 사용할 수 없으므로 적용을 거부한다.
 */
class GripperTransformAdapter final
{
public:
    /**
     * @brief root 아래에서 사양에 적힌 손가락 관절을 찾아 기준 방향과 부모 연결을 기록한다.
     * @param gripperRoot GLB의 Gripper node에 해당하는 Entity다. 원점이 아닌 장착 위치와 방향도 보존한다.
     * @param specification 관절 이름과 적용 순서를 제공하는 모델 사양이다. 생성할 때만 읽는다.
     * @throws std::runtime_error root 또는 필수 관절이 없거나 이름이 겹치거나, 관절과 root 사이의 부모 연결이 잘못되었거나 대상 물체가 이미 삭제된 경우.
     * @throws std::invalid_argument 관절 위치·회전·크기가 유효하지 않거나 부모 연결에 `(1,1,1)`이 아닌 크기 배율이 있는 경우.
     */
    GripperTransformAdapter(
        const grasplink::scene::Entity& gripperRoot,
        const ::grasplink::robotics::models::GripperSpecification& specification);

    /**
     * @brief 모든 관절 회전 입력을 확인한 뒤 각 손가락 관절의 부모 기준 방향을 바꾼다.
     * @param state 사양 순서로 정렬된 유한한 관절각과 축·각 회전 quaternion을 담은 상태다.
     * @throws std::invalid_argument 결과 관절 수, 각도 또는 회전 값이 올바르지 않은 경우.
     * @throws std::runtime_error Scene에서 Entity가 삭제되었거나 저장해 둔 부모 경로가 달라진 경우.
     * @details 회전 입력을 모두 확인하고 새 방향을 계산한 뒤 한꺼번에 저장하므로 잘못된 값이 와도 일부 손가락만 움직인 상태로 남지 않는다.
     * 매 호출마다 처음 GLB 기준 방향에서 계산해 반복 적용에 따른 오차가 쌓이지 않는다.
     */
    void Apply(const ::grasplink::robotics::kinematics::GripperKinematicState& state);

    /**
     * @brief 그리퍼 관절 상태와 후보 Gripper 월드 변환으로 프록시들의 월드 변환을 계산한다.
     * @details 결과는 입력 Entity 순서와 같으며, 장면의 관절이나 월드 변환은 변경하지 않는다.
     */
    void BuildCandidateWorldTransforms(
        const ::grasplink::robotics::kinematics::GripperKinematicState& state,
        const glm::mat4& gripperRootWorldTransform,
        const std::vector<grasplink::scene::Entity>& targets,
        std::vector<glm::mat4>& output);

private:
    struct JointBinding
    {
        grasplink::scene::Entity entity;
        ::grasplink::robotics::models::QuaternionWxyz bindRotation;
        // 첫 물체는 관절이고 마지막 물체는 gripperRoot다. 중간 GLB 물체까지 보관해 모델 연결이 바뀌었는지 확인한다.
        std::vector<grasplink::scene::Entity> ancestry;
    };

    grasplink::scene::Entity gripperRoot_;
    std::vector<JointBinding> joints_;
    // 입력 확인과 계산이 끝날 때까지 새 관절 방향을 임시 저장한다. 매 4 ms 갱신에서 메모리를 다시 할당하지 않는다.
    std::vector<glm::quat> preparedRotations_;
    // 후보 충돌 위치를 계산할 때 자식 Entity에서 Gripper root까지의 부모 경로를 재사용한다.
    std::vector<grasplink::scene::Entity> parentPathScratch_;

    void PrepareRotations(const ::grasplink::robotics::kinematics::GripperKinematicState& state);
};

} // namespace grasplink::simulation::robotics
