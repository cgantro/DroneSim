#pragma once

#include "scene/Entity.h"
#include "model/RobotModel.h"
#include "physics/PhysicsTypes.h"
#include "robotics/models/ModelTypes.h"

#include <vector>

namespace grasplink::scene { class Scene; }

namespace grasplink::robotics::kinematics
{
struct RobotKinematicState;
}

namespace grasplink::simulation
{

/**
 * @brief URDF 충돌 도형을 물리 계산에 등록하고 FK 자세를 각 링크에 전달한다.
 * @details FK는 관절 각도에서 링크의 위치와 방향을 계산한다. 각 충돌 도형은 URDF 링크 좌표계에 놓이며 GLB 화면 모양과 별도로 관리한다.
 * Scene이 충돌 proxy Entity를 소유하고 이 어댑터는 Entity 핸들만 빌려 보관한다.
 * Scene이 먼저 제거되면 Apply가 저장한 Entity를 찾지 못해 실패한다.
 */
class RobotPhysicsAdapter final
{
public:
    struct CandidateWorldTransform
    {
        grasplink::scene::Entity entity;
        glm::mat4 worldTransform{1.0F};
    };

    /**
     * @brief 로봇 모델의 링크를 충돌용 Entity에 연결한다.
     * @param scene 새 프록시 Entity를 소유하는 Scene.
     * @param robotRoot GLB 계층의 모델 기준 Entity. FK 자세의 상위 변환으로 사용한다.
     * @param model 링크와 URDF 충돌 도형을 담은 로봇 모델.
     * @throws std::invalid_argument 모델 배열이나 링크 충돌 정보가 맞지 않을 때.
     */
    RobotPhysicsAdapter(
        grasplink::scene::Scene& scene,
        const grasplink::scene::Entity& robotRoot,
        const grasplink::model::RobotModel& model);

    /**
     * @brief FK 결과를 대응하는 충돌 proxy의 부모 기준 위치와 회전으로 저장한다.
     * @param state Robot base 기준 Link 위치 [m]와 quaternion [w,x,y,z]을 담은 FK 결과다.
     * robotRoot에 설정한 Scene 위치와 회전은 부모 계층을 통해 더해진다.
     * @throws std::invalid_argument 결과에 연결된 관절 자세가 없을 때 발생한다.
     * @throws std::runtime_error Scene이 없어 저장한 Entity 핸들을 찾지 못할 때 발생한다.
     */
    void Apply(const grasplink::robotics::kinematics::RobotKinematicState& state);

    /**
     * @brief FK 결과와 로봇 기준 Entity의 월드 변환으로 충돌 프록시의 후보 월드 변환을 계산한다.
     * @details 이 함수는 Scene Entity를 변경하거나 TransformSystemModule을 실행하지 않는다.
     */
    void BuildCandidateWorldTransforms(
        const grasplink::robotics::kinematics::RobotKinematicState& state,
        const glm::mat4& robotRootWorldTransform,
        std::vector<CandidateWorldTransform>& output) const;

private:
    struct LinkBinding
    {
        /// Scene이 소유하며 FK 결과에 따라 움직이는 링크의 충돌용 Entity다.
        grasplink::scene::Entity entity;
        /// RobotKinematicState에서 이 충돌용 Entity가 따라갈 관절 번호다.
        std::size_t jointIndex = 0;
        /// 기준 관절의 자식 링크에서 URDF 링크까지의 고정 변환이다.
        grasplink::robotics::models::Pose3 poseInJointFrame{};
    };

    /** @brief FK 결과에 링크의 고정 오프셋을 더해 로봇 기준 변환을 계산한다. */
    [[nodiscard]] glm::mat4 GetLinkLocalTransform(
        const LinkBinding& link,
        const grasplink::robotics::kinematics::RobotKinematicState& state) const;

    /** @brief 적용 전에 모든 충돌 proxy와 FK 결과가 서로 대응하는지 확인한다. */
    void ValidateState(const grasplink::robotics::kinematics::RobotKinematicState& state) const;

    /// Scene이 소유하며 URDF base_link 충돌 도형을 담는 고정 Entity.
    grasplink::scene::Entity base_;
    std::vector<LinkBinding> links_;
};

}
