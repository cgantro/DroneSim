#pragma once

#include "scene/Entity.h"
#include "physics/PhysicsTypes.h"

#include <vector>

namespace grasplink::physics { class PhysicsWorld; }
namespace grasplink::robotics::backends::simulation { class SimGripperController; }

namespace grasplink::simulation
{
class PhysicsSystemModule;

/**
 * @brief 양쪽 손끝의 실제 접촉과 물체를 유지하는 물리 연결 상태를 구분해 보관한다.
 * @details 한쪽 접촉은 양쪽 손끝이 물체를 끼울 때까지 닫힘을 계속한다.
 * grasped는 같은 Dynamic 물체에 양쪽 손끝이 반대 방향으로 닿은 뒤 고정 연결이 남아 있다는 뜻이다.
 * object는 연결된 물체의 비소유 Body 핸들이며 힘이나 마찰 안정성을 측정한 값이 아니다.
 */
struct GripperGraspState
{
    bool leftContact = false;
    bool rightContact = false;
    bool grasped = false;
    physics::PhysicsBodyHandle object;
};

/**
 * @brief TwoF85 손끝의 Jolt 접촉을 개폐 정지와 물체 파지 연결로 바꾼다.
 * @details collision proxy는 화면 메시를 따라 움직이는 별도 충돌 물체다.
 * 왼쪽과 오른쪽 손끝 proxy가 같은 Dynamic 물체에 서로 반대 방향으로 실제로 닿은 경우에만 본체와 물체 사이의 고정 constraint를 만든다.
 * constraint는 두 물체의 상대 위치와 방향을 물리 계산에서 유지하는 연결이며 실제 손가락 마찰이나 forceRequest에 대응하는 힘 [N]을 재현하지 않는다.
 * 한쪽 손끝이 먼저 닿아도 공통 개폐 비율을 계속 진행해 반대 손끝이 같은 물체에 닿을 기회를 준다.
 * 양쪽 손끝이 같은 물체를 끼운 순간 개폐를 멈춘다. 독립 손가락 적응은 없으므로 심하게 비대칭인 배치는 파지로 이어지지 않을 수 있다.
 * Scene은 Entity를, PhysicsWorld는 Body와 constraint를 소유한다. 이 객체는 그 핸들만 보관하고 파괴할 때 연결을 해제한다.
 * Scene, PhysicsWorld, PhysicsSystemModule과 Controller는 이 객체보다 오래 살아야 한다.
 */
class GripperGraspAdapter final
{
public:
    /** @brief 물리 상태와 장면 Body 조회, 개폐 Controller를 빌려 연결한다. */
    GripperGraspAdapter(physics::PhysicsWorld& world, PhysicsSystemModule& system,
        ::grasplink::robotics::backends::simulation::SimGripperController& controller);
    /** @brief 남은 물리 파지 연결을 해제한다. */
    ~GripperGraspAdapter();
    GripperGraspAdapter(const GripperGraspAdapter&) = delete;
    GripperGraspAdapter& operator=(const GripperGraspAdapter&) = delete;

    /**
     * @brief robotRoot 아래의 본체와 양쪽 손끝 충돌 proxy를 찾는다.
     * @details Body가 아직 생성되지 않아도 proxy Entity가 있으면 연결한다. 기존 파지는 해제한다.
     * @return 필수 proxy가 살아 있으면 true다.
     */
    bool Bind(const grasplink::scene::Entity& robotRoot);
    /**
     * @brief 열기, Reset, Disconnect와 삭제된 Body를 확인해 다음 물리 계산 전에 파지를 해제한다.
     * @details Controller.Update와 FK보다 먼저 호출한다. FK는 관절 각도에서 화면 부품의 위치와 방향을 계산하는 과정이다.
     */
    void BeforePhysicsStep();
    /**
     * @brief 물리 계산이 끝난 접촉 목록으로 개폐를 멈추고 양쪽 접촉이면 파지 연결을 만든다.
     * @details Controller.Update, FK, 장면 World 변환 갱신, PhysicsSystemModule.Step을 실행한 뒤 호출한다.
     * 닿은 틱의 연속 개폐 위치를 유지하므로 다음 틱 FK는 그 위치부터 계산한다.
     * Jolt callback 안에서 constraint를 만들지 않아 worker와 Body 잠금이 충돌하지 않는다.
     */
    void AfterPhysicsStep();
    /**
     * @brief 파지 연결을 제거하고 관찰 상태를 비운다. 물체는 다음 Step부터 기존 속도와 중력으로 움직인다.
     * @details 이전 닫힘 명령으로 즉시 다시 잡지 않으며 새 Command를 수락한 뒤에만 파지 연결을 다시 만들 수 있다.
     */
    void Release();
    /** @brief 현재 접촉과 연결 상태의 독립된 복사본을 반환한다. */
    [[nodiscard]] GripperGraspState GetState() const;

private:
    struct ContactCandidate
    {
        physics::PhysicsBodyHandle object;
        glm::vec3 fingerToObject{};
    };

    /** @brief 열기·Reset 요청과 교체된 충돌 Body를 확인해 오래된 파지를 해제한다. */
    void RefreshBindingAndReleaseState();

    physics::PhysicsWorld& world_;
    PhysicsSystemModule& system_;
    ::grasplink::robotics::backends::simulation::SimGripperController& controller_;
    grasplink::scene::Entity anchor_;
    grasplink::scene::Entity left_;
    grasplink::scene::Entity right_;
    physics::PhysicsBodyHandle heldAnchor_;
    physics::PhysicsBodyHandle heldLeft_;
    physics::PhysicsBodyHandle heldRight_;
    physics::PhysicsConstraintHandle constraint_;
    GripperGraspState state_;
    std::uint64_t releaseRevision_ = 0;
    std::uint64_t releasedCommandRevision_ = 0;
    bool disarmed_ = false;
    physics::Transform previousLeft_;
    physics::Transform previousRight_;
    bool previousTipsValid_ = false;
    std::vector<physics::ContactSnapshot> contactSnapshots_;
    std::vector<ContactCandidate> leftContactCandidates_;
    std::vector<ContactCandidate> rightContactCandidates_;
};
}
