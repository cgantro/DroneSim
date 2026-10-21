#pragma once

#include "physics/PhysicsTypes.h"

#include <memory>

namespace grasplink::physics
{

/**
 * @brief Jolt 물리 공간을 만들고, 그 안에 생성된 Body의 사용과 해제를 관리한다.
 * @details
 * Flecs Entity와 Body를 연결하고 고정 업데이트마다 Step을 부르는 일은 simulation 모듈이 맡는다.
 * 각 PhysicsWorld는 충돌과 움직임을 계산하는 Jolt PhysicsSystem, 계산 중 임시로 쓰는 메모리, 작업자 thread pool을 가진다.
 * Jolt 타입 등록은 프로그램 전체가 공유하므로 첫 PhysicsWorld를 만들 때 준비하고 마지막 World가 끝날 때 해제한다.
 */
class PhysicsWorld final
{
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    PhysicsWorld(PhysicsWorld&&) = delete;
    PhysicsWorld& operator=(PhysicsWorld&&) = delete;

    /**
     * @brief 고정 시간 간격 하나만큼 Jolt 물리 상태를 한 번 진행한다.
     * @param fixedDeltaSeconds 계산 간격 [s]. 유한한 양수여야 한다.
     * @details 한 호출은 추가 분할 없이 Jolt Update 한 번으로 처리한다. 유효하지 않은 간격은 조용히 무시한다.
     */
    void Step(double fixedDeltaSeconds);

    /**
     * @brief Box 충돌 모양 하나를 가진 물리 Body를 만든다.
     * @param description Box 반쪽 길이 [m], Body 원점의 World 자세, 움직임 방식과 충돌 그룹이다.
     * @return 이 PhysicsWorld에서만 사용할 수 있는 Body 핸들이다.
     * @throws std::invalid_argument 자세, 그룹 또는 치수가 유효하지 않을 때 발생한다.
     */
    PhysicsBodyHandle CreateBox(
        const BoxBodyDescription& description);

    /**
     * @brief Box 또는 ConvexHull 충돌 모양을 하나의 물리 Body로 묶는다.
     * ConvexHull은 점들을 모두 포함하는 볼록한 껍질이다. 입력 점 네 개는 최소 개수일 뿐이며 Jolt가 부피를 만들 수 있는 점을 추가로 요구한다.
     * @param description 형상별 Body 기준 배치와 크기, Body 원점의 World 자세, 움직임 방식과 충돌 그룹이다.
     * @return 이 PhysicsWorld에서만 유효한 핸들이다.
     * @throws std::invalid_argument 형상 목록이 비었거나 자세, 그룹, 치수가 유효하지 않을 때 발생한다.
     * @throws std::runtime_error Jolt가 형상 또는 Body를 만들지 못할 때 발생한다.
     * @details 여러 형상이 한 Body를 구성하므로 모두 같은 움직임 방식으로 움직인다.
     * 공개 함수의 자세 기준은 모델이 정한 Body 원점이다.
     * 형상이 비대칭이면 Jolt 계산에 쓰는 무게중심(COM)이 이 원점과 다를 수 있다.
     * Jolt가 그 차이를 내부에서 처리하므로 호출자는 COM 보정을 더하지 않는다.
     */
    PhysicsBodyHandle CreateBody(
        const BodyDescription& description);

    /** @brief 핸들이 이 World에서 현재 물리 계산에 등록된 Body를 가리키는지 확인한다. */
    [[nodiscard]]
    bool IsBodyValid(
        PhysicsBodyHandle handle) const;

    /** @brief 유효한 Body를 움직이는 주체인 Static, Kinematic, Dynamic 중 하나를 반환한다. */
    [[nodiscard]] BodyMotionType GetBodyMotionType(PhysicsBodyHandle handle) const;

    /** @brief 현재 Body의 충돌 그룹을 반환한다. 접촉 snapshot에서 장면 바닥 접촉과 물체 접촉을 구별할 때 사용한다. */
    [[nodiscard]] CollisionLayer GetCollisionLayer(PhysicsBodyHandle handle) const;

    /**
     * @brief 마지막 물리 계산에서 남아 있는 접촉 값을 전달한 벡터에 복사한다.
     * @param output 접촉 목록을 덮어쓸 벡터다. 기존 저장 공간은 유지하므로 매 tick 재사용할 수 있다.
     * @details Jolt worker callback은 내부 mutex로 보호한 접촉 목록만 갱신하며 이 함수는 Step이 끝난 뒤 호출한다.
     * 삭제된 Body의 항목은 복사하지 않는다. 이 API를 포함한 World 조작과 Step은 호출자 스레드에서 순서대로 실행해야 한다.
     */
    void CopyContactsTo(std::vector<ContactSnapshot>& output) const;

    /**
     * @brief Kinematic 기준 물체에 Dynamic 물체의 현재 상대 자세를 고정하는 연결을 만든다.
     * @details 연결은 Jolt가 물리 계산에서 상대 위치와 회전을 유지하도록 한다.
     * 물체의 위치를 직접 옮기거나 Scene 부모를 바꾸지 않으며 실제 마찰이나 그리퍼 힘 [N]을 계산하는 모델은 아니다.
     * Step과 접촉 callback이 끝난 뒤 호출해야 하며 연결된 Body가 삭제되면 연결도 먼저 제거한다.
     * @throws std::invalid_argument 핸들이 무효이거나 필요한 움직임 종류와 다를 때 발생한다.
     */
    PhysicsConstraintHandle CreateFixedConstraint(PhysicsBodyHandle anchor, PhysicsBodyHandle object);

    /** @brief 연결이 이 World에 남아 있는지 확인한다. */
    [[nodiscard]] bool IsConstraintValid(PhysicsConstraintHandle handle) const;

    /** @brief 물리 연결을 해제한다. 무효이거나 다른 World의 핸들은 무시한다. */
    void DestroyConstraint(PhysicsConstraintHandle handle);

    /**
     * @brief Body의 모델 기준점 위치와 회전을 World 좌표로 읽는다.
     * @param handle 이 World에서 생성한 유효한 Body 핸들.
     * @return 위치 [m]와 회전. Entity에 물리 계산 결과를 반영할 때 사용한다.
     * @throws std::invalid_argument 핸들이 무효이거나 다른 World 소유인 경우.
     * @details COM이 아니라 Body 원점을 반환하므로 비대칭 형상에서도 모델 기준점을 유지한다.
     */
    [[nodiscard]]
    Transform GetBodyTransform(
        PhysicsBodyHandle handle) const;

    /**
     * @brief 물리 계산을 진행하기 전에 지정 자세의 Body 형상이 Environment와 겹치는지 확인한다.
     * @param handle 이 PhysicsWorld가 만든 형상을 조회할 Body 핸들이다.
     * @param targetTransform 검사할 Body 원점의 World 위치 [m]와 회전이다.
     * @return Jolt 형상 검사에서 Environment와 겹치면 true다.
     * @details Kinematic Body는 Static Environment를 밀거나 막지 않으므로 Contact callback만으로 목표 침투를 예방할 수 없다.
     * 이 검사는 현재 목표 자세 하나를 검사하며 이동 구간 전체의 장애물 회피 경로를 만들지는 않는다.
     */
    [[nodiscard]] bool OverlapsEnvironmentAt(
        PhysicsBodyHandle handle,
        const Transform& targetTransform) const;

    /**
     * @brief Body가 지정한 Environment와의 접촉만 제외하고 나머지 Environment와 겹치는지 확인한다.
     * @param handle 검사할 Body handle이다.
     * @param targetTransform 검사할 Body 원점의 World 위치와 회전이다.
     * @param ignoredEnvironmentBody 검사에서 제외할 Environment Body handle이다.
     * @details 두 Body가 실제로 맞닿아 있어야 하는 연결부에서만 사용한다. 예를 들어 로봇의 Link1과 Base가 관절에서 맞닿는 경우 Base만 제외할 수 있으며, 다른 바닥이나 장애물 검사는 계속 수행한다.
     * @throws std::invalid_argument 제외할 Body handle이 유효하지 않거나 Environment로 분류되지 않은 경우 발생한다.
     */
    [[nodiscard]] bool OverlapsEnvironmentAt(
        PhysicsBodyHandle handle,
        const Transform& targetTransform,
        PhysicsBodyHandle ignoredEnvironmentBody) const;

    /**
     * @brief 두 Collider를 지정한 자세에 배치했을 때 서로 겹치는지 검사한다.
     * @details 두 Body의 현재 Jolt 자세 대신 전달한 자세를 모두 사용하므로 FK 후보 자세의 자가 충돌을 검사할 수 있다.
     */
    [[nodiscard]] bool OverlapsBodiesAt(
        PhysicsBodyHandle first,
        const Transform& firstTransform,
        PhysicsBodyHandle second,
        const Transform& secondTransform) const;

    /**
     * @brief Body를 지정한 World 위치와 회전에 즉시 배치한다. 이동 경로를 따라가지는 않는다.
     * @param handle 이 World에서 생성한 유효한 Body 핸들.
     * @param transform 목표 위치 [m]와 회전.
     * @throws std::invalid_argument 핸들이 무효이거나 자세 성분이 유한하지 않고 회전이 0인 경우.
     * @details 경로를 따라가는 이동이 아니라 Reset/Teleport용 직접 배치다. 회전은 전달 전에 정규화되며 Dynamic Body는 기존 속도도 초기화한다.
     */
    void SetBodyTransform(
        PhysicsBodyHandle handle,
        const Transform& transform);

    /**
     * @brief Scene 목표까지 Kinematic Body가 충돌을 고려하며 움직이도록 Jolt에 전달한다.
     * @param handle 이 PhysicsWorld가 만든 Kinematic Body 핸들이다.
     * @param targetTransform 목표 Body 원점의 World 위치 [m]와 회전이다.
     * @param fixedDeltaSeconds 이번 이동 간격 [s]이며 유한한 양수여야 한다.
     * @throws std::invalid_argument 핸들, 목표 자세 또는 간격이 유효하지 않을 때 발생한다.
     * @throws std::logic_error Body가 Kinematic이 아닐 때 발생한다.
     * @details Jolt는 현재 자세에서 목표까지 이번 간격 동안 움직일 속도를 계산해 충돌을 처리한다.
     * 경로를 따라가지 않고 즉시 배치할 때는 SetBodyTransform을 사용한다.
     */
    void MoveKinematic(
        PhysicsBodyHandle handle,
        const Transform& targetTransform,
        double fixedDeltaSeconds);

    /**
     * @brief 목표에 도달한 Kinematic Body에 남아 있는 직선·회전 속도를 없애 정지시킨다.
     * @details MoveKinematic은 목표에서 이동 속도를 만들며 Step 이후에도 그 속도가 남는다.
     * 목표 전달을 생략하기 전에 한 번 호출해 정지시킨다. 위치·회전은 바꾸지 않는다.
     * @throws std::invalid_argument 핸들이 유효하지 않을 때.
     * @throws std::logic_error Body가 Kinematic이 아닐 때.
     */
    void StopKinematic(PhysicsBodyHandle handle);

    /** @brief Body를 다음 물리 계산에서 제외한 뒤 메모리를 해제한다. 유효하지 않거나 다른 World의 핸들은 무시한다. */
    void DestroyBody(
        PhysicsBodyHandle handle);

private:
    [[nodiscard]] bool OverlapsEnvironmentAtImpl(
        PhysicsBodyHandle handle,
        const Transform& targetTransform,
        const PhysicsBodyHandle* ignoredEnvironmentBody) const;

    // Jolt 타입과 자원은 Impl에서 관리한다.
    struct Impl;

    std::unique_ptr<Impl> impl_;
};

} // namespace grasplink::physics
