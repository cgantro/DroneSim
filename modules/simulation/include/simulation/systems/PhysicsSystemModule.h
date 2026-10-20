#pragma once

#include <flecs.h>
#include "physics/PhysicsTypes.h"

#include <memory>

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::diagnostics
{
class Logger;
}

namespace grasplink::simulation
{

/**
 * @brief ECS 설정을 Jolt 물리 Body로 만들고 매 고정 간격마다 장면과 물리 상태를 동기화한다.
 * @details Body는 위치, 회전, 충돌을 물리 계산에 맡기는 물체다. `RigidBody`와 `Colliders`가 함께 있는 Entity만 Body가 된다.
 * 충돌 모양은 화면 Mesh와 별도로 지정하며, 크기는 미터 단위로 읽고 Entity의 scale을 곱하지 않는다.
 * `Local` 변환은 부모 기준이고 `World` 변환은 부모 계층을 합친 장면 기준이다. 이 모듈은 `World` 자세로 Body를 만들고,
 * Dynamic Body의 물리 결과를 `Local` 값으로 돌려준다. 그래서 Dynamic Body는 변환된 부모 아래에 둘 수 없고,
 * 물리 계산하는 Body를 또 다른 물리 Body의 자식으로 둘 수도 없다. 화면 바닥 크기를 위한 scale은 Static Environment에만 허용한다.
 * Static은 움직이지 않고, Kinematic은 장면이 정한 목표를 따라가며, Dynamic은 중력과 충돌 결과에 따라 움직인다.
 */
class PhysicsSystemModule final
{
public:
    /**
     * @brief Flecs 설정 변화를 관찰해 Jolt Body의 생성과 해제를 연결한다.
     * @param world 설정 Entity와 observer가 살아 있는 Flecs World다.
     * @param physicsWorld 만든 Body를 실제로 소유하고 해제하는 PhysicsWorld다.
     * 두 World는 이 모듈보다 오래 살아야 한다.
     * @param logger 진단 기록을 남길 Logger다. 전달하면 이 모듈보다 오래 살아야 한다.
     * 생성 시 이미 붙어 있는 component도 찾아 연결한다.
     * 새 설정으로 Body를 만들거나 다시 만드는 일은 첫 Step에서 실행한다.
     */
    PhysicsSystemModule(flecs::world& world, grasplink::physics::PhysicsWorld& physicsWorld,
        grasplink::diagnostics::Logger* logger = nullptr);

    /** @brief 모듈이 만든 Body와 Entity 연결을 정리한 뒤 this를 참조하는 observer를 해제한다. */
    ~PhysicsSystemModule();

    PhysicsSystemModule(const PhysicsSystemModule&) = delete;
    PhysicsSystemModule& operator=(const PhysicsSystemModule&) = delete;

    /**
     * @brief Scene이 정한 자세를 Jolt에 보내고 Dynamic 계산 결과를 Entity에 기록한다.
     * @param fixedDeltaSeconds 이번 물리 계산 간격 [s]다. 유한한 양수만 처리한다.
     * @details 호출 전에 Controller와 FK가 관절 자세를 갱신해야 한다.
     * TransformSystem은 부모 Local 변환을 합쳐 최신 World 행렬을 계산한다.
     * Step은 예약한 Body 설정을 적용하고 Static 위치와 Kinematic 목표를 Jolt에 전달한다.
     * 이어서 Jolt를 고정 간격 한 번 계산하고 Dynamic Body의 World 위치·회전을 Entity Local 값에 기록한다.
     * 다음 렌더링에서 이 결과를 사용하려면 호출자가 TransformSystem을 다시 실행해야 한다.
     * 같은 Kinematic 목표를 반복 전달하지 않으며 목표에 도달한 속도는 한 번 정지시킨다.
     * 충돌 모양에는 scale을 적용하지 않으므로 Static Environment의 화면용 scale을 제외하고 관련 Entity와 조상은 단위 scale이어야 한다.
     */
    void Step(double fixedDeltaSeconds);

    /**
     * @brief Scene Entity에 현재 연결된 물리 Body의 비소유 핸들을 반환한다.
     * @details Entity는 Flecs World가 소유한 장면 물체를 가리킨다. 아직 Body가 없거나 Entity가 삭제됐으면 무효 핸들을 반환한다.
     * 설정 변경으로 Body가 재생성될 수 있으므로 adapter는 각 Step에서 다시 조회해야 한다.
     */
    [[nodiscard]] grasplink::physics::PhysicsBodyHandle GetBodyHandle(flecs::entity entity) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
