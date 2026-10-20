#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <vector>

namespace grasplink::physics
{

/**
 * @brief PhysicsWorld가 만든 물리 Body를 찾아갈 때 사용하는 식별 정보.
 * @details
 * Body 메모리와 Jolt ID는 PhysicsWorld가 소유하고, 이 구조체는 그 Body를 다시 찾는 번호를 보관한다.
 * World 식별 번호도 함께 비교하므로 다른 PhysicsWorld에서 같은 번호를 받은 Body를 잘못 가리키지 않는다.
 * Jolt는 삭제한 Body의 저장 칸을 새 Body에 재사용할 수 있다.
 * 핸들 안의 순번도 비교해 오래된 핸들이 같은 칸의 새 Body를 가리키지 않게 한다.
 * Body가 파괴된 뒤에는 핸들을 다시 사용하면 안 된다.
 */
struct PhysicsBodyHandle
{
    static constexpr std::uint32_t InvalidValue = 0xFFFFFFFFU;

    std::uint32_t value = InvalidValue;
    // Body ID 숫자가 같더라도 핸들에 들어 있는 World token이 다르면 다른 World의 Body다. 이 핸들은 현재 World에 전달해 사용할 수 없다.
    std::uint64_t worldToken = 0;

    [[nodiscard]]
    bool IsValid() const noexcept
    {
        return value != InvalidValue && worldToken != 0;
    }
};

/**
 * @brief PhysicsWorld가 소유하는 두 물체 사이의 고정 연결을 가리킨다.
 * @details 연결은 두 물체의 상대 위치와 회전을 유지하는 Jolt constraint다.
 * 핸들은 연결 자체를 소유하지 않으며 World 번호와 재사용하지 않는 연결 번호로 삭제된 연결을 구별한다.
 */
struct PhysicsConstraintHandle
{
    std::uint64_t value = 0;
    std::uint64_t worldToken = 0;
    [[nodiscard]] bool IsValid() const noexcept { return value != 0 && worldToken != 0; }
};

/**
 * @brief Jolt가 실제 충돌 검사에서 발견한 두 물체의 접촉을 복사해 보관한다.
 * @details snapshot은 물리 계산이 끝난 시점에 읽는 값의 복사본이며 Body 메모리를 소유하지 않는다.
 * normal은 장면 전체 기준 World 방향이고 first에서 second를 향한다.
 * point는 first 표면에서 접촉한 지점의 World 위치 [m]다.
 * penetrationMeters는 겹침 깊이 [m]이며 음수이면 다음 이동을 대비해 미리 발견한 간격이다.
 * 같은 두 물체에서도 서로 다른 충돌 모양이 닿으면 여러 항목이 생길 수 있다.
 */
struct ContactSnapshot
{
    PhysicsBodyHandle first;
    PhysicsBodyHandle second;
    glm::vec3 normal{0.0F};
    float penetrationMeters = 0.0F;
    glm::vec3 point{0.0F};
};


/**
 * @brief 물체의 위치와 회전을 함께 나타낸다.
 * @details 위치는 장면 전체 기준 World 좌표 [m]다.
 * quaternion은 회전 방향을 네 숫자 (w,x,y,z)로 저장한다.
 * Body 자세는 모델 기준점인 Body 원점의 World 위치와 회전이다.
 * localTransform은 Body 원점에서 Collider 중심까지의 위치와 회전을 지정한다.
 * Collider는 화면 Mesh와 별도로 접촉 여부를 계산하는 모양이다.
 * 비대칭 모양에서 Jolt 계산용 무게중심(COM)은 Body 원점과 다를 수 있지만 API는 Body 원점을 계속 사용한다.
 */
struct Transform
{
    glm::vec3 position{0.0F};

    glm::quat rotation{
        1.0F,
        0.0F,
        0.0F,
        0.0F
    };
};


/**
 * @brief Physics Body의 위치와 회전을 누가 정하는지 선택한다.
 * @details Physics Body는 Jolt가 위치, 속도, 충돌을 계산하는 물체다.
 * Static은 움직이지 않는 바닥이나 벽에 쓴다.
 * Kinematic은 로봇 팔처럼 코드가 목표 자세를 정하는 물체에 쓴다.
 * Dynamic은 박스처럼 Jolt가 중력과 충돌 결과로 움직이는 물체에 쓴다.
 */
enum class BodyMotionType : std::uint8_t
{
    Static,
    Kinematic,
    Dynamic
};

/** @brief Static과 Kinematic은 Scene이 계산한 자세를 물리 공간에 전달해 사용한다. */
constexpr bool IsSceneDriven(BodyMotionType type) noexcept
{
    return type == BodyMotionType::Static || type == BodyMotionType::Kinematic;
}

/** @brief Dynamic은 충돌과 중력 계산으로 Physics가 자세를 바꾸고, Scene은 그 결과를 화면에 반영한다. */
constexpr bool IsPhysicsDriven(BodyMotionType type) noexcept
{
    return type == BodyMotionType::Dynamic;
}

/**
 * @brief 물체를 충돌 그룹별로 구분한다.
 * @details Environment, Robot, Gripper, DynamicObject처럼 이 물체가 어떤 종류인지를 나타낸다.
 * 예를 들어 Robot과 DynamicObject는 서로 충돌하게 하고 Robot끼리는 충돌하지 않도록 규칙을 만들 때 쓴다.
 * 이 값은 Static, Kinematic, Dynamic처럼 물체를 누가 움직이는지 정하는 BodyMotionType과는 별개다.
 * PhysicsWorld는 두 설정을 함께 사용해 어떤 물체 쌍의 충돌을 검사할지 결정한다.
 */
enum class CollisionLayer : std::uint8_t
{
    Environment,
    Robot,
    Gripper,
    DynamicObject,
    /// 실제 충돌 그룹 수를 나타내며 Body에 지정할 수 없다.
    Count
};

/** @brief Body 주변의 Box 또는 ConvexHull 충돌 모양을 고른다. */
enum class CollisionShapeType : std::uint8_t
{
    Box,
    ConvexHull
};

/**
 * @brief 물리 접촉에 사용할 Collider의 종류, 치수와 Body 기준 배치를 지정한다.
 * @details Collider는 접촉 여부를 계산하는 모양이고 화면 Mesh와 별개다.
 * 치수와 정점은 [m]이며 Mesh나 Entity scale에서 자동으로 가져오지 않는다.
 * localTransform으로 Body 원점에 위치와 회전을 지정한다.
 * 여러 Collider는 한 Body에 속해 같은 방식으로 움직인다.
 */
struct CollisionShapeDescription
{
    CollisionShapeType type = CollisionShapeType::Box;
    // 각 축에서 Box 중심부터 면까지의 길이 [m]다. 실제 전체 Box 크기는 이 값의 두 배다. {0.5, 0.5, 0.5}이면 전체 크기는 1 m.
    glm::vec3 halfExtentsMeters{0.5F};
    // 형상의 위치와 회전은 Body 원점 기준이다. GLM quaternion 성분은 (w,x,y,z) 순서다.
    Transform localTransform;
    // ConvexHull은 입력한 모든 점을 감싸는 볼록한 껍질이며 점은 형상 기준 좌표 [m]다.
    // 점 네 개는 최소 개수일 뿐이며 한 평면에 놓여 부피를 만들지 못하면 Jolt가 거부한다.
    // localTransform은 이 형상을 Body 원점에 배치한다.
    std::vector<glm::vec3> pointsMeters;
};

/**
 * @brief 여러 충돌 모양을 하나의 물리 Body로 만들기 위한 설정이다.
 * @details Body는 Jolt가 위치, 속도, 충돌을 계산하는 물체다.
 * Box나 ConvexHull 같은 각 collider는 Body에 붙는 접촉 판정용 모양이며 별도로 움직이지 않는다.
 * 모든 모양은 Body의 움직임 방식과 CollisionLayer를 함께 사용한다.
 * transform은 모델 기준 Body 원점의 World 위치 [m]와 회전이다.
 * 모양 배치가 한쪽으로 치우치면 Jolt의 계산 기준인 무게중심(COM)이 Body 원점과 다를 수 있다.
 * 공개 API는 계속 Body 원점의 자세를 읽고 설정하므로 호출자가 COM 차이를 더하지 않는다.
 */
struct BodyDescription
{
    // 아래 형상들은 별도 Body가 아니라 하나의 Body를 구성하므로 같은 자세, Static/Kinematic/Dynamic 움직임 방식, 충돌 범주를 공유한다.
    std::vector<CollisionShapeDescription> shapes;
    Transform transform;
    BodyMotionType motionType = BodyMotionType::Dynamic;
    CollisionLayer collisionLayer = CollisionLayer::DynamicObject;
};


/** @brief Box 하나로 Body를 만들 때 쓰는 간단한 설정. 각 축의 크기는 halfExtentsMeters 값의 두 배다. */
struct BoxBodyDescription
{
    // 각 축에서 Box 중심부터 면까지의 길이 [m]다. 실제 전체 Box 크기는 이 값의 두 배다.
    glm::vec3 halfExtentsMeters{
        0.5F,
        0.5F,
        0.5F
    };

    Transform transform;

    BodyMotionType motionType =
        BodyMotionType::Dynamic;

    CollisionLayer collisionLayer =
        CollisionLayer::DynamicObject;
};

} // namespace grasplink::physics
