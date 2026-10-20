#pragma once

#include <cstddef>

/**
 * @brief 화면 모델 대신 충돌 계산에 사용할 proxy Entity임을 표시한다.
 * @details Proxy는 원래 시각 메시와 별도로 만든 대리 물체이며 충돌 모양을 가진다.
 * RobotPhysicsAdapter가 FK 결과로 이 Entity의 위치와 회전을 정한다.
 * 움직임 방식은 RigidBody에, 접촉 모양은 Colliders에 따로 기록한다.
 * 이 표식은 GLB 메시나 Jolt Body를 소유하지 않고 자체적으로 충돌 동작을 바꾸지 않는다.
 */
struct RobotCollisionProxy
{
    /// linkIndex나 jointIndex에 연결 정보가 없음을 나타낸다.
    static constexpr std::size_t InvalidLinkIndex = static_cast<std::size_t>(-1);

    /// 로봇 팔에서 충돌 모양을 등록한 순서다. 관절 번호와 다를 수 있다.
    std::size_t linkIndex = InvalidLinkIndex;

    /// 이 충돌 모양을 움직이는 base_link부터의 관절 번호다. 고정 링크는 같은 번호를 공유한다.
    std::size_t jointIndex = InvalidLinkIndex;
};

/** @brief 로봇 Base의 고정 Environment proxy를 이름 없이 식별하는 표식이다. */
struct RobotBaseEnvironmentProxy {};

/** @brief 그리퍼 proxy가 파지 adapter에서 맡는 역할이다. */
enum class GripperCollisionPart
{
    Other,
    Body,
    LeftFingerTip,
    RightFingerTip
};

/**
 * @brief 그리퍼 충돌 proxy의 역할을 이름과 분리해 ECS에 보관한다.
 * @details Gripper collision setup이 역할을 기록하고 GripperGraspAdapter가 Body와 좌우 손끝을 찾아 쓴다.
 */
struct GripperCollisionProxy
{
    GripperCollisionPart part = GripperCollisionPart::Other;
};
