#pragma once

#include <cstddef>
#include <cstdint>

namespace grasplink::simulation::robotics
{

enum class SelfCollisionPart : std::uint8_t
{
    Base,
    Link,
    GripperBody
};

struct SelfCollisionIdentity
{
    /// 배열 번호가 없거나 부품이 Base 또는 GripperBody이면 쓴다.
    static constexpr std::size_t InvalidIndex = static_cast<std::size_t>(-1);

    /// Base, 로봇 링크, 그리퍼 몸체 중 이 충돌 모양의 역할이다.
    SelfCollisionPart part = SelfCollisionPart::Link;
    /// 진단 메시지에 표시할 충돌 모양의 순번이다.
    std::size_t linkIndex = InvalidIndex;

    /// 연결 관계를 검사할 때 쓰는 움직이는 관절의 순번이다.
    std::size_t jointIndex = InvalidIndex;
};

/**
 * @brief 로봇의 이웃한 부품과 장착된 그리퍼가 맞닿는 충돌을 허용한다.
 * @details linkIndex는 충돌 모양의 표시 순번이고, jointIndex는 해당 모양을 움직이는 관절 번호다.
 * 같은 움직이는 관절에 고정된 링크와 바로 이웃한 관절에 연결된 링크의 접촉은 허용한다.
 * 그리퍼는 끝 관절에 고정되므로 같은 jointIndex를 가진 링크와의 접촉을 허용한다.
 */
[[nodiscard]] constexpr bool IsAllowedSelfCollision(
    SelfCollisionIdentity first,
    SelfCollisionIdentity second) noexcept
{
    if (first.part == SelfCollisionPart::Link && second.part == SelfCollisionPart::Link)
    {
        if (first.jointIndex == SelfCollisionIdentity::InvalidIndex ||
            second.jointIndex == SelfCollisionIdentity::InvalidIndex)
            return false;
        const std::size_t distance = first.jointIndex > second.jointIndex
            ? first.jointIndex - second.jointIndex
            : second.jointIndex - first.jointIndex;
        return distance <= 1;
    }

    const bool isBaseAndLink =
        (first.part == SelfCollisionPart::Base && second.part == SelfCollisionPart::Link) ||
        (second.part == SelfCollisionPart::Base && first.part == SelfCollisionPart::Link);
    if (isBaseAndLink)
    {
        const SelfCollisionIdentity& link = first.part == SelfCollisionPart::Link ? first : second;
        return link.jointIndex == 0;
    }

    const bool isLinkAndGripper =
        (first.part == SelfCollisionPart::Link && second.part == SelfCollisionPart::GripperBody) ||
        (second.part == SelfCollisionPart::Link && first.part == SelfCollisionPart::GripperBody);
    if (isLinkAndGripper)
    {
        const SelfCollisionIdentity& link = first.part == SelfCollisionPart::Link ? first : second;
        const SelfCollisionIdentity& gripper = first.part == SelfCollisionPart::GripperBody ? first : second;
        return link.jointIndex != SelfCollisionIdentity::InvalidIndex &&
            link.jointIndex == gripper.jointIndex;
    }

    return false;
}

} // namespace grasplink::simulation::robotics
