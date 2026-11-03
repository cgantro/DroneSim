#pragma once

#include <glm/vec3.hpp>

#include <cstdint>
#include <random>

namespace dronesim::simulation
{
/**
 * @brief 훈련장에서 천천히 방향이 바뀌는 수평 바람을 만든다.
 * @details 같은 난수 시드와 같은 Tick 순서를 사용하면 바람 변화도 다시 재현된다.
 */
class WindField final
{
public:
    /** @brief 반복 가능한 바람을 만들 난수 시드를 정한다. */
    explicit WindField(std::uint32_t seed = 0x44524F4EU) noexcept;

    /** @brief 한 고정 Tick만큼 바람을 갱신하고 초당 미터 단위 속도를 돌려준다. */
    [[nodiscard]] glm::dvec3 AdvanceOneTick();

private:
    std::mt19937 generator_;
    glm::dvec3 velocityMetersPerSecond_{0.0};
    glm::dvec3 targetMetersPerSecond_{0.0};
    std::uint32_t ticksUntilNextGust_ = 0U;
};
} // namespace dronesim::simulation
