#include "simulation_core/WindField.h"

#include <random>

namespace dronesim::simulation
{
namespace
{
// 60Hz 고정 Tick에서 240회는 새 바람 목표를 약 4초마다 고른다는 뜻이다.
constexpr std::uint32_t gustChangeIntervalTicks = 240U;
constexpr double maximumWindMetersPerSecond = 0.5;
constexpr double smoothingPerTick = 0.01;
}

WindField::WindField(std::uint32_t seed) noexcept
    : generator_(seed)
{
}

glm::dvec3 WindField::AdvanceOneTick()
{
    if (ticksUntilNextGust_ == 0U)
    {
        std::uniform_real_distribution<double> chooseWind(
            -maximumWindMetersPerSecond, maximumWindMetersPerSecond);
        targetMetersPerSecond_ = {
            chooseWind(generator_), chooseWind(generator_), 0.0};
        ticksUntilNextGust_ = gustChangeIntervalTicks;
    }

    --ticksUntilNextGust_;

    // 순간적으로 바람이 튀지 않도록 현재 속도의 1%만 새 목표 쪽으로 옮긴다.
    velocityMetersPerSecond_ +=
        (targetMetersPerSecond_ - velocityMetersPerSecond_) * smoothingPerTick;
    return velocityMetersPerSecond_;
}
} // namespace dronesim::simulation
