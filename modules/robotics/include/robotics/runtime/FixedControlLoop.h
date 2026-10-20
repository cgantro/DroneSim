#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <type_traits>
#include <stdexcept>

namespace grasplink::robotics::runtime
{
/**
 * @brief 화면 갱신 간격이 달라도 제어와 물리를 일정한 시간 간격으로 진행한다.
 * @details 화면은 일정하지 않은 간격으로 다시 그려질 수 있다. 이 클래스는 화면 갱신 사이에
 * 지난 시간을 누적하고, 한 번의 tick에 필요한 시간이 모일 때마다 tick 함수를 호출한다.
 * 짧은 화면 간격은 남은 시간을 다음 호출에 보관하고, 긴 간격은 여러 tick으로 나누어 처리한다.
 * 다만 한 번에 받아들이는 시간을 제한하므로, 디버거 정지처럼 긴 멈춤 뒤에는 시뮬레이션 시간이
 * 실제 시간보다 늦어질 수 있다. 이 클래스는 별도 thread를 만들거나 실시간 응답을 보장하지 않는다.
 * 현재 Viewer는 250 Hz, 즉 0.004 s 간격으로 tick을 실행하며 한 번에 최대 0.1 s를 받는다.
 */
class FixedControlLoop final
{
public:
    /**
     * @brief 한 번의 제어 tick 간격과 화면 갱신 하나에서 받아들일 시간의 상한을 정한다.
     * @param fixedDeltaSeconds tick 간격 [s]. 유한한 양수여야 한다.
     * @param maxFrameDeltaSeconds 한 번의 호출에서 누적할 시간의 상한 [s]. 유한한 양수여야 한다.
     * @throws std::invalid_argument 두 값 중 하나가 유한한 양수가 아니면 발생한다.
     */
    explicit FixedControlLoop(double fixedDeltaSeconds, double maxFrameDeltaSeconds = 0.1)
        : fixedDeltaSeconds_(fixedDeltaSeconds), maxFrameDeltaSeconds_(maxFrameDeltaSeconds)
    {
        if (!std::isfinite(fixedDeltaSeconds_) || fixedDeltaSeconds <= 0.0)
        {
            throw std::invalid_argument("FixedControlLoop: fixedDeltaSeconds must be finite and > 0.");
        }

        if (!std::isfinite(maxFrameDeltaSeconds_) || maxFrameDeltaSeconds_ <= 0.0)
        {
            throw std::invalid_argument("FixedControlLoop: maxFrameDeltaSeconds must be finite and > 0.");
        }
    }

    /**
     * @brief 지난 화면 갱신 이후의 시간을 누적하고, 준비된 tick을 실행한다.
     * @param frameDeltaSeconds 직전 화면 갱신 이후 지난 시간 [s]다. 유한한 양수만 처리한다.
     * @param tick fixedDeltaSeconds를 받아 제어나 시뮬레이션을 한 번 진행하는 함수다.
     * @return 이번 호출에서 실행한 tick 횟수다.
     * @details 유효하지 않은 시간은 무시하고 0을 반환한다. 한 번에 받은 시간이 상한을 넘으면
     * 초과분은 버리며, tick 간격보다 짧게 남은 누적 시간은 다음 호출에 보관한다.
     * tick이 예외를 던지면 호출자에게 전달하며 해당 tick 시간은 누적값에 남는다.
     * 이미 실행된 tick의 외부 상태 변경은 이 클래스가 되돌리지 않는다.
     */
    template <typename TickFunction>
    std::size_t Advance(double frameDeltaSeconds, TickFunction&& tick)
    {
        static_assert(std::is_invocable_v<TickFunction&, double>,
            "TickFunction must be callable with double fixedDeltaSeconds.");

        // 0 이하이거나 유한하지 않은 입력은 시뮬레이션 시간에 더하지 않는다.
        if (!std::isfinite(frameDeltaSeconds) || frameDeltaSeconds <= 0.0)
        {
            return 0;
        }

        // 긴 멈춤 뒤 밀린 시간을 한꺼번에 따라잡지 않도록 이번 입력을 제한한다.
        const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds, maxFrameDeltaSeconds_);

        // 상한을 넘은 시간은 버린다. 긴 멈춤 뒤 시뮬레이션이 실제 시간보다 늦어질 수 있다.
        accumulatorSeconds_ += clampedFrameDeltaSeconds;
        std::size_t executedSteps = 0;

        // 모든 tick에 같은 시간 간격을 전달해 화면 갱신 주기와 물리 계산을 분리한다.
        while (accumulatorSeconds_ >= fixedDeltaSeconds_)
        {
            tick(fixedDeltaSeconds_);
            accumulatorSeconds_ -= fixedDeltaSeconds_;
            ++executedSteps;
        }

        return executedSteps;
    }

    /** @brief 남은 시간을 버린다. 시뮬레이션을 초기화하거나 이전 시간축을 끊을 때 사용한다. */
    void Reset() noexcept
    {
        accumulatorSeconds_ = 0.0;
    }

    /** @brief 한 tick의 고정 시간 간격 [s]. */
    [[nodiscard]] double GetFixedDeltaSeconds() const noexcept
    {
        return fixedDeltaSeconds_;
    }

    /** @brief 다음 tick까지 모인 시간 [s]. 보통 0 이상 fixedDeltaSeconds 미만이다. */
    [[nodiscard]] double GetAccumulatorSeconds() const noexcept
    {
        return accumulatorSeconds_;
    }

    /** @brief 다음 tick까지 모인 시간의 비율 [0,1). 이전·현재 상태를 화면에서 보간할 때 사용한다. */
    [[nodiscard]]
    double GetInterpolationAlpha() const noexcept
    {
        return accumulatorSeconds_ / fixedDeltaSeconds_;
    }

private:
    // 모든 제어·물리 tick이 동일하게 사용하는 시간 간격 [s]다.
    double fixedDeltaSeconds_;

    // 화면 갱신 한 번에서 누적기에 더할 수 있는 시간의 상한 [s]다.
    double maxFrameDeltaSeconds_;

    // 다음 tick을 실행하기에 부족해 누적기에 남겨 둔 시간 [s]다.
    double accumulatorSeconds_ = 0.0;
};
} // namespace grasplink::robotics::runtime
