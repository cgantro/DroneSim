#pragma once

#include <chrono>

namespace dronesim::simulation
{

/**
 * @brief 지정한 코드 구간이 끝날 때 걸린 시간을 마이크로초 단위로 기록한다.
 * @details 결과를 덮어쓰거나 기존 값에 더할 수 있다. 기록 대상이 없으면 시계를 읽지 않는다.
 */
class ScopedDuration final
{
public:
    enum class Mode
    {
        Replace,
        Accumulate
    };

    explicit ScopedDuration(double* microseconds, Mode mode = Mode::Replace) noexcept
        : microseconds_(microseconds),
          mode_(mode),
          startedAt_(microseconds != nullptr ? Clock::now() : Clock::time_point{})
    {
    }

    ~ScopedDuration() noexcept
    {
        if (microseconds_ == nullptr)
            return;

        const double elapsed = std::chrono::duration<double, std::micro>(
            Clock::now() - startedAt_).count();
        if (mode_ == Mode::Accumulate)
            *microseconds_ += elapsed;
        else
            *microseconds_ = elapsed;
    }

    ScopedDuration(const ScopedDuration&) = delete;
    ScopedDuration& operator=(const ScopedDuration&) = delete;

private:
    using Clock = std::chrono::steady_clock;

    double* microseconds_;
    Mode mode_;
    Clock::time_point startedAt_;
};

} // namespace dronesim::simulation
