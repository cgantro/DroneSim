#include "simulation_core/BatteryModel.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dronesim::simulation
{
bool BatteryModelParameters::IsValid() const noexcept
{
    // 에너지는 양의 용량을 기준으로 계산한다. 전력은 0도 허용하고,
    // 예비량은 전체 용량의 0~100% 사이여야 한다.
    return std::isfinite(capacityWattHours)
        && capacityWattHours > 0.0
        && std::isfinite(hoverPowerWatts)
        && hoverPowerWatts >= 0.0
        && std::isfinite(flightPowerWatts)
        && flightPowerWatts >= 0.0
        && std::isfinite(reservePercent)
        && reservePercent >= 0.0
        && reservePercent <= 100.0;
}

/** @brief 배터리 설정과 시작 잔량으로 사용 가능 에너지를 초기화한다. */
BatteryModel::BatteryModel(
    BatteryModelParameters parameters, double initialPercent)
    : parameters_(parameters)
{
    if (!parameters_.IsValid()
        || !std::isfinite(initialPercent)
        || initialPercent < 0.0
        || initialPercent > 100.0)
    {
        throw std::invalid_argument("배터리 용량·전력·예비량 설정이 올바르지 않습니다.");
    }

    initialWattHours_ = parameters_.capacityWattHours * initialPercent / 100.0;
    remainingWattHours_ = initialWattHours_;
}

void BatteryModel::Advance(FlightMode mode) noexcept
{
    if (remainingWattHours_ <= 0.0 || mode == FlightMode::Grounded)
        return;

    // 호버링만 호버 전력을 쓰고, 이륙·이동·착륙은 비행 전력으로 근사한다.
    // 실제 속도나 로터 추력에 따라 전력이 달라지는 효과는 이 간단한 모델에서 다루지 않는다.
    const double powerWatts = mode == FlightMode::Hovering
        ? parameters_.hoverPowerWatts : parameters_.flightPowerWatts;
    // 와트 × 초를 와트시로 바꿔 고정 Tick마다 남은 에너지에서 뺀다.
    constexpr double secondsPerHour = 3'600.0;
    const double usedWattHours = powerWatts * DroneModel::fixedStepSeconds
        / secondsPerHour;
    remainingWattHours_ = std::max(0.0, remainingWattHours_ - usedWattHours);
}

void BatteryModel::Reset() noexcept
{
    // 재시작 시 현재 잔량과 저전압 복귀 표시를 초기 상태로 되돌린다.
    remainingWattHours_ = initialWattHours_;
    returningForLowBattery_ = false;
}

double BatteryModel::CapacityWattHours() const noexcept
{
    return parameters_.capacityWattHours;
}

double BatteryModel::InitialWattHours() const noexcept
{
    return initialWattHours_;
}

double BatteryModel::RemainingWattHours() const noexcept
{
    return remainingWattHours_;
}

double BatteryModel::RemainingPercent() const noexcept
{
    return 100.0 * remainingWattHours_ / parameters_.capacityWattHours;
}

double BatteryModel::ConsumedWattHours() const noexcept
{
    return initialWattHours_ - remainingWattHours_;
}

bool BatteryModel::ShouldReturnForLowBattery() const noexcept
{
    return RemainingPercent() <= parameters_.reservePercent;
}

bool BatteryModel::IsReturningForLowBattery() const noexcept
{
    return returningForLowBattery_;
}

bool BatteryModel::TryBeginLowBatteryReturn() noexcept
{
    if (returningForLowBattery_ || !ShouldReturnForLowBattery())
        return false;

    // 복귀 상태를 기억해 이후 Tick에서 같은 복귀 명령을 반복 시작하지 않게 한다.
    returningForLowBattery_ = true;
    return true;
}
}
