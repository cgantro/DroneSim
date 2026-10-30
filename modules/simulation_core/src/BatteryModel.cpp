#include "simulation_core/BatteryModel.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dronesim::simulation
{
/** @brief 배터리 설정과 시작 잔량으로 사용 가능 에너지를 초기화한다. */
BatteryModel::BatteryModel(
    BatteryModelParameters parameters, double initialPercent)
    : parameters_(parameters)
{
    if (!std::isfinite(parameters_.capacityWattHours)
        || parameters_.capacityWattHours <= 0.0
        || !std::isfinite(parameters_.hoverPowerWatts)
        || parameters_.hoverPowerWatts < 0.0
        || !std::isfinite(parameters_.flightPowerWatts)
        || parameters_.flightPowerWatts < 0.0
        || !std::isfinite(parameters_.reservePercent)
        || parameters_.reservePercent < 0.0
        || parameters_.reservePercent > 100.0
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

void BatteryModel::MarkReturningForLowBattery() noexcept
{
    returningForLowBattery_ = true;
}
}
