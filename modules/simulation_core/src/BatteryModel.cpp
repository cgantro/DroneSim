#include "simulation_core/BatteryModel.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace aerosimlab::simulation
{
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
    constexpr double secondsPerHour = 3'600.0;
    const double usedWattHours = powerWatts * DroneModel::fixedStepSeconds
        / secondsPerHour;
    remainingWattHours_ = std::max(0.0, remainingWattHours_ - usedWattHours);
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
