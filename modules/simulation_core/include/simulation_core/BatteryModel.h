#pragma once

#include "simulation_core/DroneModel.h"

namespace dronesim::simulation
{
/** @brief 배터리 소모 계산에 사용하는 기체별 기준값이다. */
struct BatteryModelParameters
{
    double capacityWattHours = 0.0;
    double hoverPowerWatts = 0.0;
    double flightPowerWatts = 0.0;
    double reservePercent = 0.0;
};

/**
 * @brief 모의 비행 상태에 따라 남은 배터리 에너지를 계산한다.
 * @details 호버링과 그 밖의 비행에 서로 다른 평균 전력을 적용하는 단순 모델이다. 실제 기체의 측정값은 아니다.
 */
class BatteryModel
{
public:
    /** @brief Flecs가 컴포넌트 저장 공간을 만들 때 사용한다. 설정은 ConfigureBattery에서 덮어쓴다. */
    BatteryModel() = default;

    BatteryModel(BatteryModelParameters parameters, double initialPercent = 100.0);

    /** @brief 현재 비행 상태에 해당하는 에너지를 고정 Tick만큼 사용한다. */
    void Advance(FlightMode mode) noexcept;

    /** @brief 배터리를 처음 설정한 충전량으로 되돌린다. */
    void Reset() noexcept;

    [[nodiscard]] double CapacityWattHours() const noexcept;
    [[nodiscard]] double InitialWattHours() const noexcept;
    [[nodiscard]] double RemainingWattHours() const noexcept;
    [[nodiscard]] double RemainingPercent() const noexcept;
    [[nodiscard]] double ConsumedWattHours() const noexcept;
    [[nodiscard]] bool ShouldReturnForLowBattery() const noexcept;
    [[nodiscard]] bool IsReturningForLowBattery() const noexcept;
    void MarkReturningForLowBattery() noexcept;

private:
    BatteryModelParameters parameters_;
    double initialWattHours_ = 0.0;
    double remainingWattHours_ = 0.0;
    bool returningForLowBattery_ = false;
};
}
