#pragma once

#include "manual_control/GamepadController.h"
#include "simulation_core/SimulationCore.h"

#include <optional>

namespace dronesim::manual_control
{
/**
 * @brief 들어온 수동 조종 입력을 다음 시뮬레이션 Tick에 적용한다.
 * @details 게임패드와 웹 조작 입력을 모아 두고, 임무가 조종 권한을 가진 동안에는
 * 입력을 적용하지 않은 채 시뮬레이션만 진행한다.
 */
class ManualControlSession final
{
public:
    /** @brief 게임패드 입력을 보관하고 연결 해제 때 남은 입력을 정리한다. */
    void AcceptGamepadUpdate(const GamepadController::Update& update,
        bool missionOwnsInput);

    /** @brief 버튼으로 들어온 비행 명령을 다음 Tick에 적용하도록 보관한다. */
    void QueueFlightAction(const simulation::DroneCommand& command);

    /** @brief 웹 조작기의 축 값을 다음 Tick에 사용할 입력으로 저장한다. */
    void SetWebAxes(const GamepadAxes& axes) noexcept;

    /** @brief 다음 Tick에 적용할 스틱 명령을 비운다. */
    void ClearPendingPilotCommand() noexcept;

    /** @brief 임무 시작 전 대기 중인 수동 조작 입력을 모두 비운다. */
    void ClearPendingInputs() noexcept;

    /** @brief 한 번의 비행 Tick에서 수동 입력을 적용하고 시뮬레이션을 진행한다. */
    void AdvanceTick(simulation::SimulationCore& simulation,
        bool manualInputBlocked);

private:
    std::optional<simulation::DroneCommand> pendingFlightAction_;
    std::optional<PilotCommand> pendingPilotCommand_;
    std::optional<GamepadAxes> webAxes_;
};
}
