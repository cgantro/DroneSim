#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/EoCameraModel.h"

#include <cstdint>

namespace aerosimlab::simulation
{
/**
 * @brief 화면이 없어도 같은 순서로 한 단계씩 실행할 수 있는 드론 모의 코어다.
 * @details 호출자가 AdvanceOneTick을 부를 때마다 모의 시간이 1/60초 진행된다. 일시정지 중에는 상태와 시간이 바뀌지 않는다.
 */
class SimulationCore
{
public:
    /** @brief 드론에 명령을 보내고 적용 결과를 반환한다. */
    [[nodiscard]] CommandResult Apply(const DroneCommand& command) noexcept;

    /** @brief 실행 중이면 고정 간격 한 단계만 진행하고, 일시정지 중이면 아무것도 바꾸지 않는다. */
    void AdvanceOneTick() noexcept;

    /** @brief 모의 시간 진행을 멈춘다. */
    void Pause() noexcept;

    /** @brief 모의 시간 진행을 다시 시작한다. */
    void Resume() noexcept;

    /** @brief 시간, 드론 위치와 상태를 처음 값으로 되돌리고 실행을 계속한다. */
    void Reset() noexcept;

    /** @brief 현재 드론 상태를 반환한다. */
    [[nodiscard]] const DroneState& State() const noexcept;

    /** @brief 모의 카메라의 상태와 프레임 생성 정보를 반환한다. */
    [[nodiscard]] const CameraState& Camera() const noexcept;

    /** @brief 카메라를 켜거나 끈다. 고장 상태는 복구 명령으로만 해제된다. */
    void SetCameraEnabled(bool enabled) noexcept;

    /** @brief 카메라 장애를 주입하거나 해제한다. */
    void SetCameraFault(bool faulted) noexcept;

    /** @brief 지금까지 실행한 고정 간격 단계 수를 반환한다. */
    [[nodiscard]] std::uint64_t TickCount() const noexcept;

    /** @brief 누적된 모의 시간을 초 단위로 반환한다. */
    [[nodiscard]] double SimulationTimeSeconds() const noexcept;

    /** @brief 모의가 일시정지 상태인지 반환한다. */
    [[nodiscard]] bool IsPaused() const noexcept;

private:
    DroneModel drone_;
    EoCameraModel camera_;
    std::uint64_t tickCount_ = 0;
    bool paused_ = false;
};
}
