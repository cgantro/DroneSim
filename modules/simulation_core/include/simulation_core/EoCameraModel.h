#pragma once

#include <cstdint>
#include <string_view>

namespace aerosimlab::simulation
{
/**
 * @brief 탑재 RGB 센서의 동작 상태를 나타낸다.
 * @details Active일 때만 모의 Tick마다 영상 프레임 번호가 증가한다. Disabled는 명령으로 끈 상태이고,
 * Fault는 장애 주입으로 영상 생성을 멈춘 상태다.
 */
enum class CameraStatus
{
    Active,
    Disabled,
    Fault
};

/** @brief 카메라 상태와 마지막으로 생성한 모의 프레임 정보를 보관한다. */
struct CameraState
{
    CameraStatus status = CameraStatus::Active;
    std::uint64_t frameCount = 0;
    std::uint64_t lastFrameTick = 0;
};

/** @brief Flecs의 카메라 상태 Component에 적용할 센서 동작을 제공한다. */
class EoCameraModel
{
public:
    /** @brief 현재 Tick에서 영상 생성이 가능하면 프레임 번호를 갱신한다. */
    static void AdvanceOneTick(CameraState& state, std::uint64_t tick) noexcept;

    /** @brief 카메라를 켜거나 끈다. */
    static void SetEnabled(CameraState& state, bool enabled) noexcept;

    /** @brief 카메라 고장을 주입한다. */
    static void InjectFault(CameraState& state) noexcept;

    /** @brief 고장을 해제하고 카메라를 다시 켠다. */
    static void Recover(CameraState& state) noexcept;

    /** @brief 프레임 번호와 센서 상태를 초기값으로 되돌린다. */
    static void Reset(CameraState& state) noexcept;
};

/** @brief 센서 상태를 시나리오 결과와 사용자 화면에 쓸 짧은 이름으로 바꾼다. */
[[nodiscard]] std::string_view ToString(CameraStatus status) noexcept;
}
