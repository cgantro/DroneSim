#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <cstdint>
#include <string_view>

namespace aerosimlab::simulation
{
struct DroneState;

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

/** @brief 카메라 모델이 만드는 영상의 크기와 시야각이다. */
struct CameraModel
{
    static constexpr int maximumImageDimensionPixels = 8192;

    int imageWidthPixels = 1280;
    int imageHeightPixels = 960;
    double verticalFieldOfViewDegrees = 70.0;

    /** @brief 영상 크기와 시야각이 사용할 수 있는 값인지 확인한다. */
    [[nodiscard]] bool IsValid() const noexcept;
};

/**
 * @brief 카메라 동작 상태와 드론에 고정된 위치·자세를 보관한다.
 * @details 위치는 월드 좌표계의 미터 단위다. 카메라 중심은 기체 앞쪽 0.12m, 아래쪽 0.10m에 둔다.
 * 위치와 자세 모두 동체에 고정하며 광축의 아래쪽 기울기도 시뮬레이션 좌표에서 계산한다.
 */
struct CameraState
{
    CameraStatus status = CameraStatus::Active;
    CameraModel model;
    std::uint64_t frameCount = 0;
    std::uint64_t lastFrameTick = 0;
    glm::dvec3 positionMeters{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
};

/** @brief Flecs의 카메라 상태 Component에 적용할 센서 동작을 제공한다. */
class EoCameraModel
{
public:
    /** @brief 현재 Tick에서 영상 생성이 가능하면 프레임 번호를 갱신한다. */
    static void AdvanceOneTick(CameraState& state, std::uint64_t tick) noexcept;

    /** @brief 드론의 월드 위치와 자세에서 탑재 카메라의 월드 자세를 갱신한다. */
    static void UpdatePose(CameraState& camera, const DroneState& drone) noexcept;

    /** @brief 현재 카메라가 바라보는 월드 기준 단위 방향을 돌려준다. */
    [[nodiscard]] static glm::dvec3 OpticalAxis(const CameraState& camera) noexcept;

    /** @brief 카메라 영상의 위쪽에 해당하는 월드 기준 단위 방향을 돌려준다. */
    [[nodiscard]] static glm::dvec3 UpAxis(const CameraState& camera) noexcept;

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
