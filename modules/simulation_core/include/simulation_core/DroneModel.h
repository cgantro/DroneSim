#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <array>
#include <optional>
#include <string_view>

namespace dronesim::simulation
{
/**
 * @brief 드론이 고정 간격 모의에서 따르는 비행 상태를 나타낸다.
 * @details Grounded에서는 명령을 받아도 움직이지 않는다. TakingOff와 Landing은 목표 높이까지 오르내리는 중이며,
 * Hovering과 Flying에서는 공중에 머물거나 속도 명령을 따른다. FollowingWaypoint는 지정한 위치까지 이동한다.
 */
enum class FlightMode
{
    Grounded,
    TakingOff,
    Hovering,
    Flying,
    FollowingWaypoint,
    Landing
};

/** @brief 비행 상태를 통신과 시나리오에서 쓰는 짧은 이름으로 바꾼다. */
[[nodiscard]] std::string_view ToString(FlightMode mode) noexcept;

/** @brief 알려진 비행 상태 이름이면 해당 값으로 바꾼다. */
[[nodiscard]] std::optional<FlightMode> TryParseFlightMode(std::string_view name) noexcept;

/**
 * @brief 한 번의 모의 단계가 끝난 뒤 드론의 위치와 움직임을 보관한다.
 * @details 위치와 선속도는 장면 좌표(ENU) 기준이다. 자세는 Quaternion, 각속도는 기체 축(FLU) 기준이며 단위는 각각 라디안과 라디안/초다.
 */
struct DroneState
{
    glm::dvec3 positionMeters{0.0};
    glm::dvec3 velocityMetersPerSecond{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 angularVelocityRadiansPerSecond{0.0};
    /** @brief 물리 엔진이 계산한 네 로터의 현재 회전 속도다. 단위는 라디안/초다. */
    std::array<double, 4> motorSpeedsRadiansPerSecond{};
    double yawRadians = 0.0;
    FlightMode mode = FlightMode::Grounded;
};

/** @brief 다음 모의 단계에 사용할 드론 제어 목표를 보관한다. */
struct DroneControlState
{
    glm::dvec3 waypointMeters{0.0};
    glm::dvec3 targetVelocityMetersPerSecond{0.0};
    double commandedYawRateRadiansPerSecond = 0.0;
    double desiredYawRadians = 0.0;
    double groundElevationMeters = 0.0;
    /** @brief 값이 있으면 비행 제어기 대신 시험용 로터 추력 비율을 사용한다. */
    std::optional<std::array<double, 4>> commandedRotorThrustRatios;
};

/**
 * @brief 화면 조작·시나리오·게임패드가 함께 사용하는 드론 명령이다.
 * @details vector에는 목표 속도 또는 월드 좌표의 경유점을 넣고, yawRate에는 라디안/초 단위 회전 속도를 넣는다.
 * 명령을 받은 시점에 드론 상태가 해당 동작을 허용하지 않으면 거부한다.
 */
struct DroneCommand
{
    enum class Kind
    {
        TakeOff,
        Land,
        SetVelocity,
        SetWaypoint,
        SetYawRate,
        SetRotorThrusts,
        Stop
    };

    Kind kind = Kind::Stop;
    glm::dvec3 vector{0.0};
    double yawRateRadiansPerSecond = 0.0;
    std::array<double, 4> rotorThrustRatios{};
    /** @brief 경유점 이동 방향과 다른 방향을 바라봐야 할 때 지정한다. */
    std::optional<double> targetYawRadians;
};

/** @brief 드론이 명령을 받았는지와 거부 이유를 나타낸다. */
enum class CommandResult
{
    Accepted,
    InvalidValue,
    InvalidFlightMode,
    SimulationPaused,
    DroneNotFound
};

/** @brief 비행 명령을 확인해 드론의 목표와 상태에 반영한다. */
class DroneModel
{
public:
    static constexpr double fixedStepSeconds = 1.0 / 60.0;
    static constexpr double takeoffHeightMeters = 1.5;
    static constexpr double maximumSpeedMetersPerSecond = 5.0;
    static constexpr double maximumYawRateRadiansPerSecond = 1.5;

    /** @brief 허용되는 비행 상태와 입력 값인지 확인한 뒤 명령을 적용한다. */
    [[nodiscard]] static CommandResult Apply(
        DroneState& state, DroneControlState& control, const DroneCommand& command) noexcept;

    /** @brief 자세 쿼터니언에서 월드 Z축 기준 방향각을 라디안으로 구한다. */
    [[nodiscard]] static double YawRadians(const DroneState& state) noexcept;

    /** @brief 드론을 원점의 지상 상태로 되돌린다. */
    static void Reset(DroneState& state, DroneControlState& control) noexcept;

private:
    static constexpr double waypointToleranceMeters = 0.05;

    [[nodiscard]] static bool IsAirborne(const DroneState& state) noexcept;
};
}
