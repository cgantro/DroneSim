#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec3.hpp>

#include <array>

namespace aerosimlab::simulation
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

/**
 * @brief 한 번의 모의 단계가 끝난 뒤 드론의 위치와 움직임을 보관한다.
 * @details 위치는 미터, 속도는 초당 미터, 자세는 Quaternion, 각속도는 라디안/초 단위다. 위치와 속도는 장면의 월드 좌표를 쓴다.
 */
struct DroneState
{
    glm::dvec3 positionMeters{0.0};
    glm::dvec3 velocityMetersPerSecond{0.0};
    glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 angularVelocityRadiansPerSecond{0.0};
    /** @brief AirSim이 계산한 네 로터의 회전 속도다. 단위는 라디안/초다. */
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
};

/**
 * @brief 지상통제·시나리오·수동 조작이 함께 사용할 드론 명령을 나타낸다.
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
        Stop
    };

    Kind kind = Kind::Stop;
    glm::dvec3 vector{0.0};
    double yawRateRadiansPerSecond = 0.0;
};

/** @brief 드론이 명령을 받았는지와 거부 이유를 나타낸다. */
enum class CommandResult
{
    Accepted,
    InvalidValue,
    InvalidFlightMode,
    UnsupportedDynamics,
    SimulationPaused,
    DroneNotFound
};

/**
 * @brief 현재 비행 규칙을 드론 상태 데이터에 적용한다.
 * @details 실제 공기 흐름이나 모터 힘은 계산하지 않는다. 이 규칙은 시나리오·명령 흐름을 시험하기 위한 것으로,
 * 실제 비행 성능을 예측하지 않는다.
 */
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

    /** @brief 고정된 1/60초만큼 위치와 방향을 갱신하고, 지면에 닿으면 정지시킨다. */
    static void AdvanceOneTick(DroneState& state, DroneControlState& control) noexcept;

    /** @brief 드론을 원점의 지상 상태로 되돌린다. */
    static void Reset(DroneState& state, DroneControlState& control) noexcept;

private:
    static constexpr double climbSpeedMetersPerSecond = 1.0;
    static constexpr double waypointToleranceMeters = 0.05;
    static constexpr double altitudeToleranceMeters = 1.0e-12;

    [[nodiscard]] static bool IsAirborne(const DroneState& state) noexcept;
};
}
