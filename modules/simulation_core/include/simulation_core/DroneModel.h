#pragma once

#include <glm/vec3.hpp>

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
 * @details 위치는 미터, 속도는 초당 미터, 방향각과 회전 속도는 라디안 단위다. 위치와 속도는 장면의 월드 좌표를 쓴다.
 */
struct DroneState
{
    glm::dvec3 positionMeters{0.0};
    glm::dvec3 velocityMetersPerSecond{0.0};
    double yawRadians = 0.0;
    FlightMode mode = FlightMode::Grounded;
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
    SimulationPaused
};

/**
 * @brief 고정 시간 간격으로 갱신하는 단순 쿼드콥터 운동 모델이다.
 * @details 실제 공기 흐름이나 모터 힘을 계산하지 않고 위치와 속도를 갱신한다. 한 단계는 1/60초이며,
 * 이 모델은 시나리오·명령 흐름을 시험하기 위한 것이므로 실제 비행 성능을 예측하지 않는다.
 */
class DroneModel
{
public:
    static constexpr double fixedStepSeconds = 1.0 / 60.0;
    static constexpr double maximumSpeedMetersPerSecond = 5.0;
    static constexpr double maximumYawRateRadiansPerSecond = 1.5;

    /** @brief 현재 비행 상태를 반환한다. */
    [[nodiscard]] const DroneState& State() const noexcept;

    /** @brief 허용되는 비행 상태와 입력 값인지 확인한 뒤 명령을 적용한다. */
    [[nodiscard]] CommandResult Apply(const DroneCommand& command) noexcept;

    /** @brief 고정된 1/60초만큼 위치와 방향을 갱신한다. */
    void AdvanceOneTick() noexcept;

    /** @brief 드론을 원점의 지상 상태로 되돌린다. */
    void Reset() noexcept;

private:
    static constexpr double takeoffHeightMeters = 1.5;
    static constexpr double climbSpeedMetersPerSecond = 1.0;
    static constexpr double waypointToleranceMeters = 0.05;
    static constexpr double altitudeToleranceMeters = 1.0e-12;

    [[nodiscard]] bool IsAirborne() const noexcept;

    DroneState state_;
    glm::dvec3 waypointMeters_{0.0};
    double commandedYawRateRadiansPerSecond_ = 0.0;
};
}
