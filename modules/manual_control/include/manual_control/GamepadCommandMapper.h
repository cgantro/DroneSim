#pragma once

#include "simulation_core/DroneModel.h"

namespace aerosimlab::manual_control
{
/**
 * @brief GLFW 게임패드의 축 값을 드론 비행 명령으로 바꾼다.
 * @details 축 값은 -1부터 1까지이며, 전진·좌우 이동은 조종기 기준으로 입력한다.
 * 드론의 현재 방향을 적용해 월드 좌표 속도로 바꾸므로 실제 상태 변경은 SimulationCore가 맡는다.
 */
struct GamepadAxes
{
    double forward = 0.0;
    double right = 0.0;
    double vertical = 0.0;
    double yaw = 0.0;
};

/** @brief 축 입력을 드론의 속도와 회전 명령 값으로 바꾼 결과다. */
struct PilotCommand
{
    glm::dvec3 velocityMetersPerSecond{0.0};
    double yawRateRadiansPerSecond = 0.0;
};

/**
 * @brief 게임패드 연결이 끊겼을 때 처리할 동작을 나타낸다.
 * @details NoChange는 이미 끊긴 상태라 대기 중인 안전 명령을 유지한다. ClearPendingAction은 연결이 끊긴 순간에 대기 중인 조종 명령을 비우고 추가 동작을 보내지 않는다.
 */
enum class GamepadDisconnectAction
{
    NoChange,
    ClearPendingAction,
    Stop,
    Land
};

/**
 * @brief 스틱 입력을 월드 좌표 속도와 회전 속도로 바꾼다.
 * @details PX4 Position Slow의 속도·회전 한도와 부드러운 스틱 감도를 참고한다.
 * 결과는 속도 목표이므로 PX4의 가속 기반 입력이나 내부 위치 제어를 그대로 재현하지 않는다.
 */
[[nodiscard]] PilotCommand MapAxes(
    const GamepadAxes& axes, double droneYawRadians) noexcept;

/** @brief 축의 작은 흔들림은 무시하고 입력 범위를 -1부터 1까지 다시 맞춘다. */
[[nodiscard]] double ApplyDeadZone(double value) noexcept;

/** @brief 지상통제나 기록 재생이 진행 중이면 게임패드 명령을 막는다. */
[[nodiscard]] bool CanGamepadControlSimulation(bool groundControlActive, bool replaying) noexcept;

/**
 * @brief 연결이 끊긴 순간에 실행할 동작을 고른다.
 * @details 이전 상태가 이미 끊김이면 NoChange를 반환해 대기 중인 정지·착륙 명령을 유지한다.
 */
[[nodiscard]] GamepadDisconnectAction ResolveGamepadDisconnectAction(
    bool wasConnected, bool manualControlBlocked, simulation::FlightMode flightMode) noexcept;
}
