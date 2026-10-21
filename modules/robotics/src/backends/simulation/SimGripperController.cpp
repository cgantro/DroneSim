#include "robotics/backends/simulation/SimGripperController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace grasplink::robotics::backends::simulation
{
namespace
{
struct SimGripperControllerMessages final
{
    inline static constexpr char notConnected[] = "SimGripperController: not connected";
};

Result Failure(ErrorCode code, const char* message)
{
    return {code, message};
}

bool IsFinitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}
} // namespace

SimGripperController::SimGripperController(
    const models::GripperSpecification& specification,
    SimGripperMotionSettings settings)
    : specification_(specification), settings_(settings)
{
    if (specification_.joints == nullptr || specification_.jointCount == 0 ||
        specification_.positionRequestMin >= specification_.positionRequestMax ||
        specification_.speedRequestMin >= specification_.speedRequestMax ||
        specification_.forceRequestMin > specification_.forceRequestMax ||
        !IsFinitePositive(specification_.nominalMasterClosedRadians))
    {
        throw std::invalid_argument("SimGripperController: invalid gripper specification");
    }

    if (!IsFinitePositive(settings_.minimumMasterVelocityRadiansPerSecond) ||
        !IsFinitePositive(settings_.maximumMasterVelocityRadiansPerSecond) ||
        settings_.minimumMasterVelocityRadiansPerSecond > settings_.maximumMasterVelocityRadiansPerSecond)
    {
        throw std::invalid_argument("SimGripperController: invalid master velocity settings");
    }
}

Result SimGripperController::Connect()
{
    if (connected_)
        return Result::Success();

    state_ = {};
    state_.mode = GripperMode::Inactive;
    state_.objectStatus = GripperObjectStatus::AtRequestedPosition;
    state_.requestedPositionEcho = specification_.positionRequestMin;
    state_.actualPosition = specification_.positionRequestMin;
    state_.currentRaw = 0;
    state_.currentValid = false;
    state_.closureFraction = 0.0;
    state_.closureFractionValid = true;
    state_.valid = true;
    targetClosureFraction_ = 0.0;
    masterVelocityRadiansPerSecond_ = 0.0;
    connected_ = true;
    contactStopped_ = false;
    requestedDirection_ = RequestedDirection::None;
    return Result::Success();
}

void SimGripperController::Disconnect() noexcept
{
    ++releaseRevision_;
    contactStopped_ = false;
    requestedDirection_ = RequestedDirection::None;
    connected_ = false;
    state_ = {};
    state_.mode = GripperMode::Disconnected;
    state_.valid = false;
    state_.closureFractionValid = false;
    targetClosureFraction_ = 0.0;
    masterVelocityRadiansPerSecond_ = 0.0;
}

bool SimGripperController::IsConnected() const noexcept
{
    return connected_;
}

Result SimGripperController::Activate()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, SimGripperControllerMessages::notConnected);

    if (state_.activated)
        return Result::Success();

    state_.activated = true;
    state_.activationStatus = 3;
    state_.goToActive = false;
    state_.mode = GripperMode::Idle;
    state_.objectStatus = IsAtTarget()
        ? GripperObjectStatus::AtRequestedPosition
        : GripperObjectStatus::Moving;
    return Result::Success();
}

Result SimGripperController::Reset()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, SimGripperControllerMessages::notConnected);

    const bool reachedRequestedPosition = IsAtTarget();
    ++releaseRevision_;
    contactStopped_ = false;
    requestedDirection_ = RequestedDirection::None;
    masterVelocityRadiansPerSecond_ = 0.0;
    state_.activated = false;
    state_.activationStatus = 0;
    state_.goToActive = false;
    state_.mode = GripperMode::Inactive;
    state_.objectStatus = reachedRequestedPosition
        ? GripperObjectStatus::AtRequestedPosition
        : GripperObjectStatus::Moving;
    return Result::Success();
}

Result SimGripperController::Command(const GripperCommand& command)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, SimGripperControllerMessages::notConnected);
    if (!state_.activated)
        return Failure(ErrorCode::Busy, "SimGripperController: not activated");

    // 위치·속도·힘 raw code 범위를 전부 확인한 뒤 요청을 받아들인다. 하나라도 잘못되면 현재 상태 복사본(snapshot)과 이동 목표를 바꾸지 않는다.
    if (command.positionRequest < specification_.positionRequestMin ||
        command.positionRequest > specification_.positionRequestMax ||
        command.speedRequest < specification_.speedRequestMin ||
        command.speedRequest > specification_.speedRequestMax ||
        command.forceRequest < specification_.forceRequestMin ||
        command.forceRequest > specification_.forceRequestMax)
    {
        return Failure(ErrorCode::InvalidCommand, "SimGripperController: request outside specification range");
    }

    const double positionRange = static_cast<double>(
        specification_.positionRequestMax - specification_.positionRequestMin);
    ++commandRevision_;
    const double requestedClosure = static_cast<double>(
        command.positionRequest - specification_.positionRequestMin) / positionRange;

    // 접촉 정지 뒤 같은 방향으로 더 닫으라는 반복 요청은 정지를 해제하지 않는다.
    // 열린 목표가 들어오면 파지 해제 번호를 바꾸므로 Reset 후 즉시 Activate한 경우도 adapter가 놓치지 않는다.
    RequestedDirection direction = RequestedDirection::None;
    if (requestedClosure > state_.closureFraction)
        direction = RequestedDirection::Closing;
    else if (requestedClosure < state_.closureFraction)
        direction = RequestedDirection::Opening;

    if (direction == RequestedDirection::Opening)
        ++releaseRevision_;
    if (contactStopped_ && direction != RequestedDirection::None && direction == requestedDirection_)
    {
        targetClosureFraction_ = requestedClosure;
        state_.requestedPositionEcho = command.positionRequest;
        return Result::Success();
    }
    targetClosureFraction_ = requestedClosure;
    requestedDirection_ = direction;
    contactStopped_ = false;

    const double speedRange = static_cast<double>(
        specification_.speedRequestMax - specification_.speedRequestMin);
    const double speedFraction = static_cast<double>(
        command.speedRequest - specification_.speedRequestMin) / speedRange;
    masterVelocityRadiansPerSecond_ = settings_.minimumMasterVelocityRadiansPerSecond +
        speedFraction * (settings_.maximumMasterVelocityRadiansPerSecond -
            settings_.minimumMasterVelocityRadiansPerSecond);

    state_.requestedPositionEcho = command.positionRequest;
    state_.goToActive = !IsAtTarget();
    state_.mode = state_.goToActive ? GripperMode::Moving : GripperMode::Idle;
    state_.objectStatus = state_.goToActive
        ? GripperObjectStatus::Moving
        : GripperObjectStatus::AtRequestedPosition;
    return Result::Success();
}

Result SimGripperController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, SimGripperControllerMessages::notConnected);

    const bool reachedRequestedPosition = IsAtTarget();
    requestedDirection_ = RequestedDirection::None;
    masterVelocityRadiansPerSecond_ = 0.0;
    state_.goToActive = false;
    state_.mode = GripperMode::Stopped;
    state_.objectStatus = reachedRequestedPosition
        ? GripperObjectStatus::AtRequestedPosition
        : GripperObjectStatus::Moving;
    return Result::Success();
}

GripperState SimGripperController::GetState() const
{
    return state_;
}

void SimGripperController::ApplyContactFeedback(bool opening)
{
    if (!connected_ || !state_.activated || requestedDirection_ == RequestedDirection::None)
        return;
    contactStopped_ = true;
    state_.goToActive = false;
    state_.mode = GripperMode::Stopped;
    state_.objectStatus = opening ? GripperObjectStatus::ContactWhileOpening : GripperObjectStatus::ContactWhileClosing;
    masterVelocityRadiansPerSecond_ = 0.0;
}

bool SimGripperController::IsClosingRequested() const noexcept
{
    return connected_ && state_.activated && requestedDirection_ == RequestedDirection::Closing;
}

bool SimGripperController::IsOpeningRequested() const noexcept
{
    return connected_ && state_.activated && requestedDirection_ == RequestedDirection::Opening;
}

void SimGripperController::Update(double dtSeconds)
{
    if (!connected_ || !state_.activated || state_.mode != GripperMode::Moving ||
        !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
        return;
    }

    // 연속 개폐 비율은 master 관절의 각속도로 진행하며 기준 닫힘 각도와 시간으로 변화량을 정한다. 관절 계산에 8-bit raw 위치를 다시 넣지 않아 정수 반올림이 운동에 반복 전달되지 않는다.
    const double maximumClosureStep = masterVelocityRadiansPerSecond_ * dtSeconds /
        specification_.nominalMasterClosedRadians;
    const double remaining = targetClosureFraction_ - state_.closureFraction;
    if (std::abs(remaining) <= maximumClosureStep)
        state_.closureFraction = targetClosureFraction_;
    else
        state_.closureFraction += std::copysign(maximumClosureStep, remaining);

    UpdateRawPosition();

    if (IsAtTarget())
    {
        state_.goToActive = false;
        state_.mode = GripperMode::Idle;
        state_.objectStatus = GripperObjectStatus::AtRequestedPosition;
        masterVelocityRadiansPerSecond_ = 0.0;
    }
}

bool SimGripperController::IsAtTarget() const noexcept
{
    return state_.closureFraction == targetClosureFraction_;
}

void SimGripperController::UpdateRawPosition() noexcept
{
    const double raw = specification_.positionRequestMin + state_.closureFraction *
        static_cast<double>(specification_.positionRequestMax - specification_.positionRequestMin);
    const long rounded = std::lround(raw);
    const long bounded = std::clamp(rounded,
        static_cast<long>(specification_.positionRequestMin),
        static_cast<long>(specification_.positionRequestMax));
    state_.actualPosition = static_cast<std::uint8_t>(bounded);
}

} // namespace grasplink::robotics::backends::simulation
