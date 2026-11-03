#include "manual_control/ManualControlSession.h"

namespace dronesim::manual_control
{
void ManualControlSession::AcceptGamepadUpdate(
    const GamepadController::Update& update, bool missionOwnsInput)
{
    if (update.connectionChanged && !update.connected)
    {
        pendingFlightAction_.reset();
        if (!missionOwnsInput)
            pendingPilotCommand_.reset();
    }

    if (update.flightAction)
        pendingFlightAction_ = *update.flightAction;
    if (update.pilotCommand)
        pendingPilotCommand_ = *update.pilotCommand;
}

void ManualControlSession::QueueFlightAction(
    const simulation::DroneCommand& command)
{
    pendingFlightAction_ = command;
}

void ManualControlSession::SetWebAxes(const GamepadAxes& axes) noexcept
{
    webAxes_ = axes;
}

void ManualControlSession::ClearPendingPilotCommand() noexcept
{
    pendingPilotCommand_.reset();
}

void ManualControlSession::ClearPendingInputs() noexcept
{
    pendingFlightAction_.reset();
    pendingPilotCommand_.reset();
    webAxes_.reset();
}

void ManualControlSession::AdvanceTick(
    simulation::SimulationCore& simulation, bool manualInputBlocked)
{
    if (manualInputBlocked)
    {
        simulation.AdvanceOneTick();
        return;
    }

    if (webAxes_)
    {
        const auto flightMode = simulation.State().mode;
        if (flightMode == dronesim::simulation::FlightMode::Hovering
            || flightMode == dronesim::simulation::FlightMode::Flying)
        {
            pendingPilotCommand_ = MapAxes(*webAxes_, simulation.State().yawRadians);
        }
        else
        {
            pendingPilotCommand_.reset();
        }
    }

    if (pendingFlightAction_)
    {
        static_cast<void>(simulation.Apply(*pendingFlightAction_));
        pendingFlightAction_.reset();
    }

    if (pendingPilotCommand_)
    {
        const auto flightMode = simulation.State().mode;
        if (flightMode == dronesim::simulation::FlightMode::Hovering
            || flightMode == dronesim::simulation::FlightMode::Flying)
        {
            const dronesim::simulation::DroneCommand velocityCommand{
                dronesim::simulation::DroneCommand::Kind::SetVelocity,
                pendingPilotCommand_->velocityMetersPerSecond};
            const dronesim::simulation::DroneCommand yawCommand{
                dronesim::simulation::DroneCommand::Kind::SetYawRate,
                glm::dvec3{0.0},
                pendingPilotCommand_->yawRateRadiansPerSecond};
            static_cast<void>(simulation.Apply(velocityCommand));
            static_cast<void>(simulation.Apply(yawCommand));
        }
    }

    simulation.AdvanceOneTick();
}
}
