#include "AeroSimLabApp.h"
#include "ExecutableDirectory.h"

#include "graphics/MultisampleFramebuffer.h"
#include "FleetDroneSpawn.h"
#include "graphics/Window.h"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include <chrono>
#include <exception>
#include <filesystem>
#include <string>
#include <variant>

namespace aerosimlab::simulator
{
namespace
{
template <typename... Callables>
struct Overloaded : Callables...
{
    using Callables::operator()...;
};

template <typename... Callables>
Overloaded(Callables...) -> Overloaded<Callables...>;
}

void AeroSimLabApp::PollGroundControl()
{
#ifdef __EMSCRIPTEN__
    return;
#else
    if (!udpSocket_)
        return;

    for (int receivedCount = 0; receivedCount < 16; ++receivedCount)
    {
        communication::UdpPeer peer;
        const auto datagram = udpSocket_->Receive(peer);
        if (!datagram)
            return;

        ProcessGroundDatagram(*datagram, peer);
    }
#endif
}

#ifndef __EMSCRIPTEN__
void AeroSimLabApp::ProcessGroundDatagram(const std::string& datagram,
    const communication::UdpPeer& peer)
{
    communication::CommandPacket packet;
    try
    {
        packet = communication::DecodeCommand(datagram);
    }
    catch (const std::exception& error)
    {
        udpSocket_->Send(communication::EncodeError(error.what()), peer);
        return;
    }

    lastGroundControlMessage_ = std::chrono::steady_clock::now();
    const bool peerChanged = activePeer_
        && (activePeer_->addressNetworkOrder != peer.addressNetworkOrder
            || activePeer_->portNetworkOrder != peer.portNetworkOrder);
    if (!activePeer_ || (peerChanged && !pendingCommand_))
    {
        activePeer_ = peer;
        if (peerChanged)
        {
            sequenceGuard_ = {};
            lastReply_.reset();
        }
    }

    const auto decision = sequenceGuard_.Inspect(packet);
    const auto makeReply = [this, &packet]
    {
        communication::CommandReply reply;
        reply.commandId = packet.commandId;
        reply.sequence = packet.sequence;
        reply.telemetry = CurrentTelemetry();
        return reply;
    };
    if (ReplyToUnqueueableGroundCommand(packet, peer, decision))
        return;

    sequenceGuard_.Commit(packet);
    const auto* fleetCommand = std::get_if<communication::FleetCommand>(
        &packet.command);
    if (fleetCommand
        && *fleetCommand == communication::FleetCommand::GetStatus)
    {
        auto reply = makeReply();
        reply.accepted = true;
        reply.status = "fleet_status";
        reply.appliedTick = simulation_.TickCount();
        lastReply_ = reply;
        udpSocket_->Send(communication::EncodeReply(reply), peer);
        return;
    }

    externalControl_ = true;
    if (!std::holds_alternative<communication::FleetCommand>(packet.command))
    {
        groundControlActive_ = true;
        window_->SetTitle("AeroSimLab | 지상통제 입력 사용 중");
    }
    pendingControllerAction_.reset();
    pendingPilotCommand_.reset();
    const auto elapsed = std::chrono::steady_clock::now() - startTime_;
    const auto receivedAtMs = static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
    pendingCommand_ = PendingGroundCommand{packet, peer, receivedAtMs};
}

bool AeroSimLabApp::ReplyToUnqueueableGroundCommand(
    const communication::CommandPacket& packet,
    const communication::UdpPeer& peer,
    communication::SequenceGuard::Decision decision)
{
    using Decision = communication::SequenceGuard::Decision;
    if (decision == Decision::Duplicate)
    {
        if (lastReply_)
            udpSocket_->Send(communication::EncodeReply(*lastReply_), peer);
        return true;
    }

    const char* status = nullptr;
    if (decision == Decision::OutOfOrder)
        status = "rejected_sequence";
    else if (pendingCommand_)
        status = "busy";
    else
        return false;

    communication::CommandReply reply;
    reply.commandId = packet.commandId;
    reply.sequence = packet.sequence;
    reply.status = status;
    reply.telemetry = CurrentTelemetry();
    udpSocket_->Send(communication::EncodeReply(reply), peer);
    return true;
}
#endif

void AeroSimLabApp::ApplyDroneCommand(
    aerosimlab::simulation::DroneId droneId,
    const aerosimlab::simulation::DroneCommand& command,
    communication::CommandReply& reply)
{
    const auto result = ApplySimulationCommand(droneId, command);
    if (result == aerosimlab::simulation::CommandResult::Accepted
        && smokeTest_
        && command.kind == aerosimlab::simulation::DroneCommand::Kind::TakeOff)
    {
        smokeFlightCommandObserved_ = true;
    }
    reply.accepted = result == aerosimlab::simulation::CommandResult::Accepted;
    if (reply.accepted)
    {
        const bool hasActiveVelocityCommand = command.kind
                == aerosimlab::simulation::DroneCommand::Kind::SetVelocity
            && glm::length(command.vector) > 0.0;
        const bool hasActiveYawRateCommand = command.kind
                == aerosimlab::simulation::DroneCommand::Kind::SetYawRate
            && command.yawRateRadiansPerSecond != 0.0;
        if (hasActiveVelocityCommand || hasActiveYawRateCommand)
        {
            groundControlCommandExpiresAtTick_[droneId] = simulation_.TickCount()
                + groundControlCommandTimeoutTicks_;
        }
        else
        {
            groundControlCommandExpiresAtTick_.erase(droneId);
        }
    }

    using Result = aerosimlab::simulation::CommandResult;
    switch (result)
    {
    case Result::Accepted: reply.status = "applied"; break;
    case Result::InvalidValue: reply.status = "invalid_value"; break;
    case Result::InvalidFlightMode: reply.status = "invalid_flight_mode"; break;
    case Result::UnsupportedDynamics: reply.status = "unsupported_dynamics"; break;
    case Result::SimulationPaused: reply.status = "simulation_paused"; break;
    case Result::DroneNotFound: reply.status = "drone_not_found"; break;
    }
}

void AeroSimLabApp::ApplyCameraCommand(
    communication::CameraCommand command,
    communication::CommandReply& reply)
{
    using Command = communication::CameraCommand;
    switch (command)
    {
    case Command::Enable: simulation_.SetCameraEnabled(true); break;
    case Command::Disable: simulation_.SetCameraEnabled(false); break;
    case Command::InjectFault: simulation_.SetCameraFault(true); break;
    case Command::Recover: simulation_.SetCameraFault(false); break;
    }

    reply.accepted = true;
    reply.status = "applied";
}

void AeroSimLabApp::ApplyAddDroneCommand(
    communication::CommandReply& reply)
{
    if (simulation_.DroneCount() >= communication::maximumFleetDroneCount)
    {
        reply.accepted = false;
        reply.status = "fleet_limit_reached";
        return;
    }

    const glm::dvec3 primaryPosition = simulation_.State().positionMeters;
    const auto spawn = detail::FindFleetDroneSpawnPose(
        terrainGrid_, primaryPosition, simulation_.DroneCount() - 1U);
    if (!spawn)
    {
        reply.accepted = false;
        reply.status = "spawn_outside_terrain";
        return;
    }

    const auto id = simulation_.AddDrone(
        spawn->positionMeters, 0.0, spawn->groundElevationMeters);
    ConfigureTrainingBattery(id);
    fleetDroneIds_.push_back(id);
    reply.accepted = true;
    reply.status = "drone_added";
}

void AeroSimLabApp::ExpireGroundControlCommand()
{
    const auto now = simulation_.TickCount();
    for (auto command = groundControlCommandExpiresAtTick_.begin();
         command != groundControlCommandExpiresAtTick_.end();)
    {
        if (now < command->second)
        {
            ++command;
            continue;
        }

        const aerosimlab::simulation::DroneCommand stop{
            aerosimlab::simulation::DroneCommand::Kind::Stop};
        static_cast<void>(ApplySimulationCommand(command->first, stop));
        command = groundControlCommandExpiresAtTick_.erase(command);
    }
}

bool AeroSimLabApp::ProcessPendingGroundCommand()
{
#ifdef __EMSCRIPTEN__
    return false;
#else
    if (!pendingCommand_)
        return false;

    const auto& pending = *pendingCommand_;
    communication::CommandReply reply;
    reply.commandId = pending.packet.commandId;
    reply.sequence = pending.packet.sequence;
    reply.receivedAtMilliseconds = pending.receivedAtMilliseconds;

    std::visit(Overloaded{
        [](std::monostate) {},
        [this, &reply](const aerosimlab::simulation::DroneCommand& command)
        {
            const auto droneId = pendingCommand_->packet.droneId
                .value_or(simulation_.PrimaryDroneId());
            ApplyDroneCommand(droneId, command, reply);
        },
        [this, &reply](communication::CameraCommand command)
        {
            ApplyCameraCommand(command, reply);
        },
        [this, &reply](communication::FleetCommand command)
        {
            if (command == communication::FleetCommand::AddDrone)
                ApplyAddDroneCommand(reply);
            else if (command == communication::FleetCommand::DispatchReconnaissance)
            {
                reply.accepted = StartFleetReconnaissance();
                reply.status = reply.accepted
                    ? "fleet_reconnaissance_started" : "fleet_reconnaissance_rejected";
            }
        }
    }, pending.packet.command);

    simulation_.AdvanceOneTick();

    reply.appliedTick = simulation_.TickCount();
    reply.telemetry = CurrentTelemetry();
    lastReply_ = reply;
    try
    {
        udpSocket_->Send(communication::EncodeReply(reply), pending.peer);
    }
    catch (const std::exception& error)
    {
#ifdef _WIN32
        const std::string message = std::string("지상통제 응답 전송 실패: ") + error.what() + "\n";
        OutputDebugStringA(message.c_str());
#else
        (void)error;
#endif
    }
    pendingCommand_.reset();
    return true;
#endif
}

communication::Telemetry AeroSimLabApp::CurrentTelemetry() const
{
    communication::Telemetry telemetry;
    telemetry.tick = simulation_.TickCount();
    telemetry.simulationTimeSeconds = simulation_.SimulationTimeSeconds();
    telemetry.primaryDroneId = simulation_.PrimaryDroneId();
    telemetry.drone = simulation_.State();
    telemetry.camera = simulation_.Camera();
    telemetry.drones.reserve(fleetDroneIds_.size());
    for (const auto id : fleetDroneIds_)
    {
        const auto& drone = simulation_.State(id);
        const auto mission = fleetReconnaissance_.find(id);
        telemetry.drones.push_back({
            id, "드론 " + std::to_string(id), drone.mode, drone.positionMeters,
            mission == fleetReconnaissance_.end() ? std::string{}
                : reconTargets_[mission->second.targetIndex].displayName,
            mission == fleetReconnaissance_.end() ? std::string{}
                : std::string(detail::FleetDroneStatusLabel(
                    mission->second.status))});
    }
    return telemetry;
}

}
