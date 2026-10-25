#include "AeroSimLabApp.h"
#include "FleetDroneSpawn.h"
#include "graphics/Window.h"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#include <chrono>
#include <exception>
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
        aerosimlab::communication::UdpPeer peer;
        const auto datagram = udpSocket_->Receive(peer);
        if (!datagram)
            return;

        ProcessGroundDatagram(*datagram, peer);
    }
#endif
}

#ifndef __EMSCRIPTEN__
void AeroSimLabApp::ProcessGroundDatagram(const std::string& datagram,
    const aerosimlab::communication::UdpPeer& peer)
{
    aerosimlab::communication::CommandPacket packet;
    try
    {
        packet = aerosimlab::communication::DecodeCommand(datagram);
    }
    catch (const std::exception& error)
    {
        udpSocket_->Send(aerosimlab::communication::EncodeError(error.what()), peer);
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
        aerosimlab::communication::CommandReply reply;
        reply.commandId = packet.commandId;
        reply.sequence = packet.sequence;
        reply.telemetry = CurrentTelemetry();
        return reply;
    };
    if (ReplyToUnqueueableGroundCommand(packet, peer, decision))
        return;

    sequenceGuard_.Commit(packet);
    const auto* fleetCommand = std::get_if<aerosimlab::communication::FleetCommand>(
        &packet.command);
    if (fleetCommand
        && *fleetCommand == aerosimlab::communication::FleetCommand::GetStatus)
    {
        auto reply = makeReply();
        reply.accepted = true;
        reply.status = "fleet_status";
        reply.appliedTick = simulation_.TickCount();
        lastReply_ = reply;
        udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
        return;
    }

    externalControl_ = true;
    if (!std::holds_alternative<aerosimlab::communication::SessionCommand>(packet.command)
        && !std::holds_alternative<aerosimlab::communication::FleetCommand>(packet.command))
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
    const aerosimlab::communication::CommandPacket& packet,
    const aerosimlab::communication::UdpPeer& peer,
    aerosimlab::communication::SequenceGuard::Decision decision)
{
    using Decision = aerosimlab::communication::SequenceGuard::Decision;
    if (decision == Decision::Duplicate)
    {
        if (lastReply_)
            udpSocket_->Send(aerosimlab::communication::EncodeReply(*lastReply_), peer);
        return true;
    }

    const char* status = nullptr;
    if (decision == Decision::OutOfOrder)
        status = "rejected_sequence";
    else if (pendingCommand_)
        status = "busy";
    else
        return false;

    aerosimlab::communication::CommandReply reply;
    reply.commandId = packet.commandId;
    reply.sequence = packet.sequence;
    reply.status = status;
    reply.telemetry = CurrentTelemetry();
    udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
    return true;
}
#endif

bool AeroSimLabApp::ApplySessionCommand(
    aerosimlab::communication::SessionCommand command,
    aerosimlab::communication::CommandReply& reply)
{
    using Command = aerosimlab::communication::SessionCommand;
    using FlightMode = aerosimlab::simulation::FlightMode;
    const auto resetSimulationState = [this]
    {
#ifndef __EMSCRIPTEN__
        // Tick이 0으로 돌아가기 전에 이전 GPU 읽기를 끝내고 프레임 번호 기준을 초기화한다.
        DrainEoReadbacks();
        lastQueuedEoFrameTick_ = 0;
#endif
        simulation_.Reset();
        eoFrame_ = {};
        groundControlCommandExpiresAtTick_.reset();
    };

    switch (command)
    {
    case Command::StartRecording:
        if (commandRecording_.IsRecording() || commandRecording_.IsReplaying()
#ifndef __EMSCRIPTEN__
            || (eoVideoRecorder_ && eoVideoRecorder_->IsRecording())
#endif
            )
        {
            reply.status = "recording_unavailable";
            return false;
        }
        if (simulation_.State().mode != FlightMode::Grounded)
        {
            reply.status = "recording_requires_grounded";
            return false;
        }

        resetSimulationState();
        (void)commandRecording_.StartRecording();
        externalControl_ = true;
        window_->SetTitle("AeroSimLab | 입력 기록 중");
        reply.accepted = true;
        reply.status = "recording_started";
        return false;

    case Command::StopRecording:
        if (!commandRecording_.IsRecording())
        {
            reply.status = "recording_not_active";
            return false;
        }

        (void)commandRecording_.StopRecording();
        window_->SetTitle("AeroSimLab | 입력 기록 완료");
        reply.accepted = true;
        reply.status = "recording_stopped";
        return false;

    case Command::ReplayRecording:
        if (commandRecording_.IsRecording() || commandRecording_.IsReplaying()
#ifndef __EMSCRIPTEN__
            || (eoVideoRecorder_ && eoVideoRecorder_->IsRecording())
#endif
            )
        {
            reply.status = "recording_unavailable";
            return false;
        }
        if (!commandRecording_.StartReplay())
        {
            reply.status = "recording_empty";
            return false;
        }

        resetSimulationState();
        smokeReplayStarted_ = smokeTest_;
        pendingControllerAction_.reset();
        pendingPilotCommand_.reset();
        window_->SetTitle("AeroSimLab | 입력 기록 재생 중");
        AdvanceReplayTick();
        reply.accepted = true;
        reply.status = "recording_replay_started";
        return true;
    }

    return false;
}

void AeroSimLabApp::ApplyVideoRecordingCommand(
    aerosimlab::communication::VideoRecordingCommand command,
    aerosimlab::communication::CommandReply& reply)
{
#ifdef __EMSCRIPTEN__
    static_cast<void>(command);
    reply.status = "video_recording_requires_native";
#else
    using Command = aerosimlab::communication::VideoRecordingCommand;
    if (command == Command::Start)
    {
        if (eoVideoRecorder_ && eoVideoRecorder_->IsRecording())
        {
            reply.status = "video_recording_already_active";
            return;
        }
        if (simulation_.Camera().status != aerosimlab::simulation::CameraStatus::Active)
        {
            reply.status = "video_recording_camera_unavailable";
            return;
        }

        DrainEoReadbacks();

        auto recorder = std::make_unique<EoVideoRecorder>();
        std::string error;
        const auto& model = simulation_.Camera().model;
        const auto path = NewEoRecordingPath();
        if (!recorder->Start(path, model.imageWidthPixels, model.imageHeightPixels,
            60.0, error))
        {
            reply.status = "video_recording_failed:" + error;
            return;
        }
        eoVideoRecorder_ = std::move(recorder);
        window_->SetTitle("DroneSim | EO 영상 녹화 중");
        reply.accepted = true;
        reply.status = "video_recording_started:" + eoVideoRecorder_->VideoPath().string();
        return;
    }

    if (!eoVideoRecorder_ || !eoVideoRecorder_->IsRecording())
    {
        reply.status = "video_recording_not_active";
        return;
    }

    DrainEoReadbacks();
    const auto path = eoVideoRecorder_->VideoPath();
    std::uint64_t savedFrames = 0;
    std::string error;
    const bool saved = eoVideoRecorder_->Stop(savedFrames, error);
    eoVideoRecorder_.reset();
    if (!saved)
    {
        window_->SetTitle("DroneSim | EO 녹화 저장 실패: " + error);
        reply.status = "video_recording_failed:" + error;
        return;
    }
    window_->SetTitle("DroneSim | EO 영상 녹화 완료");
    reply.accepted = true;
    reply.status = "video_recording_stopped:" + std::to_string(savedFrames)
        + ":" + path.string();
#endif
}

void AeroSimLabApp::ApplyDroneCommand(
    const aerosimlab::simulation::DroneCommand& command,
    aerosimlab::communication::CommandReply& reply)
{
    if (commandRecording_.IsReplaying())
    {
        commandRecording_.StopReplay();
        window_->SetTitle("AeroSimLab | 지상통제 입력 사용 중");
    }

    const auto result = ApplyAndRecordCommand(command);
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
            groundControlCommandExpiresAtTick_ = simulation_.TickCount()
                + groundControlCommandTimeoutTicks_;
        }
        else
        {
            groundControlCommandExpiresAtTick_.reset();
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
    aerosimlab::communication::CameraCommand command,
    aerosimlab::communication::CommandReply& reply)
{
    using Command = aerosimlab::communication::CameraCommand;
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
    aerosimlab::communication::CommandReply& reply)
{
    if (simulation_.DroneCount() >= aerosimlab::communication::maximumFleetDroneCount)
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
    fleetDroneIds_.push_back(id);
    reply.accepted = true;
    reply.status = "drone_added";
}

void AeroSimLabApp::ExpireGroundControlCommand()
{
    if (!groundControlCommandExpiresAtTick_
        || simulation_.TickCount() < *groundControlCommandExpiresAtTick_)
        return;

    const aerosimlab::simulation::DroneCommand stop{
        aerosimlab::simulation::DroneCommand::Kind::Stop};
    static_cast<void>(ApplyAndRecordCommand(stop));
    groundControlCommandExpiresAtTick_.reset();
}

bool AeroSimLabApp::ProcessPendingGroundCommand()
{
#ifdef __EMSCRIPTEN__
    return false;
#else
    if (!pendingCommand_)
        return false;

    const auto& pending = *pendingCommand_;
    aerosimlab::communication::CommandReply reply;
    reply.commandId = pending.packet.commandId;
    reply.sequence = pending.packet.sequence;
    reply.receivedAtMilliseconds = pending.receivedAtMilliseconds;

    const bool wasRecordingAtTickStart = commandRecording_.IsRecording();
    bool replayTickAdvanced = false;
    std::visit(Overloaded{
        [](std::monostate) {},
        [this, &reply](const aerosimlab::simulation::DroneCommand& command)
        {
            ApplyDroneCommand(command, reply);
        },
        [this, &reply](aerosimlab::communication::CameraCommand command)
        {
            ApplyCameraCommand(command, reply);
        },
        [this, &reply, &replayTickAdvanced](
            aerosimlab::communication::SessionCommand command)
        {
            replayTickAdvanced = ApplySessionCommand(command, reply);
        },
        [this, &reply](aerosimlab::communication::VideoRecordingCommand command)
        {
            ApplyVideoRecordingCommand(command, reply);
        },
        [this, &reply](aerosimlab::communication::FleetCommand command)
        {
            if (command == aerosimlab::communication::FleetCommand::AddDrone)
                ApplyAddDroneCommand(reply);
        }
    }, pending.packet.command);

    if (!replayTickAdvanced)
        simulation_.AdvanceOneTick();
    commandRecording_.FinishTick(wasRecordingAtTickStart);

    reply.appliedTick = simulation_.TickCount();
    reply.telemetry = CurrentTelemetry();
    lastReply_ = reply;
    try
    {
        udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), pending.peer);
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

void AeroSimLabApp::ApplyRecordedCommands()
{
    for (const auto& entry : commandRecording_.CommandsForReplayTick())
    {
        (void)simulation_.Apply(entry.command);
    }
}

aerosimlab::communication::Telemetry AeroSimLabApp::CurrentTelemetry() const
{
    aerosimlab::communication::Telemetry telemetry;
    telemetry.tick = simulation_.TickCount();
    telemetry.simulationTimeSeconds = simulation_.SimulationTimeSeconds();
    telemetry.drone = simulation_.State();
    telemetry.camera = simulation_.Camera();
    telemetry.drones.reserve(fleetDroneIds_.size());
    for (const auto id : fleetDroneIds_)
    {
        const auto& drone = simulation_.State(id);
        telemetry.drones.push_back({
            id, "드론 " + std::to_string(id), drone.mode, drone.positionMeters});
    }
    return telemetry;
}

}
