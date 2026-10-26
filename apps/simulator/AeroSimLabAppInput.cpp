#include "AeroSimLabApp.h"

#include "graphics/MultisampleFramebuffer.h"
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

#ifndef __EMSCRIPTEN__
#include "media/RtpVideoStream.h"
#include "media/RecordedCaptureTransfer.h"
#endif

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
        StartEoVideoStream(peer);
        if (!captureTransferPending_.load()
            && std::chrono::steady_clock::now() >= nextCaptureTransferAttempt_)
        {
            if (captureTransferWorker_.joinable())
                captureTransferWorker_.join();
            const std::string destination = peer.AddressIpv4();
            captureTransferPending_ = true;
            nextCaptureTransferAttempt_ = std::chrono::steady_clock::now()
                + std::chrono::seconds(2);
            captureTransferWorker_ = std::jthread([this, destination]
            {
                try
                {
                    for (const auto& manifestPath : EoVideoRecorder::FindSavedCaptures())
                    {
                        const std::string manifestName = manifestPath.filename().string();
                        constexpr std::string_view manifestSuffix = ".manifest.json";
                        const std::string baseName = manifestName.substr(
                            0, manifestName.size() - manifestSuffix.size());
                        const aerosimlab::media::RecordedCaptureFiles files{
                            .video = manifestPath.parent_path() / (baseName + ".mp4"),
                            .metadata = manifestPath.parent_path() / (baseName + ".jsonl"),
                            .manifest = manifestPath};
                        std::string transferError;
                        if (!aerosimlab::media::SendRecordedCapture(destination,
                                aerosimlab::media::defaultCaptureTransferPort,
                                files, transferError))
                            break;
                        static_cast<void>(EoVideoRecorder::MarkCaptureDelivered(
                            manifestPath, transferError));
                    }
                }
                catch (...)
                {
                    // 전송을 못한 자료는 SAVED 상태로 남아 다음 연결 확인 때 다시 시도한다.
                }
                captureTransferPending_ = false;
            });
        }
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

void AeroSimLabApp::StartEoVideoStream(
    const aerosimlab::communication::UdpPeer& peer)
{
    const std::string destination = peer.AddressIpv4();
    if (eoVideoStartPending_.load())
        return;
    if (eoVideoSender_ && eoVideoSender_->IsRunning()
        && eoVideoDestinationAddress_ == destination)
        return;

    if (eoVideoStartWorker_.joinable())
        eoVideoStartWorker_.join();
    if (!eoVideoStartError_.empty() && eoVideoAttemptedAddress_ == destination)
    {
        if (!eoVideoStartErrorReported_)
        {
            window_->SetTitle("DroneSim | EO 영상 전송을 시작하지 못했습니다: "
                + eoVideoStartError_);
            eoVideoStartErrorReported_ = true;
        }
        return;
    }

    if (!eoVideoSender_)
        eoVideoSender_ = std::make_unique<aerosimlab::media::RtpVideoSender>();
    const int width = cameraTarget_->GetWidth();
    const int height = cameraTarget_->GetHeight();
    eoVideoStartPending_ = true;
    eoVideoAttemptedAddress_ = destination;
    eoVideoStartError_.clear();
    eoVideoStartErrorReported_ = false;
    eoVideoStartWorker_ = std::jthread([this, destination, width, height]
    {
        eoVideoSender_->Stop();
        std::string error;
        if (eoVideoSender_->Start(width, height, 30, destination,
            aerosimlab::media::EoVideoStreamPorts::receiverRtp,
            aerosimlab::media::EoVideoStreamPorts::receiverRtcp,
            aerosimlab::media::EoVideoStreamPorts::senderRtcp, error))
        {
            eoVideoDestinationAddress_ = destination;
        }
        else
        {
            eoVideoStartError_ = std::move(error);
        }
        eoVideoStartPending_ = false;
    });
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
        groundControlCommandExpiresAtTick_.clear();
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
            60.0, inspectionMission_.facilityId, inspectionMission_.zoneId, error))
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
    aerosimlab::simulation::DroneId droneId,
    const aerosimlab::simulation::DroneCommand& command,
    aerosimlab::communication::CommandReply& reply)
{
    if (commandRecording_.IsReplaying())
    {
        commandRecording_.StopReplay();
        window_->SetTitle("AeroSimLab | 지상통제 입력 사용 중");
    }

    if (droneId != simulation_.PrimaryDroneId() && commandRecording_.IsRecording())
    {
        reply.status = "recording_secondary_unsupported";
        return;
    }

    const auto result = ApplyAndRecordCommand(droneId, command);
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
        static_cast<void>(ApplyAndRecordCommand(command->first, stop));
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
            const auto droneId = pendingCommand_->packet.droneId
                .value_or(simulation_.PrimaryDroneId());
            ApplyDroneCommand(droneId, command, reply);
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
        [this, &reply](aerosimlab::communication::InspectionCommand command)
        {
            if (command == aerosimlab::communication::InspectionCommand::Start)
            {
                reply.accepted = RequestInspectionMission();
                reply.status = reply.accepted
                    ? "inspection_mission_started" : InspectionMissionStatus();
                return;
            }

            if (command == aerosimlab::communication::InspectionCommand::Pause)
            {
                reply.accepted = PauseInspectionMission();
                reply.status = reply.accepted
                    ? "inspection_mission_paused" : "inspection_mission_cannot_pause";
                return;
            }

            if (command == aerosimlab::communication::InspectionCommand::Resume)
            {
                reply.accepted = ResumeInspectionMission();
                reply.status = reply.accepted
                    ? "inspection_mission_resumed" : "inspection_mission_not_paused";
                return;
            }

            reply.accepted = CancelInspectionMission();
            reply.status = reply.accepted
                ? "inspection_mission_cancelled" : "no_active_inspection_mission";
        },
        [this, &reply](aerosimlab::communication::FleetCommand command)
        {
            if (command == aerosimlab::communication::FleetCommand::AddDrone)
                ApplyAddDroneCommand(reply);
            else if (command == aerosimlab::communication::FleetCommand::DispatchReconnaissance)
            {
                reply.accepted = StartFleetReconnaissance();
                reply.status = reply.accepted
                    ? "fleet_reconnaissance_started" : "fleet_reconnaissance_rejected";
            }
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
    telemetry.primaryDroneId = simulation_.PrimaryDroneId();
    telemetry.drone = simulation_.State();
    telemetry.camera = simulation_.Camera();
    telemetry.inspectionMissionStatus = InspectionMissionStatus();
    telemetry.drones.reserve(fleetDroneIds_.size());
    for (const auto id : fleetDroneIds_)
    {
        const auto& drone = simulation_.State(id);
        const auto mission = fleetReconnaissance_.find(id);
        telemetry.drones.push_back({
            id, "드론 " + std::to_string(id), drone.mode, drone.positionMeters,
            mission == fleetReconnaissance_.end() ? std::string{}
                : inspectionTargets_[mission->second.targetIndex].displayName,
            mission == fleetReconnaissance_.end() ? std::string{} : mission->second.status});
    }
    return telemetry;
}

}
