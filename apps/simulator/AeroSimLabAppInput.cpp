#include "AeroSimLabApp.h"
#include "graphics/Window.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/ScenarioRunner.h"

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

#ifndef __EMSCRIPTEN__
#include <GLFW/glfw3.h>
#endif

#include <chrono>
#include <exception>
#include <string>

namespace aerosimlab::simulator
{
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
    try
    {
        const auto packet = aerosimlab::communication::DecodeCommand(datagram);
        lastGroundControlMessage_ = std::chrono::steady_clock::now();
        if (activePeer_ && (activePeer_->addressNetworkOrder != peer.addressNetworkOrder
            || activePeer_->portNetworkOrder != peer.portNetworkOrder) && !pendingCommand_)
        {
            activePeer_ = peer;
            sequenceGuard_ = {};
            lastReply_.reset();
        }
        else if (!activePeer_)
        {
            activePeer_ = peer;
        }

        const auto decision = sequenceGuard_.Inspect(packet);
        if (decision == aerosimlab::communication::SequenceGuard::Decision::Duplicate)
        {
            if (lastReply_)
                udpSocket_->Send(aerosimlab::communication::EncodeReply(*lastReply_), peer);
            return;
        }
        if (decision == aerosimlab::communication::SequenceGuard::Decision::OutOfOrder)
        {
            aerosimlab::communication::CommandReply reply;
            reply.commandId = packet.commandId;
            reply.sequence = packet.sequence;
            reply.status = "rejected_sequence";
            reply.telemetry = CurrentTelemetry();
            udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
            return;
        }
        if (pendingCommand_)
        {
            aerosimlab::communication::CommandReply reply;
            reply.commandId = packet.commandId;
            reply.sequence = packet.sequence;
            reply.status = "busy";
            reply.telemetry = CurrentTelemetry();
            udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
            return;
        }

        sequenceGuard_.Commit(packet);
        if (packet.fleetCommand == aerosimlab::communication::FleetCommand::GetStatus)
        {
            aerosimlab::communication::CommandReply reply;
            reply.commandId = packet.commandId;
            reply.sequence = packet.sequence;
            reply.accepted = true;
            reply.status = "fleet_status";
            reply.appliedTick = simulation_.TickCount();
            reply.telemetry = CurrentTelemetry();
            lastReply_ = reply;
            udpSocket_->Send(aerosimlab::communication::EncodeReply(reply), peer);
            return;
        }

        externalControl_ = true;
        if (!packet.sessionCommand && !packet.fleetCommand)
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
    catch (const std::exception& error)
    {
        udpSocket_->Send(aerosimlab::communication::EncodeError(error.what()), peer);
    }
}
#endif

void AeroSimLabApp::PollGamepad()
{
#ifdef __EMSCRIPTEN__
    return;
#else
    GLFWgamepadstate state{};
    int connectedJoystickId = -1;
    int unsupportedJoystickId = -1;
    for (int joystickId = GLFW_JOYSTICK_1; joystickId <= GLFW_JOYSTICK_LAST; ++joystickId)
    {
        if (glfwJoystickPresent(joystickId) != GLFW_TRUE)
            continue;

        if (glfwJoystickIsGamepad(joystickId) == GLFW_TRUE
            && glfwGetGamepadState(joystickId, &state) == GLFW_TRUE)
        {
            connectedJoystickId = joystickId;
            break;
        }

        if (unsupportedJoystickId < 0)
            unsupportedJoystickId = joystickId;
    }

    if (connectedJoystickId < 0)
    {
        HandleGamepadDisconnected();
        gamepadJoystickId_ = -1;

        if (unsupportedJoystickId >= 0
            && unsupportedJoystickId_ != unsupportedJoystickId)
        {
            const char* deviceName = glfwGetJoystickName(unsupportedJoystickId);
            const std::string name = deviceName != nullptr ? deviceName : "이름 없음";
            window_->SetTitle("AeroSimLab | 게임패드 형식 미지원: " + name);
            unsupportedJoystickId_ = unsupportedJoystickId;
        }
        else if (unsupportedJoystickId < 0 && unsupportedJoystickId_ >= 0)
        {
            window_->SetTitle(groundControlActive_
                ? "AeroSimLab | 지상통제 입력 사용 중"
                : "AeroSimLab | 드론 관찰 · EO 영상");
            unsupportedJoystickId_ = -1;
        }
        return;
    }

    unsupportedJoystickId_ = -1;

    if (gamepadJoystickId_ != connectedJoystickId)
    {
        previousTakeoffButton_ = false;
        previousLandButton_ = false;
        pendingPilotCommand_.reset();
        gamepadJoystickId_ = connectedJoystickId;
    }

    const bool takeoffPressed = state.buttons[GLFW_GAMEPAD_BUTTON_A] == GLFW_PRESS;
    const bool landPressed = state.buttons[GLFW_GAMEPAD_BUTTON_B] == GLFW_PRESS;
    std::optional<aerosimlab::manual_control::GamepadAxes> pilotAxes;
    if (!groundControlActive_)
    {
        const auto mode = simulation_.State().mode;
        const bool airborne = mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying;
        if (airborne)
        {
            // GLFW 트리거 값은 -1부터 시작하므로 0~1로 바꾼 뒤 양쪽 입력 차이를 높이 명령으로 쓴다.
            const auto triggerValue = [](float value)
            {
                return (static_cast<double>(value) + 1.0) * 0.5;
            };
            pilotAxes = aerosimlab::manual_control::GamepadAxes{
                .forward = -static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_Y]),
                .right = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_LEFT_X]),
                .vertical = triggerValue(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER])
                    - triggerValue(state.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]),
                .yaw = static_cast<double>(state.axes[GLFW_GAMEPAD_AXIS_RIGHT_X])
            };
        }
    }
    HandleGamepadConnected(takeoffPressed, landPressed, pilotAxes);
#endif
}

void AeroSimLabApp::HandleGamepadDisconnected()
{
    using DisconnectAction = aerosimlab::manual_control::GamepadDisconnectAction;
    const auto action = aerosimlab::manual_control::ResolveGamepadDisconnectAction(
        gamepadConnected_, groundControlActive_ || commandRecording_.IsReplaying(), simulation_.State().mode);
    if (action == DisconnectAction::NoChange)
        return;

    pendingControllerAction_.reset();
    window_->SetTitle(groundControlActive_
        ? "AeroSimLab | 지상통제 입력 사용 중"
        : "AeroSimLab | 드론 관찰 · EO 영상");
    switch (action)
    {
    case DisconnectAction::Land:
        pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
            aerosimlab::simulation::DroneCommand::Kind::Land};
        break;
    case DisconnectAction::Stop:
        pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
            aerosimlab::simulation::DroneCommand::Kind::Stop};
        break;
    case DisconnectAction::NoChange:
    case DisconnectAction::ClearPendingAction:
        break;
    }
    if (!groundControlActive_)
        pendingPilotCommand_.reset();
    gamepadConnected_ = false;
    previousTakeoffButton_ = false;
    previousLandButton_ = false;
}

void AeroSimLabApp::HandleGamepadConnected(bool takeoffPressed, bool landPressed,
    const std::optional<aerosimlab::manual_control::GamepadAxes>& pilotAxes)
{
    if (!gamepadConnected_ && !groundControlActive_)
        window_->SetTitle("AeroSimLab | 게임패드 연결됨");
    gamepadConnected_ = true;
    externalControl_ = true;

    if (aerosimlab::manual_control::CanGamepadControlSimulation(
            groundControlActive_, commandRecording_.IsReplaying()))
    {
        if (takeoffPressed && !previousTakeoffButton_
            && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded)
        {
            pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::TakeOff};
        }
        if (landPressed && !previousLandButton_
            && simulation_.State().mode != aerosimlab::simulation::FlightMode::Grounded)
        {
            pendingControllerAction_ = aerosimlab::simulation::DroneCommand{
                aerosimlab::simulation::DroneCommand::Kind::Land};
        }

        if (pilotAxes)
            pendingPilotCommand_ = aerosimlab::manual_control::MapAxes(
                *pilotAxes, simulation_.State().yawRadians);
    }
    else if (commandRecording_.IsReplaying())
    {
        pendingControllerAction_.reset();
        pendingPilotCommand_.reset();
    }

    previousTakeoffButton_ = takeoffPressed;
    previousLandButton_ = landPressed;
}

bool AeroSimLabApp::ApplySessionCommand(
    aerosimlab::communication::SessionCommand command,
    aerosimlab::communication::CommandReply& reply)
{
    using Command = aerosimlab::communication::SessionCommand;
    using FlightMode = aerosimlab::simulation::FlightMode;

    switch (command)
    {
    case Command::StartRecording:
        if (commandRecording_.IsRecording() || commandRecording_.IsReplaying())
        {
            reply.status = "recording_unavailable";
            return false;
        }
        if (simulation_.State().mode != FlightMode::Grounded)
        {
            reply.status = "recording_requires_grounded";
            return false;
        }

        simulation_.Reset();
        eoFrame_ = {};
        groundControlCommandExpiresAtTick_.reset();
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
        if (commandRecording_.IsRecording() || commandRecording_.IsReplaying())
        {
            reply.status = "recording_unavailable";
            return false;
        }
        if (!commandRecording_.StartReplay())
        {
            reply.status = "recording_empty";
            return false;
        }

        simulation_.Reset();
        eoFrame_ = {};
        groundControlCommandExpiresAtTick_.reset();
        smokeReplayStarted_ = smokeTest_;
        pendingControllerAction_.reset();
        pendingPilotCommand_.reset();
        window_->SetTitle("AeroSimLab | 입력 기록 재생 중");
        ApplyRecordedCommands();
        simulation_.AdvanceOneTick();
        commandRecording_.AdvanceReplayTick();
        reply.accepted = true;
        reply.status = "recording_replay_started";
        return true;
    }

    return false;
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

    const auto result = simulation_.Apply(command);
    reply.accepted = result == aerosimlab::simulation::CommandResult::Accepted;
    if (reply.accepted)
    {
        RecordCommand(command);
        const bool velocityCommand = command.kind
            == aerosimlab::simulation::DroneCommand::Kind::SetVelocity;
        const bool yawRateCommand = command.kind
            == aerosimlab::simulation::DroneCommand::Kind::SetYawRate;
        const bool hasNonzeroInput = velocityCommand
            ? glm::length(command.vector) > 0.0
            : command.yawRateRadiansPerSecond != 0.0;
        if ((velocityCommand || yawRateCommand) && hasNonzeroInput)
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

    const std::size_t slot = simulation_.DroneCount() - 1U;
    const double eastMeters = static_cast<double>(slot % 3U + 1U) * 5.0;
    const double northMeters = static_cast<double>(slot / 3U) * 5.0;
    const glm::dvec3 primaryPosition = simulation_.State().positionMeters;
    const glm::dvec2 spawnPosition{
        primaryPosition.x + eastMeters,
        primaryPosition.y + northMeters};
    const double groundHeightMeters = terrainGrid_.HeightAtLocalMeters(spawnPosition);
    const glm::dvec3 positionMeters{
        spawnPosition.x,
        spawnPosition.y,
        groundHeightMeters
            + aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters};
    const auto id = simulation_.AddDrone(positionMeters, 0.0, groundHeightMeters);
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
    if (simulation_.Apply(stop) == aerosimlab::simulation::CommandResult::Accepted)
        RecordCommand(stop);
    groundControlCommandExpiresAtTick_.reset();
}

void AeroSimLabApp::ApplyGroundCommand()
{
#ifdef __EMSCRIPTEN__
    return;
#else
    if (!pendingCommand_)
        return;

    const auto& pending = *pendingCommand_;
    aerosimlab::communication::CommandReply reply;
    reply.commandId = pending.packet.commandId;
    reply.sequence = pending.packet.sequence;
    reply.receivedAtMilliseconds = pending.receivedAtMilliseconds;

    const bool wasRecordingAtTickStart = commandRecording_.IsRecording();
    bool replayTickAdvanced = false;
    if (pending.packet.sessionCommand)
        replayTickAdvanced = ApplySessionCommand(*pending.packet.sessionCommand, reply);
    else if (pending.packet.droneCommand)
        ApplyDroneCommand(*pending.packet.droneCommand, reply);
    else if (pending.packet.cameraCommand)
        ApplyCameraCommand(*pending.packet.cameraCommand, reply);
    else if (pending.packet.fleetCommand
        == aerosimlab::communication::FleetCommand::AddDrone)
        ApplyAddDroneCommand(reply);

    if (!replayTickAdvanced)
        simulation_.AdvanceOneTick();
    commandRecording_.FinishTick(wasRecordingAtTickStart);

    if (commandRecording_.IsReplaying() && commandRecording_.ReplayFinished())
    {
        commandRecording_.StopReplay();
        window_->SetTitle("AeroSimLab | 기록 재생 완료");
    }
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
#endif
}

void AeroSimLabApp::ApplyRecordedCommands()
{
    for (const auto& entry : commandRecording_.CommandsForReplayTick())
    {
        (void)simulation_.Apply(entry.command);
    }
}

void AeroSimLabApp::RecordCommand(const aerosimlab::simulation::DroneCommand& command)
{
    commandRecording_.Record(command);
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
