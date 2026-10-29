#include "communication/Protocol.h"

#include <gtest/gtest.h>

#include <array>
#include <stdexcept>

namespace aerosimlab::communication
{
void Verify_Protocol_RoundTripsCommandsAndReplyTelemetry()
{
    using Kind = simulation::DroneCommand::Kind;
    const std::array droneCommands{
        simulation::DroneCommand{Kind::TakeOff},
        simulation::DroneCommand{Kind::Land},
        simulation::DroneCommand{Kind::Stop},
        simulation::DroneCommand{Kind::SetVelocity, {1.0, 2.0, 3.0}},
        simulation::DroneCommand{Kind::SetWaypoint, {2.0, -1.0, 1.5}},
        simulation::DroneCommand{Kind::SetYawRate, {}, 0.25}};
    std::uint32_t commandId = 1;
    for (const simulation::DroneCommand& expected : droneCommands)
    {
        CommandPacket packet;
        packet.commandId = commandId;
        packet.sequence = commandId;
        packet.droneId = 42;
        packet.command = expected;
        const auto decoded = DecodeCommand(EncodeCommand(packet));
        const auto* decodedDrone = std::get_if<simulation::DroneCommand>(&decoded.command);
        ASSERT_NE(decodedDrone, nullptr);
        EXPECT_EQ(decoded.commandId, commandId);
        EXPECT_EQ(decoded.sequence, commandId);
        EXPECT_EQ(decoded.droneId, 42U);
        EXPECT_EQ(decodedDrone->kind, expected.kind);
        EXPECT_EQ(decodedDrone->vector, expected.vector);
        EXPECT_DOUBLE_EQ(decodedDrone->yawRateRadiansPerSecond,
            expected.yawRateRadiansPerSecond);
        ++commandId;
    }

    CommandPacket camera;
    camera.commandId = 8;
    camera.sequence = 12;
    camera.command = CameraCommand::InjectFault;
    EXPECT_EQ(std::get<CameraCommand>(DecodeCommand(EncodeCommand(camera)).command),
        CameraCommand::InjectFault);

    CommandPacket fleetStatus;
    fleetStatus.commandId = 11;
    fleetStatus.sequence = 15;
    fleetStatus.command = FleetCommand::GetStatus;
    EXPECT_EQ(std::get<FleetCommand>(DecodeCommand(EncodeCommand(fleetStatus)).command),
        FleetCommand::GetStatus);

    CommandReply reply;
    reply.commandId = 3;
    reply.sequence = 5;
    reply.accepted = true;
    reply.status = "applied";
    reply.receivedAtMilliseconds = 34;
    reply.appliedTick = 9;
    reply.telemetry.tick = 9;
    reply.telemetry.simulationTimeSeconds = 0.15;
    reply.telemetry.primaryDroneId = 1;
    reply.telemetry.drone.mode = simulation::FlightMode::Hovering;
    reply.telemetry.drone.positionMeters = {1.0, 2.0, 3.0};
    reply.telemetry.drone.orientation = {0.9238795325, 0.0, 0.0, 0.3826834324};
    reply.telemetry.drone.angularVelocityRadiansPerSecond = {0.1, -0.2, 0.3};
    reply.telemetry.drone.motorSpeedsRadiansPerSecond = {120.0, 121.0, 119.0, 122.0};
    reply.telemetry.camera.status = simulation::CameraStatus::Fault;
    reply.telemetry.camera.frameCount = 8;
    reply.telemetry.drones = {
        {1, "드론 1", simulation::FlightMode::Hovering, {1.0, 2.0, 3.0}}};
    const std::string encodedReply = EncodeReply(reply);
    EXPECT_LE(encodedReply.size(), maximumDatagramBytes);
    EXPECT_EQ(encodedReply.find("\"mission_target\""), std::string::npos);
    EXPECT_EQ(encodedReply.find("\"mission_status\""), std::string::npos);
    const auto decoded = DecodeReply(encodedReply);
    EXPECT_EQ(decoded.commandId, 3);
    EXPECT_EQ(decoded.appliedTick, 9);
    EXPECT_EQ(decoded.telemetry.drone.mode, simulation::FlightMode::Hovering);
    EXPECT_EQ(decoded.telemetry.drone.positionMeters, glm::dvec3(1.0, 2.0, 3.0));
    EXPECT_EQ(decoded.telemetry.drone.orientation, reply.telemetry.drone.orientation);
    EXPECT_EQ(
        decoded.telemetry.drone.angularVelocityRadiansPerSecond,
        reply.telemetry.drone.angularVelocityRadiansPerSecond);
    EXPECT_EQ(decoded.telemetry.drone.motorSpeedsRadiansPerSecond,
        reply.telemetry.drone.motorSpeedsRadiansPerSecond);
    EXPECT_EQ(decoded.telemetry.camera.status, simulation::CameraStatus::Fault);
    ASSERT_EQ(decoded.telemetry.drones.size(), 1U);
    EXPECT_EQ(decoded.telemetry.drones[0].id, 1U);
    EXPECT_EQ(decoded.telemetry.drones[0].name, "드론 1");
    EXPECT_EQ(decoded.telemetry.primaryDroneId, 1U);

    CommandReply fleetReply = reply;
    fleetReply.status = "fleet_status";
    DroneTelemetry secondDrone{
        2, "드론 2", simulation::FlightMode::Grounded, {6.0, 2.0, 0.24}};
    secondDrone.missionTarget = "구역 2";
    secondDrone.missionStatus = "정찰 중";
    fleetReply.telemetry.drones.push_back(secondDrone);
    const auto encodedFleet = EncodeReply(fleetReply);
    EXPECT_NE(encodedFleet.find("\"mission_target\":\"구역 2\""),
        std::string::npos);
    EXPECT_NE(encodedFleet.find("\"mission_status\":\"정찰 중\""),
        std::string::npos);
    const auto decodedFleet = DecodeReply(encodedFleet);
    ASSERT_EQ(decodedFleet.telemetry.drones.size(), 2U);
    EXPECT_EQ(decodedFleet.telemetry.drones[1].mode, simulation::FlightMode::Grounded);
    EXPECT_EQ(decodedFleet.telemetry.drones[1].positionMeters, glm::dvec3(6.0, 2.0, 0.24));

    CommandReply fullFleet = fleetReply;
    for (std::uint64_t id = 3; id <= maximumFleetDroneCount; ++id)
    {
        fullFleet.telemetry.drones.push_back({id, "드론 " + std::to_string(id),
            simulation::FlightMode::Grounded, {static_cast<double>(id) * 5.0, 0.0, 0.24}});
    }
    const std::string fullFleetDatagram = EncodeReply(fullFleet);
    EXPECT_LE(fullFleetDatagram.size(), maximumDatagramBytes);
    EXPECT_EQ(DecodeReply(fullFleetDatagram).telemetry.drones.size(), maximumFleetDroneCount);
}

void Verify_Protocol_RejectsInvalidPacketsAndSequenceNumbers()
{
    CommandPacket packet;
    packet.commandId = 1;
    packet.sequence = 1;
    packet.command = simulation::DroneCommand{simulation::DroneCommand::Kind::TakeOff};
    const auto valid = EncodeCommand(packet);
    packet.command = std::monostate{};
    EXPECT_THROW(static_cast<void>(EncodeCommand(packet)), std::invalid_argument);
    auto wrongLength = valid;
    const auto lengthPosition = wrongLength.find("\"length\":");
    ASSERT_NE(lengthPosition, std::string::npos);
    wrongLength.replace(lengthPosition + 9, 1, "9");
    EXPECT_THROW(static_cast<void>(DecodeCommand(wrongLength)), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(DecodeCommand("{}")), std::invalid_argument);
    const std::string addDronePayload =
        R"({"action":"add_drone","command_id":1,"sequence":1,"target":"fleet"})";
    const std::string addDroneRequest = "{\"version\":1,\"type\":\"command\",\"length\":"
        + std::to_string(addDronePayload.size()) + ",\"payload\":" + addDronePayload + "}";
    EXPECT_THROW(static_cast<void>(DecodeCommand(addDroneRequest)), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(DecodeCommand(std::string(maximumDatagramBytes + 1, 'x'))), std::invalid_argument);

    CommandReply oversizedFleet;
    oversizedFleet.telemetry.drones.resize(maximumFleetDroneCount + 1U);
    EXPECT_THROW(static_cast<void>(EncodeReply(oversizedFleet)), std::invalid_argument);

    SequenceGuard guard;
    CommandPacket first{10, 4};
    EXPECT_EQ(guard.Inspect(first), SequenceGuard::Decision::New);
    guard.Commit(first);
    EXPECT_EQ(guard.Inspect(first), SequenceGuard::Decision::Duplicate);
    EXPECT_EQ(guard.Inspect(CommandPacket{9, 3}), SequenceGuard::Decision::OutOfOrder);
    EXPECT_EQ(guard.Inspect(CommandPacket{11, 4}), SequenceGuard::Decision::OutOfOrder);
    EXPECT_EQ(guard.Inspect(CommandPacket{11, 5}), SequenceGuard::Decision::New);
}

TEST(ProtocolTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_Protocol_RoundTripsCommandsAndReplyTelemetry());
    EXPECT_NO_FATAL_FAILURE(Verify_Protocol_RejectsInvalidPacketsAndSequenceNumbers());
}

}
