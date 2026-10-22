#include "communication/Protocol.h"

#include <gtest/gtest.h>

#include <stdexcept>

namespace aerosimlab::communication
{
void Verify_Protocol_RoundTripsCommandsAndReplyTelemetry()
{
    CommandPacket drone;
    drone.commandId = 7;
    drone.sequence = 11;
    drone.droneCommand = simulation::DroneCommand{
        simulation::DroneCommand::Kind::SetWaypoint, glm::dvec3{2.0, -1.0, 1.5}};
    const auto decodedDrone = DecodeCommand(EncodeCommand(drone));
    ASSERT_TRUE(decodedDrone.droneCommand);
    EXPECT_EQ(decodedDrone.commandId, 7);
    EXPECT_EQ(decodedDrone.sequence, 11);
    EXPECT_EQ(decodedDrone.droneCommand->kind, simulation::DroneCommand::Kind::SetWaypoint);
    EXPECT_EQ(decodedDrone.droneCommand->vector, glm::dvec3(2.0, -1.0, 1.5));

    CommandPacket camera;
    camera.commandId = 8;
    camera.sequence = 12;
    camera.cameraCommand = CameraCommand::InjectFault;
    EXPECT_EQ(DecodeCommand(EncodeCommand(camera)).cameraCommand, CameraCommand::InjectFault);

    CommandPacket recording;
    recording.commandId = 9;
    recording.sequence = 13;
    recording.sessionCommand = SessionCommand::ReplayRecording;
    EXPECT_EQ(DecodeCommand(EncodeCommand(recording)).sessionCommand,
        SessionCommand::ReplayRecording);

    CommandPacket addDrone;
    addDrone.commandId = 10;
    addDrone.sequence = 14;
    addDrone.fleetCommand = FleetCommand::AddDrone;
    EXPECT_EQ(DecodeCommand(EncodeCommand(addDrone)).fleetCommand, FleetCommand::AddDrone);
    CommandPacket fleetStatus;
    fleetStatus.commandId = 11;
    fleetStatus.sequence = 15;
    fleetStatus.fleetCommand = FleetCommand::GetStatus;
    EXPECT_EQ(DecodeCommand(EncodeCommand(fleetStatus)).fleetCommand, FleetCommand::GetStatus);

    CommandReply reply;
    reply.commandId = 3;
    reply.sequence = 5;
    reply.accepted = true;
    reply.status = "applied";
    reply.receivedAtMilliseconds = 34;
    reply.appliedTick = 9;
    reply.telemetry.tick = 9;
    reply.telemetry.simulationTimeSeconds = 0.15;
    reply.telemetry.drone.mode = simulation::FlightMode::Hovering;
    reply.telemetry.drone.positionMeters = {1.0, 2.0, 3.0};
    reply.telemetry.drone.orientation = {0.9238795325, 0.0, 0.0, 0.3826834324};
    reply.telemetry.drone.angularVelocityRadiansPerSecond = {0.1, -0.2, 0.3};
    reply.telemetry.drone.motorSpeedsRadiansPerSecond = {120.0, 121.0, 119.0, 122.0};
    reply.telemetry.camera.status = simulation::CameraStatus::Fault;
    reply.telemetry.camera.frameCount = 8;
    reply.telemetry.drones = {
        {1, "드론 1", simulation::FlightMode::Hovering, {1.0, 2.0, 3.0}},
        {2, "드론 2", simulation::FlightMode::Grounded, {6.0, 2.0, 0.24}}};
    const std::string encodedReply = EncodeReply(reply);
    EXPECT_LE(encodedReply.size(), maximumDatagramBytes);
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
    ASSERT_EQ(decoded.telemetry.drones.size(), 2U);
    EXPECT_EQ(decoded.telemetry.drones[0].id, 1U);
    EXPECT_EQ(decoded.telemetry.drones[0].name, "드론 1");
    EXPECT_EQ(decoded.telemetry.drones[1].mode, simulation::FlightMode::Grounded);
    EXPECT_EQ(decoded.telemetry.drones[1].positionMeters, glm::dvec3(6.0, 2.0, 0.24));

    CommandReply fullFleet = reply;
    fullFleet.status = "fleet_status";
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
    packet.droneCommand = simulation::DroneCommand{simulation::DroneCommand::Kind::TakeOff};
    const auto valid = EncodeCommand(packet);
    packet.cameraCommand = CameraCommand::Enable;
    EXPECT_THROW(static_cast<void>(EncodeCommand(packet)), std::invalid_argument);
    packet.cameraCommand.reset();
    auto wrongLength = valid;
    const auto lengthPosition = wrongLength.find("\"length\":");
    ASSERT_NE(lengthPosition, std::string::npos);
    wrongLength.replace(lengthPosition + 9, 1, "9");
    EXPECT_THROW(static_cast<void>(DecodeCommand(wrongLength)), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(DecodeCommand("{}")), std::invalid_argument);
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
