#include "communication/Protocol.h"

#include <gtest/gtest.h>

#include <stdexcept>

namespace aerosimlab::communication
{
TEST(Protocol, RoundTripsDroneAndCameraCommandsWithCheckedLength)
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
}

TEST(Protocol, RejectsWrongVersionTypeLengthAndOversizedMessages)
{
    CommandPacket packet;
    packet.commandId = 1;
    packet.sequence = 1;
    packet.droneCommand = simulation::DroneCommand{simulation::DroneCommand::Kind::TakeOff};
    const auto valid = EncodeCommand(packet);
    auto wrongLength = valid;
    const auto lengthPosition = wrongLength.find("\"length\":");
    ASSERT_NE(lengthPosition, std::string::npos);
    wrongLength.replace(lengthPosition + 9, 1, "9");
    EXPECT_THROW(static_cast<void>(DecodeCommand(wrongLength)), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(DecodeCommand("{}")), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(DecodeCommand(std::string(maximumDatagramBytes + 1, 'x'))), std::invalid_argument);
}

TEST(Protocol, SequenceGuardRecognizesDuplicatesAndRejectsOlderCommands)
{
    SequenceGuard guard;
    CommandPacket first;
    first.commandId = 10;
    first.sequence = 4;
    EXPECT_EQ(guard.Inspect(first), SequenceGuard::Decision::New);
    guard.Commit(first);
    EXPECT_EQ(guard.Inspect(first), SequenceGuard::Decision::Duplicate);

    CommandPacket older{9, 3};
    EXPECT_EQ(guard.Inspect(older), SequenceGuard::Decision::OutOfOrder);
    CommandPacket reusedSequence{11, 4};
    EXPECT_EQ(guard.Inspect(reusedSequence), SequenceGuard::Decision::OutOfOrder);
    CommandPacket next{11, 5};
    EXPECT_EQ(guard.Inspect(next), SequenceGuard::Decision::New);
}

TEST(Protocol, RoundTripsAcknowledgementAndTelemetry)
{
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
    reply.telemetry.camera.status = simulation::CameraStatus::Fault;
    reply.telemetry.camera.frameCount = 8;
    const auto decoded = DecodeReply(EncodeReply(reply));
    EXPECT_EQ(decoded.commandId, 3);
    EXPECT_EQ(decoded.appliedTick, 9);
    EXPECT_EQ(decoded.telemetry.drone.mode, simulation::FlightMode::Hovering);
    EXPECT_EQ(decoded.telemetry.drone.positionMeters, glm::dvec3(1.0, 2.0, 3.0));
    EXPECT_EQ(decoded.telemetry.camera.status, simulation::CameraStatus::Fault);
}
}
