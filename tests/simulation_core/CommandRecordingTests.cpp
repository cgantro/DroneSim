#include "simulation_core/CommandRecording.h"
#include "simulation_core/SimulationCore.h"

#include <gtest/gtest.h>

#include <optional>

using namespace aerosimlab::simulation;

namespace
{
void ReplayMatchesRecordedSimulation(SimulationDynamics dynamics)
{
    const bool usesRigidBody = dynamics == SimulationDynamics::RigidBody;
    const int takeoffTicks = usesRigidBody ? 240 : 100;
    const int movementTicks = usesRigidBody ? 60 : 30;
    const int landingTicks = usesRigidBody ? 360 : 120;

    SimulationCore recordedSimulation{dynamics};
    CommandRecording recording;
    ASSERT_TRUE(recording.StartRecording());

    recordedSimulation.Reset();
    recordedSimulation.AdvanceOneTick();
    recording.FinishTick();

    // 기록과 재생은 모두 명령을 적용한 다음 같은 고정 Tick을 한 번 진행한다.
    const auto advanceRecordedTick = [&](const std::optional<DroneCommand>& command = std::nullopt)
    {
        if (command)
        {
            recording.Record(*command);
            ASSERT_EQ(recordedSimulation.Apply(*command), CommandResult::Accepted);
        }
        recordedSimulation.AdvanceOneTick();
        recording.FinishTick();
    };

    advanceRecordedTick(DroneCommand{DroneCommand::Kind::TakeOff});
    for (int tick = 0; tick < takeoffTicks; ++tick)
        advanceRecordedTick();
    advanceRecordedTick(DroneCommand{
        DroneCommand::Kind::SetVelocity, glm::dvec3{1.0, 0.2, 0.0}});
    for (int tick = 0; tick < movementTicks; ++tick)
        advanceRecordedTick();
    advanceRecordedTick(DroneCommand{DroneCommand::Kind::Land});
    for (int tick = 0; tick < landingTicks; ++tick)
        advanceRecordedTick();

    const bool wasRecordingAtTickStart = recording.IsRecording();
    ASSERT_TRUE(recording.StopRecording());
    recordedSimulation.AdvanceOneTick();
    recording.FinishTick(wasRecordingAtTickStart);

    SimulationCore replayedSimulation{dynamics};
    replayedSimulation.Reset();
    ASSERT_TRUE(recording.StartReplay());
    const auto applyReplayCommands = [&]
    {
        for (const auto& entry : recording.CommandsForReplayTick())
            ASSERT_EQ(replayedSimulation.Apply(entry.command), CommandResult::Accepted);
    };

    applyReplayCommands();
    replayedSimulation.AdvanceOneTick();
    recording.AdvanceReplayTick();
    while (!recording.ReplayFinished())
    {
        applyReplayCommands();
        replayedSimulation.AdvanceOneTick();
        recording.AdvanceReplayTick();
    }
    recording.StopReplay();

    const DroneState& recordedState = recordedSimulation.State();
    const DroneState& replayedState = replayedSimulation.State();
    EXPECT_EQ(replayedSimulation.TickCount(), recordedSimulation.TickCount());
    EXPECT_EQ(replayedState.mode, recordedState.mode);
    EXPECT_EQ(replayedState.positionMeters, recordedState.positionMeters);
    EXPECT_EQ(replayedState.velocityMetersPerSecond, recordedState.velocityMetersPerSecond);
    EXPECT_EQ(replayedState.orientation, recordedState.orientation);
    EXPECT_EQ(replayedState.angularVelocityRadiansPerSecond,
        recordedState.angularVelocityRadiansPerSecond);

    if (usesRigidBody)
        EXPECT_EQ(recordedState.mode, FlightMode::Grounded);
}
}

void Verify_CommandRecording_ReplayMatchesRecordedKinematicSimulation()
{
    ReplayMatchesRecordedSimulation(SimulationDynamics::Kinematic);
}

TEST(CommandRecordingTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_CommandRecording_ReplayMatchesRecordedKinematicSimulation());
}
