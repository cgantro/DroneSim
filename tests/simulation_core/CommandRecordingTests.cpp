#include "simulation_core/CommandRecording.h"

#include <gtest/gtest.h>

using namespace aerosimlab::simulation;

TEST(CommandRecording, ReplaysEveryTickIncludingStartAndStopTicks)
{
    CommandRecording recording;
    ASSERT_TRUE(recording.StartRecording());

    recording.FinishTick();
    const DroneCommand takeoff{DroneCommand::Kind::TakeOff};
    recording.Record(takeoff);
    recording.FinishTick();

    ASSERT_TRUE(recording.StopRecording());
    recording.FinishTick(true);
    ASSERT_EQ(recording.TickCount(), 3);
    ASSERT_TRUE(recording.StartReplay());

    EXPECT_TRUE(recording.CommandsForReplayTick().empty());
    recording.AdvanceReplayTick();

    const auto commandTick = recording.CommandsForReplayTick();
    ASSERT_EQ(commandTick.size(), 1);
    EXPECT_EQ(commandTick.front().command.kind, DroneCommand::Kind::TakeOff);
    recording.AdvanceReplayTick();

    EXPECT_TRUE(recording.CommandsForReplayTick().empty());
    recording.AdvanceReplayTick();
    EXPECT_TRUE(recording.ReplayFinished());
}

TEST(CommandRecording, DoesNotStartReplayWithoutRecordedCommands)
{
    CommandRecording recording;
    EXPECT_FALSE(recording.StartReplay());

    ASSERT_TRUE(recording.StartRecording());
    recording.FinishTick();
    ASSERT_TRUE(recording.StopRecording());
    recording.FinishTick(true);
    EXPECT_FALSE(recording.StartReplay());
}
