#include "application/PickPlaceMission.h"
#include "application/PickPlaceConfig.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <optional>
#include <utility>

namespace
{
using namespace grasplink::robotics;

class RecordingRobotController final : public IRobotController
{
public:
    Result Connect() override { return {}; }
    void Disconnect() noexcept override {}
    bool IsConnected() const noexcept override { return true; }
    Result MoveJoint(const JointMoveCommand&) override { return {}; }
    Result MovePose(const CartesianPose&, double, double) override { return {}; }
    Result MoveLinear(const LinearMoveCommand&) override { return {}; }
    Result MoveLinearPath(const LinearPathMoveCommand& command) override
    {
        ++linearPathRequests;
        requestedLinearPath = command;
        return {};
    }
    Result Stop() override { return {}; }
    RobotState GetState() const override { return state; }
    void Update(double) override {}

    Result BeginPosePlanningWithLinearContinuation(const CartesianPose& approach,
        const LinearPathMoveCommand& continuation, double, double) override
    {
        ++planningRequests;
        requestedApproach = approach;
        requestedContinuation = continuation;
        planning = true;
        return {};
    }

    bool IsMotionPlanning() const noexcept override { return planning; }
    std::optional<Result> TakeMotionPlanningResult() override
    {
        auto result = planningResult;
        planningResult.reset();
        return result;
    }

    void CompletePlanning(Result result)
    {
        planning = false;
        planningResult = std::move(result);
    }

    RobotState state{};
    std::size_t planningRequests = 0;
    std::size_t linearPathRequests = 0;
    bool planning = false;
    std::optional<Result> planningResult;
    CartesianPose requestedApproach{};
    LinearPathMoveCommand requestedContinuation{};
    LinearPathMoveCommand requestedLinearPath{};
};

class RecordingGripperController final : public IGripperController
{
public:
    Result Connect() override { return {}; }
    void Disconnect() noexcept override {}
    bool IsConnected() const noexcept override { return true; }
    Result Activate() override { return {}; }
    Result Reset() override { return {}; }
    Result Command(const GripperCommand& command) override
    {
        lastCommand = command;
        return {};
    }
    Result Stop() override { return {}; }
    GripperState GetState() const override
    {
        GripperState state;
        state.mode = GripperMode::Idle;
        state.objectStatus = GripperObjectStatus::AtRequestedPosition;
        return state;
    }
    void Update(double) override {}

    GripperCommand lastCommand{};
};
}

TEST(PickPlaceMission, PlanningPauseAndResumeFollowControllerContract)
{
    using namespace grasplink;
    application::PickPlaceMission mission;
    RecordingRobotController controller;
    RecordingGripperController gripper;
    controller.state.mode = robotics::RobotMode::Idle;
    controller.state.valid = true;
    controller.state.tcpPoseValid = true;

    const robotics::CartesianPose graspPose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    const robotics::CartesianPose placementPose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    mission.Update(controller.state, controller, gripper, false, graspPose, placementPose);
    application::PickPlaceMissionActions actions;
    actions.start = true;
    mission.ApplyActions(actions, controller.state, controller, false, graspPose);

    EXPECT_EQ(controller.planningRequests, 1U);
    ASSERT_EQ(controller.requestedContinuation.targetPoses.size(), 1U);
    EXPECT_GT(controller.requestedApproach.positionMeters[1],
        controller.requestedContinuation.targetPoses.front().positionMeters[1]);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Planning motion");

    controller.CompletePlanning({});
    mission.Update(controller.state, controller, gripper, false, graspPose, placementPose);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the box");

    actions.stop = true;
    mission.ApplyActions(actions, controller.state, controller, false, graspPose);
    ASSERT_TRUE(mission.Snapshot().paused);

    actions = {};
    for (int frame = 0; frame < 2; ++frame)
        mission.Update(controller.state, controller, gripper, false, graspPose, placementPose);

    EXPECT_EQ(controller.planningRequests, 1U);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the box");

    actions.resume = true;
    mission.ApplyActions(actions, controller.state, controller, false, graspPose);

    EXPECT_FALSE(mission.Snapshot().paused);
    EXPECT_EQ(controller.planningRequests, 2U);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Planning motion");

    controller.CompletePlanning({robotics::ErrorCode::Unreachable, "test planning failure"});
    mission.Update(controller.state, controller, gripper, false, graspPose, placementPose);

    EXPECT_EQ(mission.Snapshot().stageLabel, "Failed");
    EXPECT_EQ(mission.Snapshot().lastMessage, "test planning failure");

    application::PickPlaceMission attachedBoxMission;
    RecordingRobotController attachedBoxController;
    RecordingGripperController attachedBoxGripper;
    attachedBoxController.state.mode = robotics::RobotMode::Idle;
    attachedBoxController.state.valid = true;
    attachedBoxController.state.tcpPoseValid = true;

    attachedBoxMission.Update(attachedBoxController.state, attachedBoxController,
        attachedBoxGripper, false, graspPose, placementPose);
    actions = {};
    actions.start = true;
    attachedBoxMission.ApplyActions(actions, attachedBoxController.state,
        attachedBoxController, false, graspPose);
    attachedBoxController.CompletePlanning({});
    attachedBoxMission.Update(attachedBoxController.state, attachedBoxController,
        attachedBoxGripper, false, graspPose, placementPose);
    attachedBoxMission.Update(attachedBoxController.state, attachedBoxController,
        attachedBoxGripper, false, graspPose, placementPose);
    attachedBoxMission.Update(attachedBoxController.state, attachedBoxController,
        attachedBoxGripper, false, graspPose, placementPose);
    attachedBoxMission.Update(attachedBoxController.state, attachedBoxController,
        attachedBoxGripper, true, graspPose, placementPose);
    EXPECT_EQ(attachedBoxMission.Snapshot().stageLabel, "Lifting the box");
    EXPECT_EQ(attachedBoxGripper.lastCommand.positionRequest,
        static_cast<std::uint8_t>(application::pick_place::config::GripperCommandSettings::PositionRequest::Closed));
    const auto linearPathRequestsBeforeResume = attachedBoxController.linearPathRequests;

    actions = {};
    actions.stop = true;
    attachedBoxMission.ApplyActions(actions, attachedBoxController.state,
        attachedBoxController, true, graspPose);
    actions = {};
    actions.resume = true;
    attachedBoxMission.ApplyActions(actions, attachedBoxController.state,
        attachedBoxController, true, graspPose);

    EXPECT_FALSE(attachedBoxMission.Snapshot().paused);
    EXPECT_EQ(attachedBoxMission.Snapshot().stageLabel, "Lifting to resume height");
    EXPECT_EQ(attachedBoxController.linearPathRequests, linearPathRequestsBeforeResume + 1U);
    ASSERT_EQ(attachedBoxController.requestedLinearPath.targetPoses.size(), 1U);
}

TEST(PickPlaceMission, CompleteStageReportsSuccessAndRaisesOneEvent)
{
    using namespace grasplink;
    application::PickPlaceMission mission;
    RecordingRobotController controller;
    RecordingGripperController gripper;
    controller.state.mode = robotics::RobotMode::Idle;
    controller.state.valid = true;
    controller.state.tcpPoseValid = true;

    const robotics::CartesianPose boxPose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    auto now = std::chrono::steady_clock::time_point{};
    const auto update = [&](bool boxGrasped)
    {
        mission.Update(controller.state, controller, gripper, boxGrasped, boxPose, boxPose, now);
    };
    update(false);
    application::PickPlaceMissionActions actions;
    actions.start = true;
    mission.ApplyActions(actions, controller.state, controller, false, boxPose);

    controller.CompletePlanning({});
    update(false);
    update(false);
    update(false);
    update(true);
    update(true);
    update(true);
    update(true);
    controller.CompletePlanning({});
    update(true);
    update(true);
    update(true);

    now += application::pick_place::config::PlacementSettings::settleDuration;
    update(true);
    update(false);
    update(false);

    const auto completed = mission.Snapshot();
    EXPECT_EQ(completed.stageLabel, "Complete");
    EXPECT_TRUE(completed.missionSucceeded);
    EXPECT_EQ(completed.completedCount, 1U);
    EXPECT_TRUE(mission.ConsumeSuccessEvent());
    EXPECT_FALSE(mission.ConsumeSuccessEvent());
}
