#include "application/PickPlaceMission.h"
#include "application/PickPlaceConfig.h"
#include "application/PickPlaceScenarioSampler.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
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

struct PickPlaceMissionHarness
{
    grasplink::application::PickPlaceMission mission;
    RecordingRobotController robot;
    RecordingGripperController gripper;
    CartesianPose graspPose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    CartesianPose placementPose{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    std::chrono::steady_clock::time_point now{};

    PickPlaceMissionHarness()
    {
        robot.state.mode = RobotMode::Idle;
        robot.state.valid = true;
        robot.state.tcpPoseValid = true;
    }

    void Update(bool boxGrasped = false)
    {
        mission.Update(robot.state, robot, gripper, boxGrasped,
            graspPose, placementPose, now);
    }

    void Apply(const grasplink::application::PickPlaceMissionActions& actions,
        bool boxGrasped = false)
    {
        mission.ApplyActions(actions, robot.state, robot, boxGrasped, graspPose);
    }

    void Start()
    {
        Update();
        grasplink::application::PickPlaceMissionActions actions;
        actions.start = true;
        Apply(actions);
    }

    void CompletePlanningAndUpdate(grasplink::robotics::Result result = {})
    {
        robot.CompletePlanning(std::move(result));
        Update();
    }
};
}

TEST(PickPlaceMission, PlanningPauseAndResumeFollowControllerContract)
{
    using namespace grasplink;
    PickPlaceMissionHarness harness;
    auto& mission = harness.mission;
    auto& controller = harness.robot;
    auto& gripper = harness.gripper;
    application::PickPlaceMissionActions actions;
    harness.Start();

    EXPECT_EQ(controller.planningRequests, 1U);
    ASSERT_EQ(controller.requestedContinuation.targetPoses.size(), 1U);
    EXPECT_GT(controller.requestedApproach.positionMeters[1],
        controller.requestedContinuation.targetPoses.front().positionMeters[1]);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Planning motion");

    harness.CompletePlanningAndUpdate();
    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the box");

    actions.stop = true;
    harness.Apply(actions);
    ASSERT_TRUE(mission.Snapshot().paused);

    actions = {};
    for (int frame = 0; frame < 2; ++frame)
        harness.Update();

    EXPECT_EQ(controller.planningRequests, 1U);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the box");

    actions.resume = true;
    harness.Apply(actions);

    EXPECT_FALSE(mission.Snapshot().paused);
    EXPECT_EQ(controller.planningRequests, 2U);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Planning motion");

    harness.CompletePlanningAndUpdate(
        {robotics::ErrorCode::Unreachable, "test planning failure"});

    EXPECT_EQ(mission.Snapshot().stageLabel, "Failed");
    EXPECT_EQ(mission.Snapshot().lastMessage, "test planning failure");

    PickPlaceMissionHarness attachedBox;
    attachedBox.Start();
    attachedBox.CompletePlanningAndUpdate();
    attachedBox.Update();
    attachedBox.Update();
    attachedBox.Update(true);
    EXPECT_EQ(attachedBox.mission.Snapshot().stageLabel, "Lifting the box");
    EXPECT_EQ(attachedBox.gripper.lastCommand.positionRequest,
        static_cast<std::uint8_t>(application::pick_place::config::GripperCommandSettings::PositionRequest::Closed));
    const auto linearPathRequestsBeforeResume = attachedBox.robot.linearPathRequests;

    actions = {};
    actions.stop = true;
    attachedBox.Apply(actions, true);
    actions = {};
    actions.resume = true;
    attachedBox.Apply(actions, true);

    EXPECT_FALSE(attachedBox.mission.Snapshot().paused);
    EXPECT_EQ(attachedBox.mission.Snapshot().stageLabel, "Lifting to resume height");
    EXPECT_EQ(attachedBox.robot.linearPathRequests, linearPathRequestsBeforeResume + 1U);
    ASSERT_EQ(attachedBox.robot.requestedLinearPath.targetPoses.size(), 1U);

    const auto verifyCollisionRecovery = [&](ErrorCode collision)
    {
        PickPlaceMissionHarness collisionHarness;
        collisionHarness.Start();
        collisionHarness.CompletePlanningAndUpdate();

        collisionHarness.robot.state.errorCode = collision;
        collisionHarness.Update();
        EXPECT_EQ(collisionHarness.mission.Snapshot().stageLabel, "Retracting after collision");
        EXPECT_EQ(collisionHarness.robot.linearPathRequests, 1U);

        collisionHarness.robot.state.errorCode = ErrorCode::None;
        collisionHarness.Update();
        EXPECT_EQ(collisionHarness.mission.Snapshot().stageLabel, "Retracting from goal");
    };

    verifyCollisionRecovery(ErrorCode::SelfCollision);
    verifyCollisionRecovery(ErrorCode::AttachedObjectCollision);
}

TEST(PickPlaceMission, CompleteStageReportsSuccessAndRaisesOneEvent)
{
    using namespace grasplink;
    PickPlaceMissionHarness harness;
    auto& mission = harness.mission;
    auto& controller = harness.robot;
    harness.Start();

    harness.CompletePlanningAndUpdate();
    harness.Update();
    harness.Update();
    harness.Update(true);
    harness.Update(true);
    harness.Update(true);
    harness.Update(true);
    harness.CompletePlanningAndUpdate();
    harness.Update(true);
    harness.Update(true);
    harness.Update(true);

    harness.now += application::pick_place::config::PlacementSettings::settleDuration;
    harness.Update(true);
    harness.Update();
    harness.Update();

    const auto completed = mission.Snapshot();
    EXPECT_EQ(completed.stageLabel, "Complete");
    EXPECT_TRUE(completed.missionSucceeded);
    EXPECT_EQ(completed.completedCount, 1U);
    EXPECT_TRUE(mission.ConsumeSuccessEvent());
    EXPECT_FALSE(mission.ConsumeSuccessEvent());
}

TEST(PickPlaceScenarioSampler, SeedReproducesValidScenariosAndResetsHistory)
{
    using grasplink::application::PickPlaceScenarioSampler;
    using namespace grasplink::application::pick_place::config;
    PickPlaceScenarioSampler first;
    PickPlaceScenarioSampler second;
    first.Seed(7319);
    second.Seed(7319);

    std::array<float, 3> previousBox{};
    std::array<float, 3> previousPlacement{};
    std::array<float, 3> initialBox{};
    for (int sample = 0; sample < 10; ++sample)
    {
        const auto box = first.SampleBoxPosition();
        const auto placement = first.SamplePlacementPosition();
        const float rotation = first.SamplePlanarRotation();
        EXPECT_EQ(box, second.SampleBoxPosition());
        EXPECT_EQ(placement, second.SamplePlacementPosition());
        EXPECT_EQ(rotation, second.SamplePlanarRotation());
        if (sample == 0)
            initialBox = box;

        const float boxRadius = std::hypot(box[0], box[2]);
        const float placementRadius = std::hypot(placement[0], placement[2]);
        EXPECT_GE(boxRadius, minimumScenarioRadiusMeters);
        EXPECT_LE(boxRadius, maximumBoxRadiusMeters);
        EXPECT_GE(placementRadius, minimumScenarioRadiusMeters);
        EXPECT_LE(placementRadius, maximumPlacementRadiusMeters);
        EXPECT_GE(rotation, -scenarioRotationLimitRadians);
        EXPECT_LE(rotation, scenarioRotationLimitRadians);
        if (sample > 0)
        {
            EXPECT_GE(std::hypot(box[0] - previousBox[0], box[2] - previousBox[2]),
                minimumBoxRepeatSeparationMeters);
            EXPECT_GE(std::hypot(placement[0] - previousPlacement[0],
                placement[2] - previousPlacement[2]), minimumPlacementRepeatSeparationMeters);
        }
        previousBox = box;
        previousPlacement = placement;
    }

    first.Seed(7319);
    EXPECT_EQ(first.SampleBoxPosition(), initialBox);
}
