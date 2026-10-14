#include "application/PickPlaceMission.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>
namespace
{
using namespace grasplink::robotics;

class StubGripper final : public grasplink::robotics::IGripperController
{
public:
    grasplink::robotics::Result Connect() override { return {}; }
    void Disconnect() noexcept override {}
    bool IsConnected() const noexcept override { return true; }
    grasplink::robotics::Result Activate() override { return {}; }
    grasplink::robotics::Result Reset() override { return {}; }
    grasplink::robotics::Result Command(const grasplink::robotics::GripperCommand& command) override
    {
        commands.push_back(command.positionRequest);
        return {};
    }
    grasplink::robotics::Result Stop() override { return {}; }
    grasplink::robotics::GripperState GetState() const override { return state; }
    void Update(double) override {}

    grasplink::robotics::GripperState state{};
    std::vector<std::uint8_t> commands;
};

class FakeRobotController final : public IRobotController
{
public:
    explicit FakeRobotController(
        const models::RobotSpecification& specification = models::hanwha::kHcr12a,
        bool completeMovesImmediately = false)
        : ik_(specification), completeMovesImmediately_(completeMovesImmediately)
    {
        state.jointPositionRadians.assign(specification.jointCount, 0.0);
        state.tcpPose = ik_.EvaluateTcp(state.jointPositionRadians);
        state.mode = RobotMode::Idle;
        state.tcpPoseValid = true;
        state.valid = true;
    }

    Result Connect() override { return {}; }
    void Disconnect() noexcept override {}
    bool IsConnected() const noexcept override { return true; }
    Result MoveJoint(const JointMoveCommand& command) override
    {
        ++jointRequests;
        if (!jointResult)
            return jointResult;
        if (completeMovesImmediately_)
        {
            state.jointPositionRadians = command.targetPositionRadians;
            state.tcpPose = ik_.EvaluateTcp(state.jointPositionRadians);
            state.mode = RobotMode::Idle;
        }
        else
            state.mode = RobotMode::Moving;
        return jointResult;
    }
    Result MovePose(const CartesianPose& target, double, double) override
    {
        ++poseRequests;
        if (poseRequests == rejectedPoseRequestNumber)
            return rejectedPoseResult;
        if (!poseResult)
            return poseResult;
        state.tcpPose = target;
        state.mode = completeMovesImmediately_ ? RobotMode::Idle : RobotMode::Moving;
        return poseResult;
    }
    Result BeginPosePlanning(const CartesianPose& target, double velocityScale, double accelerationScale) override
    {
        if (!deferPosePlanning || poseRequests + 1 != deferredPosePlanningRequest)
            return MovePose(target, velocityScale, accelerationScale);
        ++poseRequests;
        if (poseRequests == rejectedPoseRequestNumber)
            return rejectedPoseResult;
        if (!poseResult)
            return poseResult;
        planning = true;
        state.mode = RobotMode::Planning;
        deferredPoseTarget = target;
        return {};
    }
    Result BeginPosePlanningWithLinearContinuation(const CartesianPose& approach,
        const LinearPathMoveCommand& continuation, double velocityScale, double accelerationScale) override
    {
        ++compositeRequests;
        compositeApproach = approach;
        compositeContinuation = continuation;
        if (!compositeResult)
            return compositeResult;
        if (!deferCompositePlanning)
            return MovePose(approach, velocityScale, accelerationScale);
        planning = true;
        compositePlanning = true;
        deferredPoseTarget = approach;
        deferredPoseVelocityScale = velocityScale;
        deferredPoseAccelerationScale = accelerationScale;
        state.mode = RobotMode::Planning;
        return {};
    }
    Result MoveLinear(const LinearMoveCommand& command) override
    {
        ++linearRequests;
        if (!linearResult)
            return linearResult;
        state.tcpPose = command.targetPose;
        state.mode = completeMovesImmediately_ ? RobotMode::Idle : RobotMode::Moving;
        return linearResult;
    }
    Result MoveLinearPath(const LinearPathMoveCommand& command) override
    {
        ++pathRequests;
        if (!pathResult)
            return pathResult;
        if (completeMovesImmediately_)
        {
            state.tcpPose = command.targetPoses.back();
            state.mode = RobotMode::Idle;
        }
        else
            state.mode = RobotMode::Moving;
        return pathResult;
    }
    Result BeginLinearPathPlanning(const LinearPathMoveCommand& command) override
    {
        const bool pathRequest = std::exchange(expectPathRequest, false);
        if (!pathRequest)
        {
            return MoveLinear({command.targetPoses.front(), command.maxLinearVelocityMetersPerSecond,
                command.maxAngularVelocityRadiansPerSecond, command.maxLinearAccelerationMetersPerSecondSquared,
                command.maxAngularAccelerationRadiansPerSecondSquared});
        }
        if (!deferPathPlanning)
            return MoveLinearPath(command);
        ++pathRequests;
        if (!pathResult)
            return pathResult;
        planning = true;
        state.mode = RobotMode::Planning;
        return {};
    }
    void AdvanceMotionPlanning(std::size_t) override {}
    bool IsMotionPlanning() const noexcept override { return planning; }
    std::optional<Result> TakeMotionPlanningResult() override
    {
        if (!planningResultReady)
            return std::nullopt;
        planningResultReady = false;
        if (!compositePlanning)
            return planningResult;
        compositePlanning = false;
        if (!planningResult)
            return planningResult;
        return MovePose(deferredPoseTarget, deferredPoseVelocityScale, deferredPoseAccelerationScale);
    }
    Result Stop() override
    {
        ++stopRequests;
        if (planning)
        {
            planning = false;
            planningResult = {ErrorCode::Cancelled, "path planning cancelled"};
            planningResultReady = true;
        }
        if (stopResult)
            state.mode = RobotMode::Stopped;
        return stopResult;
    }
    RobotState GetState() const override { return state; }
    void Update(double) override {}

    RobotState state;
    Result jointResult{};
    Result poseResult{};
    Result rejectedPoseResult{ErrorCode::Unsupported, "pose execution unavailable"};
    Result compositeResult{};
    Result linearResult{};
    Result pathResult{};
    Result stopResult{};
    int jointRequests = 0;
    int poseRequests = 0;
    int compositeRequests = 0;
    int linearRequests = 0;
    int pathRequests = 0;
    int stopRequests = 0;
    bool deferPathPlanning = false;
    bool deferPosePlanning = false;
    bool deferCompositePlanning = false;
    bool compositePlanning = false;
    int deferredPosePlanningRequest = 0;
    int rejectedPoseRequestNumber = 0;
    bool expectPathRequest = false;
    bool planning = false;
    bool planningResultReady = false;
    Result planningResult{};
    CartesianPose deferredPoseTarget{};
    CartesianPose compositeApproach{};
    LinearPathMoveCommand compositeContinuation{};
    double deferredPoseVelocityScale = 1.0;
    double deferredPoseAccelerationScale = 1.0;

    void CompletePathPlanning(Result result = {})
    {
        planning = false;
        planningResult = std::move(result);
        planningResultReady = true;
        state.mode = planningResult ? RobotMode::Moving : RobotMode::Idle;
        state.errorCode = planningResult.code;
    }

    void CompletePosePlanning(Result result = {})
    {
        planning = false;
        planningResult = std::move(result);
        planningResultReady = true;
        if (planningResult)
            state.tcpPose = deferredPoseTarget;
        state.mode = planningResult ? RobotMode::Moving : RobotMode::Idle;
        state.errorCode = planningResult.code;
    }

    void CompleteCompositePlanning(Result result = {})
    {
        planning = false;
        planningResult = std::move(result);
        planningResultReady = true;
        state.mode = RobotMode::Idle;
        state.errorCode = planningResult.code;
    }

private:
    kinematics::DampedLeastSquaresIk ik_;
    bool completeMovesImmediately_ = false;
};

}

TEST(PickPlaceMissionTests, StartPauseAndResumeWithHeldObject)
{
    using namespace grasplink::robotics;
    using grasplink::robotics::backends::simulation::SimRobotController;
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "mission controller connects";
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    const auto state = controller.GetState();
    CartesianPose box = state.tcpPose;
    CartesianPose goal = state.tcpPose;

    mission.Update(controller.GetStateView(), controller, gripper, false, box, goal);
    mission.ApplyActions({true, false, false}, controller.GetStateView(), controller, false, box);
    ASSERT_EQ(mission.Snapshot().stageLabel, "Unwinding J6 before pickup");

    mission.ApplyActions({false, false, true}, controller.GetStateView(), controller, false, box);
    ASSERT_TRUE(mission.Snapshot().paused);
    mission.ApplyActions({false, true, false}, controller.GetStateView(), controller, true, box);
    const auto resumed = mission.Snapshot();
    ASSERT_FALSE(resumed.paused);
    ASSERT_EQ(resumed.stageLabel, "Lifting to resume height");
}

TEST(PickPlaceMissionTests, UnreachablePickupFailsWithoutMotion)
{
    using namespace grasplink::robotics;
    using grasplink::robotics::backends::simulation::SimRobotController;
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "mission controller connects for failed pickup";
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    const auto initial = controller.GetState();
    CartesianPose unreachableBox = initial.tcpPose;
    unreachableBox.positionMeters[0] += 5.0;
    const CartesianPose goal = initial.tcpPose;

    mission.Update(controller.GetStateView(), controller, gripper, false, unreachableBox, goal);
    mission.ApplyActions({true, false, false}, controller.GetStateView(), controller, false, unreachableBox);
    ASSERT_EQ(mission.Snapshot().stageLabel, "Unwinding J6 before pickup");

    for (std::size_t frame = 0; frame < 20000 && mission.Snapshot().stageLabel != "Failed"; ++frame)
    {
        if (controller.IsMotionPlanning())
            controller.AdvanceMotionPlanning(2);
        mission.Update(controller.GetStateView(), controller, gripper, false, unreachableBox, goal);
    }
    const auto failed = mission.Snapshot();
    ASSERT_EQ(failed.stageLabel, "Failed");
    EXPECT_TRUE(failed.hasResult);
    EXPECT_FALSE(failed.lastRequestAccepted);
    EXPECT_FALSE(failed.missionSucceeded);
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Idle);
}

TEST(PickPlaceMissionTests, RejectedStartCommandFailsImmediately)
{
    FakeRobotController controller;
    StubGripper gripper;
    controller.jointResult = {ErrorCode::Busy, "controller is busy"};
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);

    mission.Update(controller.state, controller, gripper, false, {}, {});
    mission.ApplyActions({true, false, false}, controller.state, controller, false, {});

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(controller.jointRequests, 1);
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_TRUE(snapshot.hasResult);
    EXPECT_FALSE(snapshot.lastRequestAccepted);
    EXPECT_EQ(snapshot.lastMessage, "controller is busy");
}

TEST(PickPlaceMissionTests, AcceptedCommandWaitsForControllerToBecomeIdle)
{
    FakeRobotController controller;
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);

    mission.Update(controller.state, controller, gripper, false, {}, {});
    mission.ApplyActions({true, false, false}, controller.state, controller, false, {});
    mission.Update(controller.state, controller, gripper, false, {}, {});

    EXPECT_EQ(mission.Snapshot().stageLabel, "Unwinding J6 before pickup");
    EXPECT_EQ(controller.jointRequests, 1);
    EXPECT_EQ(controller.linearRequests, 0);
}

TEST(PickPlaceMissionTests, PreflightsAlignedPickupApproachAndVerticalDescent)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    mission.Update(controller.state, controller, gripper, false, box, box);

    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the box");
    EXPECT_EQ(controller.poseRequests, 1);
    EXPECT_EQ(controller.compositeRequests, 1);
    EXPECT_EQ(controller.linearRequests, 0);
    EXPECT_EQ(controller.pathRequests, 0);
    ASSERT_EQ(controller.compositeContinuation.targetPoses.size(), 1U);
    EXPECT_EQ(controller.compositeApproach.orientationXyzw,
        controller.compositeContinuation.targetPoses.front().orientationXyzw);
    EXPECT_NEAR(controller.compositeApproach.positionMeters[1] -
        controller.compositeContinuation.targetPoses.front().positionMeters[1], 0.225, 1e-9);

    mission.Update(controller.state, controller, gripper, false, box, box);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Lowering to the box");
    EXPECT_EQ(controller.poseRequests, 1);
    EXPECT_EQ(controller.linearRequests, 1);
}

TEST(PickPlaceMissionTests, CompositePreflightFailureDoesNotStartApproachOrDescent)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    controller.compositeResult = {ErrorCode::Unsupported, "approach continuation is not reachable"};
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    mission.Update(controller.state, controller, gripper, false, box, box);

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_EQ(snapshot.lastMessage, "approach continuation is not reachable");
    EXPECT_EQ(controller.compositeRequests, 1);
    EXPECT_EQ(controller.poseRequests, 0);
    EXPECT_EQ(controller.linearRequests, 0);
}

TEST(PickPlaceMissionTests, WaitsForCompositePreflightBeforeStartingPickupApproach)
{
    FakeRobotController controller;
    controller.deferCompositePlanning = true;
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    controller.state.mode = RobotMode::Idle;
    mission.Update(controller.state, controller, gripper, false, box, box);

    ASSERT_EQ(mission.Snapshot().stageLabel, "Planning motion");
    EXPECT_EQ(controller.compositeRequests, 1);
    EXPECT_EQ(controller.poseRequests, 0);
    EXPECT_EQ(controller.linearRequests, 0);
    mission.Update(controller.state, controller, gripper, false, box, box);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Planning motion");

    controller.CompleteCompositePlanning();
    mission.Update(controller.state, controller, gripper, false, box, box);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the box");
    EXPECT_EQ(controller.poseRequests, 1);
    EXPECT_EQ(controller.linearRequests, 0);
}

TEST(PickPlaceMissionTests, FailedCompositePlanDoesNotStartMoveJ)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    controller.deferCompositePlanning = true;
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    mission.Update(controller.state, controller, gripper, false, box, box);
    ASSERT_EQ(mission.Snapshot().stageLabel, "Planning motion");
    EXPECT_EQ(controller.poseRequests, 0);
    EXPECT_EQ(controller.linearRequests, 0);

    controller.CompleteCompositePlanning({ErrorCode::JointLimitReached, "approach is blocked by a joint limit"});
    mission.Update(controller.state, controller, gripper, false, box, box);

    EXPECT_EQ(mission.Snapshot().stageLabel, "Failed");
    EXPECT_EQ(mission.Snapshot().lastMessage, "approach is blocked by a joint limit");
    EXPECT_EQ(controller.poseRequests, 0);
    EXPECT_EQ(controller.linearRequests, 0);
}

TEST(PickPlaceMissionTests, SimulationPosePlanningAdvancesIkAndJointPathInBoundedWork)
{
    using grasplink::robotics::backends::simulation::SimRobotController;
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect()));
    const CartesianPose target = controller.GetState().tcpPose;

    ASSERT_TRUE(static_cast<bool>(controller.BeginPosePlanning(target)));
    ASSERT_TRUE(controller.IsMotionPlanning());
    EXPECT_EQ(controller.GetState().mode, RobotMode::Planning);

    controller.AdvanceMotionPlanning(1);
    EXPECT_EQ(controller.GetState().mode, RobotMode::Planning);
    EXPECT_TRUE(controller.IsMotionPlanning());

    for (std::size_t slice = 0; slice < 500 && controller.IsMotionPlanning(); ++slice)
        controller.AdvanceMotionPlanning(1);
    EXPECT_FALSE(controller.IsMotionPlanning());
    const auto result = controller.TakeMotionPlanningResult();
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(static_cast<bool>(*result)) << result->message;
    EXPECT_TRUE(controller.GetState().mode == RobotMode::Moving ||
        controller.GetState().mode == RobotMode::Idle);
}

TEST(PickPlaceMissionTests, EnvironmentContactRequestsRetreat)
{
    FakeRobotController controller;
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = {0.4, 0.2, 0.1};

    mission.Update(controller.state, controller, gripper, false, box, {});
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    controller.state.mode = RobotMode::Idle;
    controller.state.jointPositionRadians.back() = 0.0;
    mission.Update(controller.state, controller, gripper, false, box, {});
    ASSERT_EQ(mission.Snapshot().stageLabel, "Moving above the box");
    controller.state.mode = RobotMode::Idle;
    controller.state.errorCode = ErrorCode::EnvironmentContact;

    mission.Update(controller.state, controller, gripper, false, box, {});

    EXPECT_EQ(mission.Snapshot().stageLabel, "Retracting after collision");
    EXPECT_EQ(controller.poseRequests, 1);
    EXPECT_EQ(controller.linearRequests, 1);
    EXPECT_TRUE(mission.Snapshot().lastRequestAccepted);
}

TEST(PickPlaceMissionTests, UnrelatedControllerErrorDoesNotTriggerCollisionRecovery)
{
    FakeRobotController controller;
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = {0.4, 0.2, 0.1};

    mission.Update(controller.state, controller, gripper, false, box, {});
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    controller.state.mode = RobotMode::Idle;
    mission.Update(controller.state, controller, gripper, false, box, {});
    controller.state.mode = RobotMode::Moving;
    controller.state.errorCode = ErrorCode::Fault;

    mission.Update(controller.state, controller, gripper, false, box, {});

    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the box");
    EXPECT_EQ(controller.poseRequests, 1);
    EXPECT_EQ(controller.linearRequests, 0);
}

TEST(PickPlaceMissionTests, RejectedStopDoesNotPauseMission)
{
    FakeRobotController controller;
    StubGripper gripper;
    controller.stopResult = {ErrorCode::Fault, "stop rejected"};
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);

    mission.ApplyActions({false, false, true}, controller.state, controller, false, {});

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(controller.stopRequests, 1);
    EXPECT_FALSE(snapshot.paused);
    EXPECT_EQ(snapshot.stageLabel, "Ready");
    EXPECT_FALSE(snapshot.lastRequestAccepted);
    EXPECT_EQ(snapshot.lastMessage, "stop rejected");
}

TEST(PickPlaceMissionTests, ResumeWithHeldObjectRequiresValidTcpFeedback)
{
    FakeRobotController controller;
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    mission.ApplyActions({false, false, true}, controller.state, controller, false, {});
    ASSERT_TRUE(mission.Snapshot().paused);
    controller.state.valid = false;
    controller.state.tcpPoseValid = false;

    mission.ApplyActions({false, true, false}, controller.state, controller, true, {});

    const auto snapshot = mission.Snapshot();
    EXPECT_FALSE(snapshot.paused);
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_FALSE(snapshot.lastRequestAccepted);
    EXPECT_EQ(snapshot.lastMessage, "RobotPanel: TCP feedback is unavailable for safe resume");
    EXPECT_EQ(controller.jointRequests, 0);
    EXPECT_EQ(controller.linearRequests, 0);
}

TEST(PickPlaceMissionTests, SuccessfulCyclePublishesOneCompletionEvent)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;
    CartesianPose placement = box;

    mission.Update(controller.state, controller, gripper, false, box, placement);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    bool boxGrasped = false;
    for (int tick = 0; tick < 40 && mission.Snapshot().stageLabel != "Complete" &&
        mission.Snapshot().stageLabel != "Failed"; ++tick)
    {
        if (mission.Snapshot().stageLabel == "Closing gripper")
            boxGrasped = true;
        if (mission.Snapshot().stageLabel == "Opening gripper")
            boxGrasped = false;
        const auto stageBeforeUpdate = mission.Snapshot().stageLabel;
        mission.Update(controller.state, controller, gripper, boxGrasped, box, placement);
        if (mission.Snapshot().stageLabel == "Failed")
            ADD_FAILURE() << "Mission failed while processing " << stageBeforeUpdate
                << ": " << mission.Snapshot().lastMessage;
    }

    const auto snapshot = mission.Snapshot();
    ASSERT_EQ(snapshot.stageLabel, "Complete") << snapshot.lastMessage;
    EXPECT_TRUE(snapshot.missionSucceeded);
    EXPECT_EQ(snapshot.completedCount, 1U);
    EXPECT_EQ(controller.compositeRequests, 2);
    ASSERT_EQ(controller.compositeContinuation.targetPoses.size(), 1U);
    EXPECT_EQ(controller.compositeApproach.orientationXyzw,
        controller.compositeContinuation.targetPoses.front().orientationXyzw);
    EXPECT_NEAR(controller.compositeApproach.positionMeters[1] -
        controller.compositeContinuation.targetPoses.front().positionMeters[1], 0.25, 1e-9);
    EXPECT_TRUE(mission.ConsumeSuccessEvent());
    EXPECT_FALSE(mission.ConsumeSuccessEvent());
}

TEST(PickPlaceMissionTests, GripperMissReleasesAndFailsWithoutSuccessEvent)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    StubGripper gripper;
    gripper.state.mode = GripperMode::Idle;
    gripper.state.objectStatus = GripperObjectStatus::AtRequestedPosition;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    for (int tick = 0; tick < 40 && mission.Snapshot().stageLabel != "Complete" &&
        mission.Snapshot().stageLabel != "Failed"; ++tick)
        mission.Update(controller.state, controller, gripper, false, box, box);

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_FALSE(snapshot.missionSucceeded);
    EXPECT_EQ(snapshot.completedCount, 0U);
    EXPECT_EQ(gripper.commands, (std::vector<std::uint8_t>{255, 0}));
    EXPECT_FALSE(mission.ConsumeSuccessEvent());
}

TEST(PickPlaceMissionTests, RejectedMoveJTransitPreservesControllerError)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    controller.rejectedPoseRequestNumber = 2;
    controller.rejectedPoseResult = {ErrorCode::Unsupported, "joint-space transit unavailable"};
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    bool boxGrasped = false;
    for (int tick = 0; tick < 40 && mission.Snapshot().stageLabel != "Complete" &&
        mission.Snapshot().stageLabel != "Failed"; ++tick)
    {
        boxGrasped = mission.Snapshot().stageLabel == "Closing gripper" || boxGrasped;
        mission.Update(controller.state, controller, gripper, boxGrasped, box, box);
    }

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_EQ(snapshot.lastMessage, "joint-space transit unavailable");
    EXPECT_EQ(controller.poseRequests, 2);
    EXPECT_FALSE(snapshot.missionSucceeded);
}

TEST(PickPlaceMissionTests, WaitsForIncrementalMoveJTransitPlanningBeforeMovingToPlacement)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    controller.deferPosePlanning = true;
    controller.deferredPosePlanningRequest = 2;
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    bool boxGrasped = false;
    for (int tick = 0; tick < 40 && mission.Snapshot().stageLabel != "Planning motion" &&
        mission.Snapshot().stageLabel != "Failed"; ++tick)
    {
        boxGrasped = boxGrasped || mission.Snapshot().stageLabel == "Closing gripper";
        mission.Update(controller.state, controller, gripper, boxGrasped, box, box);
    }
    ASSERT_EQ(mission.Snapshot().stageLabel, "Planning motion");
    ASSERT_TRUE(controller.IsMotionPlanning());
    ASSERT_EQ(controller.poseRequests, 2);
    const int linearRequestsBeforePlanning = controller.linearRequests;

    for (int frame = 0; frame < 4; ++frame)
        mission.Update(controller.state, controller, gripper, true, box, box);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Planning motion");
    EXPECT_EQ(controller.poseRequests, 2);
    EXPECT_EQ(controller.linearRequests, linearRequestsBeforePlanning)
        << "the mission does not submit another command while MoveJ planning is pending";

    controller.CompletePosePlanning();
    mission.Update(controller.state, controller, gripper, true, box, box);
    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the goal");
    EXPECT_EQ(controller.linearRequests, linearRequestsBeforePlanning);
    mission.Update(controller.state, controller, gripper, true, box, box);
    EXPECT_EQ(controller.linearRequests, linearRequestsBeforePlanning)
        << "the next pose waits until controller motion completes";

    controller.state.mode = RobotMode::Idle;
    mission.Update(controller.state, controller, gripper, true, box, box);
    EXPECT_EQ(controller.linearRequests, linearRequestsBeforePlanning);
    EXPECT_EQ(controller.poseRequests, 3);
    EXPECT_EQ(controller.compositeRequests, 2)
        << "the final-orientation approach and attached-box descent are preflighted together";
}

TEST(PickPlaceMissionTests, RuntimeControllerFaultFailsActiveMission)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    StubGripper gripper;
    grasplink::application::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    controller.state.mode = RobotMode::Idle;
    mission.Update(controller.state, controller, gripper, false, box, box);
    ASSERT_EQ(mission.Snapshot().stageLabel, "Moving above the box");

    controller.state.mode = RobotMode::Fault;
    controller.state.errorCode = ErrorCode::IkDidNotConverge;
    mission.Update(controller.state, controller, gripper, true, box, box);

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_FALSE(snapshot.lastRequestAccepted);
    EXPECT_FALSE(snapshot.autoRepeat);
    EXPECT_FALSE(snapshot.missionSucceeded);
    EXPECT_NE(snapshot.lastMessage.find("runtime IK did not converge"), std::string_view::npos);
}
