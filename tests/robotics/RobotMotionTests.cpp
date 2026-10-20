#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/planning/JointPathPlanner.h"
#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

using namespace grasplink::robotics;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;
using grasplink::robotics::kinematics::LevenbergMarquardtIk;

static_assert(!std::is_constructible_v<SimRobotController, models::RobotSpecification&&>);

namespace
{
double PositionDistance(const CartesianPose& left, const CartesianPose& right)
{
    double squared = 0.0;
    for (std::size_t i = 0; i < 3; ++i)
    {
        const double difference = left.positionMeters[i] - right.positionMeters[i];
        squared += difference * difference;
    }
    return std::sqrt(squared);
}

double OrientationDistance(const CartesianPose& left, const CartesianPose& right)
{
    double dot = 0.0;
    double leftNorm = 0.0;
    double rightNorm = 0.0;
    for (std::size_t i = 0; i < 4; ++i)
    {
        dot += left.orientationXyzw[i] * right.orientationXyzw[i];
        leftNorm += left.orientationXyzw[i] * left.orientationXyzw[i];
        rightNorm += right.orientationXyzw[i] * right.orientationXyzw[i];
    }
    dot = std::abs(dot) / std::sqrt(leftNorm * rightNorm);
    return 2.0 * std::acos(std::clamp(dot, 0.0, 1.0));
}

bool AdvanceUntilIdle(SimRobotController& controller, double dtSeconds)
{
    for (int i = 0; i < 10000 && controller.GetState().mode == RobotMode::Moving; ++i)
        controller.Update(dtSeconds);
    return controller.GetState().mode == RobotMode::Idle;
}

void CheckPoseApproachWithLinearContinuation()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    LevenbergMarquardtIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE(static_cast<bool>(controller.Connect()));

    const JointVector approachJoints{0.03, -0.04, 0.02, 0.01, -0.02, 0.03};
    const JointVector continuationJoints{0.05, -0.03, 0.03, 0.01, -0.02, 0.03};
    LinearPathMoveCommand continuation;
    continuation.targetPoses.push_back(ik.EvaluateTcp(continuationJoints));

    const Result begin = controller.BeginPosePlanningWithLinearContinuation(
        ik.EvaluateTcp(approachJoints), continuation);
    ASSERT_TRUE(static_cast<bool>(begin)) << begin.message;

    for (std::size_t step = 0; step < 10000 && controller.IsMotionPlanning(); ++step)
        controller.AdvanceMotionPlanning(64);
    ASSERT_FALSE(controller.IsMotionPlanning());
    const auto planningResult = controller.TakeMotionPlanningResult();
    ASSERT_TRUE(planningResult.has_value());
    ASSERT_TRUE(static_cast<bool>(*planningResult)) << planningResult->message;
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004));

    const Result continueMove = controller.MoveLinearPath(continuation);
    ASSERT_TRUE(static_cast<bool>(continueMove)) << continueMove.message;
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004));
    EXPECT_LT(PositionDistance(controller.GetStateView().tcpPose,
        ik.EvaluateTcp(continuationJoints)), 0.001);
}

void CheckLinearPathPlanningBudgetAndCancellation()
{
    const auto& specification = models::hanwha::kHcr12a;
    LevenbergMarquardtIk inverse(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    JointVector targetJoints = startJoints;
    targetJoints[0] = 0.2;
    auto checkerContext = std::make_shared<int>(0);
    std::weak_ptr<int> checkerContextLifetime = checkerContext;
    StateValidityChecker checker = [checkerContext](const JointVector&)
    {
        (void)checkerContext;
        return JointStateInvalidity::None;
    };

    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp(targetJoints));
    LinearPathPlanningJob job;
    const Result begin = job.Begin(command, specification, startJoints,
        inverse.EvaluateTcp(startJoints), inverse, checker);
    checkerContext.reset();
    checker = {};
    ASSERT_TRUE(static_cast<bool>(begin)) << begin.message;
    EXPECT_FALSE(checkerContextLifetime.expired());
    ASSERT_EQ(job.GetState(), LinearPathPlanningState::Running);

    EXPECT_EQ(job.Advance(0), LinearPathPlanningState::Running);
    EXPECT_EQ(job.Advance(1), LinearPathPlanningState::Running);
    job.Cancel();
    EXPECT_EQ(job.GetState(), LinearPathPlanningState::Cancelled);
    EXPECT_EQ(job.GetResult().code, ErrorCode::Cancelled);
    EXPECT_TRUE(checkerContextLifetime.expired());
    EXPECT_EQ(job.Advance(64), LinearPathPlanningState::Cancelled);
    EXPECT_FALSE(job.TakePlan().has_value());

    auto completedCheckerContext = std::make_shared<int>(0);
    std::weak_ptr<int> completedCheckerLifetime = completedCheckerContext;
    StateValidityChecker completedChecker = [completedCheckerContext](const JointVector&)
    {
        (void)completedCheckerContext;
        return JointStateInvalidity::None;
    };
    const Result restarted = job.Begin(command, specification, startJoints,
        inverse.EvaluateTcp(startJoints), inverse, completedChecker);
    completedCheckerContext.reset();
    completedChecker = {};
    ASSERT_TRUE(static_cast<bool>(restarted)) << restarted.message;
    EXPECT_EQ(job.GetState(), LinearPathPlanningState::Running);

    for (std::size_t step = 0; step < 10000 &&
        job.GetState() == LinearPathPlanningState::Running; ++step)
    {
        job.Advance(64);
    }
    ASSERT_EQ(job.GetState(), LinearPathPlanningState::Completed);
    EXPECT_TRUE(completedCheckerLifetime.expired());
    auto completedPlan = job.TakePlan();
    ASSERT_TRUE(completedPlan.has_value());
    EXPECT_TRUE(completedPlan->hasMotion);

    const Result restartedAfterCompletion = job.Begin(command, specification, startJoints,
        inverse.EvaluateTcp(startJoints), inverse, {});
    ASSERT_TRUE(static_cast<bool>(restartedAfterCompletion)) << restartedAfterCompletion.message;
    EXPECT_EQ(job.GetState(), LinearPathPlanningState::Running);
    job.Cancel();

    std::size_t wrongThreadCheckerCalls = 0;
    StateValidityChecker wrongThreadChecker = [&](const JointVector&)
    {
        ++wrongThreadCheckerCalls;
        return JointStateInvalidity::None;
    };
    LinearPathPlanningJob wrongThreadJob;
    const Result wrongThreadBegin = wrongThreadJob.Begin(command, specification,
        startJoints, inverse.EvaluateTcp(startJoints), inverse, wrongThreadChecker);
    ASSERT_TRUE(static_cast<bool>(wrongThreadBegin)) << wrongThreadBegin.message;
    EXPECT_EQ(wrongThreadJob.Advance(1), LinearPathPlanningState::Running);

    LinearPathPlanningState wrongThreadState = LinearPathPlanningState::Idle;
    std::thread wrongThread([&]
    {
        wrongThreadState = wrongThreadJob.Advance(1);
    });
    wrongThread.join();
    EXPECT_EQ(wrongThreadState, LinearPathPlanningState::Failed);
    EXPECT_EQ(wrongThreadJob.GetResult().code, ErrorCode::InvalidCommand);
    EXPECT_EQ(wrongThreadCheckerCalls, 0);
}

void CheckFourAxisControllerUsesPositionIk()
{
    std::array<models::JointSpecification, 4> joints{};
    std::copy_n(models::hanwha::kHcr12a.joints, joints.size(), joints.begin());
    const models::RobotSpecification specification{
        "Test", "FourAxis", joints.data(), joints.size(), nullptr, 0,
        models::hanwha::kHcr12a.toolFrameInLastJoint, true};
    LevenbergMarquardtIk inverse(specification);
    SimRobotController controller(specification);
    ASSERT_TRUE(static_cast<bool>(controller.Connect()));

    const JointVector targetJoints{0.12, -0.1, 0.08, 0.05};
    CartesianPose target = inverse.EvaluateTcp(targetJoints);
    target.orientationXyzw = {0.0, 0.0, 0.0, 1.0};
    const Result result = controller.MovePose(target);
    ASSERT_TRUE(static_cast<bool>(result)) << result.message;
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004));
    EXPECT_LT(PositionDistance(controller.GetStateView().tcpPose, target), 1e-3);
}

void CheckMovePoseAndFailurePreservation()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    LevenbergMarquardtIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "controller connects";
    const JointVector seed{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    JointMoveCommand seedCommand;
    seedCommand.targetPositions = seed;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(seedCommand)))) << "joint command prepares a regular IK seed";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "motion reaches target within bounded updates";
    const JointVector targetJoints{0.25, -0.37, 0.34, 0.17, -0.23, 0.11};
    const CartesianPose target = ik.EvaluateTcp(targetJoints);
    ASSERT_TRUE((static_cast<bool>(controller.MovePose(target, 0.5)))) << "MovePose accepts FK-generated reachable pose";
    ASSERT_TRUE((controller.GetState().mode == RobotMode::Moving)) << "MovePose starts joint motion";

    for (int i = 0; i < 8; ++i)
        controller.Update(0.004);
    const RobotState beforeRejected = controller.GetState();
    const CartesianPose farTarget{{50.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
    const Result unreachablePose = controller.MovePose(farTarget);
    ASSERT_TRUE((unreachablePose.code == ErrorCode::Unreachable))
        << "MovePose maps unreachable IK status: " << unreachablePose.message;
    const RobotState afterRejectedPose = controller.GetState();
    ASSERT_TRUE((afterRejectedPose.jointPositions == beforeRejected.jointPositions &&
        afterRejectedPose.jointVelocities == beforeRejected.jointVelocities &&
        afterRejectedPose.mode == beforeRejected.mode)) << "failed MovePose preserves the active motion state";
    const Result unreachable = controller.MoveLinear(
        {farTarget, 0.05, 0.2});
    ASSERT_TRUE((unreachable.code == ErrorCode::Unreachable))
        << "a line with a definitely unreachable endpoint is rejected early: " << unreachable.message;
    const RobotState afterRejected = controller.GetState();
    ASSERT_TRUE((afterRejected.jointPositions == beforeRejected.jointPositions &&
        afterRejected.jointVelocities == beforeRejected.jointVelocities &&
        afterRejected.mode == beforeRejected.mode)) << "failed replacement preserves the active motion state";

    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "motion reaches target within bounded updates";
    const RobotState reached = controller.GetState();
    ASSERT_TRUE((reached.tcpPoseValid)) << "simulation publishes model TCP feedback";
    ASSERT_TRUE((PositionDistance(reached.tcpPose, target) < 2e-5)) << "MovePose reaches requested TCP position";
    ASSERT_TRUE((OrientationDistance(reached.tcpPose, target) < 2e-4)) << "MovePose reaches requested TCP orientation";

    JointMoveCommand replacement;
    replacement.targetPositions.assign(6, 0.0);
    replacement.targetPositions[0] = -0.2;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(replacement)))) << "MoveJoint retarget succeeds";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.006)) << "motion reaches target within bounded updates";
    ASSERT_NEAR(controller.GetState().jointPositions[0], -0.2, 1e-8) << "MoveJoint replaces a pose target";
    CheckPoseApproachWithLinearContinuation();
    CheckFourAxisControllerUsesPositionIk();
    CheckLinearPathPlanningBudgetAndCancellation();
}

void CheckMoveJointRejectsInvalidPath();

void CheckPrismaticJointPathUsesMeterSpacing()
{
    std::array<models::JointSpecification, 1> joints{};
    joints[0].name = "slide";
    joints[0].maximumPosition = 0.2;
    joints[0].maximumVelocity = 1.0;
    joints[0].motionType = models::JointMotionType::Prismatic;

    models::RobotSpecification specification;
    specification.manufacturer = "Test";
    specification.model = "Prismatic";
    specification.joints = joints.data();
    specification.jointCount = joints.size();

    JointPathPlannerOptions options;
    options.validationPolicy.maximumPathIntervals = 10;
    options.validationPolicy.jointCollisionSampleSpacingRadians = 0.08;
    options.validationPolicy.prismaticJointCollisionSampleSpacingMeters = 0.02;

    std::vector<double> synchronousPositions;
    const StateValidityChecker synchronousChecker = [&](const JointVector& positions)
    {
        synchronousPositions.push_back(positions[0]);
        return JointStateInvalidity::None;
    };
    EXPECT_EQ(ValidateJointPath({0.0}, {0.1}, specification, synchronousChecker,
        options.validationPolicy), JointStateInvalidity::None);
    ASSERT_EQ(synchronousPositions.size(), 5);
    EXPECT_NEAR(synchronousPositions[0], 0.02, 1e-12);
    EXPECT_NEAR(synchronousPositions[3], 0.08, 1e-12);

    std::vector<double> checkedPositions;
    auto checkerContext = std::make_shared<int>(0);
    std::weak_ptr<int> checkerContextLifetime = checkerContext;
    StateValidityChecker checker = [&, checkerContext](const JointVector& positions)
    {
        (void)checkerContext;
        checkedPositions.push_back(positions[0]);
        return JointStateInvalidity::None;
    };

    JointPathPlanningJob job;
    const Result begin = job.Begin(specification, {0.0}, {0.1}, checker, options);
    checkerContext.reset();
    checker = {};
    ASSERT_TRUE(static_cast<bool>(begin)) << begin.message;
    EXPECT_FALSE(checkerContextLifetime.expired());
    for (std::size_t check = 0; check < 5; ++check)
        EXPECT_EQ(job.Advance(1), JointPathPlanningState::Running);
    EXPECT_EQ(checkedPositions.size(), 5);
    EXPECT_EQ(job.Advance(1), JointPathPlanningState::Completed);
    EXPECT_TRUE(checkerContextLifetime.expired());

    ASSERT_EQ(checkedPositions.size(), 6);
    EXPECT_NEAR(checkedPositions[0], 0.0, 1e-12);
    EXPECT_NEAR(checkedPositions[1], 0.1, 1e-12);
    EXPECT_NEAR(checkedPositions[2], 0.02, 1e-12);
    EXPECT_NEAR(checkedPositions[5], 0.08, 1e-12);
}

void CheckJointStateValidityReasons()
{
    using grasplink::robotics::planning::MapJointStateInvalidity;
    using grasplink::robotics::planning::ValidateJointState;

    const auto& specification = models::hanwha::kHcr12a;
    JointVector joints(specification.jointCount, 0.0);
    std::size_t collisionChecks = 0;
    const auto environmentChecker = [&](const JointVector&)
    {
        ++collisionChecks;
        return JointStateInvalidity::EnvironmentCollision;
    };

    ASSERT_TRUE((ValidateJointState(specification, joints, environmentChecker) ==
        JointStateInvalidity::EnvironmentCollision)) << "valid joint values propagate the environment collision reason";
    ASSERT_TRUE((collisionChecks == 1)) << "the environment checker runs after valid joint values";

    JointVector outsideLimit = joints;
    outsideLimit[0] = specification.joints[0].maximumPosition + 0.01;
    ASSERT_TRUE((ValidateJointState(specification, outsideLimit, environmentChecker) ==
        JointStateInvalidity::JointLimitViolation)) << "joint limits have a reason distinct from environment collision";
    ASSERT_TRUE((collisionChecks == 1)) << "invalid joint limits prevent the collision checker from running";

    JointVector nonFinite = joints;
    nonFinite[0] = std::numeric_limits<double>::infinity();
    ASSERT_TRUE((ValidateJointState(specification, nonFinite, environmentChecker) ==
        JointStateInvalidity::NonFinitePosition)) << "non-finite joint values are rejected before collision checks";
    ASSERT_TRUE((ValidateJointState(specification, JointVector{}, environmentChecker) ==
        JointStateInvalidity::JointCountMismatch)) << "joint count mismatch has its own invalidity reason";

    std::array<models::JointSpecification, 6> continuousJoints{};
    std::copy_n(specification.joints, continuousJoints.size(), continuousJoints.begin());
    continuousJoints[0].motionType = models::JointMotionType::Continuous;
    continuousJoints[0].minimumPosition = -std::numeric_limits<double>::infinity();
    continuousJoints[0].maximumPosition = std::numeric_limits<double>::infinity();
    auto continuousSpecification = specification;
    continuousSpecification.joints = continuousJoints.data();
    JointVector manyTurns(continuousJoints.size(), 0.0);
    manyTurns[0] = 3.0 * models::kFullTurnRadians + 0.1;
    EXPECT_EQ(ValidateJointState(continuousSpecification, manyTurns, {}), JointStateInvalidity::None)
        << "continuous joints accept any finite angle without a position limit";

    SimRobotController continuousController(continuousSpecification);
    ASSERT_TRUE(static_cast<bool>(continuousController.Connect()));
    JointMoveCommand continuousMove;
    continuousMove.targetPositions = manyTurns;
    ASSERT_TRUE(static_cast<bool>(continuousController.MoveJoint(continuousMove)))
        << "simulation accepts a continuous-joint target outside one turn";
    ASSERT_TRUE(AdvanceUntilIdle(continuousController, 0.01));
    EXPECT_NEAR(continuousController.GetState().jointPositions[0], 0.1, 1e-8)
        << "simulation follows the nearest equivalent turn from its current state";

    ASSERT_TRUE((MapJointStateInvalidity(JointStateInvalidity::EnvironmentCollision).code == ErrorCode::EnvironmentContact)) << "environment collision retains its existing controller error code";
    ASSERT_TRUE((MapJointStateInvalidity(JointStateInvalidity::SelfCollision).code == ErrorCode::SelfCollision)) << "self collision is not reported as environment contact";
    ASSERT_TRUE((MapJointStateInvalidity(JointStateInvalidity::AttachedObjectCollision).code == ErrorCode::AttachedObjectCollision)) << "attached-object collision is not reported as environment contact";

    PlanningPolicy policy;
    policy.maximumPathIntervals = 4;
    policy.jointCollisionSampleSpacingRadians = 0.1;
    JointVector pathEnd = joints;
    pathEnd[0] = 0.5;
    EXPECT_EQ(ValidateJointPath(joints, pathEnd, specification, {}, policy),
        JointStateInvalidity::PathSamplingLimitExceeded);
    EXPECT_EQ(MapJointStateInvalidity(JointStateInvalidity::PathSamplingLimitExceeded).code,
        ErrorCode::InvalidCommand);

    policy.jointCollisionSampleSpacingRadians = std::numeric_limits<double>::denorm_min();
    EXPECT_EQ(ValidateJointPath(joints, pathEnd, specification, {}, policy),
        JointStateInvalidity::PathSamplingLimitExceeded);

    CheckPrismaticJointPathUsesMeterSpacing();
    CheckMoveJointRejectsInvalidPath();
}

void CheckMoveJointRejectsInvalidPath()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "controller connects before joint path validation";

    std::size_t collisionChecks = 0;
    double firstCollisionCheckedPosition = -1.0;
    controller.SetJointStateValidityChecker([&](const JointVector& joints)
    {
        ++collisionChecks;
        if (collisionChecks == 1)
            firstCollisionCheckedPosition = joints[0];
        return joints[0] > 0.06 && joints[0] < 0.14 ?
            JointStateInvalidity::EnvironmentCollision : JointStateInvalidity::None;
    });

    JointVector target = controller.GetState().jointPositions;
    target[0] = 0.2;
    const Result result = controller.MoveJoint({target, 1.0, 1.0});
    ASSERT_TRUE((result.code == ErrorCode::EnvironmentContact)) << "MoveJoint rejects a collision between valid endpoints";
    ASSERT_TRUE((collisionChecks >= 1)) << "MoveJoint checks joint states along the path";
    ASSERT_TRUE((firstCollisionCheckedPosition > 0.0 && firstCollisionCheckedPosition < target[0]))
        << "MoveJoint checks an intermediate state before the target";
    ASSERT_TRUE((controller.GetState().mode == RobotMode::Idle)) << "rejected path leaves the controller idle";
    ASSERT_NEAR(controller.GetState().jointPositions[0], 0.0, 1e-12) << "rejected path leaves the current joints unchanged";
}

void CheckSelfCollisionFailureIncludesPairDiagnostic()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "controller connects before diagnostic validation";
    controller.SetJointStateValidityChecker([](const JointVector&)
    {
        return JointStateInvalidity::SelfCollision;
    });
    controller.SetJointStateValidityDiagnosticProvider([]
    {
        return std::string{"Link2 / Link5 at joints [deg] 0 0 0 0 0 0"};
    });

    JointVector target = controller.GetStateView().jointPositions;
    target[0] = 0.2;
    const Result result = controller.MoveJoint({target, 1.0, 1.0});
    EXPECT_EQ(result.code, ErrorCode::SelfCollision);
    EXPECT_NE(result.message.find("Link2 / Link5"), std::string::npos);
    EXPECT_NE(result.message.find("joints [deg]"), std::string::npos);
}

void CheckChangingValidityCheckerCancelsPosePlanning()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect()));
    const CartesianPose currentPose = controller.GetStateView().tcpPose;

    const Result begin = controller.BeginPosePlanning(currentPose);
    ASSERT_TRUE(static_cast<bool>(begin)) << begin.message;
    ASSERT_TRUE(controller.IsMotionPlanning());

    controller.SetJointStateValidityChecker([](const JointVector&)
    {
        return JointStateInvalidity::None;
    });

    EXPECT_FALSE(controller.IsMotionPlanning());
    const auto result = controller.TakeMotionPlanningResult();
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->code, ErrorCode::Cancelled);
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Idle);

    ASSERT_TRUE(static_cast<bool>(controller.BeginPosePlanning(currentPose)));
    for (std::size_t step = 0; step < 10000 && controller.IsMotionPlanning(); ++step)
        controller.AdvanceMotionPlanning(64);
    EXPECT_FALSE(controller.IsMotionPlanning());
    const auto noMotionResult = controller.TakeMotionPlanningResult();
    ASSERT_TRUE(noMotionResult.has_value());
    EXPECT_TRUE(static_cast<bool>(*noMotionResult));
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Idle);

    ASSERT_TRUE(static_cast<bool>(controller.BeginPosePlanning(currentPose)));
    controller.Disconnect();
    EXPECT_FALSE(controller.IsMotionPlanning());
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Disconnected);
    EXPECT_FALSE(controller.TakeMotionPlanningResult().has_value());
}

}


TEST(RobotMotion, MovePoseAndFailurePreservation) { CheckMovePoseAndFailurePreservation(); }
TEST(RobotMotion, JointStateValidityAndSelfCollisionDiagnostics)
{
    CheckJointStateValidityReasons();
    CheckSelfCollisionFailureIncludesPairDiagnostic();
    CheckChangingValidityCheckerCancelsPosePlanning();
}
