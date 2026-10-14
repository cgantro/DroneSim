#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>

using namespace grasplink::robotics;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;
using grasplink::robotics::kinematics::DampedLeastSquaresIk;

namespace grasplink::robotics::backends::simulation
{
struct SimRobotControllerTestAccess
{
    static bool ReorientForLinear(SimRobotController& controller, double availableSeconds)
    {
        return controller.ReorientForLinear(
            controller.linearPath_[controller.linearSegment_].joints, availableSeconds);
    }

    static void SetPosePath(SimRobotController& controller, std::vector<JointVector> points)
    {
        controller.posePathPoints_ = std::move(points);
        controller.posePathSegment_ = 1;
    }

    static std::size_t PosePathSize(const SimRobotController& controller)
    {
        return controller.posePathPoints_.size();
    }
};
} // namespace grasplink::robotics::backends::simulation

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

CartesianPose InterpolatePose(const CartesianPose& start, const CartesianPose& end, double fraction)
{
    CartesianPose result;
    for (std::size_t i = 0; i < 3; ++i)
        result.positionMeters[i] = start.positionMeters[i] + fraction * (end.positionMeters[i] - start.positionMeters[i]);

    // SLERP는 두 회전 사이를 일정한 각도 비율로 잇는 구면 보간이다. quaternion의 부호가 반대여도 같은 회전을 나타내므로 짧은 회전 방향을 선택한다.
    double dot = 0.0;
    for (std::size_t i = 0; i < 4; ++i)
        dot += start.orientationXyzw[i] * end.orientationXyzw[i];
    const double sign = dot < 0.0 ? -1.0 : 1.0;
    dot = std::clamp(std::abs(dot), 0.0, 1.0);
    const double angle = std::acos(dot);
    if (angle < 1e-10)
    {
        for (std::size_t i = 0; i < 4; ++i)
            result.orientationXyzw[i] = start.orientationXyzw[i] + fraction * (sign * end.orientationXyzw[i] - start.orientationXyzw[i]);
    }
    else
    {
        const double denominator = std::sin(angle);
        const double startWeight = std::sin((1.0 - fraction) * angle) / denominator;
        const double endWeight = std::sin(fraction * angle) / denominator * sign;
        for (std::size_t i = 0; i < 4; ++i)
            result.orientationXyzw[i] = startWeight * start.orientationXyzw[i] + endWeight * end.orientationXyzw[i];
    }
    double norm = 0.0;
    for (const double component : result.orientationXyzw)
        norm += component * component;
    norm = std::sqrt(norm);
    for (double& component : result.orientationXyzw)
        component /= norm;
    return result;
}

bool AdvanceUntilIdle(SimRobotController& controller, double dtSeconds)
{
    for (int i = 0; i < 10000 && controller.GetState().mode == RobotMode::Moving; ++i)
        controller.Update(dtSeconds);
    return controller.GetState().mode == RobotMode::Idle;
}

void CheckHomeSeedCanReachFoldedButValidPosture()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "home-seed controller connects without a collision filter";
    const JointVector reachableJoints{0.0, -0.8, -0.1, 0.5, -0.4, 0.2};
    const CartesianPose target = ik.EvaluateTcp(reachableJoints);
    const Result moveResult = controller.MovePose(target);
    ASSERT_TRUE((static_cast<bool>(moveResult))) << "controller runs bounded IK branch recovery when no environment validator is registered: " + moveResult.message;
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "motion reaches target within bounded updates";
    ASSERT_TRUE((PositionDistance(controller.GetState().tcpPose, target) < 2e-5)) << "home-seed recovery reaches the reachable target position";
    ASSERT_TRUE((OrientationDistance(controller.GetState().tcpPose, target) < 2e-4)) << "home-seed recovery reaches the reachable target orientation";
}

void CheckMovePoseAndFailurePreservation()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "controller connects";
    const JointVector seed{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    JointMoveCommand seedCommand;
    seedCommand.targetPositionRadians = seed;
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
    ASSERT_TRUE((afterRejectedPose.jointPositionRadians == beforeRejected.jointPositionRadians &&
        afterRejectedPose.jointVelocityRadiansPerSecond == beforeRejected.jointVelocityRadiansPerSecond &&
        afterRejectedPose.mode == beforeRejected.mode)) << "failed MovePose preserves the active motion state";
    const Result unreachable = controller.MoveLinear(
        {farTarget, 0.05, 0.2});
    ASSERT_TRUE((unreachable.code == ErrorCode::Unreachable))
        << "a line with a definitely unreachable endpoint is rejected early: " << unreachable.message;
    const RobotState afterRejected = controller.GetState();
    ASSERT_TRUE((afterRejected.jointPositionRadians == beforeRejected.jointPositionRadians &&
        afterRejected.jointVelocityRadiansPerSecond == beforeRejected.jointVelocityRadiansPerSecond &&
        afterRejected.mode == beforeRejected.mode)) << "failed replacement preserves the active motion state";

    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "motion reaches target within bounded updates";
    const RobotState reached = controller.GetState();
    ASSERT_TRUE((reached.tcpPoseValid)) << "simulation publishes model TCP feedback";
    ASSERT_TRUE((PositionDistance(reached.tcpPose, target) < 2e-5)) << "MovePose reaches requested TCP position";
    ASSERT_TRUE((OrientationDistance(reached.tcpPose, target) < 2e-4)) << "MovePose reaches requested TCP orientation";

    JointMoveCommand replacement;
    replacement.targetPositionRadians.assign(6, 0.0);
    replacement.targetPositionRadians[0] = -0.2;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(replacement)))) << "MoveJoint retarget succeeds";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.006)) << "motion reaches target within bounded updates";
    ASSERT_NEAR(controller.GetState().jointPositionRadians[0], -0.2, 1e-8) << "MoveJoint replaces a pose target";
}

void CheckEquivalentJointTargetUsesNearestLegalTurn()
{
    SimRobotController controller(models::hanwha::kHcr12a, models::Pose3{{0.04, 0.0, 0.0}, {}});
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "turn-continuity controller connects";

    constexpr double degreesToRadians = 3.14159265358979323846 / 180.0;
    JointVector start(6, 0.0);
    start[5] = 170.0 * degreesToRadians;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint({start, 1.0, 1.0})))) << "J6 reaches the positive 170 degree representation";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "motion reaches target within bounded updates";

    JointVector wrappedTarget = start;
    wrappedTarget[5] = -170.0 * degreesToRadians;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint({wrappedTarget, 1.0, 1.0})))) << "equivalent J6 target is accepted";
    int j6Ticks = 0;
    while (controller.GetState().mode == RobotMode::Moving && j6Ticks < 10000)
    {
        controller.Update(0.004);
        ++j6Ticks;
    }
    ASSERT_TRUE((controller.GetState().mode == RobotMode::Idle)) << "J6 reaches its equivalent target";
    ASSERT_NEAR(controller.GetState().jointPositionRadians[5], 190.0 * degreesToRadians, 1e-8) << "J6 uses the equivalent 190 degree representation that stays inside its plus/minus 360 degree limits";
    ASSERT_TRUE((j6Ticks < 100)) << "J6 crosses the 180 degree representation boundary by the short rotation";

    start[0] = 170.0 * degreesToRadians;
    start[5] = 0.0;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint({start, 1.0, 1.0})))) << "J1 reaches its positive 170 degree limit neighborhood";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "motion reaches target within bounded updates";
    wrappedTarget = start;
    wrappedTarget[0] = -170.0 * degreesToRadians;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint({wrappedTarget, 1.0, 1.0})))) << "J1 target within its hard limits is accepted";
    int j1Ticks = 0;
    while (controller.GetState().mode == RobotMode::Moving && j1Ticks < 10000)
    {
        controller.Update(0.004);
        ++j1Ticks;
    }
    ASSERT_TRUE((controller.GetState().mode == RobotMode::Idle)) << "J1 reaches its legal target representation";
    ASSERT_NEAR(controller.GetState().jointPositionRadians[0], -170.0 * degreesToRadians, 1e-8) << "J1 stays at the requested representation because its plus/minus 180 degree limits exclude 190 degrees";
    ASSERT_TRUE((j1Ticks > 500)) << "J1 does not cross its hard limit to take an unavailable short rotation";
}

void CheckExplicitWristUnwindPreservesZeroRepresentation()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "wrist unwind controller connects";

    JointMoveCommand nearNegativeLimit;
    nearNegativeLimit.targetPositionRadians.assign(6, 0.0);
    nearNegativeLimit.targetPositionRadians.back() = -314.5 * 3.14159265358979323846 / 180.0;
    nearNegativeLimit.preserveJointTurns = true;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(nearNegativeLimit)))) << "J6 accepts its explicit negative-turn representation";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.01)) << "motion reaches target within bounded updates";
    ASSERT_NEAR(controller.GetState().jointPositionRadians.back(), nearNegativeLimit.targetPositionRadians.back(), 1e-8) << "J6 reaches negative 314.5 degrees without changing to an equivalent turn";

    JointMoveCommand unwind;
    unwind.targetPositionRadians = controller.GetState().jointPositionRadians;
    unwind.targetPositionRadians.back() = 0.0;
    unwind.preserveJointTurns = true;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(unwind)))) << "explicit J6 unwind request is accepted";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.01)) << "motion reaches target within bounded updates";
    ASSERT_NEAR(controller.GetState().jointPositionRadians.back(), 0.0, 1e-8) << "J6 returns to the central zero degree representation instead of the nearby negative 360 degree turn";
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
    outsideLimit[0] = specification.joints[0].maxPositionRadians + 0.01;
    ASSERT_TRUE((ValidateJointState(specification, outsideLimit, environmentChecker) ==
        JointStateInvalidity::JointLimitViolation)) << "joint limits have a reason distinct from environment collision";
    ASSERT_TRUE((collisionChecks == 1)) << "invalid joint limits prevent the collision checker from running";

    JointVector nonFinite = joints;
    nonFinite[0] = std::numeric_limits<double>::infinity();
    ASSERT_TRUE((ValidateJointState(specification, nonFinite, environmentChecker) ==
        JointStateInvalidity::NonFinitePosition)) << "non-finite joint values are rejected before collision checks";
    ASSERT_TRUE((ValidateJointState(specification, JointVector{}, environmentChecker) ==
        JointStateInvalidity::JointCountMismatch)) << "joint count mismatch has its own invalidity reason";

    ASSERT_TRUE((MapJointStateInvalidity(JointStateInvalidity::EnvironmentCollision).code == ErrorCode::EnvironmentContact)) << "environment collision retains its existing controller error code";
    ASSERT_TRUE((MapJointStateInvalidity(JointStateInvalidity::SelfCollision).code == ErrorCode::SelfCollision)) << "self collision is not reported as environment contact";
    ASSERT_TRUE((MapJointStateInvalidity(JointStateInvalidity::AttachedObjectCollision).code == ErrorCode::AttachedObjectCollision)) << "attached-object collision is not reported as environment contact";
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

    JointVector target = controller.GetState().jointPositionRadians;
    target[0] = 0.2;
    const Result result = controller.MoveJoint({target, 1.0, 1.0});
    ASSERT_TRUE((result.code == ErrorCode::EnvironmentContact)) << "MoveJoint rejects a collision between valid endpoints";
    ASSERT_TRUE((collisionChecks >= 1)) << "MoveJoint checks joint states along the path";
    ASSERT_TRUE((firstCollisionCheckedPosition > 0.0 && firstCollisionCheckedPosition < target[0]))
        << "MoveJoint checks an intermediate state before the target";
    ASSERT_TRUE((controller.GetState().mode == RobotMode::Idle)) << "rejected path leaves the controller idle";
    ASSERT_NEAR(controller.GetState().jointPositionRadians[0], 0.0, 1e-12) << "rejected path leaves the current joints unchanged";
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

    JointVector target = controller.GetStateView().jointPositionRadians;
    target[0] = 0.2;
    const Result result = controller.MoveJoint({target, 1.0, 1.0});
    EXPECT_EQ(result.code, ErrorCode::SelfCollision);
    EXPECT_NE(result.message.find("Link2 / Link5"), std::string::npos);
    EXPECT_NE(result.message.find("joints [deg]"), std::string::npos);
}

void CheckMoveJointExecutesTheValidatedJointPath()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "controller connects before synchronized joint motion";
    std::size_t pathChecks = 0;
    controller.SetJointStateValidityChecker([&](const JointVector& joints)
    {
        ++pathChecks;
        return std::abs(joints[2] - 2.0 * joints[0]) <= 1e-9 ?
            JointStateInvalidity::None : JointStateInvalidity::EnvironmentCollision;
    });
    JointVector target = controller.GetStateView().jointPositionRadians;
    target[0] = 0.4;
    target[2] = 0.8;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint({target, 1.0, 1.0}))) << "controller accepts a collision-free joint path";
    ASSERT_GT(pathChecks, 0u) << "registered validity checker accepts points on the joint-space line";

    bool sawIntermediateState = false;
    for (int tick = 0; tick < 10000 && controller.GetStateView().mode == RobotMode::Moving; ++tick)
    {
        controller.Update(0.004);
        const RobotState state = controller.GetStateView();
        ASSERT_NEAR(state.jointPositionRadians[2], 2.0 * state.jointPositionRadians[0], 1e-10)
            << "all joints use the same normalized progress as the validated joint-space line";
        for (std::size_t joint = 0; joint < state.jointVelocityRadiansPerSecond.size(); ++joint)
            ASSERT_LE(std::abs(state.jointVelocityRadiansPerSecond[joint]),
                models::hanwha::kHcr12a.joints[joint].maxVelocityRadiansPerSecond + 1e-9)
                << "synchronized joint motion respects each joint speed limit";
        sawIntermediateState = sawIntermediateState || state.jointPositionRadians[0] > 0.0;
    }

    ASSERT_TRUE(sawIntermediateState) << "joint motion exposes intermediate synchronized states";
    ASSERT_EQ(controller.GetStateView().mode, RobotMode::Idle) << "synchronized joint motion reaches its endpoint";
    ASSERT_NEAR(controller.GetStateView().jointPositionRadians[0], target[0], 1e-12)
        << "synchronized motion reaches the requested J1 target";
    ASSERT_NEAR(controller.GetStateView().jointPositionRadians[2], target[2], 1e-12)
        << "synchronized motion reaches the requested J3 target";
}

void CheckMoveJointAccelerationAndRetargetContinuity()
{
    using grasplink::robotics::backends::simulation::SimRobotController;
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "acceleration fixture connects";

    JointVector target(6, 0.0);
    target[0] = 1.0;
    constexpr double accelerationScale = 0.5;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint({target, 1.0, accelerationScale})))
        << "joint trajectory accepts its velocity and acceleration scales";

    constexpr double dt = 0.004;
    const double accelerationLimit = models::hanwha::kHcr12a.joints[0].maxVelocityRadiansPerSecond /
        0.20 * accelerationScale;
    double previousVelocity = 0.0;
    for (int tick = 0; tick < 1000 && controller.GetStateView().mode == RobotMode::Moving; ++tick)
    {
        controller.Update(dt);
        const double velocity = controller.GetStateView().jointVelocityRadiansPerSecond[0];
        ASSERT_LE(std::abs(velocity), models::hanwha::kHcr12a.joints[0].maxVelocityRadiansPerSecond + 1e-9)
            << "joint trajectory respects its maximum speed";
        ASSERT_LE(std::abs(velocity - previousVelocity), accelerationLimit * dt + 1e-8)
            << "joint trajectory applies the simulation acceleration policy";
        previousVelocity = velocity;
    }
    ASSERT_EQ(controller.GetStateView().mode, RobotMode::Idle) << "accelerated move reaches its endpoint";
    ASSERT_NEAR(controller.GetStateView().jointPositionRadians[0], target[0], 1e-12)
        << "accelerated move reaches the exact requested angle";

    target[0] = -0.5;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint({target, 1.0, accelerationScale})))
        << "second move starts before retarget continuity is checked";
    for (int tick = 0; tick < 20; ++tick)
        controller.Update(dt);
    const double velocityBeforeRetarget = controller.GetStateView().jointVelocityRadiansPerSecond[0];
    target[0] = 0.4;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint({target, 1.0, accelerationScale})))
        << "retarget is accepted after its post-braking path validates";
    ASSERT_NEAR(controller.GetStateView().jointVelocityRadiansPerSecond[0], velocityBeforeRetarget, 1e-12)
        << "accepting a retarget does not instantaneously change velocity";
    previousVelocity = velocityBeforeRetarget;
    for (int tick = 0; tick < 2000 && controller.GetStateView().mode == RobotMode::Moving; ++tick)
    {
        controller.Update(dt);
        const double velocity = controller.GetStateView().jointVelocityRadiansPerSecond[0];
        ASSERT_LE(std::abs(velocity - previousVelocity), accelerationLimit * dt + 1e-8)
            << "braking and the replacement trajectory preserve the acceleration bound";
        previousVelocity = velocity;
    }
    ASSERT_EQ(controller.GetStateView().mode, RobotMode::Idle) << "retargeted trajectory reaches its endpoint";
    ASSERT_NEAR(controller.GetStateView().jointPositionRadians[0], target[0], 1e-12)
        << "retargeted trajectory reaches the replacement angle";
}

void CheckShortJointMoveAndRejectedRetarget()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "short-move fixture connects";
    JointVector target(6, 0.0);
    target[0] = 0.005;
    const double velocityLimit = models::hanwha::kHcr12a.joints[0].maxVelocityRadiansPerSecond;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint({target, 0.1, 0.5})))
        << "short triangular move is accepted with a reduced speed cap";
    double maximumObservedVelocity = 0.0;
    for (int tick = 0; tick < 1000 && controller.GetStateView().mode == RobotMode::Moving; ++tick)
    {
        controller.Update(0.004);
        maximumObservedVelocity = std::max(maximumObservedVelocity,
            std::abs(controller.GetStateView().jointVelocityRadiansPerSecond[0]));
    }
    const double accelerationLimit = velocityLimit / 0.20 * 0.5;
    ASSERT_LT(maximumObservedVelocity, velocityLimit * 0.1)
        << "short travel uses a triangular profile and does not reach its speed ceiling";
    ASSERT_NEAR(maximumObservedVelocity, std::sqrt(accelerationLimit * target[0]),
        accelerationLimit * 0.004 + 1e-8)
        << "peak speed follows the distance and acceleration limit, independent of velocityScale";
    ASSERT_EQ(controller.GetStateView().mode, RobotMode::Idle) << "short triangular move completes";

    controller.SetJointStateValidityChecker([](const JointVector& joints)
    {
        return joints[0] < -1e-9 ? JointStateInvalidity::EnvironmentCollision :
            JointStateInvalidity::None;
    });
    target[0] = 0.8;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint({target, 1.0, 0.5})))
        << "validated positive joint path starts";
    controller.Update(0.08);
    const RobotState beforeRejectedRetarget = controller.GetState();
    JointVector rejectedTarget(6, 0.0);
    rejectedTarget[0] = -0.5;
    const Result rejected = controller.MoveJoint({rejectedTarget, 1.0, 0.5});
    ASSERT_EQ(rejected.code, ErrorCode::EnvironmentContact)
        << "retarget whose post-braking path crosses an invalid state is rejected";
    const RobotState afterRejectedRetarget = controller.GetState();
    ASSERT_EQ(afterRejectedRetarget.mode, beforeRejectedRetarget.mode)
        << "rejected retarget preserves the active motion";
    ASSERT_EQ(afterRejectedRetarget.jointPositionRadians, beforeRejectedRetarget.jointPositionRadians)
        << "rejected retarget does not alter the current position";
    ASSERT_EQ(afterRejectedRetarget.jointVelocityRadiansPerSecond,
        beforeRejectedRetarget.jointVelocityRadiansPerSecond)
        << "rejected retarget does not alter the current velocity";
}

void CheckLinearReorientationRejectsInvalidJointPath()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk inverse(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "reorientation fixture connects";
    bool rejectRuntimeCandidate = false;
    std::size_t checkerCalls = 0;
    controller.SetJointStateValidityChecker([&](const JointVector&)
    {
        ++checkerCalls;
        return rejectRuntimeCandidate ? JointStateInvalidity::EnvironmentCollision : JointStateInvalidity::None;
    });
    const JointVector targetJoints{0.15, -0.4, 0.25, 0.1, -0.2, 0.1};
    LinearMoveCommand command;
    command.targetPose = inverse.EvaluateTcp(targetJoints);
    command.maxLinearVelocityMetersPerSecond = 0.1;
    command.maxAngularVelocityRadiansPerSecond = 0.5;
    ASSERT_TRUE(static_cast<bool>(controller.MoveLinear(command))) << "reorientation fixture plans with a permissive checker";
    ASSERT_GT(checkerCalls, 0u) << "planner validates the accepted line path";
    const CartesianPose pathStart = inverse.EvaluateTcp(controller.GetStateView().jointPositionRadians);
    const std::size_t checksAfterPlanning = checkerCalls;
    const bool validReorientation = grasplink::robotics::backends::simulation::SimRobotControllerTestAccess::ReorientForLinear(
        controller, 10.0);
    ASSERT_TRUE(validReorientation) << "runtime reorientation accepts a valid joint path that preserves the current TCP line pose";
    ASSERT_GT(checkerCalls, checksAfterPlanning) << "runtime reorientation checks its candidate joint path";
    ASSERT_LT(PositionDistance(controller.GetStateView().tcpPose, pathStart), 1e-6)
        << "accepted reorientation remains at the current Cartesian path pose";
    const JointVector originalJoints = controller.GetStateView().jointPositionRadians;
    rejectRuntimeCandidate = true;

    const bool accepted = grasplink::robotics::backends::simulation::SimRobotControllerTestAccess::ReorientForLinear(
        controller, 10.0);

    ASSERT_FALSE(accepted) << "runtime reorientation rejects candidates blocked by the registered checker";
    ASSERT_GT(checkerCalls, 1u) << "runtime reorientation checks candidate paths with the registered checker";
    ASSERT_EQ(controller.GetStateView().jointPositionRadians, originalJoints)
        << "rejected reorientation leaves the current joint state unchanged";
}

void CheckEnvironmentCollisionCanBeRetried()
{
    SimRobotController controller(models::hanwha::kHcr12a, models::Pose3{{0.04, 0.0, 0.0}, {}});
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "controller connects before a collision rollback";
    const JointVector safePosition = controller.GetState().jointPositionRadians;
    ASSERT_TRUE((controller.RestoreCollisionSafeState(safePosition))) << "collision rollback restores the last safe joints";
    const RobotState stopped = controller.GetState();
    ASSERT_TRUE((stopped.mode == RobotMode::Idle)) << "collision rollback leaves a stopped controller that accepts recovery motion";
    ASSERT_TRUE((stopped.errorCode == ErrorCode::EnvironmentContact)) << "collision rollback preserves the reason for stopping";
    ASSERT_TRUE((controller.RestoreCollisionSafeState(safePosition, ErrorCode::SelfCollision)))
        << "rollback also preserves self-collision as a distinct stop cause";
    EXPECT_EQ(controller.GetState().errorCode, ErrorCode::SelfCollision);
    JointVector recovery = safePosition;
    recovery[0] += 0.05;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint({recovery, 1.0, 1.0})))) << "the controller accepts a new retreat command after collision rollback";
}

void CheckCollisionAwareIkSelectsAnotherBranch()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController reference(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(reference.Connect()))) << "reference controller connects";
    const JointVector start{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    ASSERT_TRUE((static_cast<bool>(reference.MoveJoint({start, 1.0, 1.0})))) << "reference controller reaches regular seed";
    ASSERT_TRUE(AdvanceUntilIdle(reference, 0.004)) << "motion reaches target within bounded updates";
    const CartesianPose target = ik.EvaluateTcp({0.25, -0.37, 0.34, 0.17, -0.23, 0.11});
    ASSERT_TRUE((static_cast<bool>(reference.MovePose(target)))) << "reference IK reaches target without obstacle filter";
    ASSERT_TRUE(AdvanceUntilIdle(reference, 0.004)) << "motion reaches target within bounded updates";
    const JointVector blockedBranch = reference.GetState().jointPositionRadians;

    SimRobotController constrained(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(constrained.Connect()))) << "collision-aware controller connects";
    ASSERT_TRUE((static_cast<bool>(constrained.MoveJoint({start, 1.0, 1.0})))) << "collision-aware controller reaches same seed";
    ASSERT_TRUE(AdvanceUntilIdle(constrained, 0.004)) << "motion reaches target within bounded updates";
    constrained.SetJointStateValidityChecker([&](const JointVector& candidate)
    {
        double squaredDistance = 0.0;
        for (std::size_t joint = 0; joint < candidate.size(); ++joint)
        {
            const double difference = candidate[joint] - blockedBranch[joint];
            squaredDistance += difference * difference;
        }
        return squaredDistance > 0.02 * 0.02 ? JointStateInvalidity::None :
            JointStateInvalidity::EnvironmentCollision;
    });

    ASSERT_TRUE((static_cast<bool>(constrained.MovePose(target)))) << "IK selects a different collision-free branch";
    ASSERT_TRUE(AdvanceUntilIdle(constrained, 0.004)) << "motion reaches target within bounded updates";
    const auto reached = constrained.GetState();
    ASSERT_TRUE((PositionDistance(reached.tcpPose, target) < 2e-5)) << "alternate branch reaches target TCP position";
    ASSERT_TRUE((OrientationDistance(reached.tcpPose, target) < 2e-4)) << "alternate branch reaches target TCP orientation";
    double branchDifferenceSquared = 0.0;
    for (std::size_t joint = 0; joint < blockedBranch.size(); ++joint)
    {
        const double difference = reached.jointPositionRadians[joint] - blockedBranch[joint];
        branchDifferenceSquared += difference * difference;
    }
    ASSERT_TRUE((branchDifferenceSquared > 0.02 * 0.02)) << "selected IK solution avoids the rejected joint-space region";

    SimRobotController blocked(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(blocked.Connect()))) << "environment-blocked controller connects";
    blocked.SetJointStateValidityChecker([](const JointVector&)
    {
        return JointStateInvalidity::EnvironmentCollision;
    });
    const Result blockedPose = blocked.MovePose(target);
    ASSERT_TRUE((blockedPose.code == ErrorCode::EnvironmentContact)) << "MovePose preserves the environment collision reason from sampled joint states";

    LinearPathMoveCommand blockedPath;
    blockedPath.targetPoses.push_back(target);
    const Result blockedLinearPath = blocked.MoveLinearPath(blockedPath);
    ASSERT_TRUE((blockedLinearPath.code == ErrorCode::EnvironmentContact)) << "MoveLinearPath preserves the environment collision reason from path samples";
}

void CheckLinearPathAndVelocityBounds(double dtSeconds)
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "linear controller connects";
    JointMoveCommand seedCommand;
    seedCommand.targetPositionRadians = {0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(seedCommand)))) << "joint command prepares a regular linear-path seed";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "motion reaches target within bounded updates";
    const CartesianPose start = controller.GetState().tcpPose;
    const CartesianPose target = ik.EvaluateTcp({0.25, -0.37, 0.34, 0.17, -0.23, 0.11});
    LinearMoveCommand command;
    command.targetPose = target;
    command.maxLinearVelocityMetersPerSecond = 0.04;
    command.maxAngularVelocityRadiansPerSecond = 0.25;
    ASSERT_TRUE((static_cast<bool>(controller.MoveLinear(command)))) << "MoveLinear accepts the reachable TCP path";

    CartesianPose previous = controller.GetState().tcpPose;
    double previousProgress = 0.0;
    double peakLinearSpeed = 0.0;
    bool sawIntermediatePose = false;
    for (int i = 0; i < 10000 && controller.GetState().mode == RobotMode::Moving; ++i)
    {
        controller.Update(dtSeconds);
        const RobotState state = controller.GetState();
        ASSERT_TRUE((state.tcpPoseValid)) << "TCP feedback stays valid during linear motion";
        const double movedMeters = PositionDistance(previous, state.tcpPose);
        const double turnedRadians = OrientationDistance(previous, state.tcpPose);
        peakLinearSpeed = std::max(peakLinearSpeed, movedMeters / dtSeconds);
        ASSERT_TRUE((movedMeters / dtSeconds <= command.maxLinearVelocityMetersPerSecond + 1e-7)) << "per-tick TCP translation stays under its requested speed";
        if (i == 0)
            ASSERT_TRUE((movedMeters / dtSeconds <= command.maxLinearAccelerationMetersPerSecondSquared * dtSeconds + 1e-5)) << "linear motion accelerates from rest within its requested acceleration";
        ASSERT_TRUE((turnedRadians / dtSeconds <= command.maxAngularVelocityRadiansPerSecond + 1e-6)) << "per-tick TCP rotation stays under its requested speed";
        if (i == 0)
            ASSERT_TRUE((turnedRadians / dtSeconds <= command.maxAngularAccelerationRadiansPerSecondSquared * dtSeconds + 1e-5)) << "angular motion accelerates from rest within its requested acceleration";
        for (std::size_t joint = 0; joint < state.jointVelocityRadiansPerSecond.size(); ++joint)
        {
            const double velocity = state.jointVelocityRadiansPerSecond[joint];
            ASSERT_TRUE((std::isfinite(velocity) &&
                std::abs(velocity) <= models::hanwha::kHcr12a.joints[joint].maxVelocityRadiansPerSecond + 1e-7)) << "reported joint velocity stays under the model limit";
        }

        const double dx = target.positionMeters[0] - start.positionMeters[0];
        const double dy = target.positionMeters[1] - start.positionMeters[1];
        const double dz = target.positionMeters[2] - start.positionMeters[2];
        const double lengthSquared = dx * dx + dy * dy + dz * dz;
        double progress = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis)
            progress += (state.tcpPose.positionMeters[axis] - start.positionMeters[axis]) *
                (target.positionMeters[axis] - start.positionMeters[axis]);
        progress = std::clamp(progress / lengthSquared, 0.0, 1.0);
        const CartesianPose onPath = InterpolatePose(start, target, progress);
        ASSERT_TRUE((PositionDistance(onPath, state.tcpPose) < 1e-3)) << "interpolated joint path stays within 1 mm of the requested TCP line";
        ASSERT_TRUE((OrientationDistance(onPath, state.tcpPose) < 1e-3)) << "interpolated joint path stays within 1 mrad of the requested TCP rotation";
        ASSERT_TRUE((progress + 1e-8 >= previousProgress)) << "linear path progress does not move backward";
        sawIntermediatePose = sawIntermediatePose || (progress > 0.01 && progress < 0.99);
        previousProgress = progress;
        previous = state.tcpPose;
    }

    ASSERT_TRUE((controller.GetState().mode == RobotMode::Idle)) << "linear path reaches its endpoint";
    ASSERT_TRUE((sawIntermediatePose)) << "linear motion exposes intermediate TCP states";
    ASSERT_TRUE((peakLinearSpeed >= command.maxLinearVelocityMetersPerSecond * 0.8)) << "linear motion reaches its requested top speed between acceleration ramps";
    ASSERT_TRUE((PositionDistance(controller.GetState().tcpPose, target) < 2e-5)) << "linear path reaches target position";
    ASSERT_TRUE((OrientationDistance(controller.GetState().tcpPose, target) < 2e-4)) << "linear path reaches target orientation";

    // 회전만 요청하면 TCP 위치는 고정하고 공구 방향만 바뀌어야 한다. 관절 해는 여러 개일 수 있으므로 관절각 대신 실제 TCP 자세를 검사한다.
    const CartesianPose rotationStart = controller.GetState().tcpPose;
    CartesianPose rotationTarget = rotationStart;
    const double halfAngle = 0.02;
    const double cosine = std::cos(halfAngle);
    const double sine = std::sin(halfAngle);
    const auto old = rotationStart.orientationXyzw;
    rotationTarget.orientationXyzw = {
        cosine * old[0] - sine * old[1],
        cosine * old[1] + sine * old[0],
        cosine * old[2] + sine * old[3],
        cosine * old[3] - sine * old[2]};
    command.targetPose = rotationTarget;
    ASSERT_TRUE((static_cast<bool>(controller.MoveLinear(command)))) << "rotation-only TCP path is accepted";
    double peakRotationSpeed = 0.0;
    for (int i = 0; i < 10000 && controller.GetState().mode == RobotMode::Moving; ++i)
    {
        const CartesianPose previousRotationPose = controller.GetState().tcpPose;
        controller.Update(dtSeconds);
        const CartesianPose currentRotationPose = controller.GetState().tcpPose;
        peakRotationSpeed = std::max(peakRotationSpeed,
            OrientationDistance(previousRotationPose, currentRotationPose) / dtSeconds);
        ASSERT_TRUE((PositionDistance(rotationStart, currentRotationPose) < 1e-3)) << "rotation-only interpolated path keeps TCP position within 1 mm";
        ASSERT_TRUE((OrientationDistance(previousRotationPose, currentRotationPose) / dtSeconds <=
            command.maxAngularVelocityRadiansPerSecond + 1e-6)) << "rotation-only path respects angular speed";
    }
    ASSERT_TRUE((controller.GetState().mode == RobotMode::Idle)) << "rotation-only path completes";
    ASSERT_TRUE((peakRotationSpeed >= command.maxAngularVelocityRadiansPerSecond * 0.8)) << "rotation-only motion reaches its requested angular top speed";
    ASSERT_TRUE((PositionDistance(rotationStart, controller.GetState().tcpPose) < 1e-3)) << "rotation-only endpoint preserves TCP position within 1 mm";
    ASSERT_TRUE((OrientationDistance(rotationTarget, controller.GetState().tcpPose) < 2e-5)) << "rotation-only endpoint reaches requested orientation";
    command.targetPose = controller.GetState().tcpPose;
    ASSERT_TRUE((static_cast<bool>(controller.MoveLinear(command)))) << "same-pose linear request is accepted";
    ASSERT_TRUE((controller.GetState().mode == RobotMode::Idle)) << "same-pose linear request does not create unnecessary motion";
}

void CheckMultiWaypointUsesOneMotionProfile()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    const JointVector startJoints{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    const CartesianPose firstTarget = ik.EvaluateTcp({0.25, -0.37, 0.34, 0.17, -0.23, 0.11});
    const CartesianPose finalTarget = ik.EvaluateTcp({0.27, -0.4, 0.36, 0.16, -0.21, 0.12});
    const double dt = 0.004;

    SimRobotController combined(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(combined.Connect()))) << "combined-path controller connects";
    ASSERT_TRUE((static_cast<bool>(combined.MoveJoint({startJoints, 1.0, 1.0})))) << "combined path reaches its seed posture";
    ASSERT_TRUE(AdvanceUntilIdle(combined, dt)) << "motion reaches target within bounded updates";
    LinearPathMoveCommand path;
    path.targetPoses = {firstTarget, finalTarget};
    path.maxLinearVelocityMetersPerSecond = 0.05;
    path.maxAngularVelocityRadiansPerSecond = 0.5;
    path.maxLinearAccelerationMetersPerSecondSquared = 0.2;
    path.maxAngularAccelerationRadiansPerSecondSquared = 2.0;
    ASSERT_TRUE((static_cast<bool>(combined.MoveLinearPath(path)))) << "multi-waypoint path is accepted";
    int combinedTicks = 0;
    while (combined.GetState().mode == RobotMode::Moving && combinedTicks < 10000)
    {
        combined.Update(dt);
        ++combinedTicks;
    }
    ASSERT_TRUE((combined.GetState().mode == RobotMode::Idle)) << "multi-waypoint path completes";
    ASSERT_TRUE((PositionDistance(combined.GetState().tcpPose, finalTarget) < 2e-5)) << "multi-waypoint path reaches its final target";

    SimRobotController split(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(split.Connect()))) << "split-path controller connects";
    ASSERT_TRUE((static_cast<bool>(split.MoveJoint({startJoints, 1.0, 1.0})))) << "split path reaches its seed posture";
    ASSERT_TRUE(AdvanceUntilIdle(split, dt)) << "motion reaches target within bounded updates";
    LinearMoveCommand segment;
    segment.maxLinearVelocityMetersPerSecond = path.maxLinearVelocityMetersPerSecond;
    segment.maxAngularVelocityRadiansPerSecond = path.maxAngularVelocityRadiansPerSecond;
    segment.maxLinearAccelerationMetersPerSecondSquared = path.maxLinearAccelerationMetersPerSecondSquared;
    segment.maxAngularAccelerationRadiansPerSecondSquared = path.maxAngularAccelerationRadiansPerSecondSquared;
    segment.targetPose = firstTarget;
    ASSERT_TRUE((static_cast<bool>(split.MoveLinear(segment)))) << "first split segment is accepted";
    int splitTicks = 0;
    while (split.GetState().mode == RobotMode::Moving && splitTicks < 10000)
    {
        split.Update(dt);
        ++splitTicks;
    }
    segment.targetPose = finalTarget;
    ASSERT_TRUE((static_cast<bool>(split.MoveLinear(segment)))) << "second split segment is accepted";
    while (split.GetState().mode == RobotMode::Moving && splitTicks < 10000)
    {
        split.Update(dt);
        ++splitTicks;
    }
    ASSERT_TRUE((split.GetState().mode == RobotMode::Idle)) << "split segments complete";
    ASSERT_TRUE((combinedTicks < splitTicks)) << "one shared acceleration profile avoids stopping and restarting at the intermediate waypoint";
}

void CheckJointLimitsSetLinearPathSpeed()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    const CartesianPose target = ik.EvaluateTcp({0.24, -0.37, 0.34, 0.18, -0.23, 0.11});
    const auto measureTicks = [&](double linearCeiling, double angularCeiling)
    {
        SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
        if (!controller.Connect())
        {
            ADD_FAILURE() << "joint-limited path controller connects";
            return -1;
        }
        JointMoveCommand seed;
        seed.targetPositionRadians = {0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
        if (!controller.MoveJoint(seed))
        {
            ADD_FAILURE() << "joint-limited path seed starts";
            return -1;
        }
        if (!AdvanceUntilIdle(controller, 0.004))
        {
            ADD_FAILURE() << "motion reaches target within bounded updates";
            return -1;
        }
        LinearMoveCommand command;
        command.targetPose = target;
        command.maxLinearVelocityMetersPerSecond = linearCeiling;
        command.maxAngularVelocityRadiansPerSecond = angularCeiling;
        if (!controller.MoveLinear(command))
        {
            ADD_FAILURE() << "joint-limited test path is accepted";
            return -1;
        }
        int ticks = 0;
        while (ticks < 10000 && controller.GetState().mode == RobotMode::Moving)
        {
            controller.Update(0.004);
            for (std::size_t joint = 0; joint < controller.GetStateView().jointVelocityRadiansPerSecond.size(); ++joint)
                EXPECT_TRUE((std::abs(controller.GetStateView().jointVelocityRadiansPerSecond[joint]) <=
                    models::hanwha::kHcr12a.joints[joint].maxVelocityRadiansPerSecond + 1e-7)) << "joint-limited path never exceeds its model velocity";
            ++ticks;
        }
        if (controller.GetState().mode != RobotMode::Idle)
        {
            ADD_FAILURE() << "joint-limited test path finishes";
            return -1;
        }
        return ticks;
    };

    const int restrictiveTicks = measureTicks(0.04, 0.25);
    if (restrictiveTicks < 0)
        return;
    const int jointLimitedTicks = measureTicks(2.0, 8.0);
    if (jointLimitedTicks < 0)
        return;
    ASSERT_TRUE((jointLimitedTicks < restrictiveTicks)) << "raising the unrelated TCP ceiling lets the model joint limits determine the faster path duration";
}

void CheckLinearPathFromHomeNearWristSingularity()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "home-singularity fixture connects";
    const CartesianPose start = controller.GetState().tcpPose;
    const CartesianPose target = ik.EvaluateTcp({0.22, -0.1, 0.08, 0.1, -0.06, 0.12});
    LinearMoveCommand command;
    command.targetPose = target;
    command.maxLinearVelocityMetersPerSecond = 0.04;
    command.maxAngularVelocityRadiansPerSecond = 0.25;
    const Result accepted = controller.MoveLinear(command);
    ASSERT_TRUE((static_cast<bool>(accepted))) << "home wrist-singularity path is accepted: " + accepted.message;

    CartesianPose previous = start;
    for (int i = 0; i < 10000 && controller.GetState().mode == RobotMode::Moving; ++i)
    {
        controller.Update(0.004);
        const RobotState state = controller.GetState();
        const double dx = target.positionMeters[0] - start.positionMeters[0];
        const double dy = target.positionMeters[1] - start.positionMeters[1];
        const double dz = target.positionMeters[2] - start.positionMeters[2];
        const double lengthSquared = dx * dx + dy * dy + dz * dz;
        double progress = 0.0;
        for (std::size_t axis = 0; axis < 3; ++axis)
            progress += (state.tcpPose.positionMeters[axis] - start.positionMeters[axis]) *
                (target.positionMeters[axis] - start.positionMeters[axis]);
        progress = std::clamp(progress / lengthSquared, 0.0, 1.0);
        const std::string faultDetails =
            " at tick=" + std::to_string(i) +
            " errorCode=" + std::to_string(static_cast<int>(state.errorCode)) +
            " q4=" + std::to_string(state.jointPositionRadians[3]) +
            " q5=" + std::to_string(state.jointPositionRadians[4]) +
            " q6=" + std::to_string(state.jointPositionRadians[5]) +
            " tcp=(" + std::to_string(state.tcpPose.positionMeters[0]) + "," +
                std::to_string(state.tcpPose.positionMeters[1]) + "," +
                std::to_string(state.tcpPose.positionMeters[2]) + ")" +
            " progress=" + std::to_string(progress);
        ASSERT_TRUE((state.mode != RobotMode::Fault)) << "home wrist-singularity path avoids a runtime IK fault" + faultDetails;
        ASSERT_TRUE((PositionDistance(previous, state.tcpPose) / 0.004 <=
            command.maxLinearVelocityMetersPerSecond + 1e-7)) << "home wrist-singularity path respects TCP linear speed";
        ASSERT_TRUE((OrientationDistance(previous, state.tcpPose) / 0.004 <=
            command.maxAngularVelocityRadiansPerSecond + 1e-6)) << "home wrist-singularity path respects TCP angular speed";

        ASSERT_TRUE((PositionDistance(InterpolatePose(start, target, progress), state.tcpPose) < 1e-3)) << "home wrist-singularity TCP stays within 1 mm of the requested line";
        previous = state.tcpPose;
    }

    const RobotState finalState = controller.GetState();
    const double dx = target.positionMeters[0] - start.positionMeters[0];
    const double dy = target.positionMeters[1] - start.positionMeters[1];
    const double dz = target.positionMeters[2] - start.positionMeters[2];
    const double lengthSquared = dx * dx + dy * dy + dz * dz;
    double finalProgress = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis)
        finalProgress += (finalState.tcpPose.positionMeters[axis] - start.positionMeters[axis]) *
            (target.positionMeters[axis] - start.positionMeters[axis]);
    finalProgress = std::clamp(finalProgress / lengthSquared, 0.0, 1.0);
    std::string finalDiagnostics =
        " final mode=" + std::to_string(static_cast<unsigned>(finalState.mode)) +
        " errorCode=" + std::to_string(static_cast<int>(finalState.errorCode)) + " q=[";
    for (std::size_t joint = 0; joint < finalState.jointPositionRadians.size(); ++joint)
    {
        if (joint != 0)
            finalDiagnostics += ",";
        finalDiagnostics += std::to_string(finalState.jointPositionRadians[joint]);
    }
    finalDiagnostics += "] tcp=(" + std::to_string(finalState.tcpPose.positionMeters[0]) + "," +
        std::to_string(finalState.tcpPose.positionMeters[1]) + "," +
        std::to_string(finalState.tcpPose.positionMeters[2]) + ") progress=" +
        std::to_string(finalProgress) + " positionErrorMeters=" +
        std::to_string(PositionDistance(finalState.tcpPose, target)) + " orientationErrorRadians=" +
        std::to_string(OrientationDistance(finalState.tcpPose, target));
    ASSERT_TRUE((finalState.mode == RobotMode::Idle)) << "home wrist-singularity path reaches its target" + finalDiagnostics;
    ASSERT_TRUE((PositionDistance(finalState.tcpPose, target) < 2e-5)) << "home wrist-singularity path reaches TCP position" + finalDiagnostics;
    ASSERT_TRUE((OrientationDistance(finalState.tcpPose, target) < 2e-4)) << "home wrist-singularity path reaches TCP orientation" + finalDiagnostics;
}

void CheckLinearPathKeepsJ1BranchContinuous()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "branch-continuity controller connects at home";
    const double initialJ1 = controller.GetStateView().jointPositionRadians[0];
    const CartesianPose target = ik.EvaluateTcp({0.0, -0.8, -0.1, 0.5, -0.4, 0.2});
    LinearMoveCommand command;
    command.targetPose = target;
    command.maxLinearVelocityMetersPerSecond = 0.25;
    command.maxAngularVelocityRadiansPerSecond = 1.0;
    ASSERT_TRUE((static_cast<bool>(controller.MoveLinear(command)))) << "linear path accepts a reachable target that needs an alternate IK posture";

    constexpr double dtSeconds = 0.004;
    double previousJ1 = initialJ1;
    double totalJ1Travel = 0.0;
    double peakJ1Step = 0.0;
    for (int tick = 0; tick < 10000 && controller.GetStateView().mode == RobotMode::Moving; ++tick)
    {
        controller.Update(dtSeconds);
        const double currentJ1 = controller.GetStateView().jointPositionRadians[0];
        const double change = std::abs(currentJ1 - previousJ1);
        totalJ1Travel += change;
        peakJ1Step = std::max(peakJ1Step, change);
        previousJ1 = currentJ1;
    }
    const RobotState finalState = controller.GetState();
    ASSERT_TRUE((finalState.mode == RobotMode::Idle)) << "alternate-posture linear path completes without stalling";
    ASSERT_TRUE((peakJ1Step <= models::hanwha::kHcr12a.joints[0].maxVelocityRadiansPerSecond * dtSeconds + 1e-8)) << "J1 never jumps farther in one tick than its speed limit allows";
    ASSERT_TRUE((totalJ1Travel <= std::abs(finalState.jointPositionRadians[0] - initialJ1) + 0.5)) << "J1 follows a nearby branch instead of accumulating a long opposite rotation";
    ASSERT_TRUE((PositionDistance(finalState.tcpPose, target) < 2e-5)) << "continuous J1 branch still reaches the target TCP position";
    ASSERT_TRUE((OrientationDistance(finalState.tcpPose, target) < 2e-4)) << "continuous J1 branch still reaches the target TCP orientation";
}

void CheckLinearPathExecutesRefinedTcpLine()
{
    const JointVector targetJoints{0.55, -0.8, 0.7, 0.45, -0.6, 0.3};
    DampedLeastSquaresIk inverse(models::hanwha::kHcr12a);
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "seed-restart line-path fixture connects at home";

    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp(targetJoints));
    command.maxLinearVelocityMetersPerSecond = 2.0;
    command.maxAngularVelocityRadiansPerSecond = 8.0;
    command.maxLinearAccelerationMetersPerSecondSquared = 30.0;
    command.maxAngularAccelerationRadiansPerSecondSquared = 120.0;
    const Result accepted = controller.MoveLinearPath(command);
    ASSERT_TRUE((static_cast<bool>(accepted))) << "the planner accepts a reachable TCP line with refined joint samples: " + accepted.message;
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "the refined full path finishes without a runtime fault";
    const RobotState finalState = controller.GetState();
    ASSERT_TRUE(finalState.tcpPoseValid) << "completed line path reports a valid TCP pose";
    ASSERT_LT(PositionDistance(finalState.tcpPose, command.targetPoses.back()), 2e-5)
        << "the refined full path reaches its target position";
    ASSERT_LT(OrientationDistance(finalState.tcpPose, command.targetPoses.back()), 2e-4)
        << "the refined full path reaches its target orientation";

}

void CheckLinearPathRejectsJointPathThatCannotMaintainTcpStraightness()
{
    constexpr double radiansPerDegree = 3.14159265358979323846 / 180.0;
    const JointVector targetJoints{
        58.5 * radiansPerDegree, -129.0 * radiansPerDegree, -61.1 * radiansPerDegree,
        16.5 * radiansPerDegree, 60.7 * radiansPerDegree, -9.6 * radiansPerDegree};
    DampedLeastSquaresIk inverse(models::hanwha::kHcr12a);
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "seed-restart line-path fixture connects at home";

    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp(targetJoints));
    command.maxLinearVelocityMetersPerSecond = 2.0;
    command.maxAngularVelocityRadiansPerSecond = 8.0;
    command.maxLinearAccelerationMetersPerSecondSquared = 30.0;
    command.maxAngularAccelerationRadiansPerSecondSquared = 120.0;
    const RobotState before = controller.GetState();
    const Result accepted = controller.MoveLinearPath(command);
    ASSERT_FALSE(static_cast<bool>(accepted))
        << "the planner must reject a joint interpolation that cannot meet the TCP straightness contract";
    ASSERT_EQ(accepted.code, ErrorCode::IkDidNotConverge)
        << "bounded refinement reports the failed Cartesian path contract";
    ASSERT_NE(accepted.message.find("TCP straightness error"), std::string::npos)
        << "failure identifies the path error instead of accepting an invalid MoveLinear trajectory";
    const RobotState after = controller.GetState();
    ASSERT_EQ(after.mode, before.mode) << "failed full-path planning leaves controller mode unchanged";
    ASSERT_EQ(after.jointPositionRadians, before.jointPositionRadians)
        << "failed full-path planning leaves the robot posture unchanged";
}

void CheckLinearPathAvoidsJ1ZeroDetourNearLimit()
{
    constexpr double radiansPerDegree = 3.14159265358979323846 / 180.0;
    const JointVector targetJoints{
        175.0 * radiansPerDegree, -0.8, -0.1, 0.5, -0.4, 0.2};
    DampedLeastSquaresIk inverse(models::hanwha::kHcr12a);
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "near-limit line-path fixture connects";
    JointMoveCommand startCommand;
    startCommand.targetPositionRadians = {
        130.0 * radiansPerDegree, -0.8, -0.1, 0.5, -0.4, 0.2};
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(startCommand)))) << "fixture starts on a nonzero J1 branch";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "fixture reaches its nonzero J1 start posture";
    const double initialJ1 = controller.GetStateView().jointPositionRadians[0];

    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp(targetJoints));
    command.maxLinearVelocityMetersPerSecond = 2.0;
    command.maxAngularVelocityRadiansPerSecond = 8.0;
    command.maxLinearAccelerationMetersPerSecondSquared = 30.0;
    command.maxAngularAccelerationRadiansPerSecondSquared = 120.0;
    const Result accepted = controller.MoveLinearPath(command);
    ASSERT_TRUE((static_cast<bool>(accepted))) << "near-limit line path is accepted: " + accepted.message;
    double previousJ1 = initialJ1;
    double totalJ1Travel = 0.0;
    for (int tick = 0; tick < 10000 && controller.GetStateView().mode == RobotMode::Moving; ++tick)
    {
        controller.Update(0.004);
        const double currentJ1 = controller.GetStateView().jointPositionRadians[0];
        totalJ1Travel += std::abs(currentJ1 - previousJ1);
        previousJ1 = currentJ1;
    }
    ASSERT_EQ(controller.GetStateView().mode, RobotMode::Idle) << "near-limit line path completes";
    const RobotState finalState = controller.GetState();
    ASSERT_LT(PositionDistance(finalState.tcpPose, command.targetPoses.back()), 2e-5)
        << "near-limit branch selection reaches the target position";
    ASSERT_LT(OrientationDistance(finalState.tcpPose, command.targetPoses.back()), 2e-4)
        << "near-limit branch selection reaches the target orientation";
    ASSERT_LE(totalJ1Travel, std::abs(finalState.jointPositionRadians[0] - initialJ1) + 0.5)
        << "branch selection near the J1 limit does not add a detour toward zero";
}

void CheckStopRetargetAndDisconnect()
{
    const models::Pose3 tcpOffset{{0.04, 0.0, 0.0}, {}};
    DampedLeastSquaresIk ik(models::hanwha::kHcr12a, tcpOffset);
    SimRobotController controller(models::hanwha::kHcr12a, tcpOffset);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "stop fixture connects";
    JointMoveCommand seedCommand;
    seedCommand.targetPositionRadians = {0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(seedCommand)))) << "stop fixture reaches a regular IK seed";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.004)) << "motion reaches target within bounded updates";
    LinearMoveCommand path;
    path.targetPose = ik.EvaluateTcp({0.4, -0.35, 0.3, 0.2, -0.25, 0.1});
    path.maxLinearVelocityMetersPerSecond = 0.025;
    path.maxAngularVelocityRadiansPerSecond = 0.15;
    const Result longPath = controller.MoveLinear(path);
    ASSERT_TRUE((static_cast<bool>(longPath))) << "long path starts: " + longPath.message;
    for (int i = 0; i < 30; ++i)
        controller.Update(0.004);
    const RobotState beforeStop = controller.GetState();
    ASSERT_TRUE((beforeStop.mode == RobotMode::Moving)) << "stop fixture remains in flight";
    ASSERT_TRUE((static_cast<bool>(controller.Stop()))) << "Stop accepts a path cancellation";
    const RobotState stopped = controller.GetState();
    ASSERT_TRUE((stopped.mode == RobotMode::Stopped)) << "Stop reports stopped mode";
    for (int i = 0; i < 20; ++i)
        controller.Update(0.004);
    ASSERT_TRUE((controller.GetState().jointPositionRadians == stopped.jointPositionRadians)) << "stopped linear path does not resume";

    JointMoveCommand jointTarget;
    jointTarget.targetPositionRadians.assign(6, 0.0);
    jointTarget.targetPositionRadians[0] = -0.25;
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(jointTarget)))) << "new joint command replaces stopped path";
    ASSERT_TRUE(AdvanceUntilIdle(controller, 0.005)) << "motion reaches target within bounded updates";
    ASSERT_NEAR(controller.GetState().jointPositionRadians[0], -0.25, 1e-8) << "replacement joint command reaches its own target";

    ASSERT_TRUE((static_cast<bool>(controller.MoveLinear(path)))) << "another linear path starts";
    controller.Disconnect();
    ASSERT_TRUE((!controller.GetState().valid && !controller.IsConnected())) << "disconnect invalidates controller state";
    ASSERT_TRUE((controller.MovePose(path.targetPose).code == ErrorCode::NotConnected)) << "disconnected controller rejects Cartesian motion";
}

void CheckModelWithoutToolFrame()
{
    auto specification = models::hanwha::kHcr12a;
    specification.hasToolFrame = false;
    SimRobotController controller(specification);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "robot without ToolFrame still connects";
    ASSERT_TRUE((!controller.GetState().tcpPoseValid)) << "robot without ToolFrame has no TCP feedback";
    JointMoveCommand jointTarget;
    jointTarget.targetPositionRadians.assign(specification.jointCount, 0.0);
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(jointTarget)))) << "robot without ToolFrame still supports joint motion";
    ASSERT_TRUE((controller.MovePose({}).code == ErrorCode::Unsupported)) << "robot without ToolFrame cannot run a TCP pose command";
    ASSERT_TRUE((controller.MoveLinear({}).code == ErrorCode::Unsupported)) << "robot without ToolFrame cannot run a TCP line command";
}

void CheckUnreachableLineInteriorPreservesActiveJointMotion()
{
    std::array<models::JointSpecification, 1> joints{{
        {"J1", {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, -1.0, 1.0, 1.0}}};
    const models::RobotSpecification specification{
        "Test", "SingleAxisArm", joints.data(), joints.size(), nullptr, 0, {{1.0, 0.0, 0.0}, {}}, true};
    DampedLeastSquaresIk ik(specification);
    SimRobotController controller(specification);
    ASSERT_TRUE((static_cast<bool>(controller.Connect()))) << "single-axis controller connects";
    const CartesianPose reachableEndpoint = ik.EvaluateTcp({0.5});
    ASSERT_TRUE((ik.Solve(reachableEndpoint, {0.0}).status == grasplink::robotics::kinematics::IkStatus::Success)) << "the line endpoint is independently reachable";

    JointMoveCommand activeJointTarget;
    activeJointTarget.targetPositionRadians = {-0.1};
    ASSERT_TRUE((static_cast<bool>(controller.MoveJoint(activeJointTarget)))) << "existing joint motion starts";
    const RobotState beforePath = controller.GetState();
    LinearMoveCommand impossibleChord;
    impossibleChord.targetPose = reachableEndpoint;
    impossibleChord.maxLinearVelocityMetersPerSecond = 0.1;
    impossibleChord.maxAngularVelocityRadiansPerSecond = 0.5;
    const Result rejected = controller.MoveLinear(impossibleChord);
    ASSERT_TRUE((rejected.code == ErrorCode::IkDidNotConverge || rejected.code == ErrorCode::JointLimitReached)) << "whole-path validation rejects a chord with an unreachable intermediate TCP pose";
    const RobotState afterPath = controller.GetState();
    ASSERT_TRUE((afterPath.jointPositionRadians == beforePath.jointPositionRadians &&
        afterPath.jointVelocityRadiansPerSecond == beforePath.jointVelocityRadiansPerSecond &&
        afterPath.mode == beforePath.mode)) << "rejected line preserves the pre-existing joint motion";
    controller.Update(0.05);
    ASSERT_TRUE((controller.GetState().jointPositionRadians[0] < beforePath.jointPositionRadians[0])) << "preserved joint target continues after failed line validation";
}
}


TEST(RobotMotion, MovePoseAndFailurePreservation) { CheckMovePoseAndFailurePreservation(); }
TEST(RobotMotion, EquivalentJointTargetUsesNearestLegalTurn) { CheckEquivalentJointTargetUsesNearestLegalTurn(); }
TEST(RobotMotion, ExplicitWristUnwindPreservesZeroRepresentation) { CheckExplicitWristUnwindPreservesZeroRepresentation(); }
TEST(RobotMotion, HomeSeedReachesFoldedValidPosture) { CheckHomeSeedCanReachFoldedButValidPosture(); }
TEST(RobotMotion, JointStateValidityReasons) { CheckJointStateValidityReasons(); }
TEST(RobotMotion, MoveJointRejectsInvalidPath) { CheckMoveJointRejectsInvalidPath(); }
TEST(RobotMotion, SelfCollisionFailureIncludesPairDiagnostic) { CheckSelfCollisionFailureIncludesPairDiagnostic(); }
TEST(RobotMotion, MoveJointExecutesTheValidatedJointPath) { CheckMoveJointExecutesTheValidatedJointPath(); }
TEST(RobotMotion, MoveJointAccelerationAndRetargetContinuity) { CheckMoveJointAccelerationAndRetargetContinuity(); }
TEST(RobotMotion, ShortJointMoveAndRejectedRetarget) { CheckShortJointMoveAndRejectedRetarget(); }
TEST(RobotMotion, LinearReorientationRejectsInvalidJointPath) { CheckLinearReorientationRejectsInvalidJointPath(); }
TEST(RobotMotion, EnvironmentCollisionCanBeRetried) { CheckEnvironmentCollisionCanBeRetried(); }
TEST(RobotMotion, CollisionAwareIkSelectsAnotherBranch) { CheckCollisionAwareIkSelectsAnotherBranch(); }
TEST(RobotMotion, LinearPathAndVelocityBoundsAtFourMilliseconds) { CheckLinearPathAndVelocityBounds(0.004); }
TEST(RobotMotion, LinearPathAndVelocityBoundsAtSevenMilliseconds) { CheckLinearPathAndVelocityBounds(0.007); }
TEST(RobotMotion, MultiWaypointUsesOneMotionProfile) { CheckMultiWaypointUsesOneMotionProfile(); }
TEST(RobotMotion, JointLimitsSetLinearPathSpeed) { CheckJointLimitsSetLinearPathSpeed(); }
TEST(RobotMotion, LinearPathFromHomeNearWristSingularity) { CheckLinearPathFromHomeNearWristSingularity(); }
TEST(RobotMotion, LinearPathKeepsJ1BranchContinuous) { CheckLinearPathKeepsJ1BranchContinuous(); }
TEST(RobotMotion, LinearPathExecutesRefinedTcpLine) { CheckLinearPathExecutesRefinedTcpLine(); }
TEST(RobotMotion, LinearPathRejectsJointPathThatCannotMaintainTcpStraightness)
{
    CheckLinearPathRejectsJointPathThatCannotMaintainTcpStraightness();
}
TEST(RobotMotion, LinearPathAvoidsJ1ZeroDetourNearLimit) { CheckLinearPathAvoidsJ1ZeroDetourNearLimit(); }

TEST(RobotMotion, IncrementalLinearPlanningDoesNotCommitBeforePathValidation)
{
    const auto& specification = grasplink::robotics::models::hanwha::kHcr12a;
    SimRobotController controller(specification);
    ASSERT_TRUE(controller.Connect());
    const JointVector initial = controller.GetStateView().jointPositionRadians;
    DampedLeastSquaresIk inverse(specification);
    const JointVector targetJoints{0.12, -0.08, 0.06, 0.12, -0.04, 0.1};
    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp(targetJoints));
    std::size_t validityCalls = 0;
    controller.SetJointStateValidityChecker([&](const JointVector&)
    {
        ++validityCalls;
        return JointStateInvalidity::None;
    });

    ASSERT_TRUE(controller.BeginLinearPathPlanning(command));
    ASSERT_TRUE(controller.IsMotionPlanning());
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Planning);
    for (std::size_t work = 0; work < 20000 && controller.IsMotionPlanning(); ++work)
    {
        controller.AdvanceMotionPlanning(1);
        EXPECT_EQ(controller.GetStateView().jointPositionRadians, initial)
            << "an incomplete path never changes the active joint state";
    }

    ASSERT_FALSE(controller.IsMotionPlanning());
    const auto result = controller.TakeMotionPlanningResult();
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(result.value());
    EXPECT_GT(validityCalls, 0u);
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Moving);
    EXPECT_EQ(controller.GetStateView().jointPositionRadians, initial)
        << "a validated path commits without teleporting the robot";
}

void CheckRejectedJointCommandPreservesPoseWaypoints()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(controller.Connect());
    grasplink::robotics::backends::simulation::SimRobotControllerTestAccess::SetPosePath(
        controller, {JointVector(6, 0.1), JointVector(6, 0.2)});

    JointMoveCommand invalid;
    invalid.targetPositionRadians = {0.0};
    EXPECT_EQ(controller.MoveJoint(invalid).code, ErrorCode::InvalidCommand);
    EXPECT_EQ(grasplink::robotics::backends::simulation::SimRobotControllerTestAccess::PosePathSize(controller), 2u)
        << "a rejected command does not erase waypoints from the accepted plan";

    JointMoveCommand accepted;
    accepted.targetPositionRadians.assign(6, 0.05);
    ASSERT_TRUE(controller.MoveJoint(accepted));
    EXPECT_EQ(grasplink::robotics::backends::simulation::SimRobotControllerTestAccess::PosePathSize(controller), 0u)
        << "an accepted external command replaces the old pose route";
}

void CheckCollisionRestoreCancelsPendingPosePlan()
{
    const auto& specification = grasplink::robotics::models::hanwha::kHcr12a;
    SimRobotController controller(specification);
    ASSERT_TRUE(controller.Connect());
    DampedLeastSquaresIk inverse(specification);
    const CartesianPose target = inverse.EvaluateTcp({0.12, -0.08, 0.06, 0.12, -0.04, 0.1});
    ASSERT_TRUE(controller.BeginPosePlanning(target));
    ASSERT_TRUE(controller.IsMotionPlanning());
    controller.AdvanceMotionPlanning(1);
    const JointVector safe = controller.GetStateView().jointPositionRadians;

    ASSERT_TRUE(controller.RestoreCollisionSafeState(safe, ErrorCode::SelfCollision));
    EXPECT_FALSE(controller.IsMotionPlanning());
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Idle);
    EXPECT_EQ(controller.GetStateView().errorCode, ErrorCode::SelfCollision);
    controller.AdvanceMotionPlanning(1000);
    EXPECT_EQ(controller.GetStateView().jointPositionRadians, safe)
        << "a cancelled plan cannot resume after collision rollback";
}

void CheckPosePlanPreflightsAndReusesLinearContinuation()
{
    const auto& specification = grasplink::robotics::models::hanwha::kHcr12a;
    SimRobotController controller(specification);
    ASSERT_TRUE(controller.Connect());
    DampedLeastSquaresIk inverse(specification);
    const JointVector approachJoints{0.12, -0.08, 0.06, 0.12, -0.04, 0.1};
    const CartesianPose approach = inverse.EvaluateTcp(approachJoints);
    LinearPathMoveCommand descent;
    descent.targetPoses.push_back(approach);
    descent.targetPoses.back().positionMeters[1] -= 0.02;

    ASSERT_TRUE(controller.BeginPosePlanningWithLinearContinuation(approach, descent));
    const JointVector initial = controller.GetStateView().jointPositionRadians;
    for (std::size_t work = 0; work < 100000 && controller.IsMotionPlanning(); ++work)
    {
        controller.AdvanceMotionPlanning(8);
        EXPECT_EQ(controller.GetStateView().jointPositionRadians, initial)
            << "planning both paths must not move before the certified approach is ready";
    }
    ASSERT_FALSE(controller.IsMotionPlanning());
    const auto planned = controller.TakeMotionPlanningResult();
    ASSERT_TRUE(planned.has_value());
    ASSERT_TRUE(*planned) << planned->message;

    for (std::size_t tick = 0; tick < 10000 && controller.GetStateView().mode == RobotMode::Moving; ++tick)
        controller.Update(0.004);
    ASSERT_EQ(controller.GetStateView().mode, RobotMode::Idle);
    EXPECT_LT(PositionDistance(controller.GetStateView().tcpPose, approach), 1e-3);

    const Result cachedDescent = controller.BeginLinearPathPlanning(descent);
    ASSERT_TRUE(cachedDescent) << cachedDescent.message;
    EXPECT_FALSE(controller.IsMotionPlanning())
        << "the command consumes the exact MoveL plan already certified for this approach posture";
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Moving);
}

TEST(RobotMotion, PosePlanningFindsKnownReachableEndpointAfterLimitStall)
{
    const auto& specification = grasplink::robotics::models::hanwha::kHcr12a;
    SimRobotController controller(specification);
    ASSERT_TRUE(controller.Connect());
    DampedLeastSquaresIk inverse(specification);
    constexpr double degreesToRadians = 3.14159265358979323846 / 180.0;
    const JointVector knownReachableJoints{
        119.776553 * degreesToRadians,
        73.182887 * degreesToRadians,
        -46.491438 * degreesToRadians,
        28.083620 * degreesToRadians,
        95.159352 * degreesToRadians,
        -72.352759 * degreesToRadians};
    const CartesianPose target = inverse.EvaluateTcp(knownReachableJoints);

    ASSERT_TRUE(controller.BeginPosePlanning(target));
    for (std::size_t frame = 0; frame < 1000 && controller.IsMotionPlanning(); ++frame)
        controller.AdvanceMotionPlanning(64);

    ASSERT_FALSE(controller.IsMotionPlanning());
    const auto result = controller.TakeMotionPlanningResult();
    ASSERT_TRUE(result.has_value());
    ASSERT_TRUE(*result) << result->message;
    EXPECT_GT(controller.GetLastPoseIkSeedAttemptCount(), 1u)
        << "the primary seed stalls on this deterministic endpoint, so a bounded fallback must solve it";

    for (std::size_t tick = 0; tick < 10000 &&
        controller.GetStateView().mode == RobotMode::Moving; ++tick)
        controller.Update(0.004);
    const auto& state = controller.GetStateView();
    ASSERT_EQ(state.mode, RobotMode::Idle);
    ASSERT_TRUE(state.tcpPoseValid);
    EXPECT_LT(PositionDistance(state.tcpPose, target), 1e-3);
    EXPECT_LT(OrientationDistance(state.tcpPose, target), 1e-3);
    EXPECT_EQ(ValidateJointState(specification, state.jointPositionRadians, {}),
        JointStateInvalidity::None);
}

TEST(RobotMotion, ReplacingValidityCheckerCancelsInFlightPlan)
{
    const auto& specification = grasplink::robotics::models::hanwha::kHcr12a;
    SimRobotController controller(specification);
    ASSERT_TRUE(controller.Connect());
    std::size_t oldCheckerCalls = 0;
    controller.SetJointStateValidityChecker([&](const JointVector&)
    {
        ++oldCheckerCalls;
        return JointStateInvalidity::None;
    });
    DampedLeastSquaresIk inverse(specification);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp({0.12, -0.08, 0.06, 0.12, -0.04, 0.1}));

    ASSERT_TRUE(controller.BeginLinearPathPlanning(command));
    ASSERT_TRUE(controller.IsMotionPlanning());
    for (std::size_t work = 0; work < 1000 && controller.IsMotionPlanning() && oldCheckerCalls == 0; ++work)
        controller.AdvanceMotionPlanning(1);
    ASSERT_GT(oldCheckerCalls, 0u);
    const std::size_t callsBeforeReplacement = oldCheckerCalls;
    controller.SetJointStateValidityChecker([](const JointVector&)
    {
        return JointStateInvalidity::EnvironmentCollision;
    });

    EXPECT_FALSE(controller.IsMotionPlanning());
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Idle);
    const auto cancelled = controller.TakeMotionPlanningResult();
    ASSERT_TRUE(cancelled.has_value());
    EXPECT_EQ(cancelled->code, ErrorCode::Cancelled);
    EXPECT_EQ(controller.TakeMotionPlanningResult(), std::nullopt);
    controller.AdvanceMotionPlanning(64);
    EXPECT_EQ(oldCheckerCalls, callsBeforeReplacement)
        << "a cancelled job never calls the replaced validity checker";
}

TEST(RobotMotion, RejectedJointCommandPreservesPoseWaypoints)
{
    CheckRejectedJointCommandPreservesPoseWaypoints();
}

TEST(RobotMotion, CollisionRestoreCancelsPendingPosePlan)
{
    CheckCollisionRestoreCancelsPendingPosePlan();
}

TEST(RobotMotion, PosePlanPreflightsAndReusesLinearContinuation)
{
    CheckPosePlanPreflightsAndReusesLinearContinuation();
}

TEST(RobotMotion, ReconnectCancelsAndClearsInFlightPlan)
{
    const auto& specification = grasplink::robotics::models::hanwha::kHcr12a;
    SimRobotController controller(specification);
    ASSERT_TRUE(controller.Connect());
    DampedLeastSquaresIk inverse(specification);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp({0.12, -0.08, 0.06, 0.12, -0.04, 0.1}));
    ASSERT_TRUE(controller.BeginLinearPathPlanning(command));
    ASSERT_TRUE(controller.IsMotionPlanning());

    ASSERT_TRUE(controller.Connect());

    EXPECT_FALSE(controller.IsMotionPlanning());
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Idle);
    EXPECT_TRUE(controller.GetStateView().valid);
    EXPECT_EQ(controller.GetStateView().jointPositionRadians, JointVector(6, 0.0));
    EXPECT_EQ(controller.TakeMotionPlanningResult(), std::nullopt);
    controller.AdvanceMotionPlanning(64);
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Idle)
        << "a plan from the previous connection cannot commit after reconnect";
}

TEST(RobotMotion, StopRetargetAndDisconnect) { CheckStopRetargetAndDisconnect(); }
TEST(RobotMotion, ModelWithoutToolFrame) { CheckModelWithoutToolFrame(); }
TEST(RobotMotion, UnreachableLineInteriorPreservesActiveJointMotion) { CheckUnreachableLineInteriorPreservesActiveJointMotion(); }
