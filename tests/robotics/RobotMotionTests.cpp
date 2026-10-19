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
TEST(RobotMotion, JointStateValidityReasons) { CheckJointStateValidityReasons(); }
TEST(RobotMotion, MoveJointRejectsInvalidPath) { CheckMoveJointRejectsInvalidPath(); }
TEST(RobotMotion, SelfCollisionFailureIncludesPairDiagnostic) { CheckSelfCollisionFailureIncludesPairDiagnostic(); }
TEST(RobotMotion, MoveJointExecutesTheValidatedJointPath) { CheckMoveJointExecutesTheValidatedJointPath(); }
TEST(RobotMotion, MoveJointAccelerationAndRetargetContinuity) { CheckMoveJointAccelerationAndRetargetContinuity(); }
TEST(RobotMotion, LinearPathAndVelocityBoundsAtFourMilliseconds) { CheckLinearPathAndVelocityBounds(0.004); }
TEST(RobotMotion, LinearPathAndVelocityBoundsAtSevenMilliseconds) { CheckLinearPathAndVelocityBounds(0.007); }
TEST(RobotMotion, LinearPathRejectsJointPathThatCannotMaintainTcpStraightness)
{
    CheckLinearPathRejectsJointPathThatCannotMaintainTcpStraightness();
}
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

TEST(RobotMotion, CollisionRestoreCancelsPendingPosePlan)
{
    CheckCollisionRestoreCancelsPendingPosePlan();
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
