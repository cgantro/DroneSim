#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;

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

std::array<double, 3> RotationRate(const CartesianPose& plus, const CartesianPose& minus, double deltaRadians)
{
    const auto& a = plus.orientationXyzw;
    const auto& b = minus.orientationXyzw;
    const double x = -a[3] * b[0] + a[0] * b[3] - a[1] * b[2] + a[2] * b[1];
    const double y = -a[3] * b[1] + a[0] * b[2] + a[1] * b[3] - a[2] * b[0];
    const double z = -a[3] * b[2] - a[0] * b[1] + a[1] * b[0] + a[2] * b[3];
    const double w = a[3] * b[3] + a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    const double vectorLength = std::sqrt(x * x + y * y + z * z);
    const double angle = 2.0 * std::atan2(vectorLength, std::abs(w));
    if (vectorLength <= 1e-15)
        return {};
    const double scale = angle / (2.0 * deltaRadians * vectorLength);
    return {x * scale, y * scale, z * scale};
}

models::RobotSpecification MakeSingleJointModel(
    std::array<models::JointSpecification, 1>& joints,
    models::Pose3 toolFrame,
    double minimumRadians = -0.1,
    double maximumRadians = 0.1)
{
    joints[0] = {"J1", {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, minimumRadians, maximumRadians, 1.0};
    return {"Test", "SingleJoint", joints.data(), joints.size(), nullptr, 0, toolFrame, true};
}

}

namespace
{
void CheckFourJointModelSolvesPositionOnly()
{
    std::array<models::JointSpecification, 4> joints{};
    for (std::size_t i = 0; i < joints.size(); ++i)
        joints[i] = models::hanwha::kHcr12a.joints[i];
    const models::RobotSpecification specification{
        "Test", "FourAxis", joints.data(), joints.size(), nullptr, 0, {}, true};
    LevenbergMarquardtIk inverse(specification);
    const JointVector seed{0.2, -0.35, 0.3, 0.2};
    const JointVector targetJoints{0.22, -0.33, 0.31, 0.19};
    CartesianPose target = inverse.EvaluateTcp(targetJoints);
    target.orientationXyzw = {0.0, 0.0, 0.0, 1.0};

    const IkOptions options = RobotDefaultIkOptions(specification);
    ASSERT_EQ(options.taskMode, IkTaskMode::PositionOnly);
    const IkResult result = inverse.Solve(target, seed, options);

    ASSERT_EQ(result.status, IkStatus::Success) << result.message;
    EXPECT_LT(PositionDistance(inverse.EvaluateTcp(result.jointPositions), target), 2e-5);
}

void CheckSevenJointModelSolvesReachablePose()
{
    std::array<models::JointSpecification, 7> joints{};
    for (std::size_t i = 0; i < models::hanwha::kHcr12a.jointCount; ++i)
        joints[i] = models::hanwha::kHcr12a.joints[i];
    const auto& last = joints[5];
    joints[6] = {"J7", {last.bindPivotMeters.x, last.bindPivotMeters.y, last.bindPivotMeters.z + 0.08},
        {0.0, 1.0, 0.0}, -2.0, 2.0, 1.0};
    const models::RobotSpecification specification{
        "Test", "SevenAxis", joints.data(), joints.size(), nullptr, 0,
        models::hanwha::kHcr12a.toolFrameInLastJoint, true};
    LevenbergMarquardtIk inverse(specification);
    const JointVector seed{0.2, -0.35, 0.3, 0.2, -0.25, 0.1, 0.0};
    const JointVector targetJoints{0.21, -0.34, 0.31, 0.19, -0.24, 0.11, 0.03};
    const CartesianPose target = inverse.EvaluateTcp(targetJoints);

    const IkOptions options = RobotDefaultIkOptions(specification);
    ASSERT_EQ(options.taskMode, IkTaskMode::Pose);
    const IkResult result = inverse.Solve(target, seed, options);

    ASSERT_EQ(result.status, IkStatus::Success) << result.message;
    EXPECT_LT(PositionDistance(inverse.EvaluateTcp(result.jointPositions), target), 2e-5);
}
}

TEST(RobotInverseKinematicsTests, SolvesPositionAndPoseForFourAndSevenJoints)
{
    EXPECT_EQ(RobotDefaultIkOptions(models::hanwha::kHcr12a).taskMode, IkTaskMode::Pose);
    CheckFourJointModelSolvesPositionOnly();
    CheckSevenJointModelSolvesReachablePose();
}

TEST(RobotInverseKinematicsTests, JacobianMatchesFiniteDifferenceWithAngledAxesAndTcpOffset)
{
    // 서로 다른 크기와 방향을 가진 회전축, 관절 중심, ToolFrame offset을 사용해 축이 Local에서 Base로 변환되는지 확인한다.
    std::array<models::JointSpecification, 3> joints{{
        {"J1", {0.1, 0.2, 0.1}, {0.0, 0.0, 2.0}, -1.5, 1.5, 1.0},
        {"J2", {0.2, 0.25, 0.15}, {0.0, 3.0, 0.0}, -1.5, 1.5, 1.0},
        {"J3", {0.25, 0.42, 0.12}, {1.0, 1.0, 0.0}, -1.5, 1.5, 1.0}}};
    const models::Pose3 toolFrame{{0.16, 0.03, 0.08}, {}};
    const models::RobotSpecification specification{
        "Test", "AngledAxes", joints.data(), joints.size(), nullptr, 0, toolFrame, true};
    const double halfAngle = 0.13;
    const models::Pose3 tcpOffset{{0.04, -0.02, 0.03}, {std::cos(halfAngle), std::sin(halfAngle), 0.0, 0.0}};
    RobotKinematics forward(specification);
    LevenbergMarquardtIk inverse(specification, tcpOffset);
    const JointVector angles{0.31, -0.27, 0.19};
    const CartesianPose center = inverse.EvaluateTcp(angles);
    const auto& fkState = forward.Update(angles);
    constexpr double deltaRadians = 1e-6;

    for (std::size_t joint = 0; joint < angles.size(); ++joint)
    {
        // 회전 관절의 위치 변화율은 Base 회전축과 TCP에서 관절 중심까지의 거리의 외적이다. 중심 차분은 같은 미소 각도 변화에서 FK가 실제로 만든 위치 변화를 구한다.
        const auto& axis = fkState.jointAxesInBaseFrame[joint];
        const auto& pivot = fkState.linkPosesInBaseFrame[joint].positionMeters;
        const double rx = center.positionMeters[0] - pivot.x;
        const double ry = center.positionMeters[1] - pivot.y;
        const double rz = center.positionMeters[2] - pivot.z;
        const std::array<double, 3> analytic{
            axis.y * rz - axis.z * ry,
            axis.z * rx - axis.x * rz,
            axis.x * ry - axis.y * rx};
        JointVector plus = angles;
        JointVector minus = angles;
        plus[joint] += deltaRadians;
        minus[joint] -= deltaRadians;
        const CartesianPose plusPose = inverse.EvaluateTcp(plus);
        const CartesianPose minusPose = inverse.EvaluateTcp(minus);
        for (std::size_t coordinate = 0; coordinate < 3; ++coordinate)
        {
            const double numeric = (plusPose.positionMeters[coordinate] - minusPose.positionMeters[coordinate]) /
                (2.0 * deltaRadians);
            EXPECT_NEAR(numeric, analytic[coordinate], 2e-7)
                << "base-frame joint axis cross TCP offset matches finite-difference FK";
        }
        const std::array<double, 3> numericRotation = RotationRate(plusPose, minusPose, deltaRadians);
        EXPECT_NEAR(numericRotation[0], axis.x, 2e-7) << "finite-difference TCP rotation x follows the base joint axis";
        EXPECT_NEAR(numericRotation[1], axis.y, 2e-7) << "finite-difference TCP rotation y follows the base joint axis";
        EXPECT_NEAR(numericRotation[2], axis.z, 2e-7) << "finite-difference TCP rotation z follows the base joint axis";
    }
}

TEST(RobotInverseKinematicsTests, RoundTripsReachablePoseAndIgnoresQuaternionSign)
{
    LevenbergMarquardtIk inverse(models::hanwha::kHcr12a);
    const JointVector currentSeed{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    const JointVector targetJoints{0.25, -0.37, 0.34, 0.17, -0.23, 0.11};
    const CartesianPose target = inverse.EvaluateTcp(targetJoints);

    // 순기구학(FK)은 관절각에서 TCP 자세를 계산하고 역기구학(IK)은 그 자세에서 관절각을 찾는다. 여기서는 같은 로봇 모델의 FK가 만든 목표를 넣어 IK 출력도 FK로 다시 검사한다.
    const IkResult result = inverse.Solve(target, currentSeed);
    ASSERT_EQ(result.status, IkStatus::Success) << "FK-generated reachable target converges: " + result.message;
    const CartesianPose recovered = inverse.EvaluateTcp(result.jointPositions);
    EXPECT_LT(PositionDistance(recovered, target), 2e-5) << "round-trip TCP position error";
    EXPECT_LT(OrientationDistance(recovered, target), 2e-4) << "round-trip TCP orientation error";

    CartesianPose equivalent = target;
    for (double& component : equivalent.orientationXyzw)
        component = -component;
    const IkResult signFlipped = inverse.Solve(equivalent, currentSeed);
    ASSERT_EQ(signFlipped.status, IkStatus::Success) << "opposite quaternion sign describes the same orientation";
    const CartesianPose signRecovered = inverse.EvaluateTcp(signFlipped.jointPositions);
    EXPECT_LT(PositionDistance(signRecovered, target), 2e-5) << "sign-flipped position error";
    EXPECT_LT(OrientationDistance(signRecovered, target), 2e-4) << "sign-flipped orientation error";
}

TEST(RobotInverseKinematicsTests, IncrementalSingleSeedMatchesSynchronousResult)
{
    LevenbergMarquardtIk synchronous(models::hanwha::kHcr12a);
    LevenbergMarquardtIk incremental(models::hanwha::kHcr12a);
    const JointVector seed{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    const JointVector targetJoints{0.9, -0.2, 0.1, 0.45, -0.1, 0.35};
    const CartesianPose target = synchronous.EvaluateTcp(targetJoints);
    const IkResult expected = synchronous.SolveSingleSeed(target, seed);
    ASSERT_EQ(expected.status, IkStatus::Success) << "the comparison target converges from the selected seed";
    const auto session = incremental.BeginSingleSeed(target, seed);

    EXPECT_EQ(incremental.StepSingleSeed(session, 0), IkSessionState::Running)
        << "zero budget reports progress without running an iteration";
    IkSessionState state = incremental.StepSingleSeed(session, 1);
    EXPECT_EQ(state, IkSessionState::Running) << "one iteration can be yielded before the solve is complete";
    while (state == IkSessionState::Running)
        state = incremental.StepSingleSeed(session, 2);

    const IkResult& actual = incremental.GetSingleSeedResult(session);
    EXPECT_EQ(state, IkSessionState::Completed) << "the incremental session reaches a terminal state";
    EXPECT_EQ(actual.status, expected.status) << "incremental and synchronous status match";
    EXPECT_EQ(actual.iterations, expected.iterations) << "incremental and synchronous iteration counts match";
    EXPECT_EQ(actual.message, expected.message) << "incremental and synchronous diagnostics match";
    EXPECT_EQ(actual.jointPositions.size(), expected.jointPositions.size());
    for (std::size_t joint = 0; joint < expected.jointPositions.size(); ++joint)
        EXPECT_DOUBLE_EQ(actual.jointPositions[joint], expected.jointPositions[joint])
            << "incremental solve preserves the synchronous joint result";
    EXPECT_DOUBLE_EQ(actual.positionErrorMeters, expected.positionErrorMeters);
    EXPECT_DOUBLE_EQ(actual.orientationErrorRadians, expected.orientationErrorRadians);
}

TEST(RobotInverseKinematicsTests, AppliesToolOffsetAndRejectsInvalidInputs)
{
    std::array<models::JointSpecification, 1> joints{};
    const double halfQuarterTurn = std::acos(-1.0) / 4.0;
    models::Pose3 tcpOffset{{0.3, 0.0, 0.0}, {std::cos(halfQuarterTurn), 0.0, 0.0, std::sin(halfQuarterTurn)}};
    const auto specification = MakeSingleJointModel(joints, {});
    LevenbergMarquardtIk toolOffsetIk(specification, tcpOffset);
    const CartesianPose offsetPose = toolOffsetIk.EvaluateTcp({0.0});
    EXPECT_NEAR(offsetPose.positionMeters[0], 0.3, 1e-12) << "TCP offset translation x";
    EXPECT_NEAR(offsetPose.positionMeters[1], 0.0, 1e-12) << "TCP offset translation y";
    EXPECT_NEAR(offsetPose.orientationXyzw[2], std::sin(halfQuarterTurn), 1e-12)
        << "model wxyz quaternion is returned as TCP xyzw";
    EXPECT_NEAR(offsetPose.orientationXyzw[3], std::cos(halfQuarterTurn), 1e-12)
        << "TCP orientation retains the tool offset rotation";

    std::array<models::JointSpecification, 1> slidingJoints{{
        {"Slide", {}, {1.0, 0.0, 0.0}, 0.0, 0.2, 0.1}}};
    slidingJoints[0].motionType = models::JointMotionType::Prismatic;
    const models::RobotSpecification slidingSpecification{
        "Test", "Slide", slidingJoints.data(), slidingJoints.size(), nullptr, 0, {}, true};
    LevenbergMarquardtIk slidingIk(slidingSpecification);
    IkOptions slidingOptions;
    slidingOptions.taskMode = IkTaskMode::PositionOnly;
    const IkResult slidingResult = slidingIk.Solve(
        CartesianPose{{0.075, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}}, {0.0}, slidingOptions);
    ASSERT_EQ(slidingResult.status, IkStatus::Success) << slidingResult.message;
    EXPECT_NEAR(slidingResult.jointPositions[0], 0.075, 1e-6);

    LevenbergMarquardtIk inverse(specification, tcpOffset);
    models::RobotSpecification withoutToolFrame = specification;
    withoutToolFrame.hasToolFrame = false;
    LevenbergMarquardtIk missingTool(withoutToolFrame);
    EXPECT_EQ(missingTool.Solve(offsetPose, {0.0}).status, IkStatus::MissingToolFrame)
        << "IK distinguishes a model without a ToolFrame";
    EXPECT_THROW((void)missingTool.EvaluateTcp({0.0}), std::invalid_argument)
        << "FK TCP evaluation rejects a missing ToolFrame";

    CartesianPose invalidTarget = offsetPose;
    invalidTarget.orientationXyzw = {0.0, 0.0, 0.0, 0.0};
    EXPECT_EQ(inverse.Solve(invalidTarget, {0.0}).status, IkStatus::InvalidInput)
        << "zero target quaternion is rejected";
    invalidTarget = offsetPose;
    invalidTarget.positionMeters[1] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(inverse.Solve(invalidTarget, {0.0}).status, IkStatus::InvalidInput)
        << "non-finite target position is rejected";
    EXPECT_EQ(inverse.Solve(offsetPose, {}).status, IkStatus::InvalidInput) << "wrong seed joint count is rejected";
    EXPECT_EQ(inverse.Solve(offsetPose, {std::numeric_limits<double>::quiet_NaN()}).status, IkStatus::InvalidInput)
        << "non-finite seed is rejected";
    EXPECT_EQ(inverse.Solve(offsetPose, {0.2}).status, IkStatus::InvalidInput) << "seed outside joint limits is rejected";

    IkOptions invalidOptions;
    invalidOptions.maxIterations = 0;
    EXPECT_EQ(inverse.Solve(offsetPose, {0.0}, invalidOptions).status, IkStatus::InvalidInput)
        << "zero iteration budget is rejected";
    invalidOptions = {};
    invalidOptions.damping = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(inverse.Solve(offsetPose, {0.0}, invalidOptions).status, IkStatus::InvalidInput)
        << "non-finite damping is rejected";
    invalidOptions = {};
    invalidOptions.positionToleranceMeters = 0.0;
    EXPECT_EQ(inverse.Solve(offsetPose, {0.0}, invalidOptions).status, IkStatus::InvalidInput)
        << "zero position tolerance is rejected";
}

TEST(RobotInverseKinematicsTests, ReportsFailureStatusesAndStabilizesNearSingularPose)
{
    std::array<models::JointSpecification, 1> joints{};
    const auto limitedModel = MakeSingleJointModel(joints, {{1.0, 0.0, 0.0}, {}});
    LevenbergMarquardtIk limited(limitedModel);
    const CartesianPose beyondJointLimit = limited.EvaluateTcp({0.5});
    const IkResult jointLimit = limited.Solve(beyondJointLimit, {0.0});
    EXPECT_EQ(jointLimit.status, IkStatus::JointLimitReached) << "joint boundary failure has its own status";
    EXPECT_EQ(jointLimit.terminationReason, IkTerminationReason::ActiveJointLimit)
        << "the local failure is tied to an active joint boundary";
    EXPECT_TRUE(std::isfinite(jointLimit.weightedJacobianMinimumSingularValue))
        << "failure records the weighted Jacobian minimum singular value";

    const IkResult unreachable = limited.Solve(
        CartesianPose{{50.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}}, {0.0});
    EXPECT_EQ(unreachable.status, IkStatus::Unreachable) << "far target is classified as unreachable";

    const IkResult exhausted = [&]
    {
        LevenbergMarquardtIk hcr(models::hanwha::kHcr12a);
        const CartesianPose reachable = hcr.EvaluateTcp({0.25, -0.37, 0.34, 0.17, -0.23, 0.11});
        IkOptions options;
        options.maxIterations = 1;
        options.positionToleranceMeters = 1e-12;
        options.orientationToleranceRadians = 1e-12;
        return hcr.Solve(reachable, {0.2, -0.35, 0.3, 0.2, -0.25, 0.1}, options);
    }();
    EXPECT_EQ(exhausted.status, IkStatus::DidNotConverge)
        << "iteration exhaustion is distinct from an unreachable target and a joint limit";
    EXPECT_EQ(exhausted.terminationReason, IkTerminationReason::IterationLimit)
        << "iteration budget exhaustion is distinguished from numerical stagnation";
    EXPECT_TRUE(exhausted.weightedJacobianMinimumSingularValue >= 0.0 &&
        !std::isnan(exhausted.weightedJacobianConditionNumber))
        << "failure diagnostics contain a nonnegative singular value and a defined condition estimate";

    IkOptions tinyStepOptions;
    tinyStepOptions.maxJointStep = std::numeric_limits<double>::denorm_min();
    const IkResult numericalFailure = limited.SolveSingleSeed(
        limited.EvaluateTcp({0.05}), {0.0}, tinyStepOptions);
    EXPECT_EQ(numericalFailure.status, IkStatus::DidNotConverge);
    EXPECT_EQ(numericalFailure.terminationReason, IkTerminationReason::NumericalFailure)
        << "a non-finite joint step is reported as a numeric failure";

    std::array<models::JointSpecification, 1> unconstrainedJoints{};
    const auto unconstrainedModel = MakeSingleJointModel(
        unconstrainedJoints, {{1.0, 0.0, 0.0}, {}}, -3.0, 3.0);
    LevenbergMarquardtIk damped(unconstrainedModel);
    const CartesianPose smallTurn = damped.EvaluateTcp({0.04});
    IkOptions dampedOptions;
    dampedOptions.damping = 1e-2;
    const IkResult nearSingular = damped.Solve(smallTurn, {0.0}, dampedOptions);
    ASSERT_EQ(nearSingular.status, IkStatus::Success) << "damping stabilizes a nearly aligned arm pose";
    EXPECT_TRUE(std::isfinite(nearSingular.jointPositions[0]) &&
        std::abs(nearSingular.jointPositions[0]) <= 3.0)
        << "damped near-singular result remains finite and within limits";
}
