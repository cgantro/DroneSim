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
    DampedLeastSquaresIk inverse(specification, tcpOffset);
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
    DampedLeastSquaresIk inverse(models::hanwha::kHcr12a);
    const JointVector currentSeed{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    const JointVector targetJoints{0.25, -0.37, 0.34, 0.17, -0.23, 0.11};
    const CartesianPose target = inverse.EvaluateTcp(targetJoints);

    // 순기구학(FK)은 관절각에서 TCP 자세를 계산하고 역기구학(IK)은 그 자세에서 관절각을 찾는다. 여기서는 같은 로봇 모델의 FK가 만든 목표를 넣어 IK 출력도 FK로 다시 검사한다.
    const IkResult result = inverse.Solve(target, currentSeed);
    ASSERT_EQ(result.status, IkStatus::Success) << "FK-generated reachable target converges: " + result.message;
    const CartesianPose recovered = inverse.EvaluateTcp(result.jointPositionRadians);
    EXPECT_LT(PositionDistance(recovered, target), 2e-5) << "round-trip TCP position error";
    EXPECT_LT(OrientationDistance(recovered, target), 2e-4) << "round-trip TCP orientation error";

    CartesianPose equivalent = target;
    for (double& component : equivalent.orientationXyzw)
        component = -component;
    const IkResult signFlipped = inverse.Solve(equivalent, currentSeed);
    ASSERT_EQ(signFlipped.status, IkStatus::Success) << "opposite quaternion sign describes the same orientation";
    const CartesianPose signRecovered = inverse.EvaluateTcp(signFlipped.jointPositionRadians);
    EXPECT_LT(PositionDistance(signRecovered, target), 2e-5) << "sign-flipped position error";
    EXPECT_LT(OrientationDistance(signRecovered, target), 2e-4) << "sign-flipped orientation error";
}

TEST(RobotInverseKinematicsTests, IncrementalSingleSeedMatchesSynchronousResult)
{
    DampedLeastSquaresIk synchronous(models::hanwha::kHcr12a);
    DampedLeastSquaresIk incremental(models::hanwha::kHcr12a);
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
    EXPECT_EQ(actual.jointPositionRadians.size(), expected.jointPositionRadians.size());
    for (std::size_t joint = 0; joint < expected.jointPositionRadians.size(); ++joint)
        EXPECT_DOUBLE_EQ(actual.jointPositionRadians[joint], expected.jointPositionRadians[joint])
            << "incremental solve preserves the synchronous joint result";
    EXPECT_DOUBLE_EQ(actual.positionErrorMeters, expected.positionErrorMeters);
    EXPECT_DOUBLE_EQ(actual.orientationErrorRadians, expected.orientationErrorRadians);
}

TEST(RobotInverseKinematicsTests, IncrementalIterationCountRespectsEachStepBudgetAtLimit)
{
    DampedLeastSquaresIk inverse(models::hanwha::kHcr12a);
    const JointVector seed{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    const JointVector targetJoints{0.9, -0.2, 0.1, 0.45, -0.1, 0.35};
    const CartesianPose target = inverse.EvaluateTcp(targetJoints);
    IkOptions options;
    options.maxIterations = 2;
    const auto session = inverse.BeginSingleSeed(target, seed, options);
    IkSessionState state = IkSessionState::Running;
    std::size_t calls = 0;

    while (state == IkSessionState::Running && calls < options.maxIterations + 1)
    {
        const std::size_t before = inverse.GetSingleSeedResult(session).iterations;
        state = inverse.StepSingleSeed(session, 1);
        const std::size_t after = inverse.GetSingleSeedResult(session).iterations;
        EXPECT_GE(after, before) << "reported solver work never moves backwards";
        EXPECT_LE(after - before, 1u)
            << "one incremental unit reports at most one completed DLS iteration";
        EXPECT_LE(after, options.maxIterations)
            << "reported work never exceeds the configured iteration boundary";
        ++calls;
    }

    ASSERT_EQ(state, IkSessionState::Completed);
    EXPECT_EQ(inverse.GetSingleSeedResult(session).iterations, options.maxIterations)
        << "the final budgeted step publishes the exact max-iteration count";
}

TEST(RobotInverseKinematicsTests, IncrementalSingleSeedCanBeCancelledAndOldSessionsExpire)
{
    DampedLeastSquaresIk inverse(models::hanwha::kHcr12a);
    const JointVector seed{0.2, -0.35, 0.3, 0.2, -0.25, 0.1};
    const JointVector targetJoints{0.9, -0.2, 0.1, 0.45, -0.1, 0.35};
    const CartesianPose target = inverse.EvaluateTcp(targetJoints);
    const auto cancelledSession = inverse.BeginSingleSeed(target, seed);

    ASSERT_EQ(inverse.StepSingleSeed(cancelledSession, 1), IkSessionState::Running);
    inverse.CancelSingleSeed(cancelledSession);
    EXPECT_EQ(inverse.StepSingleSeed(cancelledSession, 10), IkSessionState::Cancelled)
        << "a cancelled session stays terminal";
    EXPECT_EQ(inverse.GetSingleSeedResult(cancelledSession).message, "IK: cancelled");

    const auto nextSession = inverse.BeginSingleSeed(target, seed);
    EXPECT_THROW((void)inverse.StepSingleSeed(cancelledSession, 1), std::invalid_argument)
        << "starting another seed invalidates the previous session identifier";
    EXPECT_EQ(inverse.StepSingleSeed(nextSession, 1), IkSessionState::Running);
}

TEST(RobotInverseKinematicsTests, FallbackEscapesSingularSeedLocalMinimum)
{
    DampedLeastSquaresIk inverse(models::hanwha::kHcr12a);
    const JointVector source{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const JointVector reachableJoints{0.0, -0.8, -0.1, 0.5, -0.4, 0.2};
    const CartesianPose target = inverse.EvaluateTcp(reachableJoints);
    const IkResult primaryAttempt = inverse.SolveSingleSeed(target, source);
    EXPECT_TRUE(primaryAttempt.status == IkStatus::DidNotConverge || primaryAttempt.status == IkStatus::JointLimitReached)
        << "single-seed solve exposes the singular home-posture local minimum";
    if (primaryAttempt.status == IkStatus::JointLimitReached)
    {
        bool isAtJointLimit = false;
        for (std::size_t joint = 0; joint < primaryAttempt.jointPositionRadians.size(); ++joint)
        {
            const auto& limits = models::hanwha::kHcr12a.joints[joint];
            isAtJointLimit = isAtJointLimit ||
                std::abs(primaryAttempt.jointPositionRadians[joint] - limits.minPositionRadians) <= 1e-8 ||
                std::abs(primaryAttempt.jointPositionRadians[joint] - limits.maxPositionRadians) <= 1e-8;
        }
        EXPECT_TRUE(isAtJointLimit) << "joint-limit status requires the returned IK state to be at an active boundary";
    }
    const IkResult solved = inverse.Solve(target, source);
    ASSERT_EQ(solved.status, IkStatus::Success)
        << "bounded posture alternatives recover an FK-reachable target from a singular home seed";
    const CartesianPose recovered = inverse.EvaluateTcp(solved.jointPositionRadians);
    EXPECT_LT(PositionDistance(recovered, target), 2e-5)
        << "fallback solution preserves the FK-reachable target position";
    EXPECT_LT(OrientationDistance(recovered, target), 2e-4)
        << "fallback solution preserves the FK-reachable target orientation";
    EXPECT_LT(solved.iterations, 200U) << "successful fallback reports the iterations from its converged seed";
}

TEST(RobotInverseKinematicsTests, AppliesToolOffsetAndRejectsInvalidInputs)
{
    std::array<models::JointSpecification, 1> joints{};
    const double halfQuarterTurn = std::acos(-1.0) / 4.0;
    models::Pose3 tcpOffset{{0.3, 0.0, 0.0}, {std::cos(halfQuarterTurn), 0.0, 0.0, std::sin(halfQuarterTurn)}};
    const auto specification = MakeSingleJointModel(joints, {});
    DampedLeastSquaresIk toolOffsetIk(specification, tcpOffset);
    const CartesianPose offsetPose = toolOffsetIk.EvaluateTcp({0.0});
    EXPECT_NEAR(offsetPose.positionMeters[0], 0.3, 1e-12) << "TCP offset translation x";
    EXPECT_NEAR(offsetPose.positionMeters[1], 0.0, 1e-12) << "TCP offset translation y";
    EXPECT_NEAR(offsetPose.orientationXyzw[2], std::sin(halfQuarterTurn), 1e-12)
        << "model wxyz quaternion is returned as TCP xyzw";
    EXPECT_NEAR(offsetPose.orientationXyzw[3], std::cos(halfQuarterTurn), 1e-12)
        << "TCP orientation retains the tool offset rotation";

    DampedLeastSquaresIk inverse(specification, tcpOffset);
    models::RobotSpecification withoutToolFrame = specification;
    withoutToolFrame.hasToolFrame = false;
    DampedLeastSquaresIk missingTool(withoutToolFrame);
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
    DampedLeastSquaresIk limited(limitedModel);
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
        DampedLeastSquaresIk hcr(models::hanwha::kHcr12a);
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

    std::array<models::JointSpecification, 1> unconstrainedJoints{};
    const auto unconstrainedModel = MakeSingleJointModel(
        unconstrainedJoints, {{1.0, 0.0, 0.0}, {}}, -3.0, 3.0);
    DampedLeastSquaresIk damped(unconstrainedModel);
    const CartesianPose smallTurn = damped.EvaluateTcp({0.04});
    IkOptions dampedOptions;
    dampedOptions.damping = 1e-2;
    const IkResult nearSingular = damped.Solve(smallTurn, {0.0}, dampedOptions);
    ASSERT_EQ(nearSingular.status, IkStatus::Success) << "damping stabilizes a nearly aligned arm pose";
    EXPECT_TRUE(std::isfinite(nearSingular.jointPositionRadians[0]) &&
        std::abs(nearSingular.jointPositionRadians[0]) <= 3.0)
        << "damped near-singular result remains finite and within limits";
}
