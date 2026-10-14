#include "robotics/planning/LinearPathPlanner.h"

#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

using namespace grasplink::robotics;
using namespace grasplink::robotics::planning;
using grasplink::robotics::kinematics::DampedLeastSquaresIk;

namespace
{
double PositionError(const CartesianPose& left, const CartesianPose& right)
{
    double squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const double difference = left.positionMeters[axis] - right.positionMeters[axis];
        squared += difference * difference;
    }
    return std::sqrt(squared);
}

double OrientationError(const CartesianPose& left, const CartesianPose& right)
{
    double dot = 0.0;
    for (std::size_t axis = 0; axis < 4; ++axis)
        dot += left.orientationXyzw[axis] * right.orientationXyzw[axis];
    return 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
}

std::vector<CartesianPose> BuildSamples(const CartesianPose& start, const CartesianPose& end,
    const PlanningPolicy& policy)
{
    const double distance = PositionError(start, end);
    const double rotation = OrientationError(start, end);
    const std::size_t count = std::max<std::size_t>(1, static_cast<std::size_t>(std::max(
        std::ceil(distance / policy.linearPositionSampleSpacingMeters),
        std::ceil(rotation / policy.linearOrientationSampleSpacingRadians))));
    std::vector<CartesianPose> samples;
    samples.reserve(count);
    const auto startPose = grasplink::robotics::kinematics::detail::FromCartesian(start);
    const auto endPose = grasplink::robotics::kinematics::detail::FromCartesian(end);
    for (std::size_t i = 1; i <= count; ++i)
    {
        const double fraction = static_cast<double>(i) / static_cast<double>(count);
        samples.push_back(grasplink::robotics::kinematics::detail::ToCartesian(
            grasplink::robotics::kinematics::detail::Interpolate(startPose, endPose, fraction)));
    }
    return samples;
}

bool BuildGreedyPreferredPath(const std::vector<CartesianPose>& samples,
    const StateValidityChecker& checker, std::vector<JointVector>& joints)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    JointVector current(specification.jointCount, 0.0);
    const auto options = PathIkOptions();
    for (const auto& sample : samples)
    {
        const auto solution = inverse.SolveSingleSeed(sample, current, options);
        if (!solution)
            return false;
        JointVector target = solution.jointPositionRadians;
        AlignEquivalentJointAngles(target, current, specification);
        if (ValidateJointPath(current, target, specification, checker) != JointStateInvalidity::None)
            return false;
        joints.push_back(target);
        current = std::move(target);
    }
    return true;
}

TEST(LinearPathPlanner, SelectsAContinuousAlternateBranchBeforeGreedyBranchIsBlocked)
{
    const auto& specification = models::hanwha::kHcr12a;
    const JointVector startJoints(specification.jointCount, 0.0);
    DampedLeastSquaresIk inverse(specification);
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    const std::array<JointVector, 1> targets{{
        {0.22, -0.1, 0.08, 0.1, -0.06, 0.12}}};

    bool foundGreedyFailureWithGlobalRecovery = false;
    for (const auto& targetJoints : targets)
    {
        const CartesianPose targetTcp = inverse.EvaluateTcp(targetJoints);
        LinearPathMoveCommand command;
        command.targetPoses.push_back(targetTcp);
        const auto samples = BuildSamples(startTcp, targetTcp, kDefaultPlanningPolicy);
        std::vector<JointVector> greedyJoints;
        if (!BuildGreedyPreferredPath(samples, {}, greedyJoints) || greedyJoints.size() < 4)
            continue;

        for (const std::size_t blockedIndex : {greedyJoints.size() / 2})
        {
            const JointVector blockedBranchState = greedyJoints[blockedIndex];
            const StateValidityChecker checker = [blockedBranchState](const JointVector& candidate)
            {
                double distanceSquared = 0.0;
                for (std::size_t joint = 0; joint < candidate.size(); ++joint)
                {
                    const double difference = candidate[joint] - blockedBranchState[joint];
                    distanceSquared += difference * difference;
                }
                return distanceSquared < 1e-5 * 1e-5 ?
                    JointStateInvalidity::EnvironmentCollision : JointStateInvalidity::None;
            };
            std::vector<JointVector> blockedGreedyPath;
            if (BuildGreedyPreferredPath(samples, checker, blockedGreedyPath))
                continue;

            LinearPathPlan plan;
            const Result result = BuildLinearPath(command, specification, startJoints, startTcp,
                inverse, checker, plan);
            if (!result)
                continue;
            foundGreedyFailureWithGlobalRecovery = true;
            break;
        }
        if (foundGreedyFailureWithGlobalRecovery)
            break;
    }

    ASSERT_TRUE(foundGreedyFailureWithGlobalRecovery)
        << "layered IK candidates retain a safe branch when the closest local branch becomes invalid later";
}

TEST(LinearPathPlanner, RefinesJointInterpolationUntilTcpFollowsTheRequestedLine)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    const JointVector targetJoints{0.55, -0.8, 0.7, 0.45, -0.6, 0.3};
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    const CartesianPose targetTcp = inverse.EvaluateTcp(targetJoints);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(targetTcp);
    PlanningPolicy policy = kDefaultPlanningPolicy;
    policy.maximumLinearTcpErrorMeters = 0.00025;
    policy.maximumAngularTcpErrorRadians = 0.00025;
    LinearPathPlan plan;

    const Result result = BuildLinearPath(command, specification, startJoints, startTcp,
        inverse, {}, plan, policy);

    ASSERT_TRUE(static_cast<bool>(result)) << result.message;
    ASSERT_GT(plan.points.size(), 2u) << "curved joint interpolation adds Cartesian refinement samples";
    for (std::size_t edge = 1; edge < plan.points.size(); ++edge)
    {
        for (std::size_t probe = 1; probe < 20; ++probe)
        {
            const double fraction = static_cast<double>(probe) / 20.0;
            JointVector joints(specification.jointCount);
            for (std::size_t joint = 0; joint < joints.size(); ++joint)
                joints[joint] = plan.points[edge - 1].joints[joint] + fraction *
                    (plan.points[edge].joints[joint] - plan.points[edge - 1].joints[joint]);
            const CartesianPose actualTcp = inverse.EvaluateTcp(joints);
            const CartesianPose expectedTcp = grasplink::robotics::kinematics::detail::ToCartesian(
                grasplink::robotics::kinematics::detail::Interpolate(
                    grasplink::robotics::kinematics::detail::FromCartesian(plan.points[edge - 1].tcpPose),
                    grasplink::robotics::kinematics::detail::FromCartesian(plan.points[edge].tcpPose), fraction));
            ASSERT_LE(PositionError(actualTcp, expectedTcp), policy.maximumLinearTcpErrorMeters + 1e-8)
                << "FK remains inside the TCP position tolerance at an independent 5%-spaced probe";
            ASSERT_LE(OrientationError(actualTcp, expectedTcp), policy.maximumAngularTcpErrorRadians + 1e-8)
                << "FK orientation remains inside tolerance at an independent 5%-spaced probe";
        }
    }
}

TEST(LinearPathPlanner, IncrementalJobMatchesSynchronousPlanAndPreservesValidityChecks)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk synchronousIk(specification);
    DampedLeastSquaresIk incrementalIk(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    const CartesianPose startTcp = synchronousIk.EvaluateTcp(startJoints);
    const CartesianPose target = synchronousIk.EvaluateTcp({0.12, -0.08, 0.06, 0.12, -0.04, 0.1});
    LinearPathMoveCommand command;
    command.targetPoses.push_back(target);
    std::size_t synchronousValidityCalls = 0;
    std::size_t incrementalValidityCalls = 0;
    const auto synchronousChecker = [&](const JointVector&) {
        ++synchronousValidityCalls;
        return JointStateInvalidity::None;
    };
    const auto incrementalChecker = [&](const JointVector&) {
        ++incrementalValidityCalls;
        return JointStateInvalidity::None;
    };

    LinearPathPlan synchronousPlan;
    ASSERT_TRUE(BuildLinearPath(command, specification, startJoints, startTcp,
        synchronousIk, synchronousChecker, synchronousPlan));

    LinearPathPlanningJob job;
    ASSERT_TRUE(job.Begin(command, specification, startJoints, startTcp,
        incrementalIk, incrementalChecker));
    std::size_t advances = 0;
    while (job.GetState() == LinearPathPlanningState::Running)
    {
        const auto& before = job.GetPlan();
        const std::array<std::size_t, 3> beforeWork{
            before.ikIterationCount, before.tcpStraightnessChecks, before.validityStateChecks};
        const auto nextState = job.Advance(1);
        EXPECT_TRUE(nextState == LinearPathPlanningState::Running ||
            nextState == LinearPathPlanningState::Completed);
        const auto& after = job.GetPlan();
        const std::size_t ikWork = after.ikIterationCount - beforeWork[0];
        const std::size_t tcpWork = after.tcpStraightnessChecks - beforeWork[1];
        const std::size_t validityWork = after.validityStateChecks - beforeWork[2];
        const std::size_t consumedWork = ikWork + tcpWork + validityWork;
        EXPECT_LE(consumedWork, 1u)
            << "one interactive work unit never performs multiple DLS, TCP, or validity checks; "
            << "DLS=" << ikWork << ", TCP=" << tcpWork << ", validity=" << validityWork;
        ++advances;
    }
    ASSERT_EQ(job.GetState(), LinearPathPlanningState::Completed);
    const auto& incrementalPlan = job.GetPlan();
    EXPECT_GT(advances, 1u) << "the job yields between TCP samples";
    ASSERT_EQ(incrementalPlan.points.size(), synchronousPlan.points.size());
    EXPECT_EQ(incrementalPlan.ikSolveCount, synchronousPlan.ikSolveCount);
    EXPECT_EQ(incrementalPlan.ikIterationCount, synchronousPlan.ikIterationCount);
    EXPECT_EQ(incrementalPlan.tcpStraightnessChecks, synchronousPlan.tcpStraightnessChecks);
    EXPECT_EQ(incrementalPlan.jointPathValidityChecks, synchronousPlan.jointPathValidityChecks);
    EXPECT_EQ(incrementalPlan.validityStateChecks, synchronousPlan.validityStateChecks);
    EXPECT_EQ(incrementalPlan.tcpRefinementCount, synchronousPlan.tcpRefinementCount);
    EXPECT_EQ(incrementalValidityCalls, synchronousValidityCalls)
        << "incremental planning retains every joint-path collision check";
    for (std::size_t point = 0; point < synchronousPlan.points.size(); ++point)
    {
        ASSERT_EQ(incrementalPlan.points[point].joints.size(), synchronousPlan.points[point].joints.size());
        for (std::size_t joint = 0; joint < synchronousPlan.points[point].joints.size(); ++joint)
            EXPECT_DOUBLE_EQ(incrementalPlan.points[point].joints[joint], synchronousPlan.points[point].joints[joint]);
        EXPECT_DOUBLE_EQ(incrementalPlan.points[point].durationSeconds,
            synchronousPlan.points[point].durationSeconds);
        for (std::size_t axis = 0; axis < 3; ++axis)
            EXPECT_DOUBLE_EQ(incrementalPlan.points[point].tcpPose.positionMeters[axis],
                synchronousPlan.points[point].tcpPose.positionMeters[axis]);
        for (std::size_t axis = 0; axis < 4; ++axis)
            EXPECT_DOUBLE_EQ(incrementalPlan.points[point].tcpPose.orientationXyzw[axis],
                synchronousPlan.points[point].tcpPose.orientationXyzw[axis]);
    }

    DampedLeastSquaresIk batchedIk(specification);
    LinearPathPlanningJob batchedJob;
    ASSERT_TRUE(batchedJob.Begin(command, specification, startJoints, startTcp,
        batchedIk, {}));
    while (batchedJob.GetState() == LinearPathPlanningState::Running)
    {
        const auto& before = batchedJob.GetPlan();
        const std::array<std::size_t, 3> beforeWork{
            before.ikIterationCount, before.tcpStraightnessChecks, before.validityStateChecks};
        const auto nextState = batchedJob.Advance(11);
        EXPECT_TRUE(nextState == LinearPathPlanningState::Running ||
            nextState == LinearPathPlanningState::Completed);
        const auto& after = batchedJob.GetPlan();
        const std::size_t consumedWork = after.ikIterationCount - beforeWork[0] +
            after.tcpStraightnessChecks - beforeWork[1] +
            after.validityStateChecks - beforeWork[2];
        EXPECT_LE(consumedWork, 11u)
            << "batched planning never exceeds the caller's work budget";
    }
    ASSERT_EQ(batchedJob.GetState(), LinearPathPlanningState::Completed)
        << batchedJob.GetResult().message;
    const auto& batchedPlan = batchedJob.GetPlan();
    EXPECT_DOUBLE_EQ(batchedPlan.plannedLinearVelocity, synchronousPlan.plannedLinearVelocity);
    EXPECT_DOUBLE_EQ(batchedPlan.plannedAngularVelocity, synchronousPlan.plannedAngularVelocity);
    EXPECT_EQ(batchedPlan.hasMotion, synchronousPlan.hasMotion);
    EXPECT_EQ(batchedPlan.ikSolveCount, synchronousPlan.ikSolveCount);
    EXPECT_EQ(batchedPlan.ikIterationCount, synchronousPlan.ikIterationCount);
    EXPECT_EQ(batchedPlan.tcpStraightnessChecks, synchronousPlan.tcpStraightnessChecks);
    EXPECT_EQ(batchedPlan.jointPathValidityChecks, synchronousPlan.jointPathValidityChecks);
    EXPECT_EQ(batchedPlan.validityStateChecks, synchronousPlan.validityStateChecks);
    EXPECT_EQ(batchedPlan.tcpRefinementCount, synchronousPlan.tcpRefinementCount);
    ASSERT_EQ(batchedPlan.points.size(), synchronousPlan.points.size());
    for (std::size_t point = 0; point < synchronousPlan.points.size(); ++point)
    {
        ASSERT_EQ(batchedPlan.points[point].joints.size(), synchronousPlan.points[point].joints.size());
        for (std::size_t joint = 0; joint < synchronousPlan.points[point].joints.size(); ++joint)
            EXPECT_DOUBLE_EQ(batchedPlan.points[point].joints[joint], synchronousPlan.points[point].joints[joint]);
        EXPECT_DOUBLE_EQ(batchedPlan.points[point].durationSeconds,
            synchronousPlan.points[point].durationSeconds);
        for (std::size_t axis = 0; axis < 3; ++axis)
            EXPECT_DOUBLE_EQ(batchedPlan.points[point].tcpPose.positionMeters[axis],
                synchronousPlan.points[point].tcpPose.positionMeters[axis]);
        for (std::size_t axis = 0; axis < 4; ++axis)
            EXPECT_DOUBLE_EQ(batchedPlan.points[point].tcpPose.orientationXyzw[axis],
                synchronousPlan.points[point].tcpPose.orientationXyzw[axis]);
    }
}

TEST(LinearPathPlanner, ReportsBoundedFailureWhenTcpRefinementBudgetIsExhausted)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    const JointVector targetJoints{0.8, -0.9, 0.7, 0.6, -0.8, 0.5};
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp(targetJoints));
    PlanningPolicy policy = kDefaultPlanningPolicy;
    policy.maximumLinearTcpErrorMeters = 1e-12;
    policy.maximumAngularTcpErrorRadians = 1e-12;
    policy.maximumTcpRefinementPasses = 0;
    LinearPathPlan plan;

    const Result result = BuildLinearPath(command, specification, startJoints, startTcp,
        inverse, {}, plan, policy);

    ASSERT_EQ(result.code, ErrorCode::IkDidNotConverge)
        << "the planner rejects a path whose joint interpolation exceeds TCP tolerance without exceeding its refinement budget";
    ASSERT_TRUE(plan.points.empty()) << "failed refinement does not publish a partial path";
}

TEST(LinearPathPlanner, RestartingAnActiveJobCancelsItsPreviousIkSession)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    LinearPathMoveCommand firstCommand;
    firstCommand.targetPoses.push_back(inverse.EvaluateTcp({0.1, -0.1, 0.05, 0.1, -0.05, 0.1}));
    LinearPathMoveCommand replacementCommand;
    replacementCommand.targetPoses.push_back(inverse.EvaluateTcp({-0.1, -0.1, 0.05, 0.1, -0.05, 0.1}));
    LinearPathPlanningJob job;

    ASSERT_TRUE(job.Begin(firstCommand, specification, startJoints, startTcp, inverse, {}));
    ASSERT_EQ(job.Advance(1), LinearPathPlanningState::Running);
    ASSERT_TRUE(job.Begin(replacementCommand, specification, startJoints, startTcp, inverse, {}));
    for (std::size_t work = 0; work < 20000 && job.GetState() == LinearPathPlanningState::Running; ++work)
        job.Advance(1);

    ASSERT_EQ(job.GetState(), LinearPathPlanningState::Completed) << job.GetResult().message;
    ASSERT_FALSE(job.GetPlan().points.empty());
    const auto& finalJoints = job.GetPlan().points.back().joints;
    EXPECT_LT(finalJoints[0], 0.0) << "the replacement job, not the cancelled job, supplies the result";
}

TEST(LinearPathPlanner, RejectsSeedBudgetSmallerThanCandidateBeam)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(startTcp);
    PlanningPolicy policy = kDefaultPlanningPolicy;
    policy.maximumIkCandidatesPerSample = 4;
    policy.maximumIkSeedAttemptsPerSample = 3;
    LinearPathPlan plan;

    const Result result = BuildLinearPath(command, specification, startJoints, startTcp,
        inverse, {}, plan, policy);

    ASSERT_EQ(result.code, ErrorCode::InvalidCommand)
        << "the declared IK seed budget must cover each mandatory beam continuation";
    ASSERT_TRUE(plan.points.empty()) << "invalid planner policy does not publish a path";
}

TEST(LinearPathPlanner, InteractivePolicyKeepsTheSeedSearchBounded)
{
    EXPECT_EQ(kInteractivePlanningPolicy.maximumIkSeedAttemptsPerSample, 16U);
    EXPECT_LE(kInteractivePlanningPolicy.maximumIkSeedAttemptsPerSample,
        kDefaultPlanningPolicy.maximumIkSeedAttemptsPerSample);
}

TEST(LinearPathPlanner, KeepsTheDiagnosticFromTheRejectedCandidate)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    const JointVector targetJoints{0.12, -0.08, 0.06, 0.12, -0.04, 0.1};
    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp(targetJoints));
    command.targetPoses.back().positionMeters[0] += 0.03;
    std::size_t rejectedStates = 0;
    std::string latestDiagnostic;
    const StateValidityChecker checker = [&](const JointVector&)
    {
        latestDiagnostic = "rejected candidate " + std::to_string(++rejectedStates);
        return JointStateInvalidity::SelfCollision;
    };
    const auto diagnosticProvider = [&]() { return latestDiagnostic; };
    LinearPathPlan plan;

    const Result result = BuildLinearPath(command, specification, JointVector(6, 0.0),
        inverse.EvaluateTcp(JointVector(6, 0.0)), inverse, checker, plan,
        kDefaultPlanningPolicy, diagnosticProvider);

    EXPECT_EQ(result.code, ErrorCode::SelfCollision);
    EXPECT_GT(rejectedStates, 1u);
    EXPECT_NE(result.message.find("rejected candidate 1"), std::string::npos)
        << "the result names the first selected rejection, not the guard's later mutable description";
    EXPECT_EQ(result.message.find("rejected candidate " + std::to_string(rejectedStates)), std::string::npos);
}
} // namespace
