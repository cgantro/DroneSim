#include "robotics/planning/JointPathPlanner.h"
#include "robotics/planning/LinearPathPlanner.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string_view>
#include <vector>

using namespace grasplink::robotics;
using namespace grasplink::robotics::planning;

namespace
{
const std::array<models::JointSpecification, 6> kJoints{{
    {"J1", {}, {}, 0.0, 1.0, 1.0},
    {"J2", {}, {}, 0.0, 1.0, 1.0},
    {"J3", {}, {}, 0.0, 1.0, 1.0},
    {"J4", {}, {}, 0.0, 1.0, 1.0},
    {"J5", {}, {}, 0.0, 1.0, 1.0},
    {"J6", {}, {}, 0.0, 1.0, 1.0}
}};

const models::RobotSpecification kSpecification{
    "Test", "JointPath", kJoints.data(), kJoints.size(), nullptr, 0, {}, false};

JointPathPlannerOptions TestOptions()
{
    JointPathPlannerOptions options;
    options.validationPolicy.jointCollisionSampleSpacingRadians = 0.025;
    options.maximumIterations = 3000;
    options.maximumNodesPerTree = 1500;
    options.maximumShortcutAttempts = 32;
    options.extensionStepFraction = 0.12;
    options.goalBias = 0.18;
    options.randomSeed = 42;
    return options;
}

JointPathPlanningState RunToEnd(JointPathPlanningJob& job)
{
    JointPathPlanningState state = job.GetState();
    for (std::size_t batch = 0; batch < 10000 && state == JointPathPlanningState::Running; ++batch)
        state = job.Advance(16);
    return state;
}

bool IsObstacle(const JointVector& joints)
{
    return joints[0] >= 0.42 && joints[0] <= 0.58 &&
        joints[1] >= 0.35 && joints[1] <= 0.65;
}

StateValidityChecker ObstacleChecker()
{
    return [](const JointVector& joints)
    {
        return IsObstacle(joints) ? JointStateInvalidity::EnvironmentCollision :
            JointStateInvalidity::None;
    };
}
}

TEST(JointPathPlanner, ReturnsValidatedDirectPathWithoutTreeSearch)
{
    JointPathPlanningJob job;
    const JointVector start{0.1, 0.1, 0.2, 0.3, 0.4, 0.5};
    const JointVector goal{0.2, 0.15, 0.25, 0.35, 0.45, 0.55};
    ASSERT_TRUE(job.Begin(kSpecification, start, goal, {}, TestOptions()));
    ASSERT_EQ(job.GetState(), JointPathPlanningState::Running);
    EXPECT_EQ(RunToEnd(job), JointPathPlanningState::Completed);
    EXPECT_EQ(job.GetState(), JointPathPlanningState::Completed);
    ASSERT_EQ(job.GetPlan().points.size(), 2u);
    EXPECT_EQ(job.GetPlan().points.front(), start);
    EXPECT_EQ(job.GetPlan().points.back(), goal);
    EXPECT_EQ(job.GetPlan().iterations, 0u);
}

TEST(JointPathPlanner, AdvanceBoundsValidityCallbacksAndDirectEdgeSampling)
{
    std::size_t callbackCalls = 0;
    const StateValidityChecker checker = [&callbackCalls](const JointVector&)
    {
        ++callbackCalls;
        return JointStateInvalidity::None;
    };
    JointPathPlanningJob job;
    const JointVector start{0.1, 0.1, 0.2, 0.3, 0.4, 0.5};
    const JointVector goal{0.2, 0.15, 0.25, 0.35, 0.45, 0.55};
    ASSERT_TRUE(job.Begin(kSpecification, start, goal, checker, TestOptions()));
    EXPECT_EQ(callbackCalls, 0u);

    const std::size_t firstBatchCalls = callbackCalls;
    ASSERT_EQ(job.Advance(2), JointPathPlanningState::Running);
    EXPECT_EQ(callbackCalls - firstBatchCalls, 2u);

    while (job.GetState() == JointPathPlanningState::Running)
    {
        const std::size_t before = callbackCalls;
        job.Advance(3);
        EXPECT_LE(callbackCalls - before, 3u);
    }
    EXPECT_EQ(job.GetState(), JointPathPlanningState::Completed);
    EXPECT_EQ(callbackCalls, 6u);
    EXPECT_EQ(job.GetPlan().validityChecks, callbackCalls);
}

TEST(JointPathPlanner, FindsDetourAroundJointSpaceObstacle)
{
    JointPathPlanningJob job;
    JointPathPlannerOptions options = TestOptions();
    const JointVector start{0.1, 0.5, 0.5, 0.5, 0.5, 0.5};
    const JointVector goal{0.9, 0.5, 0.5, 0.5, 0.5, 0.5};
    ASSERT_TRUE(job.Begin(kSpecification, start, goal, ObstacleChecker(), options));
    ASSERT_EQ(job.GetState(), JointPathPlanningState::Running);
    ASSERT_EQ(RunToEnd(job), JointPathPlanningState::Completed) << job.GetResult().message;

    const auto& points = job.GetPlan().points;
    ASSERT_GE(points.size(), 3u);
    EXPECT_EQ(points.front(), start);
    EXPECT_EQ(points.back(), goal);
    for (std::size_t edge = 1; edge < points.size(); ++edge)
    {
        EXPECT_EQ(ValidateJointPath(points[edge - 1], points[edge], kSpecification,
            ObstacleChecker(), options.validationPolicy), JointStateInvalidity::None);
        EXPECT_FALSE(IsObstacle(points[edge]));
    }
}

TEST(JointPathPlanner, StopsWhenIterationBudgetIsExhausted)
{
    JointPathPlanningJob job;
    JointPathPlannerOptions options = TestOptions();
    options.maximumIterations = 3;
    options.maximumNodesPerTree = 16;
    const JointVector start{0.1, 0.5, 0.5, 0.5, 0.5, 0.5};
    const JointVector goal{0.9, 0.5, 0.5, 0.5, 0.5, 0.5};
    const StateValidityChecker wall = [](const JointVector& joints)
    {
        return std::abs(joints[0] - 0.5) < 0.07 ? JointStateInvalidity::EnvironmentCollision :
            JointStateInvalidity::None;
    };
    ASSERT_TRUE(job.Begin(kSpecification, start, goal, wall, options));
    EXPECT_EQ(RunToEnd(job), JointPathPlanningState::Failed);
    EXPECT_EQ(job.GetPlan().iterations, 0u);
    EXPECT_EQ(job.GetResult().code, ErrorCode::Unreachable);
}

TEST(JointPathPlanner, RejectsInvalidEndpointAndCancelsRunningPlan)
{
    JointPathPlanningJob invalidJob;
    JointVector invalid{0.1, 0.5, 0.5, 0.5, 0.5, 0.5};
    invalid[2] = 1.1;
    EXPECT_FALSE(invalidJob.Begin(kSpecification, invalid,
        {0.9, 0.5, 0.5, 0.5, 0.5, 0.5}, {}, TestOptions()));
    EXPECT_EQ(invalidJob.GetState(), JointPathPlanningState::Failed);

    JointPathPlanningJob job;
    ASSERT_TRUE(job.Begin(kSpecification,
        {0.1, 0.5, 0.5, 0.5, 0.5, 0.5},
        {0.9, 0.5, 0.5, 0.5, 0.5, 0.5}, ObstacleChecker(), TestOptions()));
    ASSERT_EQ(job.GetState(), JointPathPlanningState::Running);
    job.Cancel();
    EXPECT_EQ(job.GetState(), JointPathPlanningState::Cancelled);
    EXPECT_EQ(job.GetResult().code, ErrorCode::Cancelled);
}

TEST(JointPathPlanner, PreservesCollisionReasonForInvalidStartAndGoal)
{
    const JointVector start{0.1, 0.5, 0.5, 0.5, 0.5, 0.5};
    const JointVector goal{0.9, 0.5, 0.5, 0.5, 0.5, 0.5};
    JointPathPlanningJob job;
    const StateValidityChecker goalCollision = [&](const JointVector& joints)
    {
        return joints == goal ? JointStateInvalidity::SelfCollision : JointStateInvalidity::None;
    };

    ASSERT_TRUE(job.Begin(kSpecification, start, goal, goalCollision, TestOptions()));
    ASSERT_EQ(job.Advance(2), JointPathPlanningState::Failed);
    EXPECT_EQ(job.GetResult().code, ErrorCode::SelfCollision);
    EXPECT_NE(job.GetResult().message.find("planned robot links overlap"), std::string::npos);
}

TEST(JointPathPlanner, FixedSeedProducesRepeatablePath)
{
    const JointVector start{0.1, 0.5, 0.5, 0.5, 0.5, 0.5};
    const JointVector goal{0.9, 0.5, 0.5, 0.5, 0.5, 0.5};
    JointPathPlanningJob first;
    JointPathPlanningJob second;
    const auto options = TestOptions();
    ASSERT_TRUE(first.Begin(kSpecification, start, goal, ObstacleChecker(), options));
    ASSERT_TRUE(second.Begin(kSpecification, start, goal, ObstacleChecker(), options));
    ASSERT_EQ(RunToEnd(first), JointPathPlanningState::Completed);
    ASSERT_EQ(RunToEnd(second), JointPathPlanningState::Completed);
    ASSERT_EQ(first.GetPlan().points.size(), second.GetPlan().points.size());
    for (std::size_t point = 0; point < first.GetPlan().points.size(); ++point)
    {
        ASSERT_EQ(first.GetPlan().points[point].size(), second.GetPlan().points[point].size());
        for (std::size_t joint = 0; joint < first.GetPlan().points[point].size(); ++joint)
            EXPECT_DOUBLE_EQ(first.GetPlan().points[point][joint], second.GetPlan().points[point][joint]);
    }
}
