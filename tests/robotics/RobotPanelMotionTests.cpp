#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/planning/WristAlignmentPlanner.h"

#include <array>
#include <cmath>
#include <gtest/gtest.h>

using namespace grasplink::robotics;
using grasplink::robotics::planning::PlanWristOnlyTarget;
using grasplink::robotics::backends::simulation::SimRobotController;

namespace
{
struct Quaternion
{
    double w;
    double x;
    double y;
    double z;
};

Quaternion Multiply(const Quaternion& a, const Quaternion& b)
{
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

std::array<double, 4> RotatedOrientationXyzw(const RobotState& state,
    const std::array<double, 3>& axis, double angleRadians)
{
    const Quaternion current{state.tcpPose.orientationXyzw[3], state.tcpPose.orientationXyzw[0],
        state.tcpPose.orientationXyzw[1], state.tcpPose.orientationXyzw[2]};
    const double halfAngle = angleRadians * 0.5;
    const double sine = std::sin(halfAngle);
    const Quaternion rotation{std::cos(halfAngle), axis[0] * sine, axis[1] * sine, axis[2] * sine};
    const Quaternion target = Multiply(rotation, current);
    return {target.x, target.y, target.z, target.w};
}

}

TEST(RobotPanelMotionTests, WristAlignmentChangesOnlyJ6AndChecksAttachedBoxSweep)
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "wrist planning controller connects";
    JointVector downFacingJoints(6, 0.0);
    downFacingJoints[4] = 1.57079632679489661923;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint({downFacingJoints, 1.0, 1.0})))
        << "the wrist first reaches the down-facing grasp orientation";
    for (int tick = 0; tick < 1000 && controller.GetState().mode == RobotMode::Moving; ++tick)
        controller.Update(0.01);
    ASSERT_EQ(controller.GetState().mode, RobotMode::Idle) << "the down-facing wrist orientation is reached";
    const RobotState state = controller.GetState();
    const std::array<double, 4> targetXyzw = RotatedOrientationXyzw(state, {0.0, -1.0, 0.0}, 0.35);

    const auto plan = PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw);
    ASSERT_TRUE(plan.has_value()) << "an on-axis TCP accepts a pure J6 rotation";
    ASSERT_EQ(plan->jointPositionRadians.size(), state.jointPositionRadians.size()) << "wrist plan preserves joint count";
    for (std::size_t joint = 0; joint + 1 < state.jointPositionRadians.size(); ++joint)
        EXPECT_NEAR(plan->jointPositionRadians[joint], state.jointPositionRadians[joint], 1e-12)
            << "wrist alignment does not change J1 through J5";
    EXPECT_NEAR(plan->jointPositionRadians.back(), 0.35, 1e-10)
        << "wrist alignment changes J6 by the requested amount";

    const std::array<double, 3> axialBoxOffset{0.0, 0.0, 0.025};
    EXPECT_TRUE(PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw, axialBoxOffset).has_value())
        << "an attached box centered on the J6 axis stays in place during wrist alignment";
    RobotState measuredToolOffset = state;
    measuredToolOffset.tcpPose.positionMeters[0] += 0.00144;
    EXPECT_TRUE(PlanWristOnlyTarget(models::hanwha::kHcr12a, measuredToolOffset, targetXyzw, axialBoxOffset).has_value())
        << "the measured 1.44 mm fingertip-center offset stays within the bounded wrist sweep";
    const std::array<double, 3> radialBoxOffset{0.02, 0.0, 0.0};
    const auto radialBoxPlan = PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw, radialBoxOffset);
    ASSERT_TRUE(radialBoxPlan.has_value()) << "a small attached-box arc does not block wrist alignment";
    EXPECT_LT(radialBoxPlan->attachedBoxSweepMeters, 0.02)
        << "the wrist plan reports the attached box center movement during rotation";
    const grasplink::robotics::planning::WristAlignmentPolicy strictSweepPolicy{0.002, 0.005};
    EXPECT_FALSE(PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw, radialBoxOffset,
        strictSweepPolicy).has_value()) << "a caller can tighten the maximum attached-box sweep policy";
    const std::array<double, 3> largeRadialBoxOffset{0.10, 0.0, 0.0};
    EXPECT_FALSE(PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw, largeRadialBoxOffset).has_value())
        << "a wrist rotation is rejected when the attached box sweep would exceed two centimeters";

    RobotState offAxisTcp = state;
    offAxisTcp.tcpPose.positionMeters[0] += 0.01;
    EXPECT_FALSE(PlanWristOnlyTarget(models::hanwha::kHcr12a, offAxisTcp, targetXyzw).has_value())
        << "wrist alignment is rejected when the actual TCP is offset from the J6 axis";

    EXPECT_FALSE(PlanWristOnlyTarget(models::hanwha::kHcr12a, state,
        RotatedOrientationXyzw(state, {1.0, 0.0, 0.0}, 0.2)).has_value())
        << "a requested tilt is rejected because J6 cannot produce it alone";
}

TEST(RobotPanelMotionTests, WristAlignmentRespectsJ6JointLimits)
{
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "limit fixture controller connects";
    JointVector start(6, 0.0);
    constexpr double pi = 3.14159265358979323846;
    start.back() = 350.0 * pi / 180.0;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint({start, 1.0, 1.0}))) << "J6 moves to 350 degrees";
    for (int tick = 0; tick < 1000 && controller.GetState().mode == RobotMode::Moving; ++tick)
        controller.Update(0.01);
    ASSERT_EQ(controller.GetState().mode, RobotMode::Idle) << "J6 reaches 350 degrees";

    const RobotState state = controller.GetState();
    const auto plan = PlanWristOnlyTarget(models::hanwha::kHcr12a, state,
        RotatedOrientationXyzw(state, {0.0, 0.0, 1.0}, 20.0 * pi / 180.0));
    ASSERT_TRUE(plan.has_value()) << "an equivalent J6 target inside the official range remains available";
    EXPECT_NEAR(plan->jointPositionRadians.back(), 10.0 * pi / 180.0, 1e-9)
        << "J6 target is represented at 10 degrees rather than exceeding its 360 degree upper limit";
    EXPECT_TRUE(plan->jointPositionRadians.back() >= models::hanwha::kHcr12a.joints[5].minPositionRadians &&
        plan->jointPositionRadians.back() <= models::hanwha::kHcr12a.joints[5].maxPositionRadians)
        << "wrist target stays inside the J6 hard limits";
}
