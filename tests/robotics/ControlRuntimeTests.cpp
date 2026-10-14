#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/runtime/FixedControlLoop.h"

#include <gtest/gtest.h>
#include <cstddef>
#include <limits>
#include <stdexcept>

using namespace grasplink::robotics;

/**
 * @brief FixedControlLoop와 시뮬레이션 Controller의 시간·명령 계약을 확인한다.
 * @details 고정 간격 누적/초과 시간 제한, 잘못된 경과 시간 무시, 관절 속도 제한과 범위 거부를 검사한다.
 * 또한 정지·재목표 동작과 TCP feedback 및 Cartesian 경로 요청 계약을 확인한다.
 */
TEST(ControlRuntimeTests, FixedLoopAndControllerContracts)
{
    // 입력 시간은 초 단위다. 250 Hz 설정은 4 ms 고정 step으로 나뉘며, 긴 정지 뒤에도 한 번에 최대 100 ms만 누적한다.
    runtime::FixedControlLoop loop(0.004, 0.1);
    std::size_t ticks = 0;
    auto tick = [&](double dt) { EXPECT_NEAR(dt, 0.004, 1e-12) << "fixed dt"; ++ticks; };
    EXPECT_EQ(loop.Advance(0.010, tick), 2) << "two ticks";
    EXPECT_NEAR(loop.GetAccumulatorSeconds(), 0.002, 1e-12) << "remaining time";
    EXPECT_EQ(loop.Advance(0.002, tick), 1) << "carry-over tick";
    loop.Reset();
    loop.Advance(3.0, tick);
    EXPECT_GE(ticks, 27);
    EXPECT_LE(ticks, 28) << "clamp discards excess elapsed time";
    EXPECT_GE(loop.GetInterpolationAlpha(), 0.0);
    EXPECT_LT(loop.GetInterpolationAlpha(), 1.0);
    const auto before = ticks;
    loop.Advance(std::numeric_limits<double>::quiet_NaN(), tick);
    loop.Advance(-1.0, tick);
    EXPECT_EQ(ticks, before) << "invalid elapsed time";

    backends::simulation::SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "connect";
    JointMoveCommand command;
    command.targetPositionRadians.assign(6, 0.0);
    command.targetPositionRadians[0] = 0.5;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint(command))) << "move";
    constexpr double fixedDeltaSeconds = 0.004;
    const double maximumVelocity = models::hanwha::kHcr12a.joints[0].maxVelocityRadiansPerSecond;
    const double accelerationLimit = maximumVelocity / 0.20;
    controller.Update(fixedDeltaSeconds);
    // 관절 가속도는 최대 속도에 0.20초 동안 도달하는 시뮬레이션 정책으로 계산한다.
    EXPECT_NEAR(controller.GetState().jointPositionRadians[0],
        0.5 * accelerationLimit * fixedDeltaSeconds * fixedDeltaSeconds, 1e-12)
        << "the simulation ramp accelerates from rest under its configured acceleration limit";
    const auto unchanged = controller.GetState().jointPositionRadians;
    command.targetPositionRadians[0] = 10.0;
    EXPECT_EQ(controller.MoveJoint(command).code, ErrorCode::InvalidCommand) << "out-of-range command";
    EXPECT_EQ(controller.GetState().jointPositionRadians, unchanged);
    command.targetPositionRadians[0] = -0.5;
    ASSERT_TRUE(static_cast<bool>(controller.MoveJoint(command))) << "re-target while moving";
    controller.Update(fixedDeltaSeconds);
    const double brakingPosition = controller.GetState().jointPositionRadians[0];
    EXPECT_GT(brakingPosition, unchanged[0]) << "retarget first brakes along the already validated path";
    controller.Update(fixedDeltaSeconds);
    EXPECT_LT(controller.GetState().jointPositionRadians[0], brakingPosition)
        << "replacement trajectory reverses only after the old path reaches its stop pose";
    ASSERT_TRUE(static_cast<bool>(controller.Stop())) << "stop";
    const auto stopped = controller.GetState().jointPositionRadians;
    controller.Update(1.0);
    EXPECT_EQ(controller.GetState().jointPositionRadians, stopped);
    EXPECT_TRUE(controller.GetState().tcpPoseValid);
    grasplink::robotics::kinematics::DampedLeastSquaresIk ik(models::hanwha::kHcr12a);
    const CartesianPose homePose = ik.EvaluateTcp(JointVector(6, 0.0));
    ASSERT_TRUE(static_cast<bool>(controller.MovePose(homePose))) << "MovePose accepts reachable TCP target";
    ASSERT_TRUE(static_cast<bool>(controller.MoveLinear({homePose, 0.05, 0.2}))) << "MoveLinear accepts a reachable TCP path";
    controller.Disconnect();
    EXPECT_FALSE(controller.GetState().valid) << "disconnected state invalid";

    auto joint = models::hanwha::kHcr12aJoints.front();
    auto specification = models::hanwha::kHcr12a;
    specification.joints = &joint;
    specification.jointCount = 1;
    joint.maxVelocityRadiansPerSecond = -1.0;
    EXPECT_THROW(backends::simulation::SimRobotController bad(specification), std::invalid_argument);
    joint.maxVelocityRadiansPerSecond = 1.0;
    joint.minPositionRadians = 0.1;
    EXPECT_THROW(backends::simulation::SimRobotController bad(specification), std::invalid_argument);
}
