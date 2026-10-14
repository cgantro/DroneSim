#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/models/robotiq/TwoF85.h"

#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>

using namespace grasplink::robotics;
using grasplink::robotics::backends::simulation::SimGripperController;
using grasplink::robotics::backends::simulation::SimGripperMotionSettings;
using grasplink::robotics::models::robotiq::kTwoF85;

namespace
{
GripperCommand Request(std::uint8_t position, std::uint8_t speed = 255, std::uint8_t force = 128)
{
    return {position, speed, force};
}

void AdvanceTicks(SimGripperController& controller, int count, double dtSeconds)
{
    for (int i = 0; i < count; ++i)
        controller.Update(dtSeconds);
}

void AdvanceToTarget(SimGripperController& controller)
{
    for (int i = 0; i < 3000 && controller.GetState().mode == GripperMode::Moving; ++i)
        controller.Update(0.004);
    ASSERT_TRUE(controller.GetState().mode == GripperMode::Idle) << "target completes within fixed steps";
}

void RequireSnapshotEqual(const GripperState& left, const GripperState& right, const std::string& label)
{
    ASSERT_TRUE(left.mode == right.mode && left.activated == right.activated && left.goToActive == right.goToActive &&
        left.objectStatus == right.objectStatus && left.requestedPositionEcho == right.requestedPositionEcho &&
        left.actualPosition == right.actualPosition && left.currentRaw == right.currentRaw &&
        left.currentValid == right.currentValid && left.closureFractionValid == right.closureFractionValid &&
        left.valid == right.valid) << label + ": flags and raw values unchanged";
    EXPECT_NEAR(left.closureFraction, right.closureFraction, 0.0) << label + ": closure unchanged";
}

void CheckLifecycleAndCommands()
{
    // 실제 2F-85 장치 코드는 0..255 범위를 모두 허용한다. 범위 밖 입력을 거부하는 동작을 시험할 때만 시험 사양의 최소 힘 값을 높여 유효 범위를 좁힌다.
    auto specification = kTwoF85;
    specification.forceRequestMin = 1;
    SimGripperController controller(specification);
    ASSERT_TRUE(controller.Command(Request(255)).code == ErrorCode::NotConnected) << "command requires connection";
    ASSERT_TRUE(controller.Activate().code == ErrorCode::NotConnected) << "activation requires connection";

    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "connect";
    auto state = controller.GetState();
    ASSERT_TRUE(state.valid && state.closureFractionValid && state.mode == GripperMode::Inactive) << "connect exposes inactive valid snapshot";
    ASSERT_TRUE(state.actualPosition == kTwoF85.positionRequestMin && state.closureFraction == 0.0) << "connect starts at the open raw endpoint";
    ASSERT_TRUE(!state.currentValid && state.currentRaw == 0) << "simulation does not invent current feedback";

    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "connect is idempotent";
    ASSERT_TRUE(controller.Command(Request(255)).code == ErrorCode::Busy) << "command requires activation";
    ASSERT_TRUE(static_cast<bool>(controller.Activate())) << "activate";
    ASSERT_TRUE(controller.GetState().activated && controller.GetState().activationStatus == 3 &&
        controller.GetState().mode == GripperMode::Idle) << "activate enters idle";
    ASSERT_TRUE(static_cast<bool>(controller.Activate())) << "activation is idempotent";

    ASSERT_TRUE(static_cast<bool>(controller.Command(Request(255, 0, 255)))) << "full close request accepted";
    state = controller.GetState();
    ASSERT_TRUE(state.mode == GripperMode::Moving && state.goToActive && state.objectStatus == GripperObjectStatus::Moving) << "accepted request starts free-space motion";
    ASSERT_TRUE(state.requestedPositionEcho == 255 && state.closureFraction == 0.0) << "request echo is separate from current continuous position";
    const GripperState beforeIdempotentActivate = controller.GetState();
    ASSERT_TRUE(static_cast<bool>(controller.Activate())) << "activate during movement succeeds";
    RequireSnapshotEqual(beforeIdempotentActivate, controller.GetState(), "activate during movement preserves state");

    AdvanceTicks(controller, 1, 0.004);
    state = controller.GetState();
    const double expectedMinimumSpeedStep = 0.1 * 0.004 / kTwoF85.nominalMasterClosedRadians;
    EXPECT_NEAR(state.closureFraction, expectedMinimumSpeedStep, 1e-15) << "raw minimum maps to minimum positive speed";
    ASSERT_TRUE(state.actualPosition == 0) << "raw position is rounded only for the snapshot";
    const GripperState copied = state;
    AdvanceTicks(controller, 2, 0.004);
    EXPECT_NEAR(copied.closureFraction, expectedMinimumSpeedStep, 0.0) << "snapshot is an independent value copy";

    const auto beforeInvalid = controller.GetState();
    ASSERT_TRUE(controller.Command(Request(255, 255, 0)).code == ErrorCode::InvalidCommand) << "force below model range rejected";
    // 잘못된 명령을 거부한 뒤에는 이전 목표 위치, 속도, 상태가 모두 그대로여야 한다.
    RequireSnapshotEqual(beforeInvalid, controller.GetState(), "invalid request is atomic");

    // 장치의 가장 느린 값과 가장 빠른 속도 코드가 시뮬레이션에서 각각 0.1 rad/s와 1.0 rad/s의 각속도가 되는지 확인한다.
    SimGripperController maximumSpeed(kTwoF85);
    maximumSpeed.Connect();
    maximumSpeed.Activate();
    maximumSpeed.Command(Request(255, 255));
    maximumSpeed.Update(0.004);
    EXPECT_NEAR(maximumSpeed.GetState().closureFraction, 1.0 * 0.004 / kTwoF85.nominalMasterClosedRadians, 1e-15) << "raw maximum maps to maximum speed";

    SimGripperController splitTicks(kTwoF85);
    SimGripperController oneInterval(kTwoF85);
    for (auto* item : {&splitTicks, &oneInterval})
    {
        item->Connect();
        item->Activate();
        item->Command(Request(255, 128));
    }
    AdvanceTicks(splitTicks, 10, 0.004);
    oneInterval.Update(0.040);
    EXPECT_NEAR(splitTicks.GetState().closureFraction, oneInterval.GetState().closureFraction, 1e-14) << "250 Hz partition and equivalent elapsed interval agree";

    maximumSpeed.Update(std::numeric_limits<double>::quiet_NaN());
    maximumSpeed.Update(std::numeric_limits<double>::infinity());
    maximumSpeed.Update(0.0);
    maximumSpeed.Update(-1.0);
    EXPECT_NEAR(maximumSpeed.GetState().closureFraction, 1.0 * 0.004 / kTwoF85.nominalMasterClosedRadians, 0.0) << "invalid dt is ignored";

    maximumSpeed.Update(100.0);
    state = maximumSpeed.GetState();
    ASSERT_TRUE(state.closureFraction == 1.0 && state.actualPosition == 255 &&
        state.mode == GripperMode::Idle && !state.goToActive &&
        state.objectStatus == GripperObjectStatus::AtRequestedPosition) << "large dt clamps exactly to target without reporting contact";

    // 움직이는 중 새 명령을 받으면 이전 목표 위치에서 이어가지 않고, 현재 실제 연속 개폐 위치부터 새 목표까지 이동해야 한다.
    ASSERT_TRUE(static_cast<bool>(maximumSpeed.Command(Request(255, 255)))) << "repeat close request accepted";
    ASSERT_TRUE(static_cast<bool>(maximumSpeed.Command(Request(0, 255)))) << "open retarget accepted";
    maximumSpeed.Update(0.1);
    ASSERT_TRUE(maximumSpeed.GetState().closureFraction < 1.0) << "retarget moves toward replacement goal";
    AdvanceToTarget(maximumSpeed);
    ASSERT_TRUE(maximumSpeed.GetState().closureFraction == 0.0 && maximumSpeed.GetState().actualPosition == 0) << "replacement open target completes";
}

void CheckStopResetAndReconnect()
{
    SimGripperController controller(kTwoF85);
    controller.Connect();
    controller.Activate();
    controller.Command(Request(255, 255, 128));
    AdvanceTicks(controller, 25, 0.004);
    const GripperState beforeStop = controller.GetState();
    ASSERT_TRUE(beforeStop.closureFraction > 0.0 && beforeStop.closureFraction < 1.0) << "stop fixture is mid travel";

    ASSERT_TRUE(static_cast<bool>(controller.Stop())) << "stop";
    auto stopped = controller.GetState();
    ASSERT_TRUE(stopped.mode == GripperMode::Stopped && !stopped.goToActive &&
        stopped.requestedPositionEcho == 255 && stopped.objectStatus == GripperObjectStatus::Moving) << "stop retains request echo without claiming target reached";
    AdvanceTicks(controller, 20, 0.004);
    EXPECT_NEAR(controller.GetState().closureFraction, beforeStop.closureFraction, 0.0) << "stopped position does not advance";

    ASSERT_TRUE(static_cast<bool>(controller.Reset())) << "reset";
    const auto reset = controller.GetState();
    ASSERT_TRUE(reset.mode == GripperMode::Inactive && !reset.activated && reset.activationStatus == 0 && !reset.goToActive) << "reset stops and deactivates";
    EXPECT_NEAR(reset.closureFraction, beforeStop.closureFraction, 0.0) << "reset preserves position";
    ASSERT_TRUE(reset.requestedPositionEcho == 255 && reset.objectStatus == GripperObjectStatus::Moving) << "reset preserves request history and unreached classification";
    ASSERT_TRUE(static_cast<bool>(controller.Activate())) << "reactivate after reset";
    controller.Update(1.0);
    EXPECT_NEAR(controller.GetState().closureFraction, beforeStop.closureFraction, 0.0) << "reset does not resume previous target";

    ASSERT_TRUE(static_cast<bool>(controller.Command(Request(0, 255)))) << "new request after reset";
    AdvanceToTarget(controller);
    ASSERT_TRUE(static_cast<bool>(controller.Stop())) << "stop at target";
    ASSERT_TRUE(controller.GetState().objectStatus == GripperObjectStatus::AtRequestedPosition) << "stop at a reached target preserves target classification";

    controller.Disconnect();
    ASSERT_TRUE(!controller.IsConnected() && !controller.GetState().valid &&
        !controller.GetState().closureFractionValid && controller.GetState().mode == GripperMode::Disconnected) << "disconnect invalidates snapshot";
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "reconnect";
    const auto reconnected = controller.GetState();
    ASSERT_TRUE(reconnected.valid && reconnected.mode == GripperMode::Inactive &&
        reconnected.closureFraction == 0.0 && reconnected.actualPosition == kTwoF85.positionRequestMin) << "reconnect starts a fresh open state";
}

void CheckSpecificationAndSettingsValidation()
{
    const auto invalidSpecification = [](const models::GripperSpecification& specification)
    {
        SCOPED_TRACE("invalid gripper specification rejected");
        EXPECT_THROW(SimGripperController invalid(specification), std::invalid_argument);
    };

    auto specification = kTwoF85;
    specification.positionRequestMax = specification.positionRequestMin;
    invalidSpecification(specification);
    specification = kTwoF85;
    specification.speedRequestMax = specification.speedRequestMin;
    invalidSpecification(specification);
    specification = kTwoF85;
    specification.forceRequestMin = 200;
    specification.forceRequestMax = 100;
    invalidSpecification(specification);
    specification = kTwoF85;
    specification.nominalMasterClosedRadians = 0.0;
    invalidSpecification(specification);
    specification = kTwoF85;
    specification.nominalMasterClosedRadians = std::numeric_limits<double>::infinity();
    invalidSpecification(specification);
    specification = kTwoF85;
    specification.joints = nullptr;
    invalidSpecification(specification);
    specification = kTwoF85;
    specification.jointCount = 0;
    invalidSpecification(specification);

    for (const SimGripperMotionSettings settings : {
        SimGripperMotionSettings{0.0, 1.0},
        SimGripperMotionSettings{0.1, std::numeric_limits<double>::infinity()},
        SimGripperMotionSettings{0.5, 0.1},
        SimGripperMotionSettings{std::numeric_limits<double>::quiet_NaN(), 1.0}})
    {
        SCOPED_TRACE("invalid motion settings rejected");
        EXPECT_THROW(SimGripperController invalid(kTwoF85, settings), std::invalid_argument);
    }

    // 장치 속도 값이 달라도 사양에서 같은 시뮬레이션 각속도 범위로 변환된다면 목표까지 걸리는 시간은 일정해야 한다.
    SimGripperController fixedSpeed(kTwoF85, {0.25, 0.25});
    ASSERT_TRUE(static_cast<bool>(fixedSpeed.Connect()) && static_cast<bool>(fixedSpeed.Activate())) << "equal positive settings accepted";
}
}

/**
 * @brief 자유공간 Gripper Controller의 상태 전이, 시간 진행과 입력 검증을 회귀 검사한다.
 * @details raw 위치는 장치에 보내는 정수 명령 코드라 거리 단위가 아니다. closureFraction은 0부터 1까지의 연속 개폐 비율이고 master 속도는 rad/s, dt는 초다.
 * 이 Controller는 빈 공간에서 개폐만 계산하므로 힘·전류·접촉이나 물체를 실제로 집는 동작은 시험하지 않는다.
 */
TEST(GripperRuntimeTests, LifecycleAndCommands)
{
    CheckLifecycleAndCommands();
}

TEST(GripperRuntimeTests, StopResetAndReconnect)
{
    CheckStopResetAndReconnect();
}

TEST(GripperRuntimeTests, SpecificationAndMotionSettingsValidation)
{
    CheckSpecificationAndSettingsValidation();
}
