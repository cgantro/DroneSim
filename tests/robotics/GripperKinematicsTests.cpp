#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/models/robotiq/TwoF85.h"

#include <cmath>
#include <gtest/gtest.h>
#include <stdexcept>
#include <type_traits>

namespace
{
using grasplink::robotics::backends::simulation::SimGripperController;
using grasplink::robotics::kinematics::GripperKinematics;
using grasplink::robotics::models::GripperSpecification;

static_assert(!std::is_constructible_v<GripperKinematics, GripperSpecification&&>);
static_assert(!std::is_constructible_v<SimGripperController, GripperSpecification&&>);

grasplink::robotics::GripperState ClosedFraction(double fraction)
{
    grasplink::robotics::GripperState state;
    state.valid = true;
    state.closureFractionValid = true;
    state.closureFraction = fraction;
    return state;
}
}

/**
 * @brief 그리퍼가 지정한 닫힘 비율로 움직이고 잘못된 입력 뒤에도 직전 결과를 보존하는지 확인한다.
 * @details 자유공간에서 각 손가락 관절은 기준 각도에 사양의 배율을 곱해 움직인다. 개폐 비율은 열린 상태 0부터 닫힌 상태 1까지이며 각도 단위는 [rad]다.
 */
TEST(GripperKinematics, MimicAnglesAndRejectedInputPreservePreviousPose)
{
    using grasplink::robotics::kinematics::GripperKinematics;
    using grasplink::robotics::models::robotiq::kTwoF85;
    using grasplink::robotics::models::robotiq::kTwoF85Joints;

    GripperKinematics kinematics(kTwoF85);
    const auto& open = kinematics.Update(ClosedFraction(0.0));
    ASSERT_EQ(open.jointAnglesRadians.size(), kTwoF85.jointCount);
    for (const double angle : open.jointAnglesRadians)
        EXPECT_DOUBLE_EQ(angle, 0.0);

    const auto& closed = kinematics.Update(ClosedFraction(1.0));
    ASSERT_EQ(closed.jointAnglesRadians.size(), kTwoF85.jointCount);
    ASSERT_EQ(closed.jointLocalRotations.size(), kTwoF85.jointCount);
    for (std::size_t i = 0; i < kTwoF85.jointCount; ++i)
    {
        const double angle = kTwoF85Joints[i].masterMultiplier *
            kTwoF85.nominalMasterClosedRadians;
        EXPECT_NEAR(closed.jointAnglesRadians[i], angle, 1.0e-12);

        const auto& axis = kTwoF85Joints[i].axis;
        const double halfAngle = angle * 0.5;
        const auto& rotation = closed.jointLocalRotations[i];
        EXPECT_NEAR(rotation.w, std::cos(halfAngle), 1.0e-12);
        EXPECT_NEAR(rotation.x, axis.x * std::sin(halfAngle), 1.0e-12);
        EXPECT_NEAR(rotation.y, axis.y * std::sin(halfAngle), 1.0e-12);
        EXPECT_NEAR(rotation.z, axis.z * std::sin(halfAngle), 1.0e-12);
    }

    const auto previousPose = closed;
    auto invalid = ClosedFraction(1.01);
    EXPECT_THROW(kinematics.Update(invalid), std::invalid_argument);
    EXPECT_EQ(closed.jointAnglesRadians, previousPose.jointAnglesRadians);
    EXPECT_EQ(closed.jointLocalRotations.size(), previousPose.jointLocalRotations.size());
    for (std::size_t i = 0; i < closed.jointLocalRotations.size(); ++i)
    {
        EXPECT_DOUBLE_EQ(closed.jointLocalRotations[i].w, previousPose.jointLocalRotations[i].w);
        EXPECT_DOUBLE_EQ(closed.jointLocalRotations[i].x, previousPose.jointLocalRotations[i].x);
        EXPECT_DOUBLE_EQ(closed.jointLocalRotations[i].y, previousPose.jointLocalRotations[i].y);
        EXPECT_DOUBLE_EQ(closed.jointLocalRotations[i].z, previousPose.jointLocalRotations[i].z);
    }
}
