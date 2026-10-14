#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <string>

using namespace grasplink::robotics;
using grasplink::robotics::kinematics::RobotKinematics;

namespace
{
void ExpectToolPosition(
    RobotKinematics& kinematics,
    RobotState& state,
    const std::array<double, 6>& jointRadians,
    const std::array<double, 3>& expectedMeters,
    const std::string& label)
{
    // 입력 관절각은 rad, 기대 위치는 로봇 기준점(Base)에서 잰 m 단위다. 관절을 하나씩 회전시켜 축 방향의 부호와 링크 변환이 누적되는 순서를 확인한다.
    state.jointPositionRadians.assign(jointRadians.begin(), jointRadians.end());
    const auto& result = kinematics.Update(state);
    SCOPED_TRACE(label);
    ASSERT_TRUE(result.toolFrameValid);
    EXPECT_EQ(result.jointLocalRotations.size(), models::hanwha::kHcr12a.jointCount);
    EXPECT_EQ(result.linkPosesInBaseFrame.size(), models::hanwha::kHcr12a.jointCount);
    EXPECT_NEAR(result.toolFrameInBaseFrame.positionMeters.x, expectedMeters[0], 1e-9);
    EXPECT_NEAR(result.toolFrameInBaseFrame.positionMeters.y, expectedMeters[1], 1e-9);
    EXPECT_NEAR(result.toolFrameInBaseFrame.positionMeters.z, expectedMeters[2], 1e-9);
}
}

/**
 * @brief HCR-12A 순기구학의 기준 자세와 입력 검증을 확인한다.
 * @details 영 자세와 각 관절의 90° 단독 회전, 혼합 회전을 별도 축 회전 계산으로 얻은 Base 기준 위치와 비교한다.
 * 자세값은 모델 링크 피벗/ToolFrame을 따라 누적한 결과이며, 잘못된 사양·상태는 계산 전에 거부되어야 한다.
 */
TEST(RobotKinematicsTests, ReferencePosesAndValidationContracts)
{
    RobotKinematics kinematics(models::hanwha::kHcr12a);
    RobotState state;
    state.valid = true;
    const double quarterTurn = std::acos(-1.0) * 0.5;

    ExpectToolPosition(kinematics, state, {0, 0, 0, 0, 0, 0},
        {0.0, 1.015, 0.9145}, "zero pose");
    ExpectToolPosition(kinematics, state, {quarterTurn, 0, 0, 0, 0, 0},
        {0.9145, 1.015, 0.0}, "J1 positive quarter turn");
    ExpectToolPosition(kinematics, state, {0, quarterTurn, 0, 0, 0, 0},
        {0.0, -0.5045, 0.805}, "J2 positive quarter turn");
    ExpectToolPosition(kinematics, state, {0, 0, quarterTurn, 0, 0, 0},
        {0.0, 0.0955, 0.205}, "J3 positive quarter turn");
    ExpectToolPosition(kinematics, state, {0, 0, 0, quarterTurn, 0, 0},
        {0.0, 1.015, 0.9145}, "J4 positive quarter turn");
    ExpectToolPosition(kinematics, state, {0, 0, 0, 0, quarterTurn, 0},
        {0.0, 0.7915, 0.691}, "J5 positive quarter turn");
    ExpectToolPosition(kinematics, state, {0, 0, 0, 0, 0, quarterTurn},
        {0.0, 1.015, 0.9145}, "J6 positive quarter turn");
    // 독립적으로 만든 3×3 축 회전 행렬로 로봇 기준 위치 [m]를 계산해 기대값으로 사용한다. 1e-9 m 오차는 double 정밀도 계산 결과가 바뀌었는지 확인한다.
    ExpectToolPosition(kinematics, state, {0.45, -0.4, 0.3, 0.5, -0.2, 0.7},
        {0.26702766609490924, 1.0867538528236094, 0.60173049560973602}, "mixed pose");

    state.jointPositionRadians.assign(6, 0.0);
    const auto& zeroResult = kinematics.Update(state);
    // 관절 회전 중심도 로봇 기준 [m] 좌표로 확인한다. 1e-12 허용 오차는 동일한 사양 상수로 계산할 때 피벗 위치 누적 순서가 바뀌는 회귀를 찾는다.
    EXPECT_NEAR(zeroResult.linkPosesInBaseFrame.front().positionMeters.y, 0.1985, 1e-12);
    EXPECT_NEAR(zeroResult.linkPosesInBaseFrame.back().positionMeters.z, 0.85475, 1e-12);

    models::RobotSpecification withoutTool = models::hanwha::kHcr12a;
    withoutTool.hasToolFrame = false;
    RobotKinematics noToolKinematics(withoutTool);
    EXPECT_FALSE(noToolKinematics.Update(state).toolFrameValid);

    RobotState invalidState = state;
    invalidState.valid = false;
    EXPECT_THROW(kinematics.Update(invalidState), std::invalid_argument);
    invalidState = state;
    invalidState.jointPositionRadians.pop_back();
    EXPECT_THROW(kinematics.Update(invalidState), std::invalid_argument);
    invalidState = state;
    invalidState.jointPositionRadians[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(kinematics.Update(invalidState), std::invalid_argument);

    auto invalidJoints = models::hanwha::kHcr12aJoints;
    auto invalidSpecification = models::hanwha::kHcr12a;
    invalidSpecification.joints = invalidJoints.data();
    invalidJoints[0].bindPivotMeters.x = std::numeric_limits<double>::infinity();
    EXPECT_THROW(RobotKinematics invalid(invalidSpecification), std::invalid_argument);

    invalidJoints = models::hanwha::kHcr12aJoints;
    invalidSpecification.joints = invalidJoints.data();
    invalidJoints[0].axis = {};
    EXPECT_THROW(RobotKinematics invalid(invalidSpecification), std::invalid_argument);

    invalidJoints[0].axis.x = std::numeric_limits<double>::infinity();
    EXPECT_THROW(RobotKinematics invalid(invalidSpecification), std::invalid_argument);

    invalidJoints = models::hanwha::kHcr12aJoints;
    invalidSpecification = models::hanwha::kHcr12a;
    invalidSpecification.joints = invalidJoints.data();
    invalidSpecification.toolFrameInLastJoint.rotation = {0.0, 0.0, 0.0, 0.0};
    EXPECT_THROW(RobotKinematics invalid(invalidSpecification), std::invalid_argument);

    invalidSpecification = models::hanwha::kHcr12a;
    invalidSpecification.toolFrameInLastJoint.positionMeters.z = std::numeric_limits<double>::infinity();
    EXPECT_THROW(RobotKinematics invalid(invalidSpecification), std::invalid_argument);
    invalidSpecification = models::hanwha::kHcr12a;
    invalidSpecification.toolFrameInLastJoint.rotation.w = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(RobotKinematics invalid(invalidSpecification), std::invalid_argument);

    auto tinyQuaternionSpecification = models::hanwha::kHcr12a;
    tinyQuaternionSpecification.toolFrameInLastJoint.rotation = {1.0e-20, 0.0, 0.0, 0.0};
    RobotKinematics tinyQuaternionKinematics(tinyQuaternionSpecification);
    const auto& tinyQuaternionResult = tinyQuaternionKinematics.Update(state);
    EXPECT_NEAR(tinyQuaternionResult.toolFrameInBaseFrame.rotation.w, 1.0, 1e-12);

    invalidSpecification = models::hanwha::kHcr12a;
    invalidSpecification.joints = nullptr;
    {
        SCOPED_TRACE("missing joint array rejected");
        EXPECT_THROW(RobotKinematics invalid(invalidSpecification), std::invalid_argument);
    }
    invalidSpecification = models::hanwha::kHcr12a;
    invalidSpecification.jointCount = 0;
    {
        SCOPED_TRACE("empty joint array rejected");
        EXPECT_THROW(RobotKinematics invalid(invalidSpecification), std::invalid_argument);
    }
}
