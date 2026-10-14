#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/models/robotiq/TwoF85.h"

#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <stdexcept>

namespace
{
using namespace grasplink::robotics;
using kinematics::GripperKinematicState;
using kinematics::GripperKinematics;

GripperState ClosureState(double fraction)
{
    GripperState state;
    state.valid = true;
    state.closureFractionValid = true;
    state.closureFraction = fraction;
    state.mode = GripperMode::Inactive;
    return state;
}

void RequireQuaternion(const models::QuaternionWxyz& actual,
    const models::QuaternionWxyz& expected, double tolerance, const std::string& label)
{
    EXPECT_NEAR(actual.w, expected.w, tolerance) << label + ": w";
    EXPECT_NEAR(actual.x, expected.x, tolerance) << label + ": x";
    EXPECT_NEAR(actual.y, expected.y, tolerance) << label + ": y";
    EXPECT_NEAR(actual.z, expected.z, tolerance) << label + ": z";
}

void RequireSamePose(const GripperKinematicState& actual,
    const GripperKinematicState& expected, const std::string& label)
{
    EXPECT_NEAR(actual.masterAngleRadians, expected.masterAngleRadians, 0.0) << label + ": master angle";
    EXPECT_TRUE(actual.jointAnglesRadians.size() == expected.jointAnglesRadians.size()) << label + ": angle count";
    EXPECT_TRUE(actual.jointLocalRotations.size() == expected.jointLocalRotations.size()) << label + ": rotation count";
    for (std::size_t i = 0; i < expected.jointAnglesRadians.size(); ++i)
    {
        EXPECT_NEAR(actual.jointAnglesRadians[i], expected.jointAnglesRadians[i], 0.0) << label + ": joint angle";
        RequireQuaternion(actual.jointLocalRotations[i], expected.jointLocalRotations[i], 0.0, label + ": rotation");
    }
}

void CheckReferencePoses()
{
    GripperKinematics calculator(models::robotiq::kTwoF85);
    // 기대값은 여섯 관절의 연동 회전 부호와 각 관절의 Local -Z 회전축을 사용한다. 관절각 단위는 rad이고 회전 quaternion 성분 순서는 [w,x,y,z]다.
    constexpr std::array<double, 6> signs{1.0, -1.0, 1.0, -1.0, -1.0, 1.0};
    for (double fraction : {0.0, 0.5, 1.0})
    {
        const auto& pose = calculator.Update(ClosureState(fraction));
        const double master = 0.7929 * fraction;
        EXPECT_NEAR(pose.masterAngleRadians, master, 1.0e-12) << "nominal master angle";
        EXPECT_TRUE(pose.jointAnglesRadians.size() == signs.size()) << "six mimic angles";
        EXPECT_TRUE(pose.jointLocalRotations.size() == signs.size()) << "six local rotations";
        for (std::size_t i = 0; i < signs.size(); ++i)
        {
            const double angle = signs[i] * master;
            const std::string label = std::string(models::robotiq::kTwoF85Joints[i].name) +
                " closure " + std::to_string(fraction);
            EXPECT_NEAR(pose.jointAnglesRadians[i], angle, 1.0e-12) << label + ": mimic angle";
            const auto& rotation = pose.jointLocalRotations[i];
            RequireQuaternion(rotation, {std::cos(angle * 0.5), 0.0, 0.0, -std::sin(angle * 0.5)},
                1.0e-12, label);
            EXPECT_NEAR(rotation.w * rotation.w + rotation.x * rotation.x +
                rotation.y * rotation.y + rotation.z * rotation.z, 1.0, 1.0e-12) << label + ": unit rotation";
        }
    }
}

void CheckContinuousInputAndRejectedState()
{
    GripperKinematics calculator(models::robotiq::kTwoF85);
    GripperState state = ClosureState(0.3725);
    state.actualPosition = 255;
    state.requestedPositionEcho = 0;
    const GripperKinematicState continuous = calculator.Update(state);
    state.actualPosition = 0;
    state.requestedPositionEcho = 255;
    state.currentRaw = 255;
    state.activated = true;
    state.mode = GripperMode::Moving;
    RequireSamePose(calculator.Update(state), continuous, "raw feedback and activation do not change pose");

    // 마지막 Update()가 반환한 내부 상태 참조를 보관해 입력이 거부된 직후에도 값이 바뀌지 않았는지 확인한다. 검사용 정상 Update를 다시 호출해 결과를 덮어쓰지 않는다.
    const auto& retained = calculator.Update(state);
    const GripperKinematicState before = retained;
    const auto reject = [&](const GripperState& invalid, const std::string& label)
    {
        SCOPED_TRACE(label);
        EXPECT_THROW(calculator.Update(invalid), std::invalid_argument);
        RequireSamePose(retained, before, label + ": previous pose retained");
    };
    GripperState invalid = state;
    invalid.valid = false;
    reject(invalid, "invalid snapshot");
    invalid = state;
    invalid.closureFractionValid = false;
    reject(invalid, "invalid fraction flag");
    for (double fraction : {-0.01, 1.01, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()})
    {
        invalid = state;
        invalid.closureFraction = fraction;
        reject(invalid, "out-of-range or non-finite fraction");
    }
}

void CheckNonunitAxis()
{
    auto joints = models::robotiq::kTwoF85Joints;
    joints[0].axis = {2.0, -3.0, 6.0};
    auto specification = models::robotiq::kTwoF85;
    specification.joints = joints.data();
    GripperKinematics calculator(specification);
    const auto& pose = calculator.Update(ClosureState(0.5));
    const double halfAngle = 0.7929 * 0.25;
    const double sine = std::sin(halfAngle);
    // 회전축 (2,-3,6)의 길이는 7이지만 quaternion 정규화 후에는 방향만 남는다. 축 벡터의 크기가 달라도 같은 회전각을 적용해야 한다.
    RequireQuaternion(pose.jointLocalRotations[0],
        {std::cos(halfAngle), sine * 2.0 / 7.0, sine * -3.0 / 7.0, sine * 6.0 / 7.0},
        1.0e-12, "arbitrary axis is normalized");
    EXPECT_NEAR(pose.jointAnglesRadians[0], 0.7929 * 0.5, 1.0e-12) << "axis length preserves joint angle";
}

void CheckInvalidSpecifications()
{
    const auto reject = [](const models::GripperSpecification& specification, const std::string& label)
    {
        SCOPED_TRACE(label);
        EXPECT_THROW(GripperKinematics invalid(specification), std::invalid_argument);
    };
    auto specification = models::robotiq::kTwoF85;
    specification.joints = nullptr;
    reject(specification, "missing joint array");
    specification = models::robotiq::kTwoF85;
    specification.jointCount = 0;
    reject(specification, "empty joint array");
    for (double angle : {0.0, -0.1, std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()})
    {
        specification = models::robotiq::kTwoF85;
        specification.nominalMasterClosedRadians = angle;
        reject(specification, "invalid nominal closed angle");
    }

    auto joints = models::robotiq::kTwoF85Joints;
    const auto rejectJoint = [&](const models::GripperJointSpecification& joint, const std::string& label)
    {
        joints = models::robotiq::kTwoF85Joints;
        joints[0] = joint;
        specification = models::robotiq::kTwoF85;
        specification.joints = joints.data();
        reject(specification, label);
    };
    auto joint = models::robotiq::kTwoF85Joints[0];
    joint.name = {};
    rejectJoint(joint, "empty joint name");
    joint = models::robotiq::kTwoF85Joints[0];
    joint.name = models::robotiq::kTwoF85Joints[1].name;
    rejectJoint(joint, "duplicate joint name");
    for (const models::Axis3 axis : {models::Axis3{},
        models::Axis3{std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0},
        models::Axis3{0.0, std::numeric_limits<double>::infinity(), 1.0},
        models::Axis3{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max()}})
    {
        joint = models::robotiq::kTwoF85Joints[0];
        joint.axis = axis;
        rejectJoint(joint, "invalid joint axis");
    }
    for (double multiplier : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
    {
        joint = models::robotiq::kTwoF85Joints[0];
        joint.masterMultiplier = multiplier;
        rejectJoint(joint, "non-finite mimic multiplier");
    }
    for (const std::array<double, 2> limits : {std::array<double, 2>{0.5, -0.5},
        std::array<double, 2>{std::numeric_limits<double>::quiet_NaN(), 0.8},
        std::array<double, 2>{0.0, std::numeric_limits<double>::infinity()},
        std::array<double, 2>{0.1, 0.8}, std::array<double, 2>{-0.8, -0.1},
        std::array<double, 2>{0.0, 0.7}})
    {
        joint = models::robotiq::kTwoF85Joints[0];
        joint.minPositionRadians = limits[0];
        joint.maxPositionRadians = limits[1];
        rejectJoint(joint, "invalid limits or excluded open-to-closed range");
    }
    joint = models::robotiq::kTwoF85Joints[0];
    joint.masterMultiplier = -1.0;
    rejectJoint(joint, "negative closed angle outside limits");

    joints = models::robotiq::kTwoF85Joints;
    specification = models::robotiq::kTwoF85;
    specification.joints = joints.data();
    specification.nominalMasterClosedRadians = std::numeric_limits<double>::max();
    joints[0].masterMultiplier = 2.0;
    joints[0].maxPositionRadians = std::numeric_limits<double>::max();
    reject(specification, "nominal mimic angle overflow");
}
}

/**
 * @brief 2F-85 연속 개폐 위치의 mimic 각도와 Local quaternion 입력 계약을 검증한다.
 * @details 열린·중간·닫힌 기준 자세, 축 정규화와 잘못된 입력 거부를 확인한다.
 * GLB bind 합성·부모 계층 전파·물리 추종은 별도의 PoseIntegration 테스트가 담당한다.
 */
TEST(GripperKinematicsTests, ReferencePosesAndMimicRotations)
{
    CheckReferencePoses();
}

TEST(GripperKinematicsTests, ContinuousInputAndRejectedState)
{
    CheckContinuousInputAndRejectedState();
}

TEST(GripperKinematicsTests, NormalizesNonunitJointAxis)
{
    CheckNonunitAxis();
}

TEST(GripperKinematicsTests, RejectsInvalidSpecifications)
{
    CheckInvalidSpecifications();
}
