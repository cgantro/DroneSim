#include "simulation_core/QuadrotorModel.h"

#include <gtest/gtest.h>

#include <cmath>

namespace aerosimlab::simulation
{
TEST(QuadrotorModel, EqualRotorSpeedsBalanceWeightAndCancelTorque)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    EXPECT_NEAR(parameters.massKilograms, 2.0643076923076924, 1.0e-12);
    EXPECT_NEAR(parameters.principalInertiaKilogramMetersSquared.x,
        0.023846692647701416, 1.0e-12);
    EXPECT_NEAR(parameters.principalInertiaKilogramMetersSquared.y,
        0.02394961745804886, 1.0e-12);
    EXPECT_NEAR(parameters.principalInertiaKilogramMetersSquared.z,
        0.043999953714003955, 1.0e-12);
    EXPECT_DOUBLE_EQ(parameters.rotors[0].positionMeters.z, 0.06);

    const double hoverSpeed = std::sqrt(
        parameters.massKilograms * 9.81
        / (QuadrotorModel::rotorCount * parameters.thrustCoefficientNewtonsPerSpeedSquared));
    QuadrotorState state;
    state.motorSpeedRadiansPerSecond.fill(hoverSpeed);

    const QuadrotorWrench wrench = QuadrotorModel::CalculateWrench(state, parameters);
    EXPECT_NEAR(wrench.forceNewtons.z, parameters.massKilograms * 9.81, 1.0e-9);
    EXPECT_NEAR(wrench.torqueNewtonMeters.x, 0.0, 1.0e-12);
    EXPECT_NEAR(wrench.torqueNewtonMeters.y, 0.0, 1.0e-12);
    EXPECT_NEAR(wrench.torqueNewtonMeters.z, 0.0, 1.0e-12);
}

TEST(QuadrotorModel, X500MotorTableEstimateMatchesPublishedMotorPoints)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500MotorTableEstimate();
    constexpr double radiansPerSecondPerRpm = 0.10471975511965977;
    // 공개된 Holybro 모터 표의 회전수(rpm), 추력(g), 토크(N·m) 세 점이다.
    constexpr std::array<double, 3> speedsRpm{4269.0, 6252.0, 10703.0};
    constexpr std::array<double, 3> thrustGrams{214.74, 470.2, 1367.04};
    constexpr std::array<double, 3> torqueNewtonMeters{0.03, 0.07, 0.19};

    EXPECT_NEAR(parameters.maximumMotorSpeedRadiansPerSecond,
        speedsRpm.back() * radiansPerSecondPerRpm, 1.0e-6);

    for (std::size_t point = 0; point < speedsRpm.size(); ++point)
    {
        QuadrotorState state;
        state.motorSpeedRadiansPerSecond.fill(
            speedsRpm[point] * radiansPerSecondPerRpm);

        const QuadrotorWrench wrench = QuadrotorModel::CalculateWrench(state, parameters);
        const double expectedThrustNewtons = thrustGrams[point] / 1000.0 * 9.80665;
        EXPECT_NEAR(wrench.forceNewtons.z / QuadrotorModel::rotorCount,
            expectedThrustNewtons, expectedThrustNewtons * 0.02);

        QuadrotorState singleRotorState;
        singleRotorState.motorSpeedRadiansPerSecond[0] =
            speedsRpm[point] * radiansPerSecondPerRpm;
        const QuadrotorWrench singleRotorWrench = QuadrotorModel::CalculateWrench(
            singleRotorState, parameters);
        EXPECT_NEAR(std::abs(singleRotorWrench.torqueNewtonMeters.z),
            torqueNewtonMeters[point], 0.0051);
    }
}

TEST(QuadrotorModel, X500CollisionBoxesIncludeFrameGearAndFourPropellerPlanes)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();

    ASSERT_EQ(parameters.collisionBoxes.size(), 9U);
    EXPECT_NEAR(parameters.collisionBoxes[0].halfExtentsMeters.x, 0.1767767F, 1.0e-6F);
    EXPECT_NEAR(parameters.collisionBoxes[1].positionMeters.z, -0.123, 1.0e-9);
    EXPECT_NEAR(parameters.collisionBoxes[4].positionMeters.z, -0.2195, 1.0e-9);
    EXPECT_NEAR(parameters.collisionBoxes[5].positionMeters.x, 0.174, 1.0e-9);
    EXPECT_NEAR(parameters.collisionBoxes[5].positionMeters.y, -0.174, 1.0e-9);
    EXPECT_NEAR(parameters.collisionBoxes[8].positionMeters.z, 0.06, 1.0e-9);
}

TEST(QuadrotorModel, MotorResponseAndUnequalRotorsChangeWrench)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    QuadrotorState state;
    std::array<double, QuadrotorModel::rotorCount> targetSpeeds;
    targetSpeeds.fill(parameters.maximumMotorSpeedRadiansPerSecond);

    QuadrotorModel::AdvanceMotors(
        state, targetSpeeds, parameters, parameters.motorTimeConstantUpSeconds);
    const double expectedAfterSpinUp = parameters.maximumMotorSpeedRadiansPerSecond
        * (1.0 - std::exp(-1.0));
    EXPECT_NEAR(state.motorSpeedRadiansPerSecond[0], expectedAfterSpinUp, 1.0e-9);

    const std::array<double, QuadrotorModel::rotorCount> differentialSpeeds{
        parameters.maximumMotorSpeedRadiansPerSecond, 0.0, 0.0, 0.0};
    state.motorSpeedRadiansPerSecond = differentialSpeeds;
    const QuadrotorWrench wrench = QuadrotorModel::CalculateWrench(state, parameters);
    EXPECT_GT(std::abs(wrench.torqueNewtonMeters.x), 0.1);
    EXPECT_GT(std::abs(wrench.torqueNewtonMeters.y), 0.1);
    EXPECT_GT(std::abs(wrench.torqueNewtonMeters.z), 0.1);

    targetSpeeds.fill(0.0);
    QuadrotorModel::AdvanceMotors(
        state, targetSpeeds, parameters, parameters.motorTimeConstantDownSeconds);
    EXPECT_NEAR(state.motorSpeedRadiansPerSecond[0],
        parameters.maximumMotorSpeedRadiansPerSecond * std::exp(-1.0), 1.0e-9);
}

TEST(QuadrotorModel, RotorDragOpposesAirRelativeVelocity)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    QuadrotorState state;
    state.motorSpeedRadiansPerSecond.fill(500.0);

    const QuadrotorWrench stillAir = QuadrotorModel::CalculateWrench(state, parameters);
    const QuadrotorWrench forwardAirflow = QuadrotorModel::CalculateWrench(
        state, parameters, glm::dvec3{5.0, 0.0, 0.0});

    EXPECT_DOUBLE_EQ(stillAir.forceNewtons.x, 0.0);
    EXPECT_LT(forwardAirflow.forceNewtons.x, 0.0);
    EXPECT_NEAR(forwardAirflow.forceNewtons.x,
        -4.0 * 500.0 * parameters.rotorDragCoefficientNewtonSecondsSquaredPerMeter * 5.0,
        1.0e-12);
}
}
