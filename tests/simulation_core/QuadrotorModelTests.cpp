#include "simulation_core/QuadrotorModel.h"

#include <gtest/gtest.h>

#include <cmath>

namespace aerosimlab::simulation
{
TEST(QuadrotorModel, EqualRotorSpeedsBalanceWeightAndCancelTorque)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
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
