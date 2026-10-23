#include "simulation_core/QuadrotorModel.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace aerosimlab::simulation
{
void Verify_QuadrotorParameters_MotorTableEstimateMatchesManufacturerReference()
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500MotorTableEstimate();

    struct TestStandPoint
    {
        double speedRpm;
        double thrustGrams;
        double torqueNewtonMeters;
    };
    // 제조사 16V 표의 회전수·추력·토크 15점이다. 전압과 프로펠러는 모든 행에서 같다.
    constexpr std::array<TestStandPoint, 15> testStand = {{
        {4042.0, 210.0, 0.03}, {4469.0, 259.0, 0.04}, {4855.0, 309.0, 0.05},
        {5301.0, 373.0, 0.05}, {5780.0, 447.0, 0.06}, {6298.0, 536.0, 0.08},
        {6800.0, 628.0, 0.09}, {7281.0, 729.0, 0.10}, {7679.0, 814.0, 0.11},
        {8096.0, 906.0, 0.12}, {8468.0, 993.0, 0.14}, {8867.0, 1087.0, 0.15},
        {9257.0, 1191.0, 0.16}, {9675.0, 1289.0, 0.18}, {9857.0, 1332.0, 0.18}}};
    constexpr double standardGravityMetersPerSecondSquared = 9.80665;
    const double radiansPerRpm = 2.0 * std::numbers::pi / 60.0;
    double coefficientNumerator = 0.0;
    double coefficientDenominator = 0.0;
    double torqueRatioNumerator = 0.0;
    double torqueRatioDenominator = 0.0;
    double measuredMaximumThrustNewtons = 0.0;
    for (const TestStandPoint& point : testStand)
    {
        const double angularSpeed = point.speedRpm * radiansPerRpm;
        const double angularSpeedSquared = angularSpeed * angularSpeed;
        const double measuredThrust = point.thrustGrams
            / 1000.0 * standardGravityMetersPerSecondSquared;
        coefficientNumerator += angularSpeedSquared * measuredThrust;
        coefficientDenominator += angularSpeedSquared * angularSpeedSquared;
        torqueRatioNumerator += measuredThrust * point.torqueNewtonMeters;
        torqueRatioDenominator += measuredThrust * measuredThrust;
        measuredMaximumThrustNewtons = std::max(
            measuredMaximumThrustNewtons, measuredThrust);
    }

    const double fittedCoefficient = coefficientNumerator / coefficientDenominator;
    const double fittedMomentToThrustRatio =
        torqueRatioNumerator / torqueRatioDenominator;
    double squaredError = 0.0;
    for (const TestStandPoint& point : testStand)
    {
        const double angularSpeed = point.speedRpm * radiansPerRpm;
        const double measuredThrust = point.thrustGrams
            / 1000.0 * standardGravityMetersPerSecondSquared;
        const double predictedThrust = fittedCoefficient * angularSpeed * angularSpeed;
        const double error = predictedThrust - measuredThrust;
        squaredError += error * error;
    }
    const double relativeRootMeanSquareError = std::sqrt(
        squaredError / static_cast<double>(testStand.size()))
        / measuredMaximumThrustNewtons;

    const double measuredMaximumRpm = parameters.maximumMotorSpeedRadiansPerSecond
        * 60.0 / (2.0 * std::numbers::pi);
    const double estimatedMaximumThrustGrams =
        parameters.thrustCoefficientNewtonsPerSpeedSquared
        * parameters.maximumMotorSpeedRadiansPerSecond
        * parameters.maximumMotorSpeedRadiansPerSecond
        / standardGravityMetersPerSecondSquared * 1000.0;

    EXPECT_NEAR(fittedCoefficient,
        parameters.thrustCoefficientNewtonsPerSpeedSquared, 1.0e-12);
    EXPECT_NEAR(fittedMomentToThrustRatio, parameters.momentToThrustRatio, 1.0e-8);
    EXPECT_NEAR(relativeRootMeanSquareError, 0.00742232, 1.0e-6);
    EXPECT_NEAR(measuredMaximumRpm, 9857.0, 0.1);
    EXPECT_NEAR(estimatedMaximumThrustGrams, 1332.0, 10.0);
    EXPECT_NEAR(parameters.momentToThrustRatio, 0.01403352, 1.0e-8);
    EXPECT_DOUBLE_EQ(parameters.motorTimeConstantUpSeconds, 0.0125);
    EXPECT_DOUBLE_EQ(parameters.motorTimeConstantDownSeconds, 0.025);
}

void Verify_X500GazeboParameters_KeepAggregatedMassInertiaAndRotorLayout()
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();

    // 공개 X500의 중심 기체와 네 로터를 한 강체로 합산한 기준값이다.
    EXPECT_NEAR(parameters.massKilograms, 2.0643076923076924, 1.0e-12);
    EXPECT_NEAR(parameters.principalInertiaKilogramMetersSquared.x,
        0.023846692647701416, 1.0e-12);
    EXPECT_NEAR(parameters.principalInertiaKilogramMetersSquared.y,
        0.02394961745804886, 1.0e-12);
    EXPECT_NEAR(parameters.principalInertiaKilogramMetersSquared.z,
        0.043999953714003955, 1.0e-12);

    constexpr std::array<glm::dvec3, QuadrotorParameters::rotorCount> expectedPositions{{
        {0.174, -0.174, 0.06}, {-0.174, 0.174, 0.06},
        {0.174, 0.174, 0.06}, {-0.174, -0.174, 0.06}}};
    constexpr std::array<RotorRotation, QuadrotorParameters::rotorCount> expectedRotations{{
        RotorRotation::CounterClockwise, RotorRotation::CounterClockwise,
        RotorRotation::Clockwise, RotorRotation::Clockwise}};
    ASSERT_EQ(parameters.rotors.size(), expectedPositions.size());
    for (std::size_t index = 0; index < parameters.rotors.size(); ++index)
    {
        EXPECT_DOUBLE_EQ(parameters.rotors[index].positionMeters.x,
            expectedPositions[index].x);
        EXPECT_DOUBLE_EQ(parameters.rotors[index].positionMeters.y,
            expectedPositions[index].y);
        EXPECT_DOUBLE_EQ(parameters.rotors[index].positionMeters.z,
            expectedPositions[index].z);
        EXPECT_EQ(parameters.rotors[index].rotation, expectedRotations[index]);
    }
}

TEST(QuadrotorModelTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_QuadrotorParameters_MotorTableEstimateMatchesManufacturerReference());
    EXPECT_NO_FATAL_FAILURE(Verify_X500GazeboParameters_KeepAggregatedMassInertiaAndRotorLayout());
}

}
