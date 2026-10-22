#include "simulation_core/PhysicsWorld.h"
#include "simulation_core/QuadrotorModel.h"
#include "terrain/BuildingFootprint.h"
#include "terrain/TerrainGrid.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace aerosimlab::simulation
{
namespace
{
RigidBodyHandle MakeX500(PhysicsWorld& world, double heightMeters)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition definition;
    definition.positionMeters.z = heightMeters;
    return world.CreateQuadrotor(definition, parameters);
}

RigidBodyState SimulateMotorInput(
    const QuadrotorParameters& parameters,
    const std::array<double, QuadrotorParameters::rotorCount>& motorTargets)
{
    PhysicsWorld world;
    RigidBodyDefinition definition;
    definition.positionMeters.z = 10.0;
    const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);
    world.SetMotorTargets(drone, motorTargets);
    for (int tick = 0; tick < 12; ++tick)
        world.Step(1.0 / 60.0);
    return world.State(drone);
}
}

void Verify_AirSimPhysics_GravityMovesTheDroneDown()
{
    PhysicsWorld world;
    const RigidBodyHandle drone = MakeX500(world, 10.0);

    for (int tick = 0; tick < 60; ++tick)
        world.Step(1.0 / 60.0);

    const RigidBodyState state = world.State(drone);
    EXPECT_LT(state.positionMeters.z, 6.0);
    EXPECT_LT(state.linearVelocityMetersPerSecond.z, -8.0);
}

void Verify_AirSimPhysics_RotorTargetsLiftTheDrone()
{
    PhysicsWorld world;
    const RigidBodyHandle drone = MakeX500(world, 2.0);
    const auto parameters = QuadrotorParameters::X500Gazebo();
    const double hoverSpeed = std::sqrt(parameters.massKilograms * 9.81
        / (parameters.rotors.size() * parameters.thrustCoefficientNewtonsPerSpeedSquared));
    std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
    motorTargets.fill(hoverSpeed * 1.2);
    world.SetMotorTargets(drone, motorTargets);

    for (int tick = 0; tick < 120; ++tick)
        world.Step(1.0 / 60.0);

    EXPECT_GT(world.State(drone).positionMeters.z, 2.0);
}

void Verify_AirSimPhysics_RotorSpeedChangesGraduallyAfterACommand()
{
    constexpr double stepSeconds = 1.0 / 60.0;
    PhysicsWorld world;
    const RigidBodyHandle drone = MakeX500(world, 2.0);
    std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
    motorTargets.fill(800.0);
    world.SetMotorTargets(drone, motorTargets);

    world.Step(stepSeconds);
    const double speedAfterFirstStep = world.MotorSpeedsRadiansPerSecond(drone)[0];
    world.Step(stepSeconds);
    const double speedAfterSecondStep = world.MotorSpeedsRadiansPerSecond(drone)[0];
    const double expectedFirstOrderResponse = motorTargets[0] * std::sqrt(
        1.0 - std::exp(-stepSeconds
            / QuadrotorParameters::X500Gazebo().motorTimeConstantUpSeconds));
    EXPECT_LT(speedAfterFirstStep, speedAfterSecondStep);
    EXPECT_NEAR(speedAfterSecondStep, expectedFirstOrderResponse, 1.0);

    for (int tick = 0; tick < 10; ++tick)
        world.Step(1.0 / 60.0);
    const double laterSpeed = world.MotorSpeedsRadiansPerSecond(drone)[0];

    EXPECT_LT(speedAfterSecondStep, laterSpeed);
    EXPECT_LT(laterSpeed, motorTargets[0]);
}

void Verify_AirSimPhysics_TerrainContactStopsTheDrone()
{
    PhysicsWorld world;
    const RigidBodyHandle drone = MakeX500(world, 2.0);

    for (int tick = 0; tick < 240; ++tick)
        world.Step(1.0 / 60.0);

    const RigidBodyState state = world.State(drone);
    EXPECT_GT(state.positionMeters.z, 0.0);
    EXPECT_LT(std::abs(state.linearVelocityMetersPerSecond.z), 0.2);
}

void Verify_AirSimPhysics_SlopedTerrainContactUsesTheDroneFootprint()
{
    terrain::TerrainGrid slope;
    slope.sampleCount = 3;
    slope.spacingXMeters = 2.0F;
    slope.spacingYMeters = 2.0F;
    slope.heightOffsetsMeters = {-1.0F, 0.0F, 1.0F,
        -1.0F, 0.0F, 1.0F,
        -1.0F, 0.0F, 1.0F};

    PhysicsWorld world;
    world.SetTerrain(std::move(slope));
    RigidBodyDefinition definition;
    definition.positionMeters.z = 0.3;
    const RigidBodyHandle drone = world.CreateQuadrotor(
        definition, QuadrotorParameters::X500Gazebo());

    for (int tick = 0; tick < 60; ++tick)
        world.Step(1.0 / 60.0);

    EXPECT_GT(world.State(drone).positionMeters.z, 0.25);
}

void Verify_AirSimPhysics_OpenStreetMapBuildingContactStopsHorizontalMotion()
{
    const std::filesystem::path mapPath = std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
        / "assets/maps/bukhansan-demo";
    auto terrain = terrain::TerrainGrid::Load(mapPath / "terrain.grid");
    const auto buildings = terrain::LoadBuildingFootprints(
        mapPath / "buildings.geojson", terrain);
    const auto building = std::find_if(buildings.begin(), buildings.end(),
        [](const terrain::BuildingFootprint& footprint)
        {
            return footprint.buildingType == "temple";
        });
    ASSERT_NE(building, buildings.end());

    const auto& outline = building->outlineMeters;
    glm::dvec2 minimum = outline.front();
    glm::dvec2 maximum = outline.front();
    for (const glm::dvec2& point : outline)
    {
        minimum = glm::min(minimum, point);
        maximum = glm::max(maximum, point);
    }
    const glm::dvec2 center = (minimum + maximum) * 0.5;
    constexpr double testGroundHeightMeters = 10.0;
    constexpr double buildingHeightMeters = 3.0;

    PhysicsWorld world;
    world.AddStaticObstacleBox(
        {center.x, center.y, testGroundHeightMeters + buildingHeightMeters * 0.5},
        {static_cast<float>((maximum.x - minimum.x) * 0.5),
            static_cast<float>(buildingHeightMeters * 0.5),
            static_cast<float>((maximum.y - minimum.y) * 0.5)});

    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition droneDefinition;
    droneDefinition.positionMeters = {
        minimum.x - 0.8, center.y,
        testGroundHeightMeters + buildingHeightMeters * 0.5};
    const RigidBodyHandle drone = world.CreateQuadrotor(droneDefinition, parameters);
    const double hoverSpeed = std::sqrt(parameters.massKilograms * 9.81
        / (parameters.rotors.size() * parameters.thrustCoefficientNewtonsPerSpeedSquared));
    std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
    motorTargets.fill(hoverSpeed);
    world.SetMotorTargets(drone, motorTargets);

    RigidBodyState state = world.State(drone);
    state.linearVelocityMetersPerSecond.x = 2.0;
    world.SetState(drone, state);

    for (int tick = 0; tick < 60; ++tick)
        world.Step(1.0 / 60.0);

    state = world.State(drone);
    EXPECT_LT(state.positionMeters.x, minimum.x);
    EXPECT_LT(state.linearVelocityMetersPerSecond.x, 0.5);
}

void Verify_AirSimCollision_UsesDroneBoxesInsteadOfTheirBoundingSphere()
{
    PhysicsWorld world;
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition definition;
    definition.positionMeters.z = 10.0;
    const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);
    world.AddStaticObstacleBox({0.45, 0.15, 10.0}, {0.1F, 0.1F, 0.3F});

    RigidBodyState state = world.State(drone);
    state.linearVelocityMetersPerSecond.x = 1.0;
    world.SetState(drone, state);
    world.Step(1.0 / 60.0);

    state = world.State(drone);
    EXPECT_GT(state.positionMeters.x, 0.0);
    EXPECT_GT(state.linearVelocityMetersPerSecond.x, 0.8);
}

void Verify_AirSimCollision_SeparatesOverlappingDrones()
{
    PhysicsWorld world;
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition leftDefinition;
    leftDefinition.positionMeters = {-0.16, 0.0, 10.0};
    RigidBodyDefinition rightDefinition;
    rightDefinition.positionMeters = {0.16, 0.0, 10.0};
    const RigidBodyHandle left = world.CreateQuadrotor(leftDefinition, parameters);
    const RigidBodyHandle right = world.CreateQuadrotor(rightDefinition, parameters);

    RigidBodyState leftState = world.State(left);
    leftState.linearVelocityMetersPerSecond.x = 1.0;
    world.SetState(left, leftState);
    RigidBodyState rightState = world.State(right);
    rightState.linearVelocityMetersPerSecond.x = -1.0;
    world.SetState(right, rightState);

    world.Step(1.0 / 60.0);

    const auto leftAfter = world.State(left);
    const auto rightAfter = world.State(right);
    EXPECT_LT(leftAfter.positionMeters.x, leftDefinition.positionMeters.x);
    EXPECT_GT(rightAfter.positionMeters.x, rightDefinition.positionMeters.x);
}

void Verify_AirSimPhysics_RejectsInvalidVehicleConfiguration()
{
    PhysicsWorld world;
    QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    parameters.massKilograms = 0.0;

    EXPECT_THROW(static_cast<void>(world.CreateQuadrotor(
        RigidBodyDefinition{}, parameters)),
        std::invalid_argument);

    parameters = QuadrotorParameters::X500Gazebo();
    parameters.maximumMotorSpeedRadiansPerSecond = 0.0;
    EXPECT_THROW(static_cast<void>(world.CreateQuadrotor(
        RigidBodyDefinition{}, parameters)),
        std::invalid_argument);

    RigidBodyDefinition invalidPose;
    invalidPose.orientation = {0.0, 0.0, 0.0, 0.0};
    EXPECT_THROW(static_cast<void>(world.CreateQuadrotor(
        invalidPose, QuadrotorParameters::X500Gazebo())),
        std::invalid_argument);

    parameters = QuadrotorParameters::X500Gazebo();
    parameters.collisionBoxes.front().halfExtentsMeters.x = 0.0F;
    EXPECT_THROW(static_cast<void>(world.CreateQuadrotor(
        RigidBodyDefinition{}, parameters)),
        std::invalid_argument);
}

void Verify_AirSimPhysics_DifferentialRotorInputAndInertiaChangeVehicleResponse()
{
    const QuadrotorParameters standard = QuadrotorParameters::X500Gazebo();
    QuadrotorParameters heavier = standard;
    heavier.massKilograms *= 2.0;
    heavier.principalInertiaKilogramMetersSquared *= 2.0;

    const double hoverSpeed = std::sqrt(standard.massKilograms * 9.81
        / (QuadrotorParameters::rotorCount
            * standard.thrustCoefficientNewtonsPerSpeedSquared));
    std::array<double, QuadrotorParameters::rotorCount> equalMotorTargets{};
    equalMotorTargets.fill(hoverSpeed * 1.1);
    const RigidBodyState standardMassState = SimulateMotorInput(standard, equalMotorTargets);
    const RigidBodyState heavierState = SimulateMotorInput(heavier, equalMotorTargets);
    EXPECT_GT(standardMassState.linearVelocityMetersPerSecond.z,
        heavierState.linearVelocityMetersPerSecond.z + 0.1);

    constexpr std::array<double, QuadrotorParameters::rotorCount> unevenMotorTargets{
        500.0, 700.0, 900.0, 1100.0};
    QuadrotorParameters higherInertia = standard;
    higherInertia.principalInertiaKilogramMetersSquared *= 4.0;
    const RigidBodyState standardInertiaState =
        SimulateMotorInput(standard, unevenMotorTargets);
    const RigidBodyState higherInertiaState =
        SimulateMotorInput(higherInertia, unevenMotorTargets);
    const auto angularSpeed = [](const glm::dvec3& velocity)
    {
        return std::sqrt(velocity.x * velocity.x
            + velocity.y * velocity.y + velocity.z * velocity.z);
    };
    EXPECT_GT(angularSpeed(standardInertiaState.angularVelocityRadiansPerSecond), 1.0e-6);
    EXPECT_GT(glm::length(glm::dvec3{standardInertiaState.orientation.x,
        standardInertiaState.orientation.y, standardInertiaState.orientation.z}), 1.0e-7);
    EXPECT_GT(angularSpeed(standardInertiaState.angularVelocityRadiansPerSecond),
        angularSpeed(higherInertiaState.angularVelocityRadiansPerSecond));
}

TEST(PhysicsWorldTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_GravityMovesTheDroneDown());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_RotorTargetsLiftTheDrone());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_RotorSpeedChangesGraduallyAfterACommand());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_TerrainContactStopsTheDrone());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_SlopedTerrainContactUsesTheDroneFootprint());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_OpenStreetMapBuildingContactStopsHorizontalMotion());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimCollision_UsesDroneBoxesInsteadOfTheirBoundingSphere());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimCollision_SeparatesOverlappingDrones());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_RejectsInvalidVehicleConfiguration());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_DifferentialRotorInputAndInertiaChangeVehicleResponse());
}

}
