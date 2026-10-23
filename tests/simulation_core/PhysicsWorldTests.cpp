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

void Verify_AirSimPhysics_UsesEastNorthUpWorldAndForwardLeftUpBodyAxes()
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition definition;
    definition.positionMeters = {0.0, 0.0, 10.0};
    definition.orientation = glm::normalize(
        glm::angleAxis(glm::half_pi<double>(), glm::dvec3{0.0, 0.0, 1.0})
        * glm::angleAxis(0.2, glm::dvec3{0.0, 1.0, 0.0}));

    PhysicsWorld world;
    const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);
    const double hoverSpeed = std::sqrt(parameters.massKilograms * 9.81
        / (parameters.rotors.size() * parameters.thrustCoefficientNewtonsPerSpeedSquared));
    std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
    motorTargets.fill(hoverSpeed);
    world.SetMotorTargets(drone, motorTargets);

    for (int tick = 0; tick < 30; ++tick)
        world.Step(1.0 / 60.0);

    const RigidBodyState state = world.State(drone);
    // 기체를 동쪽으로 기울인 뒤 북쪽을 향하게 했으므로 추력은 북쪽으로 이동시킨다.
    EXPECT_GT(state.positionMeters.y, definition.positionMeters.y + 0.05);
    EXPECT_LT(std::abs(state.positionMeters.x - definition.positionMeters.x), 0.01);
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

    EXPECT_LT(world.State(left).positionMeters.x, leftDefinition.positionMeters.x);
    EXPECT_GT(world.State(right).positionMeters.x, rightDefinition.positionMeters.x);
}

TEST(PhysicsWorldTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_RotorSpeedChangesGraduallyAfterACommand());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysics_UsesEastNorthUpWorldAndForwardLeftUpBodyAxes());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_TerrainContactStopsTheDrone());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_OpenStreetMapBuildingContactStopsHorizontalMotion());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimCollision_UsesDroneBoxesInsteadOfTheirBoundingSphere());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimCollision_SeparatesOverlappingDrones());
}

}
