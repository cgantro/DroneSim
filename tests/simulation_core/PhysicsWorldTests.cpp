#include "simulation_core/PhysicsWorld.h"
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/QuadrotorModel.h"
#include "CollisionGeometry.h"
#include "terrain/BuildingFootprint.h"
#include "terrain/TerrainGrid.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <random>
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

void Verify_AirSimFlightController_HoldsAStationaryWaypoint()
{
    constexpr double stepSeconds = 1.0 / 60.0;
    constexpr int simulationTicks = 600;
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();

    PhysicsWorld world;
    RigidBodyDefinition definition;
    definition.positionMeters.z = 10.0;
    const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);
    DroneControlState control;
    control.waypointMeters = definition.positionMeters;

    for (int tick = 0; tick < simulationTicks; ++tick)
    {
        const RigidBodyState physicsState = world.State(drone);
        DroneState controllerState;
        controllerState.positionMeters = physicsState.positionMeters;
        controllerState.velocityMetersPerSecond = physicsState.linearVelocityMetersPerSecond;
        controllerState.orientation = physicsState.orientation;
        controllerState.angularVelocityRadiansPerSecond =
            physicsState.angularVelocityRadiansPerSecond;
        controllerState.mode = FlightMode::FollowingWaypoint;
        world.SetMotorTargets(drone, QuadrotorFlightController::MotorTargets(
            controllerState, control, parameters));
        world.Step(stepSeconds);
    }

    const RigidBodyState state = world.State(drone);
    EXPECT_NEAR(state.positionMeters.z, control.waypointMeters.z, 0.1);
    EXPECT_NEAR(state.linearVelocityMetersPerSecond.z, 0.0, 0.05);
}

void Verify_AirSimPhysics_AsymmetricRotorTargetsChangeAttitude()
{
    PhysicsWorld world;
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition definition;
    definition.positionMeters.z = 10.0;
    const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);

    std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
    motorTargets.fill(760.0);
    // 두 로터의 목표 속도를 높여 한쪽으로 기울이는 토크를 만든다.
    motorTargets[0] = 900.0;
    motorTargets[2] = 900.0;
    world.SetMotorTargets(drone, motorTargets);

    for (int tick = 0; tick < 60; ++tick)
        world.Step(1.0 / 60.0);

    const RigidBodyState state = world.State(drone);
    EXPECT_GT(std::abs(state.angularVelocityRadiansPerSecond.y), 0.1);
    EXPECT_GT(std::abs(state.orientation.y), 0.01);
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
    const auto verifyGroundContact = [](double terrainHeightMeters, bool useHeightGrid)
    {
        PhysicsWorld world;
        if (useHeightGrid)
        {
            terrain::TerrainGrid terrainGrid;
            terrainGrid.sampleCount = 3;
            terrainGrid.spacingXMeters = 2.0F;
            terrainGrid.spacingYMeters = 2.0F;
            terrainGrid.heightOffsetsMeters.assign(9U,
                static_cast<float>(terrainHeightMeters));
            world.SetTerrain(std::move(terrainGrid));
        }

        const RigidBodyHandle drone = MakeX500(world, terrainHeightMeters + 2.0);
        for (int tick = 0; tick < 240; ++tick)
            world.Step(1.0 / 60.0);

        const RigidBodyState state = world.State(drone);
        EXPECT_NEAR(state.positionMeters.z,
            terrainHeightMeters
                + QuadrotorFlightController::groundedModelOriginHeightMeters,
            0.03);
        EXPECT_LT(std::abs(state.linearVelocityMetersPerSecond.z), 0.2);
    };

    verifyGroundContact(0.0, false);
    verifyGroundContact(2.0, true);

    PhysicsWorld world;
    terrain::TerrainGrid terrainGrid;
    terrainGrid.sampleCount = 3;
    terrainGrid.spacingXMeters = 2.0F;
    terrainGrid.spacingYMeters = 2.0F;
    terrainGrid.heightOffsetsMeters.assign(9U, 5.0F);
    world.SetTerrain(std::move(terrainGrid));

    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition definition;
    definition.positionMeters = {3.0, 0.0, 2.0};
    const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);
    for (int tick = 0; tick < 240; ++tick)
        world.Step(1.0 / 60.0);

    EXPECT_NEAR(world.State(drone).positionMeters.z,
        QuadrotorFlightController::groundedModelOriginHeightMeters, 0.03);
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

    constexpr float assumedBuildingHeightMeters = 3.0F;
    const auto bounds = terrain::MakeBuildingBoundsMeters(
        *building, terrain, assumedBuildingHeightMeters);

    PhysicsWorld world;
    world.AddStaticObstacleBox(bounds.centerMeters,
        glm::vec3{bounds.halfExtentsMeters});

    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition droneDefinition;
    droneDefinition.positionMeters = {
        bounds.centerMeters.x - bounds.halfExtentsMeters.x - 0.8,
        bounds.centerMeters.y, bounds.centerMeters.z};
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
    EXPECT_LT(state.positionMeters.x,
        bounds.centerMeters.x - bounds.halfExtentsMeters.x);
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

void Verify_CollisionBroadphase_KeepsTouchingBoxesForPreciseTest()
{
    using detail::BroadphaseMayOverlap;
    using detail::CollisionRadius;
    using detail::FindBoxContact;
    using detail::ToWorldBox;

    const std::array<RigidBodyBox, 1> boxes{
        RigidBodyBox{glm::vec3{3.0F, 4.0F, 12.0F}, glm::dvec3{0.0},
            glm::dquat{1.0, 0.0, 0.0, 0.0}}};
    const double radius = CollisionRadius(boxes);
    const detail::OrientedBox first = ToWorldBox(
        glm::dvec3{0.0}, glm::dquat{1.0, 0.0, 0.0, 0.0}, boxes.front());
    const detail::OrientedBox second = ToWorldBox(
        glm::dvec3{6.0, 8.0, 24.0},
        glm::dquat{1.0, 0.0, 0.0, 0.0}, boxes.front());

    ASSERT_TRUE(FindBoxContact(first, second));
    EXPECT_TRUE(BroadphaseMayOverlap(
        first.centerMeters, radius, second.centerMeters, radius));

    // 고정 시드로 여러 위치와 자세를 만들고, 정밀 검사에서 겹친 상자를
    // 빠른 후보 검사에서 놓치지 않는지 확인한다.
    std::mt19937 random{0xA3E05U};
    std::uniform_real_distribution<double> positionMeters{-3.0, 3.0};
    std::uniform_real_distribution<double> localOffsetMeters{-0.5, 0.5};
    std::uniform_real_distribution<double> halfExtentMeters{0.1, 1.5};
    std::uniform_real_distribution<double> angleRadians{-glm::pi<double>(), glm::pi<double>()};
    const auto randomOrientation = [&]()
    {
        return glm::angleAxis(angleRadians(random), glm::dvec3{0.0, 0.0, 1.0})
            * glm::angleAxis(angleRadians(random), glm::dvec3{0.0, 1.0, 0.0})
            * glm::angleAxis(angleRadians(random), glm::dvec3{1.0, 0.0, 0.0});
    };

    std::size_t exactContacts = 0;
    for (std::size_t sample = 0; sample < 2'000; ++sample)
    {
        const glm::dvec3 firstCenter{
            positionMeters(random), positionMeters(random), positionMeters(random)};
        const glm::dvec3 secondCenter{
            positionMeters(random), positionMeters(random), positionMeters(random)};
        const std::array<RigidBodyBox, 1> firstBoxes{
            RigidBodyBox{
                glm::vec3{halfExtentMeters(random), halfExtentMeters(random), halfExtentMeters(random)},
                glm::dvec3{localOffsetMeters(random), localOffsetMeters(random), localOffsetMeters(random)},
                randomOrientation()}};
        const std::array<RigidBodyBox, 1> secondBoxes{
            RigidBodyBox{
                glm::vec3{halfExtentMeters(random), halfExtentMeters(random), halfExtentMeters(random)},
                glm::dvec3{localOffsetMeters(random), localOffsetMeters(random), localOffsetMeters(random)},
                randomOrientation()}};
        const auto firstWorldBox = ToWorldBox(firstCenter, randomOrientation(), firstBoxes.front());
        const auto secondWorldBox = ToWorldBox(secondCenter, randomOrientation(), secondBoxes.front());
        if (!FindBoxContact(firstWorldBox, secondWorldBox))
            continue;

        ++exactContacts;
        EXPECT_TRUE(BroadphaseMayOverlap(
            firstCenter, CollisionRadius(firstBoxes),
            secondCenter, CollisionRadius(secondBoxes))) << "표본 " << sample;
    }
    EXPECT_GT(exactContacts, 0U);
}

void Verify_AirSimCollision_SeparatesOverlappingDrones()
{
    PhysicsWorld world;
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    constexpr std::size_t pairCount = 4;
    std::array<glm::dvec3, pairCount> leftPositions{};
    std::array<glm::dvec3, pairCount> rightPositions{};
    std::array<RigidBodyHandle, pairCount> leftBodies{};
    std::array<RigidBodyHandle, pairCount> rightBodies{};

    for (std::size_t index = 0; index < pairCount; ++index)
    {
        const double xOffset = static_cast<double>(index) * 0.02;
        const double yOffset = static_cast<double>(index) * 10.0;
        leftPositions[index] = {-0.16 + xOffset, yOffset, 10.0};
        rightPositions[index] = {0.16 + xOffset, yOffset, 10.0};

        RigidBodyDefinition definition;
        definition.positionMeters = rightPositions[index];
        rightBodies[index] = world.CreateQuadrotor(definition, parameters);
        // 등록 순서를 공간상 순서와 반대로 두어 정렬 단계도 함께 확인한다.
        definition.positionMeters = leftPositions[index];
        leftBodies[index] = world.CreateQuadrotor(definition, parameters);

        RigidBodyState leftState = world.State(leftBodies[index]);
        leftState.linearVelocityMetersPerSecond.x = 1.0;
        world.SetState(leftBodies[index], leftState);
        RigidBodyState rightState = world.State(rightBodies[index]);
        rightState.linearVelocityMetersPerSecond.x = -1.0;
        world.SetState(rightBodies[index], rightState);
    }

    world.Step(1.0 / 60.0);

    for (std::size_t index = 0; index < pairCount; ++index)
    {
        EXPECT_LT(world.State(leftBodies[index]).positionMeters.x,
            leftPositions[index].x) << "겹친 드론 쌍 " << index;
        EXPECT_GT(world.State(rightBodies[index]).positionMeters.x,
            rightPositions[index].x) << "겹친 드론 쌍 " << index;
    }
}

void Verify_AirSimBoundary_RejectsValuesItCannotRepresent()
{
    PhysicsWorld world;
    const RigidBodyHandle drone = MakeX500(world, 10.0);
    std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
    motorTargets.fill(800.0);
    world.SetMotorTargets(drone, motorTargets);
    world.Step(1.0 / 60.0);

    auto invalidMotorTargets = motorTargets;
    invalidMotorTargets.back() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(world.SetMotorTargets(drone, invalidMotorTargets), std::invalid_argument);
    world.Step(1.0 / 60.0);
    const double expectedMotorSpeed = motorTargets[0] * std::sqrt(
        1.0 - std::exp(-1.0 / 60.0
            / QuadrotorParameters::X500Gazebo().motorTimeConstantUpSeconds));
    for (const double motorSpeed : world.MotorSpeedsRadiansPerSecond(drone))
        EXPECT_NEAR(motorSpeed, expectedMotorSpeed, 1.0);

    const RigidBodyState stateBeforeRejectedUpdate = world.State(drone);
    const auto motorSpeedsBeforeRejectedUpdate = world.MotorSpeedsRadiansPerSecond(drone);
    const double tooLargeForFloat =
        static_cast<double>(std::numeric_limits<float>::max()) * 2.0;

    EXPECT_THROW(world.SetWindVelocityMetersPerSecond({tooLargeForFloat, 0.0, 0.0}),
        std::invalid_argument);

    RigidBodyState unrepresentableState = stateBeforeRejectedUpdate;
    unrepresentableState.positionMeters.x = tooLargeForFloat;
    EXPECT_THROW(world.SetState(drone, unrepresentableState), std::invalid_argument);

    const RigidBodyState stateAfterRejectedUpdate = world.State(drone);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.positionMeters.x,
        stateBeforeRejectedUpdate.positionMeters.x);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.positionMeters.y,
        stateBeforeRejectedUpdate.positionMeters.y);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.positionMeters.z,
        stateBeforeRejectedUpdate.positionMeters.z);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.linearVelocityMetersPerSecond.x,
        stateBeforeRejectedUpdate.linearVelocityMetersPerSecond.x);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.linearVelocityMetersPerSecond.y,
        stateBeforeRejectedUpdate.linearVelocityMetersPerSecond.y);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.linearVelocityMetersPerSecond.z,
        stateBeforeRejectedUpdate.linearVelocityMetersPerSecond.z);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.orientation.w,
        stateBeforeRejectedUpdate.orientation.w);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.orientation.x,
        stateBeforeRejectedUpdate.orientation.x);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.orientation.y,
        stateBeforeRejectedUpdate.orientation.y);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.orientation.z,
        stateBeforeRejectedUpdate.orientation.z);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.angularVelocityRadiansPerSecond.x,
        stateBeforeRejectedUpdate.angularVelocityRadiansPerSecond.x);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.angularVelocityRadiansPerSecond.y,
        stateBeforeRejectedUpdate.angularVelocityRadiansPerSecond.y);
    EXPECT_DOUBLE_EQ(stateAfterRejectedUpdate.angularVelocityRadiansPerSecond.z,
        stateBeforeRejectedUpdate.angularVelocityRadiansPerSecond.z);
    EXPECT_EQ(world.MotorSpeedsRadiansPerSecond(drone), motorSpeedsBeforeRejectedUpdate);

    QuadrotorParameters unrepresentableMass = QuadrotorParameters::X500Gazebo();
    unrepresentableMass.massKilograms = tooLargeForFloat;
    EXPECT_THROW(static_cast<void>(world.CreateQuadrotor({}, unrepresentableMass)),
        std::invalid_argument);

    QuadrotorParameters massThatRoundsToZero = QuadrotorParameters::X500Gazebo();
    massThatRoundsToZero.massKilograms = std::numeric_limits<double>::denorm_min();
    EXPECT_THROW(static_cast<void>(world.CreateQuadrotor({}, massThatRoundsToZero)),
        std::invalid_argument);
}

TEST(PhysicsWorldTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_RotorSpeedChangesGraduallyAfterACommand());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimFlightController_HoldsAStationaryWaypoint());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_AsymmetricRotorTargetsChangeAttitude());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysics_UsesEastNorthUpWorldAndForwardLeftUpBodyAxes());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_TerrainContactStopsTheDrone());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_OpenStreetMapBuildingContactStopsHorizontalMotion());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimCollision_UsesDroneBoxesInsteadOfTheirBoundingSphere());
    EXPECT_NO_FATAL_FAILURE(Verify_CollisionBroadphase_KeepsTouchingBoxesForPreciseTest());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimCollision_SeparatesOverlappingDrones());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimBoundary_RejectsValuesItCannotRepresent());
}

}
