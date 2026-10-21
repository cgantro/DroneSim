#include "simulation_core/PhysicsWorld.h"
#include "terrain/BuildingFootprint.h"
#include "terrain/StaticFacility.h"
#include "terrain/TerrainGrid.h"
#include "simulation_core/QuadrotorModel.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iterator>
#include <stdexcept>
#include <vector>

namespace aerosimlab::simulation
{
namespace
{
RigidBodyHandle MakeBody(PhysicsWorld& world, glm::dvec3 position)
{
    RigidBodyDefinition definition;
    definition.halfExtentsMeters = {0.1F, 0.1F, 0.1F};
    definition.positionMeters = position;
    definition.massKilograms = 2.0;
    return world.CreateBox(definition);
}
}

TEST(PhysicsWorld, GravityAcceleratesAFreeBodyDownward)
{
    PhysicsWorld world;
    constexpr double stepSeconds = 1.0 / 60.0;
    constexpr int tickCount = 60;
    constexpr double gravityMetersPerSecondSquared = 9.81;
    const RigidBodyHandle body = MakeBody(world, {0.0, 0.0, 10.0});

    double physicsStepDurationMicroseconds = 0.0;
    world.Step(stepSeconds, 1, &physicsStepDurationMicroseconds);
    EXPECT_GT(physicsStepDurationMicroseconds, 0.0);
    for (int tick = 1; tick < tickCount; ++tick)
        world.Step(stepSeconds);

    const RigidBodyState state = world.State(body);
    // Jolt는 매 단계에서 속도를 먼저 갱신한 뒤, 그 속도로 위치를 움직인다.
    const double expectedPosition = 10.0
        - gravityMetersPerSecondSquared * stepSeconds * stepSeconds
            * tickCount * (tickCount + 1) / 2.0;
    EXPECT_NEAR(state.linearVelocityMetersPerSecond.z,
        -gravityMetersPerSecondSquared * tickCount * stepSeconds, 0.03);
    EXPECT_NEAR(state.positionMeters.z, expectedPosition, 0.01);
}

TEST(PhysicsWorld, ConstantForceProducesMassDependentAcceleration)
{
    PhysicsWorld world;
    RigidBodyDefinition lightDefinition;
    lightDefinition.positionMeters = {-1.0, 0.0, 10.0};
    lightDefinition.massKilograms = 2.0;
    const RigidBodyHandle lightBody = world.CreateBox(lightDefinition);

    RigidBodyDefinition heavyDefinition;
    heavyDefinition.positionMeters = {1.0, 0.0, 10.0};
    heavyDefinition.massKilograms = 4.0;
    const RigidBodyHandle heavyBody = world.CreateBox(heavyDefinition);

    for (int tick = 0; tick < 60; ++tick)
    {
        world.ApplyForce(lightBody, {0.0, 0.0, 24.0});
        world.ApplyForce(heavyBody, {0.0, 0.0, 24.0});
        world.Step(1.0 / 60.0);
    }

    // 위쪽을 양수로 두면 가속도는 힘/질량에서 중력을 뺀 값이다.
    const double gravityMetersPerSecondSquared = 9.81;
    // Z축에만 토크를 주므로 각가속도는 토크/해당 축 관성이다.
    const double elapsedSeconds = 1.0;
    const double lightExpectedVelocity = (24.0 / 2.0 - gravityMetersPerSecondSquared)
        * elapsedSeconds;
    const double heavyExpectedVelocity = (24.0 / 4.0 - gravityMetersPerSecondSquared)
        * elapsedSeconds;
    EXPECT_NEAR(world.State(lightBody).linearVelocityMetersPerSecond.z,
        lightExpectedVelocity, 0.03);
    EXPECT_NEAR(world.State(heavyBody).linearVelocityMetersPerSecond.z,
        heavyExpectedVelocity, 0.03);
}

TEST(PhysicsWorld, X500RotorThrustKeepsTheBodyHovering)
{
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    const double hoverSpeed = std::sqrt(
        parameters.massKilograms * 9.81
        / (QuadrotorModel::rotorCount * parameters.thrustCoefficientNewtonsPerSpeedSquared));
    QuadrotorState motors;
    motors.motorSpeedRadiansPerSecond.fill(hoverSpeed);
    const QuadrotorWrench wrench = QuadrotorModel::CalculateWrench(motors, parameters);

    PhysicsWorld world;
    RigidBodyDefinition definition;
    definition.positionMeters = {0.0, 0.0, 2.0};
    definition.massKilograms = parameters.massKilograms;
    definition.principalInertiaKilogramMetersSquared =
        parameters.principalInertiaKilogramMetersSquared;
    const RigidBodyHandle body = world.CreateBox(definition);

    for (int tick = 0; tick < 120; ++tick)
    {
        world.ApplyForce(body, wrench.forceNewtons);
        world.ApplyTorque(body, wrench.torqueNewtonMeters);
        world.Step(1.0 / 60.0);
    }

    const RigidBodyState result = world.State(body);
    EXPECT_NEAR(result.positionMeters.z, 2.0, 0.01);
    EXPECT_NEAR(result.linearVelocityMetersPerSecond.z, 0.0, 0.01);
}

TEST(PhysicsWorld, AngularResponseUsesProvidedInertia)
{
    PhysicsWorld world;
    RigidBodyDefinition lowInertiaDefinition;
    lowInertiaDefinition.positionMeters = {-1.0, 0.0, 10.0};
    lowInertiaDefinition.principalInertiaKilogramMetersSquared = {0.02, 0.02, 0.02};
    const RigidBodyHandle lowInertiaBody = world.CreateBox(lowInertiaDefinition);

    RigidBodyDefinition highInertiaDefinition;
    highInertiaDefinition.positionMeters = {1.0, 0.0, 10.0};
    highInertiaDefinition.principalInertiaKilogramMetersSquared = {0.02, 0.02, 0.08};
    const RigidBodyHandle highInertiaBody = world.CreateBox(highInertiaDefinition);

    for (int tick = 0; tick < 60; ++tick)
    {
        world.ApplyTorque(lowInertiaBody, {0.0, 0.0, 0.2});
        world.ApplyTorque(highInertiaBody, {0.0, 0.0, 0.2});
        world.Step(1.0 / 60.0);
    }

    const double elapsedSeconds = 1.0;
    const double appliedTorqueNewtonMeters = 0.2;
    EXPECT_NEAR(
        world.State(lowInertiaBody).angularVelocityRadiansPerSecond.z,
        appliedTorqueNewtonMeters / 0.02 * elapsedSeconds,
        0.05);
    EXPECT_NEAR(
        world.State(highInertiaBody).angularVelocityRadiansPerSecond.z,
        appliedTorqueNewtonMeters / 0.08 * elapsedSeconds,
        0.05);
}

TEST(PhysicsWorld, TorqueChangesAngularVelocityAndOrientation)
{
    PhysicsWorld world;
    const RigidBodyHandle body = MakeBody(world, {0.0, 0.0, 10.0});

    for (int tick = 0; tick < 30; ++tick)
    {
        world.ApplyTorque(body, {0.0, 0.0, 0.2});
        world.Step(1.0 / 60.0);
    }

    const RigidBodyState state = world.State(body);
    EXPECT_GT(state.angularVelocityRadiansPerSecond.z, 0.1);
    EXPECT_GT(std::abs(state.orientation.z), 0.01);
}

TEST(PhysicsWorld, GroundStopsAFallingBody)
{
    PhysicsWorld world;
    const RigidBodyHandle body = MakeBody(world, {0.0, 0.0, 2.0});

    for (int tick = 0; tick < 240; ++tick)
        world.Step(1.0 / 60.0);

    const RigidBodyState state = world.State(body);
    EXPECT_GE(state.positionMeters.z, 0.08);
    EXPECT_LT(std::abs(state.linearVelocityMetersPerSecond.z), 0.1);
}

TEST(PhysicsWorld, CompoundBoxesCanRestOnSeparatedRaisedPlatforms)
{
    PhysicsWorld world;
    RigidBodyDefinition platform;
    platform.isStatic = true;
    platform.halfExtentsMeters = {0.2F, 0.2F, 0.5F};
    platform.positionMeters = {-0.5, 0.0, 0.5};
    static_cast<void>(world.CreateBox(platform));
    platform.positionMeters.x = 0.5;
    static_cast<void>(world.CreateBox(platform));

    const std::array<RigidBodyBox, 2> collisionBoxes{{
        {{0.1F, 0.1F, 0.1F}, {-0.5, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}},
        {{0.1F, 0.1F, 0.1F}, {0.5, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}}}};
    RigidBodyDefinition bodyDefinition;
    bodyDefinition.positionMeters = {0.0, 0.0, 3.0};
    const RigidBodyHandle body = world.CreateCompoundBoxes(bodyDefinition, collisionBoxes);

    for (int tick = 0; tick < 240; ++tick)
        world.Step(1.0 / 60.0);

    const RigidBodyState state = world.State(body);
    EXPECT_NEAR(state.positionMeters.z, 1.1, 0.03);
    EXPECT_LT(std::abs(state.linearVelocityMetersPerSecond.z), 0.1);
}

TEST(PhysicsWorld, TerrainHeightFieldStopsABodyAtTheHighestPointInTheTestArea)
{
    const aerosimlab::terrain::TerrainGrid terrain = aerosimlab::terrain::TerrainGrid::Load(
        std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
        / "assets/maps/bukhansan-demo/terrain.grid");
    const auto peak = std::max_element(
        terrain.heightOffsetsMeters.begin(), terrain.heightOffsetsMeters.end());
    const std::size_t peakIndex = static_cast<std::size_t>(
        std::distance(terrain.heightOffsetsMeters.begin(), peak));
    const std::size_t peakRow = peakIndex / terrain.sampleCount;
    const std::size_t peakColumn = peakIndex % terrain.sampleCount;
    const double peakX = (static_cast<double>(peakColumn)
        - static_cast<double>(terrain.sampleCount - 1U) * 0.5)
        * terrain.spacingXMeters;
    const double peakY = (static_cast<double>(terrain.sampleCount - 1U) * 0.5
        - static_cast<double>(peakRow)) * terrain.spacingYMeters;
    const double peakHeight = *peak;

    PhysicsWorld world;
    world.SetTerrain(terrain);
    RigidBodyDefinition bodyDefinition;
    bodyDefinition.positionMeters = {peakX, peakY, peakHeight + 5.0};
    bodyDefinition.halfExtentsMeters = glm::vec3{0.1F};
    const RigidBodyHandle body = world.CreateBox(bodyDefinition);
    for (int step = 0; step < 240; ++step)
        world.Step(1.0 / 60.0);

    EXPECT_NEAR(world.State(body).positionMeters.x, peakX, 0.05);
    EXPECT_NEAR(world.State(body).positionMeters.y, peakY, 0.05);
    EXPECT_NEAR(world.State(body).positionMeters.z, peakHeight + 0.1, 0.12);
}

TEST(PhysicsWorld, OSMBuildingBoxStopsABodyAboveItsTerrainSample)
{
    const std::filesystem::path root{AEROSIMLAB_SOURCE_DIR};
    const aerosimlab::terrain::TerrainGrid terrain =
        aerosimlab::terrain::TerrainGrid::Load(
            root / "assets/maps/bukhansan-demo/terrain.grid");
    const auto buildings = aerosimlab::terrain::LoadBuildingFootprints(
        root / "assets/maps/bukhansan-demo/buildings.geojson", terrain);
    ASSERT_FALSE(buildings.empty());

    glm::dvec2 minimum = buildings.front().outlineMeters.front();
    glm::dvec2 maximum = minimum;
    for (const glm::dvec2& point : buildings.front().outlineMeters)
    {
        minimum = glm::min(minimum, point);
        maximum = glm::max(maximum, point);
    }
    const glm::dvec2 footprintCenter = (minimum + maximum) * 0.5;
    constexpr float assumedBuildingHeightMeters = 3.0F;
    const double groundHeightMeters = terrain.HeightAtLocalMeters(footprintCenter);

    PhysicsWorld world;
    world.SetTerrain(terrain);
    RigidBodyDefinition building;
    building.isStatic = true;
    building.positionMeters = {
        footprintCenter.x, footprintCenter.y,
        groundHeightMeters + assumedBuildingHeightMeters * 0.5};
    building.halfExtentsMeters = {
        static_cast<float>(maximum.x - minimum.x) * 0.5F,
        static_cast<float>(maximum.y - minimum.y) * 0.5F,
        assumedBuildingHeightMeters * 0.5F};
    static_cast<void>(world.CreateBox(building));

    const RigidBodyHandle body = MakeBody(
        world, {footprintCenter.x, footprintCenter.y, groundHeightMeters + 10.0});
    for (int tick = 0; tick < 240; ++tick)
        world.Step(1.0 / 60.0);

    const RigidBodyState result = world.State(body);
    EXPECT_GE(result.positionMeters.z,
        groundHeightMeters + assumedBuildingHeightMeters - 0.2);
    EXPECT_LT(result.positionMeters.z,
        groundHeightMeters + assumedBuildingHeightMeters + 1.0);
    EXPECT_LT(std::abs(result.linearVelocityMetersPerSecond.z), 0.1);
}

TEST(PhysicsWorld, SurveyedFacilityBoundsStopAFallingBody)
{
    const std::vector<aerosimlab::terrain::StaticFacility> facilities =
        aerosimlab::terrain::LoadStaticFacilities(
            std::filesystem::path{AEROSIMLAB_SOURCE_DIR}
            / "assets/maps/bukhansan-demo/facilities.json");
    const auto facility = std::find_if(facilities.begin(), facilities.end(),
        [](const auto& value) { return value.id == "B-M-01"; });
    ASSERT_NE(facility, facilities.end());

    PhysicsWorld world;
    RigidBodyDefinition obstacle;
    obstacle.isStatic = true;
    obstacle.positionMeters = facility->centerMeters + glm::dvec3{0.0, 0.0, 100.0};
    obstacle.halfExtentsMeters = facility->halfExtentsMeters;
    static_cast<void>(world.CreateBox(obstacle));

    const glm::dvec3 center = obstacle.positionMeters;
    const double top = center.z + facility->halfExtentsMeters.z;
    const RigidBodyHandle body = MakeBody(world, {center.x, center.y, top + 5.0});
    for (int tick = 0; tick < 240; ++tick)
        world.Step(1.0 / 60.0);

    EXPECT_NEAR(world.State(body).positionMeters.z, top + 0.1, 0.1);
}

TEST(PhysicsWorld, RejectsInvalidBodySize)
{
    PhysicsWorld world;
    RigidBodyDefinition definition;
    definition.halfExtentsMeters.x = 0.0F;
    EXPECT_THROW(static_cast<void>(world.CreateBox(definition)), std::invalid_argument);
}

TEST(PhysicsWorld, RejectsInvalidPrincipalInertia)
{
    PhysicsWorld world;
    RigidBodyDefinition definition;
    definition.principalInertiaKilogramMetersSquared = {0.02, 0.0, 0.04};

    EXPECT_THROW(static_cast<void>(world.CreateBox(definition)), std::invalid_argument);
}

TEST(PhysicsWorld, DestroyBodyInvalidatesItsHandle)
{
    PhysicsWorld world;
    const RigidBodyHandle body = MakeBody(world, {0.0, 0.0, 1.0});

    EXPECT_TRUE(world.DestroyBody(body));
    EXPECT_FALSE(world.DestroyBody(body));
    EXPECT_THROW((void)world.State(body), std::out_of_range);
}
}
