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
    const double expectedFirstOrderResponse = motorTargets[0]
        * (1.0 - std::exp(-stepSeconds
            / QuadrotorParameters::X500Gazebo().motorTimeConstantUpSeconds));
    EXPECT_LT(speedAfterFirstStep, speedAfterSecondStep);
    EXPECT_NEAR(speedAfterSecondStep, expectedFirstOrderResponse, 1.0);

    for (int tick = 0; tick < 10; ++tick)
        world.Step(1.0 / 60.0);
    const double laterSpeed = world.MotorSpeedsRadiansPerSecond(drone)[0];

    EXPECT_LT(speedAfterSecondStep, laterSpeed);
    EXPECT_LT(laterSpeed, motorTargets[0]);

    motorTargets.fill(0.0);
    world.SetMotorTargets(drone, motorTargets);
    world.Step(stepSeconds);
    const double speedAfterFirstDownStep =
        world.MotorSpeedsRadiansPerSecond(drone)[0];
    world.Step(stepSeconds);
    const double speedAfterSecondDownStep =
        world.MotorSpeedsRadiansPerSecond(drone)[0];
    const double expectedDownResponse = speedAfterFirstDownStep
        * std::exp(-stepSeconds
            / QuadrotorParameters::X500Gazebo().motorTimeConstantDownSeconds);
    EXPECT_LT(speedAfterSecondDownStep, speedAfterFirstDownStep);
    EXPECT_NEAR(speedAfterSecondDownStep, expectedDownResponse, 0.5);
}

void Verify_AirSimPhysics_EqualHoverThrustBalancesGravityAfterMotorSpinUp()
{
    constexpr double stepSeconds = 1.0 / 60.0;
    constexpr int simulationTicks = 120;
    const std::array<QuadrotorParameters, 2> models{
        QuadrotorParameters::X500Gazebo(),
        QuadrotorParameters::X500MotorTableEstimate()};

    for (std::size_t modelIndex = 0; modelIndex < models.size(); ++modelIndex)
    {
        SCOPED_TRACE(modelIndex == 0 ? "PX4 Gazebo model" : "motor table estimate");
        const QuadrotorParameters& parameters = models[modelIndex];
        PhysicsWorld world;
        RigidBodyDefinition definition;
        definition.positionMeters.z = 10.0;
        const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);
        const double hoverSpeed = std::sqrt(parameters.massKilograms * 9.81
            / (parameters.rotors.size() * parameters.thrustCoefficientNewtonsPerSpeedSquared));
        const double expectedRotorThrust = parameters.thrustCoefficientNewtonsPerSpeedSquared
            * hoverSpeed * hoverSpeed;
        std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
        motorTargets.fill(hoverSpeed);
        world.SetMotorTargets(drone, motorTargets);

        // 모터가 안정된 뒤 로터 추력과 중력이 서로 상쇄되는지 확인한다.
        for (int tick = 0; tick < simulationTicks; ++tick)
            world.Step(stepSeconds);

        const RigidBodyState state = world.State(drone);
        for (const double speed : world.MotorSpeedsRadiansPerSecond(drone))
            EXPECT_NEAR(speed, hoverSpeed, 1.0);
        for (const double thrust : world.MotorThrustsNewtons(drone))
            EXPECT_NEAR(thrust, expectedRotorThrust, 0.05);
        const PhysicsDynamicsSample dynamics = world.DynamicsSample(drone);
        EXPECT_NEAR(dynamics.appliedForceNewtons.z,
            parameters.massKilograms * 9.81, 0.1);
        EXPECT_NEAR(dynamics.gravityMetersPerSecondSquared.z, -9.80665, 1.0e-4);
        EXPECT_NEAR(dynamics.linearAccelerationMetersPerSecondSquared.z,
            dynamics.appliedForceNewtons.z / parameters.massKilograms
                + dynamics.gravityMetersPerSecondSquared.z,
            1.0e-3);
        EXPECT_NEAR(dynamics.linearAccelerationMetersPerSecondSquared.z, 0.0, 0.02);
        EXPECT_NEAR(state.positionMeters.x, definition.positionMeters.x, 0.01);
        EXPECT_NEAR(state.positionMeters.y, definition.positionMeters.y, 0.01);
        EXPECT_LT(glm::length(state.angularVelocityRadiansPerSecond), 0.01);
        EXPECT_NEAR(state.orientation.w, 1.0, 1.0e-4);
        EXPECT_NEAR(state.orientation.x, 0.0, 1.0e-4);
        EXPECT_NEAR(state.orientation.y, 0.0, 1.0e-4);
        EXPECT_NEAR(state.orientation.z, 0.0, 1.0e-4);
    }
}

void Verify_AirSimPhysics_GravityAndCollectiveThrustChangeVerticalMotion()
{
    constexpr double stepSeconds = 1.0 / 60.0;
    constexpr int simulationTicks = 30;
    constexpr double gravityMetersPerSecondSquared = 9.80665;
    const std::array<QuadrotorParameters, 2> models{
        QuadrotorParameters::X500Gazebo(),
        QuadrotorParameters::X500MotorTableEstimate()};

    for (std::size_t modelIndex = 0; modelIndex < models.size(); ++modelIndex)
    {
        SCOPED_TRACE(modelIndex == 0 ? "PX4 Gazebo model" : "motor table estimate");
        const QuadrotorParameters& parameters = models[modelIndex];
        const double hoverSpeed = std::sqrt(parameters.massKilograms
            * gravityMetersPerSecondSquared
            / (static_cast<double>(parameters.rotors.size())
                * parameters.thrustCoefficientNewtonsPerSpeedSquared));

        const auto runWithCollectiveThrust = [&](double hoverSpeedRatio)
        {
            PhysicsWorld world;
            RigidBodyDefinition definition;
            definition.positionMeters.z = 10.0;
            const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);
            std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
            motorTargets.fill(hoverSpeed * hoverSpeedRatio);
            world.SetMotorTargets(drone, motorTargets);
            for (int tick = 0; tick < simulationTicks; ++tick)
                world.Step(stepSeconds);
            return world.State(drone);
        };

        // 호버 속도보다 낮은 입력은 하강하고, 높은 입력은 상승해야 한다.
        const RigidBodyState freeFall = runWithCollectiveThrust(0.0);
        const RigidBodyState belowHover = runWithCollectiveThrust(0.8);
        const RigidBodyState aboveHover = runWithCollectiveThrust(1.2);
        const double elapsedSeconds = simulationTicks * stepSeconds;
        const double expectedFreeFallHeight = 10.0
            - 0.5 * gravityMetersPerSecondSquared * elapsedSeconds * elapsedSeconds;

        EXPECT_NEAR(freeFall.positionMeters.z, expectedFreeFallHeight, 0.3);
        EXPECT_LT(freeFall.linearVelocityMetersPerSecond.z, -4.0);
        EXPECT_LT(belowHover.linearVelocityMetersPerSecond.z, 0.0);
        EXPECT_GT(aboveHover.linearVelocityMetersPerSecond.z, 0.0);
        EXPECT_GT(aboveHover.positionMeters.z, belowHover.positionMeters.z);
    }
}

void Verify_AirSimPhysics_MassChangesMotionUnderTheSameRotorInput()
{
    constexpr int settlingTicks = 10;
    constexpr double stepSeconds = 1.0 / 60.0;
    QuadrotorParameters baseline = QuadrotorParameters::X500Gazebo();
    QuadrotorParameters heavier = baseline;
    heavier.massKilograms *= 1.25;
    const double baselineHoverSpeed = std::sqrt(baseline.massKilograms * 9.81
        / (baseline.rotors.size()
            * baseline.thrustCoefficientNewtonsPerSpeedSquared));

    const auto run = [&](const QuadrotorParameters& parameters)
    {
        PhysicsWorld world;
        RigidBodyDefinition definition;
        definition.positionMeters.z = 10.0;
        const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);
        std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
        motorTargets.fill(baselineHoverSpeed);
        world.SetMotorTargets(drone, motorTargets);
        for (int tick = 0; tick < settlingTicks; ++tick)
            world.Step(stepSeconds);
        return world.DynamicsSample(drone);
    };

    // 같은 로터 명령이 안정된 뒤 질량만 바꾸고 힘과 가속도를 비교한다.
    const PhysicsDynamicsSample baselineSample = run(baseline);
    const PhysicsDynamicsSample heavierSample = run(heavier);
    EXPECT_NEAR(baselineSample.appliedForceNewtons.z,
        heavierSample.appliedForceNewtons.z, 0.1);
    EXPECT_GT(baselineSample.linearAccelerationMetersPerSecondSquared.z,
        heavierSample.linearAccelerationMetersPerSecondSquared.z + 1.0);
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

void Verify_AirSimFlightController_GainsChangeYawMotorTargets()
{
    DroneState state;
    state.positionMeters.z = 10.0;
    state.angularVelocityRadiansPerSecond.z = 0.2;
    state.mode = FlightMode::Hovering;
    DroneControlState control;
    control.desiredYawRadians = 0.5;
    const QuadrotorParameters model = QuadrotorParameters::X500Gazebo();

    QuadrotorControllerParameters lowAttitudeGain;
    lowAttitudeGain.attitudeGain = 2.0;
    QuadrotorControllerParameters highAttitudeGain;
    highAttitudeGain.attitudeGain = 20.0;
    const auto lowAttitudeTargets = QuadrotorFlightController::MotorTargets(
        state, control, model, lowAttitudeGain);
    const auto highAttitudeTargets = QuadrotorFlightController::MotorTargets(
        state, control, model, highAttitudeGain);
    EXPECT_NE(lowAttitudeTargets, highAttitudeTargets);

    QuadrotorControllerParameters weakRateDamping;
    weakRateDamping.angularRateGain = 2.0;
    QuadrotorControllerParameters strongRateDamping;
    strongRateDamping.angularRateGain = 14.0;
    const auto weakDampingTargets = QuadrotorFlightController::MotorTargets(
        state, control, model, weakRateDamping);
    const auto strongDampingTargets = QuadrotorFlightController::MotorTargets(
        state, control, model, strongRateDamping);
    EXPECT_NE(weakDampingTargets, strongDampingTargets);
}

void Verify_AirSimFlightController_HoverTargetsFollowMassAndThrustCoefficient()
{
    DroneState state;
    state.positionMeters.z = 10.0;
    state.mode = FlightMode::Hovering;
    DroneControlState control;
    const QuadrotorParameters baseline = QuadrotorParameters::X500Gazebo();
    QuadrotorParameters heavier = baseline;
    heavier.massKilograms *= 1.25;
    QuadrotorParameters strongerThrust = baseline;
    strongerThrust.thrustCoefficientNewtonsPerSpeedSquared *= 1.5;

    const auto baselineTargets = QuadrotorFlightController::MotorTargets(
        state, control, baseline);
    const auto heavierTargets = QuadrotorFlightController::MotorTargets(
        state, control, heavier);
    const auto strongerThrustTargets = QuadrotorFlightController::MotorTargets(
        state, control, strongerThrust);
    constexpr double gravityMetersPerSecondSquared = 9.81;
    const auto expectedHoverSpeed = [gravityMetersPerSecondSquared](
        const QuadrotorParameters& parameters)
    {
        return std::sqrt(parameters.massKilograms
            * gravityMetersPerSecondSquared
            / (static_cast<double>(parameters.rotors.size())
                * parameters.thrustCoefficientNewtonsPerSpeedSquared));
    };

    for (std::size_t rotor = 0; rotor < baselineTargets.size(); ++rotor)
    {
        EXPECT_NEAR(baselineTargets[rotor], expectedHoverSpeed(baseline), 1.0e-9);
        EXPECT_NEAR(heavierTargets[rotor], expectedHoverSpeed(heavier), 1.0e-9);
        EXPECT_NEAR(strongerThrustTargets[rotor], expectedHoverSpeed(strongerThrust), 1.0e-9);
    }
    EXPECT_GT(heavierTargets[0], baselineTargets[0]);
    EXPECT_LT(strongerThrustTargets[0], baselineTargets[0]);
}

void Verify_AirSimPhysics_RotorTorqueResponseScalesWithPitchInertia()
{
    const auto pitchRateAfterAsymmetricInput = [](const QuadrotorParameters& parameters)
    {
        PhysicsWorld world;
        RigidBodyDefinition definition;
        definition.positionMeters.z = 10.0;
        const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);

        std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
        motorTargets.fill(760.0);
        world.SetMotorTargets(drone, motorTargets);
        for (int tick = 0; tick < 120; ++tick)
            world.Step(1.0 / 60.0);

        // 앞쪽 두 로터를 빠르게 돌려 기체의 좌우 축에 회전 토크를 만든다.
        motorTargets[0] = 900.0;
        motorTargets[2] = 900.0;
        world.SetMotorTargets(drone, motorTargets);
        for (int tick = 0; tick < 60; ++tick)
            world.Step(1.0 / 60.0);
        return std::abs(world.State(drone).angularVelocityRadiansPerSecond.y);
    };

    const std::array<QuadrotorParameters, 2> models{
        QuadrotorParameters::X500Gazebo(),
        QuadrotorParameters::X500MotorTableEstimate()};
    for (std::size_t modelIndex = 0; modelIndex < models.size(); ++modelIndex)
    {
        SCOPED_TRACE(modelIndex == 0 ? "PX4 Gazebo model" : "motor table estimate");
        const QuadrotorParameters& baseline = models[modelIndex];
        QuadrotorParameters higherPitchInertia = baseline;
        higherPitchInertia.principalInertiaKilogramMetersSquared.y *= 2.0;

        const double baselinePitchRate = pitchRateAfterAsymmetricInput(baseline);
        const double higherInertiaPitchRate = pitchRateAfterAsymmetricInput(higherPitchInertia);
        EXPECT_GT(baselinePitchRate, 0.1);
        EXPECT_NEAR(higherInertiaPitchRate / baselinePitchRate, 0.5, 0.2);
    }
}

void Verify_AirSimPhysics_YawResponseScalesWithMomentToThrustRatio()
{
    const auto yawRateAfterAsymmetricInput = [](const QuadrotorParameters& parameters)
    {
        PhysicsWorld world;
        RigidBodyDefinition definition;
        definition.positionMeters.z = 10.0;
        const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);

        std::array<double, QuadrotorParameters::rotorCount> motorTargets{};
        motorTargets.fill(760.0);
        world.SetMotorTargets(drone, motorTargets);
        for (int tick = 0; tick < 120; ++tick)
            world.Step(1.0 / 60.0);

        // 같은 회전 방향의 두 로터만 빠르게 돌려 수직축 토크를 만든다.
        motorTargets[0] = 900.0;
        motorTargets[1] = 900.0;
        world.SetMotorTargets(drone, motorTargets);
        for (int tick = 0; tick < 30; ++tick)
            world.Step(1.0 / 60.0);
        return std::abs(world.State(drone).angularVelocityRadiansPerSecond.z);
    };

    const std::array<QuadrotorParameters, 2> models{
        QuadrotorParameters::X500Gazebo(),
        QuadrotorParameters::X500MotorTableEstimate()};
    for (std::size_t modelIndex = 0; modelIndex < models.size(); ++modelIndex)
    {
        SCOPED_TRACE(modelIndex == 0 ? "PX4 Gazebo model" : "motor table estimate");
        const QuadrotorParameters& baseline = models[modelIndex];
        QuadrotorParameters higherMomentRatio = baseline;
        higherMomentRatio.momentToThrustRatio *= 2.0;

        const double baselineYawRate = yawRateAfterAsymmetricInput(baseline);
        const double higherMomentRatioYawRate = yawRateAfterAsymmetricInput(higherMomentRatio);
        EXPECT_GT(baselineYawRate, 0.1);
        EXPECT_NEAR(higherMomentRatioYawRate / baselineYawRate, 2.0, 0.2);
    }
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

void Verify_AirSimPhysicsKeepsContactAcrossMicroscopicGroundGaps()
{
    PhysicsWorld world;
    const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
    RigidBodyDefinition definition;
    // X500 로터 가드의 가장 낮은 면은 기체 중심에서 0.227m 아래에 있다.
    definition.positionMeters.z = 0.227 + 1.0e-8;
    const RigidBodyHandle drone = world.CreateQuadrotor(definition, parameters);

    RigidBodyState fallingState;
    fallingState.positionMeters = definition.positionMeters;
    fallingState.linearVelocityMetersPerSecond.z = -0.1;
    world.SetState(drone, fallingState);
    world.Step(1.0 / 60.0);

    const RigidBodyState state = world.State(drone);
    EXPECT_NEAR(state.positionMeters.z, definition.positionMeters.z, 5.0e-4);
    EXPECT_NEAR(state.linearVelocityMetersPerSecond.z, 0.0, 1.0e-4);
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
    const double expectedMotorSpeed = motorTargets[0]
        * (1.0 - std::exp(-1.0 / 60.0
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
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysics_EqualHoverThrustBalancesGravityAfterMotorSpinUp());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysics_GravityAndCollectiveThrustChangeVerticalMotion());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysics_MassChangesMotionUnderTheSameRotorInput());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimFlightController_HoldsAStationaryWaypoint());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimFlightController_GainsChangeYawMotorTargets());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimFlightController_HoverTargetsFollowMassAndThrustCoefficient());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysics_RotorTorqueResponseScalesWithPitchInertia());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysics_YawResponseScalesWithMomentToThrustRatio());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysics_UsesEastNorthUpWorldAndForwardLeftUpBodyAxes());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_TerrainContactStopsTheDrone());
    EXPECT_NO_FATAL_FAILURE(
        Verify_AirSimPhysicsKeepsContactAcrossMicroscopicGroundGaps());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimPhysics_OpenStreetMapBuildingContactStopsHorizontalMotion());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimCollision_UsesDroneBoxesInsteadOfTheirBoundingSphere());
    EXPECT_NO_FATAL_FAILURE(Verify_CollisionBroadphase_KeepsTouchingBoxesForPreciseTest());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimCollision_SeparatesOverlappingDrones());
    EXPECT_NO_FATAL_FAILURE(Verify_AirSimBoundary_RejectsValuesItCannotRepresent());
}

}
