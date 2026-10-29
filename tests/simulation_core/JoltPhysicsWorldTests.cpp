#include "simulation_core/JoltPhysicsWorld.h"
#include "simulation_core/QuadrotorModel.h"
#include "terrain/TerrainGrid.h"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

namespace aerosimlab::simulation {
namespace {
JoltRigidBodyHandle MakeFallingBox(JoltPhysicsWorld &world, double heightMeters,
                                   double massKilograms = 1.0,
                                   double northMeters = 0.0) {
  JoltBoxDefinition definition;
  definition.positionMeters = {0.0, northMeters, heightMeters};
  definition.halfExtentsMeters = {0.25, 0.25, 0.25};
  definition.massKilograms = massKilograms;
  return world.CreateDynamicBox(definition);
}
} // namespace

TEST(JoltPhysicsWorldTests, RigidBodyBasics) {
  JoltPhysicsWorld world;
  const JoltRigidBodyHandle ground =
      world.AddStaticBox({0.0, 0.0, -0.5}, {5.0, 5.0, 0.5});
  const JoltRigidBodyHandle fallingBox = MakeFallingBox(world, 2.0);

  const JoltRigidBodyState initialState = world.State(fallingBox);
  world.StepFixed();
  const JoltRigidBodyState afterOneStep = world.State(fallingBox);
  EXPECT_LT(afterOneStep.positionMeters.z, initialState.positionMeters.z);
  EXPECT_LT(afterOneStep.linearVelocityMetersPerSecond.z, 0.0);

  for (int step = 0; step < 180; ++step)
    world.StepFixed();

  const JoltRigidBodyState restingState = world.State(fallingBox);
  EXPECT_NEAR(restingState.positionMeters.z, 0.25, 0.025);
  EXPECT_NEAR(restingState.linearVelocityMetersPerSecond.z, 0.0, 0.05);

  JoltRigidBodyState resetState = restingState;
  resetState.positionMeters = {1.0, 2.0, 3.0};
  resetState.orientation = glm::angleAxis(0.4, glm::dvec3{0.0, 0.0, 1.0});
  resetState.linearVelocityMetersPerSecond = {0.0, 0.0, 0.0};
  resetState.angularVelocityRadiansPerSecond = {0.0, 0.0, 0.0};
  world.SetState(fallingBox, resetState);
  const JoltRigidBodyState restoredState = world.State(fallingBox);
  EXPECT_NEAR(restoredState.positionMeters.x, 1.0, 1e-5);
  EXPECT_NEAR(restoredState.positionMeters.y, 2.0, 1e-5);
  EXPECT_NEAR(restoredState.positionMeters.z, 3.0, 1e-5);
  EXPECT_NEAR(glm::dot(restoredState.orientation, resetState.orientation), 1.0,
              1e-5);

  EXPECT_TRUE(world.DestroyBody(fallingBox));
  EXPECT_FALSE(world.DestroyBody(fallingBox));
  EXPECT_NO_THROW(static_cast<void>(world.State(ground)));
  EXPECT_THROW(static_cast<void>(world.State(fallingBox)), std::out_of_range);
}

TEST(JoltPhysicsWorldTests, TerrainAndBuildingMeshesStopFallingBodies) {
  JoltPhysicsWorld world;
  aerosimlab::terrain::TerrainGrid terrain;
  terrain.sampleCount = 3;
  terrain.spacingXMeters = 10.0F;
  terrain.spacingYMeters = 10.0F;
  terrain.heightOffsetsMeters.assign(9, 0.0F);
  world.SetTerrain(terrain);
  world.AddStaticPrism(
      {{4.0, -2.0}, {6.0, -2.0}, {6.0, 2.0}, {5.0, 0.5}, {4.0, 2.0}}, 0.0, 3.0);

  const JoltRigidBodyHandle fallingBox = MakeFallingBox(world, 2.0);
  for (int step = 0; step < 180; ++step)
    world.StepFixed();
  EXPECT_NEAR(world.State(fallingBox).positionMeters.z, 0.25, 0.03);

  JoltBoxDefinition approachingDefinition;
  approachingDefinition.positionMeters = {2.0, 0.0, 1.0};
  approachingDefinition.halfExtentsMeters = {0.25, 0.25, 0.25};
  const JoltRigidBodyHandle approachingBox =
      world.CreateDynamicBox(approachingDefinition);
  JoltRigidBodyState movingState = world.State(approachingBox);
  movingState.linearVelocityMetersPerSecond = {3.0, 0.0, 0.0};
  world.SetState(approachingBox, movingState);
  for (int step = 0; step < 120; ++step)
    world.StepFixed();
  EXPECT_LT(world.State(approachingBox).positionMeters.x, 3.8);
}

TEST(JoltPhysicsWorldTests, EqualForceProducesMassDependentAcceleration) {
  JoltPhysicsWorld world;
  const JoltRigidBodyHandle lightBox = MakeFallingBox(world, 20.0, 1.0);
  const JoltRigidBodyHandle heavyBox = MakeFallingBox(world, 20.0, 2.0, 2.0);
  world.ApplyForce(lightBox, {12.0, 0.0, 0.0});
  world.ApplyForce(heavyBox, {12.0, 0.0, 0.0});
  world.StepFixed();

  const double lightVelocity =
      world.State(lightBox).linearVelocityMetersPerSecond.x;
  const double heavyVelocity =
      world.State(heavyBox).linearVelocityMetersPerSecond.x;
  EXPECT_GT(lightVelocity, heavyVelocity);
  EXPECT_NEAR(lightVelocity, 0.2, 0.015);
  EXPECT_NEAR(heavyVelocity, 0.1, 0.015);
}

TEST(JoltPhysicsWorldTests, EqualTorqueProducesInertiaDependentRotation) {
  JoltPhysicsWorld world;
  JoltBoxDefinition smallDefinition;
  smallDefinition.positionMeters = {0.0, 0.0, 20.0};
  smallDefinition.halfExtentsMeters = {0.25, 0.25, 0.25};
  smallDefinition.massKilograms = 1.0;
  JoltBoxDefinition largeDefinition = smallDefinition;
  largeDefinition.positionMeters.y = 2.0;
  largeDefinition.halfExtentsMeters = {0.5, 0.5, 0.5};

  const auto smallBox = world.CreateDynamicBox(smallDefinition);
  const auto largeBox = world.CreateDynamicBox(largeDefinition);
  world.ApplyTorque(smallBox, {0.0, 0.0, 0.1});
  world.ApplyTorque(largeBox, {0.0, 0.0, 0.1});
  world.StepFixed();

  const double smallRotation =
      world.State(smallBox).angularVelocityRadiansPerSecond.z;
  const double largeRotation =
      world.State(largeBox).angularVelocityRadiansPerSecond.z;
  EXPECT_GT(std::abs(smallRotation), std::abs(largeRotation));
  EXPECT_NEAR(std::abs(smallRotation / largeRotation), 4.0, 0.15);
}

TEST(JoltPhysicsWorldTests, X500RotorsProduceHoverAndClimbForces) {
  JoltPhysicsWorld world;
  const QuadrotorParameters parameters = QuadrotorParameters::X500Gazebo();
  JoltBoxDefinition definition;
  definition.positionMeters = {0.0, 0.0, 20.0};
  definition.halfExtentsMeters = {0.2, 0.2, 0.05};
  const JoltRigidBodyHandle drone =
      world.CreateQuadrotor(definition, parameters);

  const double hoverSpeed =
      std::sqrt(parameters.massKilograms * 9.81 /
                (QuadrotorParameters::rotorCount *
                 parameters.thrustCoefficientNewtonsPerSpeedSquared));
  world.SetInitialMotorSpeeds(drone,
                              {hoverSpeed, hoverSpeed, hoverSpeed, hoverSpeed});
  for (int step = 1; step < 600; ++step)
    world.StepFixed();

  const JoltRigidBodyState hoveringState = world.State(drone);
  EXPECT_NEAR(hoveringState.positionMeters.z, 20.0, 0.15);
  EXPECT_NEAR(hoveringState.linearVelocityMetersPerSecond.z, 0.0, 0.1);

  definition.positionMeters.x = 2.0;
  const JoltRigidBodyHandle climbingDrone =
      world.CreateQuadrotor(definition, parameters);
  world.SetMotorTargets(climbingDrone, {900.0, 900.0, 900.0, 900.0});
  world.StepFixed();
  const double firstMotorSpeed =
      world.MotorSpeedsRadiansPerSecond(climbingDrone)[0];
  EXPECT_GT(firstMotorSpeed, 0.0);
  EXPECT_LT(firstMotorSpeed, 900.0);
  EXPECT_GT(world.MotorThrustsNewtons(climbingDrone)[0], 0.0);

  for (int step = 0; step < 120; ++step)
    world.StepFixed();
  EXPECT_GT(world.State(climbingDrone).positionMeters.z, 21.0);

  world.SetMotorTargets(climbingDrone, {900.0, 900.0, 700.0, 700.0});
  for (int step = 0; step < 60; ++step)
    world.StepFixed();
  EXPECT_GT(world.State(climbingDrone).angularVelocityRadiansPerSecond.z, 0.0);
}

TEST(JoltPhysicsWorldTests, RejectsInvalidOrInapplicableOperations) {
  JoltPhysicsWorld world;
  const JoltRigidBodyHandle ground =
      world.AddStaticBox({0.0, 0.0, -0.5}, {5.0, 5.0, 0.5});
  EXPECT_THROW(static_cast<void>(world.AddStaticBox({}, {1.0, -1.0, 1.0})),
               std::invalid_argument);
  EXPECT_THROW(world.ApplyForce(ground, {1.0, 0.0, 0.0}), std::logic_error);
  EXPECT_THROW(world.ApplyTorque(ground, {0.0, 0.0, 1.0}), std::logic_error);

  JoltBoxDefinition invalidDefinition;
  invalidDefinition.massKilograms = 0.0;
  EXPECT_THROW(static_cast<void>(world.CreateDynamicBox(invalidDefinition)),
               std::invalid_argument);
  invalidDefinition.massKilograms = std::numeric_limits<double>::denorm_min();
  EXPECT_THROW(static_cast<void>(world.CreateDynamicBox(invalidDefinition)),
               std::invalid_argument);
  invalidDefinition.massKilograms = 1.0;
  invalidDefinition.halfExtentsMeters.x =
      std::numeric_limits<double>::denorm_min();
  EXPECT_THROW(static_cast<void>(world.CreateDynamicBox(invalidDefinition)),
               std::invalid_argument);
}
} // namespace aerosimlab::simulation
