#include "simulation_core/JoltPhysicsWorld.h"
#include "simulation_core/QuadrotorModel.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace aerosimlab::simulation {
namespace {
using JoltObjectLayer = JPH::ObjectLayer;

constexpr JoltObjectLayer nonMovingLayer = 0;
constexpr JoltObjectLayer movingLayer = 1;
constexpr JPH::uint objectLayerCount = 2;
constexpr JPH::uint broadPhaseLayerCount = 2;
constexpr JPH::uint maximumBodyCount = 65536;
constexpr JPH::uint maximumBodyPairCount = 65536;
constexpr JPH::uint maximumContactCount = 10240;
constexpr double fixedStepSeconds = 1.0 / 60.0;
constexpr double gravityMetersPerSecondSquared = 9.81;

/** @brief Jolt의 전역 형식 등록을 프로세스에서 한 번만 관리한다. */
class JoltRuntime final {
public:
  JoltRuntime() {
    JPH::RegisterDefaultAllocator();
    factory_ = std::make_unique<JPH::Factory>();
    JPH::Factory::sInstance = factory_.get();
    JPH::RegisterTypes();
  }

  ~JoltRuntime() {
    JPH::UnregisterTypes();
    JPH::Factory::sInstance = nullptr;
  }

  JoltRuntime(const JoltRuntime &) = delete;
  JoltRuntime &operator=(const JoltRuntime &) = delete;

private:
  std::unique_ptr<JPH::Factory> factory_;
};

JoltRuntime &EnsureJoltRuntime() {
  static JoltRuntime runtime;
  return runtime;
}

bool IsFinite(const glm::dvec3 &value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool IsValidOrientation(const glm::dquat &value) {
  const double lengthSquared = glm::dot(value, value);
  return std::isfinite(value.w) && std::isfinite(value.x) &&
         std::isfinite(value.y) && std::isfinite(value.z) &&
         std::isfinite(lengthSquared) && lengthSquared > 0.0;
}

float ToJoltFloat(double value) {
  constexpr double maximum = std::numeric_limits<float>::max();
  if (!std::isfinite(value) || value < -maximum || value > maximum)
    throw std::invalid_argument(
        "물리값이 Jolt에서 표현할 수 있는 범위를 벗어났습니다.");
  return static_cast<float>(value);
}

float ToJoltPositiveFloat(double value) {
  const float converted = ToJoltFloat(value);
  if (!(converted > 0.0F))
    throw std::invalid_argument("양수 물리값이 Jolt 변환 뒤 0이 됩니다.");
  return converted;
}

JPH::Vec3 ToJoltVector(const glm::dvec3 &enu) {
  // ENU의 북쪽 축을 Jolt의 -Z로 옮기면 오른손 좌표계를 유지할 수 있다.
  return {ToJoltFloat(enu.x), ToJoltFloat(enu.z), ToJoltFloat(-enu.y)};
}

JPH::Vec3 ToJoltHalfExtents(const glm::dvec3 &enuHalfExtents) {
  return {ToJoltPositiveFloat(enuHalfExtents.x),
          ToJoltPositiveFloat(enuHalfExtents.z),
          ToJoltPositiveFloat(enuHalfExtents.y)};
}

glm::dvec3 ToProjectVector(JPH::Vec3Arg jolt) {
  return {jolt.GetX(), -jolt.GetZ(), jolt.GetY()};
}

const glm::dquat &ProjectToJoltRotation() {
  static const glm::dquat rotation =
      glm::angleAxis(glm::half_pi<double>(), glm::dvec3{1.0, 0.0, 0.0});
  return rotation;
}

JPH::Quat ToJoltRotation(const glm::dquat &enu) {
  const glm::dquat normalized = glm::normalize(enu);
  const glm::dquat converted = ProjectToJoltRotation() * normalized *
                               glm::conjugate(ProjectToJoltRotation());
  return {ToJoltFloat(converted.x), ToJoltFloat(converted.y),
          ToJoltFloat(converted.z), ToJoltFloat(converted.w)};
}

glm::dquat ToProjectRotation(JPH::QuatArg jolt) {
  const glm::dquat converted{jolt.GetW(), jolt.GetX(), jolt.GetY(),
                             jolt.GetZ()};
  return glm::normalize(glm::conjugate(ProjectToJoltRotation()) * converted *
                        ProjectToJoltRotation());
}

bool HasPositiveComponents(const glm::dvec3 &value) {
  return IsFinite(value) && value.x > 0.0 && value.y > 0.0 && value.z > 0.0;
}

class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface {
public:
  BroadPhaseLayers()
      : layers_{JPH::BroadPhaseLayer(0), JPH::BroadPhaseLayer(1)} {}

  JPH::uint GetNumBroadPhaseLayers() const override {
    return broadPhaseLayerCount;
  }

  JPH::BroadPhaseLayer
  GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
    return layers_.at(layer);
  }

private:
  std::array<JPH::BroadPhaseLayer, objectLayerCount> layers_;
};

class ObjectVsBroadPhaseFilter final
    : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
  bool ShouldCollide(JPH::ObjectLayer objectLayer,
                     JPH::BroadPhaseLayer broadPhaseLayer) const override {
    if (objectLayer == nonMovingLayer)
      return broadPhaseLayer == JPH::BroadPhaseLayer(1);
    return objectLayer == movingLayer;
  }
};

class ObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter {
public:
  bool ShouldCollide(JPH::ObjectLayer first,
                     JPH::ObjectLayer second) const override {
    return first == movingLayer || second == movingLayer;
  }
};

JPH::RefConst<JPH::Shape> MakeBoxShape(const glm::dvec3 &halfExtentsMeters) {
  return new JPH::BoxShape(ToJoltHalfExtents(halfExtentsMeters));
}
} // namespace

struct JoltPhysicsWorld::Impl {
  struct QuadrotorActuation {
    QuadrotorParameters parameters;
    std::array<double, QuadrotorParameters::rotorCount> targetSpeeds{};
    std::array<double, QuadrotorParameters::rotorCount> motorSpeeds{};
    bool hasStepped = false;
  };

  struct BodyRecord {
    JPH::BodyID id;
    bool isStatic = false;
    std::optional<QuadrotorActuation> quadrotor;
  };

  JoltRuntime &runtime;
  BroadPhaseLayers broadPhaseLayers;
  ObjectVsBroadPhaseFilter objectVsBroadPhaseFilter;
  ObjectLayerPairFilter objectLayerPairFilter;
  JPH::PhysicsSystem physicsSystem;
  JPH::TempAllocatorImpl tempAllocator{10U * 1024U * 1024U};
  JPH::JobSystemSingleThreaded jobSystem{1024};
  std::unordered_map<std::uint32_t, BodyRecord> bodies;
  std::uint32_t nextHandle = 1;

  Impl() : runtime(EnsureJoltRuntime()) {
    static_cast<void>(runtime);
    // 도시 지도의 정적 건물 수용을 위해 Jolt 권장 규모로 여유를 둔다.
    physicsSystem.Init(maximumBodyCount, 0, maximumBodyPairCount,
                       maximumContactCount, broadPhaseLayers,
                       objectVsBroadPhaseFilter, objectLayerPairFilter);
    physicsSystem.SetGravity(
        {0.0F, -static_cast<float>(gravityMetersPerSecondSquared), 0.0F});
  }

  ~Impl() {
    JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
    for (const auto &[handle, record] : bodies) {
      static_cast<void>(handle);
      bodyInterface.RemoveBody(record.id);
      bodyInterface.DestroyBody(record.id);
    }
  }

  JoltRigidBodyHandle
  AddBody(const JPH::BodyCreationSettings &settings,
          JPH::EActivation activation, bool isStatic,
          std::optional<QuadrotorActuation> quadrotor = std::nullopt) {
    if (nextHandle == 0)
      throw std::overflow_error("강체 식별값을 더 만들 수 없습니다.");

    JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
    const JPH::BodyID id = bodyInterface.CreateAndAddBody(settings, activation);
    if (id.IsInvalid())
      throw std::runtime_error("Jolt 물리 월드에 강체를 만들지 못했습니다.");

    const std::uint32_t handle = nextHandle++;
    try {
      bodies.emplace(handle, BodyRecord{id, isStatic, std::move(quadrotor)});
    } catch (...) {
      bodyInterface.RemoveBody(id);
      bodyInterface.DestroyBody(id);
      throw;
    }
    return {handle};
  }

  BodyRecord &GetBody(JoltRigidBodyHandle handle) {
    const auto found = bodies.find(handle.value);
    if (found == bodies.end())
      throw std::out_of_range("물리 월드에서 강체를 찾을 수 없습니다.");
    return found->second;
  }

  const BodyRecord &GetBody(JoltRigidBodyHandle handle) const {
    const auto found = bodies.find(handle.value);
    if (found == bodies.end())
      throw std::out_of_range("물리 월드에서 강체를 찾을 수 없습니다.");
    return found->second;
  }
};

JoltPhysicsWorld::JoltPhysicsWorld() : impl_(std::make_unique<Impl>()) {}

JoltPhysicsWorld::~JoltPhysicsWorld() = default;

JoltRigidBodyHandle
JoltPhysicsWorld::AddStaticBox(const glm::dvec3 &centerMeters,
                               const glm::dvec3 &halfExtentsMeters) {
  if (!IsFinite(centerMeters) || !HasPositiveComponents(halfExtentsMeters))
    throw std::invalid_argument("고정 상자의 위치와 크기를 확인해 주세요.");

  const auto shape = MakeBoxShape(halfExtentsMeters);
  const JPH::BodyCreationSettings settings(
      shape.GetPtr(), ToJoltVector(centerMeters), JPH::Quat::sIdentity(),
      JPH::EMotionType::Static, nonMovingLayer);
  return impl_->AddBody(settings, JPH::EActivation::DontActivate, true);
}

JoltRigidBodyHandle
JoltPhysicsWorld::CreateDynamicBox(const JoltBoxDefinition &definition) {
  if (!IsFinite(definition.positionMeters) ||
      !IsValidOrientation(definition.orientation) ||
      !HasPositiveComponents(definition.halfExtentsMeters) ||
      !std::isfinite(definition.massKilograms) ||
      definition.massKilograms <= 0.0 || !std::isfinite(definition.friction) ||
      definition.friction < 0.0 || !std::isfinite(definition.restitution) ||
      definition.restitution < 0.0 || definition.restitution > 1.0) {
    throw std::invalid_argument(
        "움직이는 상자의 모양과 물리값을 확인해 주세요.");
  }

  const auto shape = MakeBoxShape(definition.halfExtentsMeters);
  JPH::BodyCreationSettings settings(shape.GetPtr(),
                                     ToJoltVector(definition.positionMeters),
                                     ToJoltRotation(definition.orientation),
                                     JPH::EMotionType::Dynamic, movingLayer);
  settings.mOverrideMassProperties =
      JPH::EOverrideMassProperties::CalculateInertia;
  settings.mMassPropertiesOverride.mMass =
      ToJoltPositiveFloat(definition.massKilograms);
  settings.mFriction = ToJoltFloat(definition.friction);
  settings.mRestitution = ToJoltFloat(definition.restitution);
  return impl_->AddBody(settings, JPH::EActivation::Activate, false);
}

JoltRigidBodyHandle
JoltPhysicsWorld::CreateQuadrotor(const JoltBoxDefinition &body,
                                  const QuadrotorParameters &parameters) {
  if (!IsFinite(body.positionMeters) || !IsValidOrientation(body.orientation) ||
      !HasPositiveComponents(body.halfExtentsMeters) ||
      !std::isfinite(body.friction) || body.friction < 0.0 ||
      !std::isfinite(body.restitution) || body.restitution < 0.0 ||
      body.restitution > 1.0 || !std::isfinite(parameters.massKilograms) ||
      parameters.massKilograms <= 0.0 ||
      !HasPositiveComponents(
          parameters.principalInertiaKilogramMetersSquared) ||
      !std::isfinite(parameters.thrustCoefficientNewtonsPerSpeedSquared) ||
      parameters.thrustCoefficientNewtonsPerSpeedSquared <= 0.0 ||
      !std::isfinite(parameters.momentToThrustRatio) ||
      parameters.momentToThrustRatio < 0.0 ||
      !std::isfinite(parameters.motorTimeConstantUpSeconds) ||
      parameters.motorTimeConstantUpSeconds <= 0.0 ||
      !std::isfinite(parameters.motorTimeConstantDownSeconds) ||
      parameters.motorTimeConstantDownSeconds <= 0.0 ||
      !std::isfinite(parameters.maximumMotorSpeedRadiansPerSecond) ||
      parameters.maximumMotorSpeedRadiansPerSecond <= 0.0) {
    throw std::invalid_argument(
        "드론의 물리 제원과 시작 상태를 확인해 주세요.");
  }
  if (!std::all_of(parameters.rotors.begin(), parameters.rotors.end(),
                   [](const RotorDefinition &rotor) {
                     return IsFinite(rotor.positionMeters);
                   })) {
    throw std::invalid_argument("로터 위치를 확인해 주세요.");
  }

  const auto shape = MakeBoxShape(body.halfExtentsMeters);
  JPH::BodyCreationSettings settings(
      shape.GetPtr(), ToJoltVector(body.positionMeters),
      ToJoltRotation(body.orientation), JPH::EMotionType::Dynamic, movingLayer);
  settings.mOverrideMassProperties =
      JPH::EOverrideMassProperties::MassAndInertiaProvided;
  settings.mMassPropertiesOverride.mMass =
      ToJoltPositiveFloat(parameters.massKilograms);
  // Jolt 축은 (동, 위, 남쪽 반대)이므로 ENU의 북·위 관성 순서를 바꾼다.
  settings.mMassPropertiesOverride.mInertia = JPH::Mat44::sScale(JPH::Vec3(
      ToJoltPositiveFloat(parameters.principalInertiaKilogramMetersSquared.x),
      ToJoltPositiveFloat(parameters.principalInertiaKilogramMetersSquared.z),
      ToJoltPositiveFloat(parameters.principalInertiaKilogramMetersSquared.y)));
  settings.mFriction = ToJoltFloat(body.friction);
  settings.mRestitution = ToJoltFloat(body.restitution);

  Impl::QuadrotorActuation actuation;
  actuation.parameters = parameters;
  return impl_->AddBody(settings, JPH::EActivation::Activate, false,
                        std::move(actuation));
}

void JoltPhysicsWorld::SetMotorTargets(
    JoltRigidBodyHandle handle,
    const std::array<double, 4> &speedRadiansPerSecond) {
  auto &body = impl_->GetBody(handle);
  if (!body.quadrotor)
    throw std::logic_error("로터 입력을 받는 드론 강체가 아닙니다.");
  const double maximumSpeed =
      body.quadrotor->parameters.maximumMotorSpeedRadiansPerSecond;
  for (std::size_t rotor = 0; rotor < speedRadiansPerSecond.size(); ++rotor) {
    if (!std::isfinite(speedRadiansPerSecond[rotor]))
      throw std::invalid_argument("로터 목표 속도는 유한한 값이어야 합니다.");
    body.quadrotor->targetSpeeds[rotor] =
        std::clamp(speedRadiansPerSecond[rotor], 0.0, maximumSpeed);
  }
}

void JoltPhysicsWorld::SetInitialMotorSpeeds(
    JoltRigidBodyHandle handle,
    const std::array<double, 4> &speedRadiansPerSecond) {
  auto &body = impl_->GetBody(handle);
  if (!body.quadrotor)
    throw std::logic_error("로터 입력을 받는 드론 강체가 아닙니다.");
  if (body.quadrotor->hasStepped)
    throw std::logic_error("초기 로터 속도는 첫 물리 단계 전에만 설정할 수 있습니다.");

  const double maximumSpeed =
      body.quadrotor->parameters.maximumMotorSpeedRadiansPerSecond;
  for (double speed : speedRadiansPerSecond) {
    if (!std::isfinite(speed) || speed < 0.0 || speed > maximumSpeed)
      throw std::invalid_argument(
          "초기 로터 속도를 허용 범위 안에서 지정해 주세요.");
  }
  body.quadrotor->motorSpeeds = speedRadiansPerSecond;
  body.quadrotor->targetSpeeds = speedRadiansPerSecond;
}

std::array<double, 4> JoltPhysicsWorld::MotorSpeedsRadiansPerSecond(
    JoltRigidBodyHandle handle) const {
  const auto &body = impl_->GetBody(handle);
  if (!body.quadrotor)
    throw std::logic_error("로터 입력을 받는 드론 강체가 아닙니다.");
  return body.quadrotor->motorSpeeds;
}

std::array<double, 4>
JoltPhysicsWorld::MotorThrustsNewtons(JoltRigidBodyHandle handle) const {
  const auto &body = impl_->GetBody(handle);
  if (!body.quadrotor)
    throw std::logic_error("로터 입력을 받는 드론 강체가 아닙니다.");
  std::array<double, QuadrotorParameters::rotorCount> thrusts{};
  const double coefficient =
      body.quadrotor->parameters.thrustCoefficientNewtonsPerSpeedSquared;
  for (std::size_t rotor = 0; rotor < thrusts.size(); ++rotor)
    thrusts[rotor] = coefficient * body.quadrotor->motorSpeeds[rotor] *
                     body.quadrotor->motorSpeeds[rotor];
  return thrusts;
}

void JoltPhysicsWorld::ApplyForce(JoltRigidBodyHandle handle,
                                  const glm::dvec3 &forceNewtons) {
  if (!IsFinite(forceNewtons))
    throw std::invalid_argument("강체에 가할 힘은 유한한 값이어야 합니다.");
  const auto &body = impl_->GetBody(handle);
  if (body.isStatic)
    throw std::logic_error("고정된 강체에는 힘을 가할 수 없습니다.");
  impl_->physicsSystem.GetBodyInterface().AddForce(body.id,
                                                   ToJoltVector(forceNewtons));
}

void JoltPhysicsWorld::ApplyTorque(JoltRigidBodyHandle handle,
                                   const glm::dvec3 &torqueNewtonMeters) {
  if (!IsFinite(torqueNewtonMeters))
    throw std::invalid_argument("강체에 가할 토크는 유한한 값이어야 합니다.");
  const auto &body = impl_->GetBody(handle);
  if (body.isStatic)
    throw std::logic_error("고정된 강체에는 토크를 가할 수 없습니다.");
  impl_->physicsSystem.GetBodyInterface().AddTorque(
      body.id, ToJoltVector(torqueNewtonMeters));
}

void JoltPhysicsWorld::SetState(JoltRigidBodyHandle handle,
                                const JoltRigidBodyState &state) {
  if (!IsFinite(state.positionMeters) ||
      !IsValidOrientation(state.orientation) ||
      !IsFinite(state.linearVelocityMetersPerSecond) ||
      !IsFinite(state.angularVelocityRadiansPerSecond)) {
    throw std::invalid_argument(
        "강체 상태는 유한한 값과 올바른 자세여야 합니다.");
  }
  const auto &body = impl_->GetBody(handle);
  if (body.isStatic)
    throw std::logic_error("고정된 강체의 상태는 바꿀 수 없습니다.");

  JPH::BodyInterface &bodyInterface = impl_->physicsSystem.GetBodyInterface();
  bodyInterface.SetPositionAndRotation(
      body.id, ToJoltVector(state.positionMeters),
      ToJoltRotation(state.orientation), JPH::EActivation::Activate);
  bodyInterface.SetLinearVelocity(
      body.id, ToJoltVector(state.linearVelocityMetersPerSecond));
  bodyInterface.SetAngularVelocity(
      body.id, ToJoltVector(state.angularVelocityRadiansPerSecond));
}

void JoltPhysicsWorld::StepFixed() {
  JPH::BodyInterface &bodyInterface = impl_->physicsSystem.GetBodyInterface();
  for (auto &[handle, body] : impl_->bodies) {
    static_cast<void>(handle);
    if (!body.quadrotor)
      continue;

    auto &actuation = *body.quadrotor;
    const QuadrotorParameters &parameters = actuation.parameters;
    JPH::RVec3 bodyPosition;
    JPH::Quat bodyRotation;
    bodyInterface.GetPositionAndRotation(body.id, bodyPosition, bodyRotation);
    const glm::dvec3 position = ToProjectVector(bodyPosition);
    const glm::dquat orientation = ToProjectRotation(bodyRotation);
    const double seconds = fixedStepSeconds;
    double reactionTorqueBody = 0.0;

    for (std::size_t rotor = 0; rotor < actuation.motorSpeeds.size(); ++rotor) {
      const double target = actuation.targetSpeeds[rotor];
      const double timeConstant = target >= actuation.motorSpeeds[rotor]
                                      ? parameters.motorTimeConstantUpSeconds
                                      : parameters.motorTimeConstantDownSeconds;
      const double blend = 1.0 - std::exp(-seconds / timeConstant);
      double &speed = actuation.motorSpeeds[rotor];
      speed += (target - speed) * blend;

      const double thrust =
          parameters.thrustCoefficientNewtonsPerSpeedSquared * speed * speed;
      const glm::dvec3 force = orientation * glm::dvec3{0.0, 0.0, thrust};
      const glm::dvec3 rotorPosition =
          position + orientation * parameters.rotors[rotor].positionMeters;
      bodyInterface.AddForce(body.id, ToJoltVector(force),
                             ToJoltVector(rotorPosition));
      const double spinSign =
          parameters.rotors[rotor].rotation == RotorRotation::CounterClockwise
              ? 1.0
              : -1.0;
      reactionTorqueBody += spinSign * parameters.momentToThrustRatio * thrust;
    }

    const glm::dvec3 reactionTorque =
        orientation * glm::dvec3{0.0, 0.0, reactionTorqueBody};
    bodyInterface.AddTorque(body.id, ToJoltVector(reactionTorque));
    actuation.hasStepped = true;
  }
  impl_->physicsSystem.Update(static_cast<float>(fixedStepSeconds), 1,
                              &impl_->tempAllocator, &impl_->jobSystem);
}

JoltRigidBodyState JoltPhysicsWorld::State(JoltRigidBodyHandle handle) const {
  const auto &body = impl_->GetBody(handle);
  JPH::RVec3 position;
  JPH::Quat orientation;
  impl_->physicsSystem.GetBodyInterface().GetPositionAndRotation(
      body.id, position, orientation);
  JPH::Vec3 linearVelocity;
  JPH::Vec3 angularVelocity;
  impl_->physicsSystem.GetBodyInterface().GetLinearAndAngularVelocity(
      body.id, linearVelocity, angularVelocity);
  return {ToProjectVector(position), ToProjectRotation(orientation),
          ToProjectVector(linearVelocity), ToProjectVector(angularVelocity)};
}

bool JoltPhysicsWorld::DestroyBody(JoltRigidBodyHandle handle) {
  const auto found = impl_->bodies.find(handle.value);
  if (found == impl_->bodies.end())
    return false;

  JPH::BodyInterface &bodyInterface = impl_->physicsSystem.GetBodyInterface();
  bodyInterface.RemoveBody(found->second.id);
  bodyInterface.DestroyBody(found->second.id);
  impl_->bodies.erase(found);
  return true;
}
} // namespace aerosimlab::simulation
