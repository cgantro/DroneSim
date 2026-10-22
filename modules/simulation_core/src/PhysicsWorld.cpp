#include "simulation_core/PhysicsWorld.h"

#include "simulation_core/DroneModel.h"
#include "simulation_core/QuadrotorModel.h"

#include <mutex>
#include <common/ClockFactory.hpp>
#include <common/SteppableClock.hpp>
#include <physics/Environment.hpp>
#include <physics/FastPhysicsEngine.hpp>
#include <physics/Kinematics.hpp>
#include <sensors/SensorFactory.hpp>
#include <vehicles/multirotor/RotorActuator.hpp>
#include <vehicles/multirotor/MultiRotorParams.hpp>
#include <vehicles/multirotor/MultiRotorPhysicsBody.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace aerosimlab::simulation
{
namespace
{
using AirSimBody = msr::airlib::MultiRotorPhysicsBody;
using AirSimVector = msr::airlib::Vector3r;

const glm::dmat3 axisReflection{
    {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}};

bool IsFinite(const glm::dvec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsValidOrientation(const glm::dquat& value)
{
    const double lengthSquared = glm::dot(value, value);
    return std::isfinite(value.w) && std::isfinite(value.x)
        && std::isfinite(value.y) && std::isfinite(value.z)
        && std::isfinite(lengthSquared) && lengthSquared > 0.0;
}

AirSimVector ToAirSimPosition(const glm::dvec3& value)
{
    return {static_cast<float>(value.x), static_cast<float>(value.y),
        static_cast<float>(-value.z)};
}

AirSimVector ToAirSimAngularVector(const glm::dvec3& value)
{
    return {static_cast<float>(-value.x), static_cast<float>(-value.y),
        static_cast<float>(value.z)};
}

glm::dvec3 ToProjectPosition(const AirSimVector& value)
{
    return {value.x(), value.y(), -value.z()};
}

glm::dvec3 ToProjectAngularVector(const AirSimVector& value)
{
    return {-value.x(), -value.y(), value.z()};
}

msr::airlib::Quaternionr ToAirSimOrientation(const glm::dquat& value)
{
    const glm::dmat3 rotation = glm::mat3_cast(glm::normalize(value));
    const glm::dquat converted = glm::quat_cast(
        axisReflection * rotation * axisReflection);
    return {static_cast<float>(converted.w), static_cast<float>(converted.x),
        static_cast<float>(converted.y), static_cast<float>(converted.z)};
}

glm::dquat ToProjectOrientation(const msr::airlib::Quaternionr& value)
{
    const glm::dmat3 rotation = glm::mat3_cast(glm::dquat{
        value.w(), value.x(), value.y(), value.z()});
    return glm::normalize(glm::quat_cast(
        axisReflection * rotation * axisReflection));
}

msr::airlib::Kinematics::State ToAirSimState(const RigidBodyState& value)
{
    auto result = msr::airlib::Kinematics::State::zero();
    result.pose.position = ToAirSimPosition(value.positionMeters);
    result.pose.orientation = ToAirSimOrientation(value.orientation);
    result.twist.linear = ToAirSimPosition(value.linearVelocityMetersPerSecond);
    result.twist.angular = ToAirSimAngularVector(value.angularVelocityRadiansPerSecond);
    return result;
}

RigidBodyState ToProjectState(const msr::airlib::Kinematics::State& value)
{
    return {
        ToProjectPosition(value.pose.position),
        ToProjectOrientation(value.pose.orientation),
        ToProjectPosition(value.twist.linear),
        ToProjectAngularVector(value.twist.angular)};
}

std::shared_ptr<msr::airlib::SteppableClock> CreateSimulationClock()
{
    return std::make_shared<msr::airlib::SteppableClock>(
        DroneModel::fixedStepSeconds);
}

/** @brief 시뮬레이터의 목표 모터 속도를 AirSim이 받는 0~1 신호로 바꾼다. */
class ActuationApi final : public msr::airlib::VehicleApiBase
{
public:
    void SetMotorTargets(
        const std::array<double, QuadrotorParameters::rotorCount>& speeds,
        double maximumSpeed)
    {
        for (std::size_t index = 0; index < signals_.size(); ++index)
        {
            const double ratio = std::clamp(speeds[index] / maximumSpeed, 0.0, 1.0);
            signals_[index] = static_cast<float>(ratio * ratio);
        }
    }

    void enableApiControl(bool enabled) override { apiControlEnabled_ = enabled; }
    bool isApiControlEnabled() const override { return apiControlEnabled_; }
    bool armDisarm(bool armed) override { armed_ = armed; return true; }
    msr::airlib::GeoPoint getHomeGeoPoint() const override { return home_; }
    std::size_t getActuatorCount() const override { return signals_.size(); }
    msr::airlib::real_T getActuation(unsigned int index) const override
    {
        return signals_.at(index);
    }

protected:
    void resetImplementation() override
    {
        signals_.fill(0.0F);
        apiControlEnabled_ = false;
        armed_ = false;
    }

private:
    std::array<float, QuadrotorParameters::rotorCount> signals_{};
    msr::airlib::GeoPoint home_{0.0, 0.0, 0.0F};
    bool apiControlEnabled_ = false;
    bool armed_ = false;
};

/** @brief 현재 기체 설정을 AirSim 다중 로터 모델의 입력 형식으로 맞춘다. */
class QuadrotorParametersAdapter final : public msr::airlib::MultiRotorParams
{
public:
    explicit QuadrotorParametersAdapter(const QuadrotorParameters& source)
        : source_(source)
    {
    }

    void Configure()
    {
        setupParams();
    }

    std::unique_ptr<msr::airlib::MultirotorApiBase> createMultirotorApi() override
    {
        return {};
    }

protected:
    void setupParams() override
    {
        Params& target = getParams();
        target.rotor_count = static_cast<msr::airlib::uint>(source_.rotors.size());
        target.mass = static_cast<float>(source_.massKilograms);
        target.inertia = msr::airlib::Matrix3x3r::Zero();
        target.inertia.diagonal() << source_.principalInertiaKilogramMetersSquared.x,
            source_.principalInertiaKilogramMetersSquared.y,
            source_.principalInertiaKilogramMetersSquared.z;
        target.body_box = AirSimVector{static_cast<float>(source_.bodySizeMeters.x),
            static_cast<float>(source_.bodySizeMeters.y),
            static_cast<float>(source_.bodySizeMeters.z)};
        target.rotor_params.max_rpm = static_cast<float>(
            source_.maximumMotorSpeedRadiansPerSecond * 60.0 / (2.0 * M_PI));
        target.rotor_params.calculateMaxThrust();
        target.rotor_params.max_speed_square = static_cast<float>(
            source_.maximumMotorSpeedRadiansPerSecond
                * source_.maximumMotorSpeedRadiansPerSecond);
        target.rotor_params.max_thrust = static_cast<float>(
            source_.thrustCoefficientNewtonsPerSpeedSquared
                * target.rotor_params.max_speed_square);
        target.rotor_params.max_torque = static_cast<float>(
            source_.momentToThrustRatio * target.rotor_params.max_thrust);
        target.rotor_params.control_signal_filter_tc = static_cast<float>(
            source_.motorTimeConstantUpSeconds);
        for (const RotorDefinition& rotor : source_.rotors)
        {
            const auto direction = rotor.rotation == RotorRotation::CounterClockwise
                ? msr::airlib::RotorTurningDirection::RotorTurningDirectionCW
                : msr::airlib::RotorTurningDirection::RotorTurningDirectionCCW;
            target.rotor_poses.emplace_back(
                ToAirSimPosition(rotor.positionMeters),
                AirSimVector{0.0F, 0.0F, -1.0F}, direction);
        }
    }

    const msr::airlib::SensorFactory* getSensorFactory() const override
    {
        return &sensorFactory_;
    }

private:
    QuadrotorParameters source_;
    msr::airlib::SensorFactory sensorFactory_;
};

struct Obstacle
{
    glm::dvec3 centerMeters;
    glm::dvec3 halfExtentsMeters;
};

struct OrientedBox
{
    glm::dvec3 centerMeters;
    glm::dvec3 halfExtentsMeters;
    std::array<glm::dvec3, 3> axes;
};

struct BoxContact
{
    glm::dvec3 normal;
    glm::dvec3 impactPointMeters;
    double penetrationMeters = 0.0;
};

double CollisionRadius(std::span<const RigidBodyBox> boxes)
{
    double radius = 0.0;
    for (const RigidBodyBox& box : boxes)
    {
        radius = std::max(radius, glm::length(box.positionMeters)
            + glm::length(glm::dvec3{box.halfExtentsMeters}));
    }
    return radius;
}

OrientedBox ToWorldBox(
    const glm::dvec3& bodyPosition,
    const glm::dquat& bodyOrientation,
    const RigidBodyBox& box)
{
    const glm::dquat orientation = glm::normalize(bodyOrientation * box.orientation);
    return {
        bodyPosition + bodyOrientation * box.positionMeters,
        glm::dvec3{box.halfExtentsMeters},
        {orientation * glm::dvec3{1.0, 0.0, 0.0},
            orientation * glm::dvec3{0.0, 1.0, 0.0},
            orientation * glm::dvec3{0.0, 0.0, 1.0}}};
}

/**
 * @brief 두 회전 상자의 겹침, 바깥쪽 방향, 기체 쪽 접촉점을 찾는다.
 * @details 분리축 정리는 상자 면 방향과 두 상자의 모서리 방향을 확인한다. 첫 번째 상자는 움직이는 기체여서 법선은 두 번째 상자에서 기체 바깥을 향한다.
 */
std::optional<BoxContact> FindBoxContact(
    const OrientedBox& droneBox,
    const OrientedBox& otherBox)
{
    std::array<glm::dvec3, 15> candidateAxes{};
    std::size_t axisCount = 0;
    for (const glm::dvec3& axis : droneBox.axes)
        candidateAxes[axisCount++] = axis;
    for (const glm::dvec3& axis : otherBox.axes)
        candidateAxes[axisCount++] = axis;
    for (const glm::dvec3& droneAxis : droneBox.axes)
    {
        for (const glm::dvec3& otherAxis : otherBox.axes)
            candidateAxes[axisCount++] = glm::cross(droneAxis, otherAxis);
    }

    const glm::dvec3 centerOffset = otherBox.centerMeters - droneBox.centerMeters;
    double smallestPenetration = std::numeric_limits<double>::infinity();
    glm::dvec3 collisionNormal{0.0};
    for (std::size_t axisIndex = 0; axisIndex < axisCount; ++axisIndex)
    {
        const glm::dvec3& candidate = candidateAxes[axisIndex];
        const double axisLengthSquared = glm::dot(candidate, candidate);
        if (axisLengthSquared < 1.0e-12)
            continue;

        const glm::dvec3 axis = candidate / std::sqrt(axisLengthSquared);
        const auto projectedRadius = [&axis](const OrientedBox& box)
        {
            return std::abs(glm::dot(axis, box.axes[0])) * box.halfExtentsMeters.x
                + std::abs(glm::dot(axis, box.axes[1])) * box.halfExtentsMeters.y
                + std::abs(glm::dot(axis, box.axes[2])) * box.halfExtentsMeters.z;
        };
        const double centerDistance = glm::dot(centerOffset, axis);
        const double penetration = projectedRadius(droneBox)
            + projectedRadius(otherBox) - std::abs(centerDistance);
        if (penetration < 0.0)
            return std::nullopt;

        if (penetration < smallestPenetration)
        {
            smallestPenetration = penetration;
            collisionNormal = axis * (centerDistance >= 0.0 ? -1.0 : 1.0);
        }
    }

    if (!std::isfinite(smallestPenetration))
        return std::nullopt;

    const glm::dvec3 towardOther = -collisionNormal;
    glm::dvec3 impactPoint = droneBox.centerMeters;
    for (std::size_t axis = 0; axis < droneBox.axes.size(); ++axis)
    {
        const double side = glm::dot(towardOther, droneBox.axes[axis]) >= 0.0
            ? 1.0 : -1.0;
        impactPoint += droneBox.axes[axis]
            * (side * droneBox.halfExtentsMeters[axis]);
    }
    return BoxContact{collisionNormal, impactPoint, smallestPenetration};
}

}

struct PhysicsWorld::Impl
{
    struct Body
    {
        QuadrotorParametersAdapter parameters;
        ActuationApi actuation;
        msr::airlib::Kinematics kinematics;
        msr::airlib::Environment environment;
        std::unique_ptr<AirSimBody> model;
        std::vector<RigidBodyBox> collisionBoxes;
        double collisionRadiusMeters = 0.0;
        double maximumMotorSpeed = 0.0;
        bool updatedSinceReset = false;

        Body(const RigidBodyDefinition& definition, const QuadrotorParameters& quadrotor)
            : parameters(quadrotor),
              kinematics(ToAirSimState({definition.positionMeters,
                  definition.orientation, {}, {}})),
              collisionBoxes(quadrotor.collisionBoxes.begin(), quadrotor.collisionBoxes.end()),
              collisionRadiusMeters(CollisionRadius(collisionBoxes)),
              maximumMotorSpeed(quadrotor.maximumMotorSpeedRadiansPerSecond)
        {
            parameters.Configure();
            msr::airlib::Environment::State environmentState;
            environmentState.position = ToAirSimPosition(definition.positionMeters);
            environmentState.geo_point.set(0.0, 0.0, 0.0F);
            environment.initialize(environmentState);
            kinematics.reset();
            actuation.reset();
            model = std::make_unique<AirSimBody>(
                &parameters, &actuation, &kinematics, &environment);
            model->reset();
            model->setPose({ToAirSimPosition(definition.positionMeters),
                ToAirSimOrientation(definition.orientation)});
        }

        void ApplyMotorTargets()
        {
            // AirSim normally copies controller output after integrating the body.
            // This simulator already has the command at the start of the fixed step.
            for (msr::airlib::uint index = 0; index < parameters.getParams().rotor_count; ++index)
            {
                auto& rotor = static_cast<msr::airlib::RotorActuator&>(
                    model->getWrenchVertex(index));
                rotor.setControlSignal(actuation.getActuation(index));
            }
        }
    };

    std::shared_ptr<msr::airlib::SteppableClock> clock = CreateSimulationClock();
    msr::airlib::FastPhysicsEngine physicsEngine{true};
    std::vector<std::unique_ptr<Body>> bodies;
    std::vector<std::pair<double, std::size_t>> bodyPositionsByX;
    std::vector<Obstacle> obstacles;
    std::optional<terrain::TerrainGrid> terrain;
    std::uint64_t stepCount = 0;

    Impl()
    {
        static_cast<void>(msr::airlib::ClockFactory::get(clock));
        physicsEngine.reset();
    }

    Body& GetBody(RigidBodyHandle handle)
    {
        if (handle.value >= bodies.size() || !bodies[handle.value])
            throw std::out_of_range("존재하지 않는 강체 번호입니다.");
        return *bodies[handle.value];
    }

    const Body& GetBody(RigidBodyHandle handle) const
    {
        if (handle.value >= bodies.size() || !bodies[handle.value])
            throw std::out_of_range("존재하지 않는 강체 번호입니다.");
        return *bodies[handle.value];
    }

    double OrderBodiesAlongXAxis()
    {
        bodyPositionsByX.clear();
        bodyPositionsByX.reserve(bodies.size());
        double largestCollisionRadius = 0.0;
        for (std::size_t index = 0; index < bodies.size(); ++index)
        {
            if (!bodies[index])
                continue;

            bodyPositionsByX.emplace_back(
                bodies[index]->kinematics.getPose().position.x(), index);
            largestCollisionRadius = std::max(
                largestCollisionRadius, bodies[index]->collisionRadiusMeters);
        }
        std::sort(bodyPositionsByX.begin(), bodyPositionsByX.end());
        return largestCollisionRadius;
    }

    double GroundHeight(const glm::dvec2& position) const
    {
        if (!terrain)
            return 0.0;
        try
        {
            return terrain->HeightAtLocalMeters(position);
        }
        catch (const std::out_of_range&)
        {
            return 0.0;
        }
    }

    /** @brief 높이 격자의 주변 변화량으로 그 지점의 바깥쪽 방향을 구한다. */
    glm::dvec3 GroundNormal(const glm::dvec2& position) const
    {
        if (!terrain)
            return {0.0, 0.0, 1.0};

        const double halfWidth = static_cast<double>(terrain->sampleCount - 1U)
            * terrain->spacingXMeters * 0.5;
        const double halfDepth = static_cast<double>(terrain->sampleCount - 1U)
            * terrain->spacingYMeters * 0.5;
        const double west = std::max(position.x - terrain->spacingXMeters, -halfWidth);
        const double east = std::min(position.x + terrain->spacingXMeters, halfWidth);
        const double south = std::max(position.y - terrain->spacingYMeters, -halfDepth);
        const double north = std::min(position.y + terrain->spacingYMeters, halfDepth);
        const double slopeEast = east > west
            ? (GroundHeight({east, position.y}) - GroundHeight({west, position.y}))
                / (east - west)
            : 0.0;
        const double slopeNorth = north > south
            ? (GroundHeight({position.x, north}) - GroundHeight({position.x, south}))
                / (north - south)
            : 0.0;
        return glm::normalize(glm::dvec3{-slopeEast, -slopeNorth, 1.0});
    }

    /**
     * @brief 기체 충돌 상자의 꼭짓점에서 지면과 겹치는 정도를 찾는다.
     * @details 가장 깊은 한 곳을 AirSim에 넘겨 위치, 방향, 겹친 깊이로 쓰게 한다.
     */
    std::optional<msr::airlib::CollisionInfo> FindTerrainContact(
        const Body& body, const RigidBodyState& state, std::uint64_t timestamp) const
    {
        std::optional<msr::airlib::CollisionInfo> contactInfo;
        double greatestPenetration = 0.0;
        for (const RigidBodyBox& box : body.collisionBoxes)
        {
            const OrientedBox worldBox = ToWorldBox(
                state.positionMeters, state.orientation, box);
            for (unsigned int cornerIndex = 0; cornerIndex < 8U; ++cornerIndex)
            {
                glm::dvec3 corner = worldBox.centerMeters;
                for (std::size_t axisIndex = 0; axisIndex < worldBox.axes.size(); ++axisIndex)
                {
                    const double direction = (cornerIndex & (1U << axisIndex)) != 0U
                        ? 1.0 : -1.0;
                    corner += worldBox.axes[axisIndex]
                        * (direction * worldBox.halfExtentsMeters[axisIndex]);
                }

                const double ground = GroundHeight({corner.x, corner.y});
                const double penetration = ground - corner.z;
                if (penetration <= greatestPenetration)
                    continue;

                greatestPenetration = penetration;
                contactInfo = msr::airlib::CollisionInfo{
                    true,
                    ToAirSimPosition(GroundNormal({corner.x, corner.y})),
                    ToAirSimPosition({corner.x, corner.y, ground}),
                    ToAirSimPosition(state.positionMeters),
                    static_cast<float>(penetration),
                    timestamp,
                    "ground",
                    0};
            }
        }
        return contactInfo;
    }

    // 단독 실행에서는 Unreal이 접촉 정보를 만들지 않으므로, 장면과의 접촉만 찾아 AirSim에 넘긴다.
    msr::airlib::CollisionInfo BuildAirSimContactInfo(
        const Body& body,
        std::size_t bodyIndex,
        double largestCollisionRadius) const
    {
        const RigidBodyState state = ToProjectState(body.kinematics.getState());
        const glm::dvec3 center = state.positionMeters;
        const std::uint64_t timestamp = stepCount + 1;

        if (auto groundContact = FindTerrainContact(body, state, timestamp))
            return *groundContact;

        // 경계 반경은 멀리 떨어진 후보만 빠르게 거른다. 실제 겹침은 아래 상자 검사로 결정한다.
        const double bodyRadius = body.collisionRadiusMeters;
        std::optional<msr::airlib::CollisionInfo> deepestContact;
        double greatestPenetration = -1.0;
        for (std::size_t index = 0; index < obstacles.size(); ++index)
        {
            const Obstacle& obstacle = obstacles[index];
            const glm::dvec3 nearest = glm::clamp(center,
                obstacle.centerMeters - obstacle.halfExtentsMeters,
                obstacle.centerMeters + obstacle.halfExtentsMeters);
            const glm::dvec3 distanceToObstacle = center - nearest;
            if (glm::dot(distanceToObstacle, distanceToObstacle)
                >= bodyRadius * bodyRadius)
                continue;

            const OrientedBox obstacleBox{
                obstacle.centerMeters,
                obstacle.halfExtentsMeters,
                {glm::dvec3{1.0, 0.0, 0.0}, glm::dvec3{0.0, 1.0, 0.0},
                    glm::dvec3{0.0, 0.0, 1.0}}};
            for (const RigidBodyBox& box : body.collisionBoxes)
            {
                const auto contact = FindBoxContact(
                    ToWorldBox(center, state.orientation, box), obstacleBox);
                if (!contact || contact->penetrationMeters <= greatestPenetration)
                    continue;

                greatestPenetration = contact->penetrationMeters;
                deepestContact = msr::airlib::CollisionInfo{
                    true,
                    ToAirSimPosition(contact->normal),
                    ToAirSimPosition(contact->impactPointMeters),
                    ToAirSimPosition(center),
                    static_cast<float>(contact->penetrationMeters),
                    timestamp,
                    "obstacle",
                    static_cast<int>(index + 1)};
            }
        }
        // X축 범위가 겹치는 기체만 비교한다. 충돌 모양을 감싸는 반경으로 후보를 거른 뒤 기존 상자 검사를 한다.
        const double searchDistance = bodyRadius + largestCollisionRadius;
        const auto firstCandidate = std::lower_bound(
            bodyPositionsByX.begin(), bodyPositionsByX.end(), center.x - searchDistance,
            [](const auto& candidate, double minimumX)
            {
                return candidate.first < minimumX;
            });
        for (auto candidate = firstCandidate;
            candidate != bodyPositionsByX.end()
                && candidate->first <= center.x + searchDistance;
            ++candidate)
        {
            const std::size_t index = candidate->second;
            if (index == bodyIndex)
                continue;

            const Body& otherBody = *bodies[index];
            const RigidBodyState other = ToProjectState(otherBody.kinematics.getState());
            const double combinedRadius = bodyRadius + otherBody.collisionRadiusMeters;
            const glm::dvec3 centerOffset = center - other.positionMeters;
            if (glm::dot(centerOffset, centerOffset) >= combinedRadius * combinedRadius)
                continue;

            for (const RigidBodyBox& ownBox : body.collisionBoxes)
            {
                const OrientedBox worldOwnBox = ToWorldBox(
                    center, state.orientation, ownBox);
                for (const RigidBodyBox& otherBox : bodies[index]->collisionBoxes)
                {
                    const auto contact = FindBoxContact(worldOwnBox,
                        ToWorldBox(other.positionMeters, other.orientation, otherBox));
                    if (!contact || contact->penetrationMeters <= greatestPenetration)
                        continue;

                    greatestPenetration = contact->penetrationMeters;
                    deepestContact = msr::airlib::CollisionInfo{
                        true,
                        ToAirSimPosition(contact->normal),
                        ToAirSimPosition(contact->impactPointMeters),
                        ToAirSimPosition(center),
                        static_cast<float>(contact->penetrationMeters),
                        timestamp,
                        "drone",
                        static_cast<int>(index + 1)};
                }
            }
        }
        return deepestContact.value_or(msr::airlib::CollisionInfo{});
    }
};

PhysicsWorld::PhysicsWorld()
    : impl_(std::make_unique<Impl>())
{
}

PhysicsWorld::~PhysicsWorld() = default;

void PhysicsWorld::AddStaticObstacleBox(
    const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters)
{
    if (!IsFinite(centerMeters)
        || !IsFinite(glm::dvec3{halfExtentsMeters})
        || glm::any(glm::lessThanEqual(halfExtentsMeters, glm::vec3{0.0F})))
    {
        throw std::invalid_argument("고정 상자의 위치와 크기를 확인해 주세요.");
    }
    impl_->obstacles.push_back({centerMeters, glm::dvec3{halfExtentsMeters}});
}

RigidBodyHandle PhysicsWorld::CreateQuadrotor(
    const RigidBodyDefinition& definition,
    const QuadrotorParameters& quadrotor)
{
    if (!IsFinite(definition.positionMeters)
        || !IsValidOrientation(definition.orientation)
        || !(quadrotor.massKilograms > 0.0)
        || !std::isfinite(quadrotor.massKilograms)
        || !IsFinite(quadrotor.principalInertiaKilogramMetersSquared)
        || glm::any(glm::lessThanEqual(
            quadrotor.principalInertiaKilogramMetersSquared, glm::dvec3{0.0}))
        || !IsFinite(quadrotor.bodySizeMeters)
        || glm::any(glm::lessThanEqual(quadrotor.bodySizeMeters, glm::dvec3{0.0}))
        || !(quadrotor.thrustCoefficientNewtonsPerSpeedSquared > 0.0)
        || !std::isfinite(quadrotor.thrustCoefficientNewtonsPerSpeedSquared)
        || !(quadrotor.momentToThrustRatio > 0.0)
        || !std::isfinite(quadrotor.momentToThrustRatio)
        || !(quadrotor.motorTimeConstantUpSeconds > 0.0)
        || !std::isfinite(quadrotor.motorTimeConstantUpSeconds)
        || !(quadrotor.maximumMotorSpeedRadiansPerSecond > 0.0)
        || !std::isfinite(quadrotor.maximumMotorSpeedRadiansPerSecond)
        || !std::all_of(quadrotor.rotors.begin(), quadrotor.rotors.end(),
            [](const RotorDefinition& rotor) { return IsFinite(rotor.positionMeters); }))
    {
        throw std::invalid_argument("드론의 물리 제원과 시작 자세를 확인해 주세요.");
    }

    for (const RigidBodyBox& box : quadrotor.collisionBoxes)
    {
        if (!IsFinite(glm::dvec3{box.halfExtentsMeters})
            || glm::any(glm::lessThanEqual(box.halfExtentsMeters, glm::vec3{0.0F}))
            || !IsFinite(box.positionMeters)
            || !IsValidOrientation(box.orientation))
        {
            throw std::invalid_argument("드론 충돌 모양의 크기와 위치를 확인해 주세요.");
        }
    }

    static_cast<void>(msr::airlib::ClockFactory::get(impl_->clock));
    auto body = std::make_unique<Impl::Body>(definition, quadrotor);
    const RigidBodyHandle handle{static_cast<std::uint32_t>(impl_->bodies.size())};
    impl_->physicsEngine.insert(body->model.get());
    impl_->bodies.push_back(std::move(body));
    return handle;
}

void PhysicsWorld::SetMotorTargets(
    RigidBodyHandle handle,
    const std::array<double, 4>& speeds)
{
    Impl::Body& body = impl_->GetBody(handle);
    body.actuation.SetMotorTargets(speeds, body.maximumMotorSpeed);
}

void PhysicsWorld::SetWindVelocityMetersPerSecond(const glm::dvec3& velocity)
{
    if (!IsFinite(velocity))
        throw std::invalid_argument("바람 속도는 유한한 값이어야 합니다.");
    impl_->physicsEngine.setWind(ToAirSimPosition(velocity));
}

void PhysicsWorld::SetTerrain(terrain::TerrainGrid source)
{
    if (source.sampleCount < 3U
        || source.heightOffsetsMeters.size()
            != static_cast<std::size_t>(source.sampleCount) * source.sampleCount
        || !std::isfinite(source.spacingXMeters) || !(source.spacingXMeters > 0.0F)
        || !std::isfinite(source.spacingYMeters) || !(source.spacingYMeters > 0.0F)
        || !std::all_of(source.heightOffsetsMeters.begin(), source.heightOffsetsMeters.end(),
            [](float value) { return std::isfinite(value); }))
    {
        throw std::invalid_argument("지형 격자 크기, 간격과 높이 값을 확인해 주세요.");
    }
    impl_->terrain = std::move(source);
}

void PhysicsWorld::SetState(RigidBodyHandle handle, const RigidBodyState& state)
{
    if (!IsFinite(state.positionMeters) || !IsFinite(state.linearVelocityMetersPerSecond)
        || !IsFinite(state.angularVelocityRadiansPerSecond)
        || !IsValidOrientation(state.orientation))
    {
        throw std::invalid_argument("강체의 초기 상태를 확인해 주세요.");
    }
    Impl::Body& body = impl_->GetBody(handle);
    static_cast<void>(msr::airlib::ClockFactory::get(impl_->clock));
    if (body.updatedSinceReset)
    {
        body.actuation.reset();
        body.model->reset();
        body.updatedSinceReset = false;
    }
    const msr::airlib::Kinematics::State airSimState = ToAirSimState(state);
    body.kinematics.setState(airSimState);
    body.model->setPose(airSimState.pose);
    body.model->setTwist(airSimState.twist);
    body.model->setCollisionInfo({});
}

void PhysicsWorld::Step(double seconds, double* durationMicroseconds)
{
    if (!std::isfinite(seconds) || !(seconds > 0.0))
        throw std::invalid_argument("물리 시간 간격을 확인해 주세요.");
    static_cast<void>(msr::airlib::ClockFactory::get(impl_->clock));
    static_cast<void>(impl_->clock->stepBy(seconds));

    auto advance = [&]
    {
        ++impl_->stepCount;
        const double largestCollisionRadius = impl_->OrderBodiesAlongXAxis();
        for (std::size_t index = 0; index < impl_->bodies.size(); ++index)
        {
            auto& body = impl_->bodies[index];
            if (!body)
                continue;
            body->environment.setPosition(body->kinematics.getPose().position);
            body->environment.update();
            body->model->setCollisionInfo(
                impl_->BuildAirSimContactInfo(*body, index, largestCollisionRadius));
            body->ApplyMotorTargets();
            body->model->update();
            body->updatedSinceReset = true;
        }
        impl_->physicsEngine.update();
    };

    if (durationMicroseconds == nullptr)
    {
        advance();
        return;
    }
    const auto start = std::chrono::steady_clock::now();
    advance();
    *durationMicroseconds = std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - start).count();
}

bool PhysicsWorld::DestroyBody(RigidBodyHandle handle)
{
    if (handle.value >= impl_->bodies.size() || !impl_->bodies[handle.value])
        return false;
    impl_->physicsEngine.erase_remove(impl_->bodies[handle.value]->model.get());
    impl_->bodies[handle.value].reset();
    return true;
}

RigidBodyState PhysicsWorld::State(RigidBodyHandle handle) const
{
    return ToProjectState(impl_->GetBody(handle).kinematics.getState());
}

std::array<double, 4> PhysicsWorld::MotorSpeedsRadiansPerSecond(
    RigidBodyHandle handle) const
{
    const Impl::Body& body = impl_->GetBody(handle);
    std::array<double, 4> speeds{};
    for (std::size_t index = 0; index < speeds.size(); ++index)
    {
        speeds[index] = body.model->getRotorOutput(
            static_cast<msr::airlib::uint>(index)).speed;
    }
    return speeds;
}
}
