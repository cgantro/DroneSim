#include "simulation_core/PhysicsWorld.h"

#include "simulation_core/DroneModel.h"
#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/ScopedDuration.h"
#include "simulation_core/CollisionGeometry.h"

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
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

namespace aerosimlab::simulation
{
namespace
{
using AirSimBody = msr::airlib::MultiRotorPhysicsBody;
using AirSimVector = msr::airlib::Vector3r;
using collision::BoxContact;
using collision::BroadphaseMayOverlap;
using collision::CollisionRadius;
using collision::FindBoxContact;
using collision::OrientedBox;
using collision::SupportPoint;
using collision::ToWorldBox;

// 장면 좌표는 동·북·위(ENU), AirSim 월드는 북·동·아래(NED)다.
const glm::dmat3 projectToAirSimWorld{
    {0.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, 0.0, -1.0}};
// 기체 좌표는 앞·왼쪽·위(FLU), AirSim 기체 좌표는 앞·오른쪽·아래(FRD)다.
const glm::dmat3 projectToAirSimBody{
    {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 0.0, -1.0}};

bool IsFinite(const glm::dvec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsPositiveFinite(double value)
{
    return std::isfinite(value) && value > 0.0;
}

/** @brief AirSim의 float이 표현할 수 있는 값만 double에서 변환한다. */
float ToAirSimFloat(double value)
{
    constexpr double maximumFloat = std::numeric_limits<float>::max();
    if (!std::isfinite(value) || value < -maximumFloat || value > maximumFloat)
        throw std::invalid_argument("물리값이 AirSim에서 표현할 수 있는 범위를 벗어났습니다.");

    return static_cast<float>(value);
}

float ToAirSimPositiveFloat(double value)
{
    const float converted = ToAirSimFloat(value);
    if (!(converted > 0.0F))
        throw std::invalid_argument("양수 물리값이 AirSim 변환 뒤 0이 됩니다.");

    return converted;
}

bool IsValidOrientation(const glm::dquat& value)
{
    const double lengthSquared = glm::dot(value, value);
    return std::isfinite(value.w) && std::isfinite(value.x)
        && std::isfinite(value.y) && std::isfinite(value.z)
        && std::isfinite(lengthSquared) && lengthSquared > 0.0;
}

bool HasValidVehicleConfiguration(
    const RigidBodyDefinition& definition,
    const QuadrotorParameters& parameters)
{
    return IsFinite(definition.positionMeters)
        && IsValidOrientation(definition.orientation)
        && IsPositiveFinite(parameters.massKilograms)
        && IsFinite(parameters.principalInertiaKilogramMetersSquared)
        && glm::all(glm::greaterThan(
            parameters.principalInertiaKilogramMetersSquared, glm::dvec3{0.0}))
        && IsFinite(parameters.bodySizeMeters)
        && glm::all(glm::greaterThan(parameters.bodySizeMeters, glm::dvec3{0.0}))
        && IsPositiveFinite(parameters.thrustCoefficientNewtonsPerSpeedSquared)
        && IsPositiveFinite(parameters.momentToThrustRatio)
        && IsPositiveFinite(parameters.motorTimeConstantUpSeconds)
        && IsPositiveFinite(parameters.motorTimeConstantDownSeconds)
        && IsPositiveFinite(parameters.maximumMotorSpeedRadiansPerSecond)
        && std::all_of(parameters.rotors.begin(), parameters.rotors.end(),
            [](const RotorDefinition& rotor)
            {
                return IsFinite(rotor.positionMeters);
            });
}

bool HasValidCollisionBox(const RigidBodyBox& box)
{
    return IsFinite(glm::dvec3{box.halfExtentsMeters})
        && glm::all(glm::greaterThan(box.halfExtentsMeters, glm::vec3{0.0F}))
        && IsFinite(box.positionMeters)
        && IsValidOrientation(box.orientation);
}

AirSimVector ToAirSimWorldVector(const glm::dvec3& value)
{
    const glm::dvec3 converted = projectToAirSimWorld * value;
    return {ToAirSimFloat(converted.x), ToAirSimFloat(converted.y),
        ToAirSimFloat(converted.z)};
}

AirSimVector ToAirSimBodyVector(const glm::dvec3& value)
{
    const glm::dvec3 converted = projectToAirSimBody * value;
    return {ToAirSimFloat(converted.x), ToAirSimFloat(converted.y),
        ToAirSimFloat(converted.z)};
}

glm::dvec3 ToProjectWorldVector(const AirSimVector& value)
{
    return projectToAirSimWorld * glm::dvec3{value.x(), value.y(), value.z()};
}

glm::dvec3 ToProjectBodyVector(const AirSimVector& value)
{
    return projectToAirSimBody * glm::dvec3{value.x(), value.y(), value.z()};
}

msr::airlib::Quaternionr ToAirSimOrientation(const glm::dquat& value)
{
    const glm::dmat3 rotation = glm::mat3_cast(glm::normalize(value));
    const glm::dquat converted = glm::quat_cast(
        projectToAirSimWorld * rotation * projectToAirSimBody);
    return {ToAirSimFloat(converted.w), ToAirSimFloat(converted.x),
        ToAirSimFloat(converted.y), ToAirSimFloat(converted.z)};
}

glm::dquat ToProjectOrientation(const msr::airlib::Quaternionr& value)
{
    const glm::dmat3 rotation = glm::mat3_cast(glm::dquat{
        value.w(), value.x(), value.y(), value.z()});
    return glm::normalize(glm::quat_cast(
        projectToAirSimWorld * rotation * projectToAirSimBody));
}

msr::airlib::Kinematics::State ToAirSimState(const RigidBodyState& value)
{
    auto result = msr::airlib::Kinematics::State::zero();
    result.pose.position = ToAirSimWorldVector(value.positionMeters);
    result.pose.orientation = ToAirSimOrientation(value.orientation);
    result.twist.linear = ToAirSimWorldVector(value.linearVelocityMetersPerSecond);
    result.twist.angular = ToAirSimBodyVector(value.angularVelocityRadiansPerSecond);
    return result;
}

RigidBodyState ToProjectState(const msr::airlib::Kinematics::State& value)
{
    return {
        ToProjectWorldVector(value.pose.position),
        ToProjectOrientation(value.pose.orientation),
        ToProjectWorldVector(value.twist.linear),
        ToProjectBodyVector(value.twist.angular)};
}

/** @brief 상승·하강 응답을 반영한 모터 속도를 AirSim 추력 신호로 바꾼다. */
class ActuationApi final : public msr::airlib::VehicleApiBase
{
public:
    ActuationApi(double timeConstantUpSeconds, double timeConstantDownSeconds)
        : timeConstantUpSeconds_(timeConstantUpSeconds),
          timeConstantDownSeconds_(timeConstantDownSeconds)
    {
    }

    void SetMotorTargets(
        const std::array<double, QuadrotorParameters::rotorCount>& speeds,
        double maximumSpeed)
    {
        for (std::size_t index = 0; index < signals_.size(); ++index)
            targetSpeeds_[index] = motorFaults_[index]
                ? 0.0 : std::clamp(speeds[index], 0.0, maximumSpeed);
    }

    void SetRotorMotorFault(std::size_t rotorIndex, bool faulted)
    {
        motorFaults_.at(rotorIndex) = faulted;
        if (faulted)
            targetSpeeds_.at(rotorIndex) = 0.0;
    }

    void SetInitialMotorSpeeds(
        const std::array<double, QuadrotorParameters::rotorCount>& speeds,
        double maximumSpeed)
    {
        for (std::size_t index = 0; index < signals_.size(); ++index)
        {
            filteredSpeeds_[index] = speeds[index];
            targetSpeeds_[index] = speeds[index];
            const double ratio = speeds[index] / maximumSpeed;
            signals_[index] = static_cast<float>(ratio * ratio);
        }
    }

    const std::array<double, QuadrotorParameters::rotorCount>& MotorSpeeds() const
    {
        return filteredSpeeds_;
    }

    /** @brief 이번 물리 Tick만큼 모터 속도를 목표 쪽으로 갱신한다. */
    void AdvanceMotorResponse(double seconds, double maximumSpeed)
    {
        for (std::size_t index = 0; index < signals_.size(); ++index)
        {
            const double targetSpeed = targetSpeeds_[index];
            const double timeConstant = targetSpeed >= filteredSpeeds_[index]
                ? timeConstantUpSeconds_ : timeConstantDownSeconds_;
            const double blend = 1.0 - std::exp(-seconds / timeConstant);
            filteredSpeeds_[index] +=
                (targetSpeed - filteredSpeeds_[index]) * blend;
            const double ratio = filteredSpeeds_[index] / maximumSpeed;
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
        targetSpeeds_.fill(0.0);
        filteredSpeeds_.fill(0.0);
        motorFaults_.fill(false);
        apiControlEnabled_ = false;
        armed_ = false;
    }

private:
    double timeConstantUpSeconds_;
    double timeConstantDownSeconds_;
    std::array<float, QuadrotorParameters::rotorCount> signals_{};
    std::array<double, QuadrotorParameters::rotorCount> targetSpeeds_{};
    std::array<double, QuadrotorParameters::rotorCount> filteredSpeeds_{};
    std::array<bool, QuadrotorParameters::rotorCount> motorFaults_{};
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
        target.mass = ToAirSimPositiveFloat(source_.massKilograms);
        target.inertia = msr::airlib::Matrix3x3r::Zero();
        target.inertia.diagonal() <<
            ToAirSimPositiveFloat(source_.principalInertiaKilogramMetersSquared.x),
            ToAirSimPositiveFloat(source_.principalInertiaKilogramMetersSquared.y),
            ToAirSimPositiveFloat(source_.principalInertiaKilogramMetersSquared.z);
        target.body_box = AirSimVector{ToAirSimPositiveFloat(source_.bodySizeMeters.x),
            ToAirSimPositiveFloat(source_.bodySizeMeters.y),
            ToAirSimPositiveFloat(source_.bodySizeMeters.z)};
        target.rotor_params.max_rpm = ToAirSimPositiveFloat(
            source_.maximumMotorSpeedRadiansPerSecond * 60.0 / (2.0 * M_PI));
        target.rotor_params.calculateMaxThrust();
        target.rotor_params.max_speed_square = ToAirSimPositiveFloat(
            source_.maximumMotorSpeedRadiansPerSecond
                * source_.maximumMotorSpeedRadiansPerSecond);
        target.rotor_params.max_thrust = ToAirSimPositiveFloat(
            source_.thrustCoefficientNewtonsPerSpeedSquared
                * target.rotor_params.max_speed_square);
        target.rotor_params.max_torque = ToAirSimPositiveFloat(
            source_.momentToThrustRatio * target.rotor_params.max_thrust);
        // 속도 응답은 위에서 상승·하강 시간을 따로 계산한다. AirSim 내부 필터는 겹치지 않게 거의 즉시 반응하도록 둔다.
        target.rotor_params.control_signal_filter_tc = 1.0e-6F;
        for (const RotorDefinition& rotor : source_.rotors)
        {
            const auto direction = rotor.rotation == RotorRotation::CounterClockwise
                ? msr::airlib::RotorTurningDirection::RotorTurningDirectionCCW
                : msr::airlib::RotorTurningDirection::RotorTurningDirectionCW;
            // 두 좌표계 사이의 변환은 오른손 회전이므로 로터 회전 방향은 유지한다.
            target.rotor_poses.emplace_back(
                ToAirSimBodyVector(rotor.positionMeters),
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
    // 외곽선이 비어 있으면 상자이고, 있으면 외곽선과 높이로 접촉을 검사한다.
    glm::dvec3 centerMeters{0.0};
    glm::dvec3 halfExtentsMeters{0.0};
    std::vector<glm::dvec2> outlineMeters;
    double bottomMeters = 0.0;
    double topMeters = 0.0;
    bool footprintOnly = false;
};

struct ContactCandidate
{
    BoxContact contact;
    const char* objectName;
    int objectId;
};

void KeepDeeperContact(
    std::optional<ContactCandidate>& current,
    const BoxContact& contact,
    const char* objectName,
    int objectId)
{
    if (current && contact.penetrationMeters <= current->contact.penetrationMeters)
        return;

    current = ContactCandidate{contact, objectName, objectId};
}

}

struct PhysicsWorld::Impl
{
    struct Body
    {
        RigidBodyHandle handle;
        QuadrotorParametersAdapter parameters;
        ActuationApi actuation;
        msr::airlib::Kinematics kinematics;
        msr::airlib::Environment environment;
        // 모델은 위 네 객체의 주소를 빌리므로 먼저 파괴되도록 뒤에 둔다.
        std::unique_ptr<AirSimBody> model;
        std::vector<RigidBodyBox> collisionBoxes;
        std::vector<OrientedBox> worldCollisionBoxes;
        RigidBodyState collisionState;
        double collisionRadiusMeters = 0.0;
        double maximumMotorSpeed = 0.0;
        bool updatedSinceReset = false;
        bool initialMotorOutputsPending = false;
        bool wasGrounded = false;

        Body(
            RigidBodyHandle bodyHandle,
            const RigidBodyDefinition& definition,
            const QuadrotorParameters& quadrotor)
            : handle(bodyHandle),
              parameters(quadrotor),
              actuation(quadrotor.motorTimeConstantUpSeconds,
                  quadrotor.motorTimeConstantDownSeconds),
              kinematics(ToAirSimState({definition.positionMeters,
                  definition.orientation, {}, {}})),
              collisionBoxes(quadrotor.collisionBoxes.begin(), quadrotor.collisionBoxes.end()),
              worldCollisionBoxes(collisionBoxes.size()),
              collisionRadiusMeters(CollisionRadius(collisionBoxes)),
              maximumMotorSpeed(quadrotor.maximumMotorSpeedRadiansPerSecond)
        {
            parameters.Configure();
            msr::airlib::Environment::State environmentState;
            environmentState.position = ToAirSimWorldVector(definition.positionMeters);
            environmentState.geo_point.set(0.0, 0.0, 0.0F);
            environment.initialize(environmentState);
            kinematics.reset();
            actuation.reset();
            model = std::make_unique<AirSimBody>(
                &parameters, &actuation, &kinematics, &environment);
            model->reset();
            model->setPose({ToAirSimWorldVector(definition.positionMeters),
                ToAirSimOrientation(definition.orientation)});
        }

        void ApplyMotorTargets()
        {
            // 고정 Tick이 시작될 때 계산된 모터 목표를 각 로터에 전달한다.
            for (msr::airlib::uint index = 0; index < parameters.getParams().rotor_count; ++index)
            {
                auto& rotor = static_cast<msr::airlib::RotorActuator&>(
                    model->getWrenchVertex(index));
                rotor.setControlSignal(actuation.getActuation(index));
            }
        }

        /**
         * @brief 첫 물리 적분 전에 AirSim 필터에 초기 회전수를 반영한다.
         * @details 첫 갱신은 필터를 진행하고, 두 번째 갱신은 그 출력으로 로터 힘을 만든다.
         */
        void InitializeMotorOutputs()
        {
            ApplyMotorTargets();
            for (msr::airlib::uint index = 0; index < parameters.getParams().rotor_count; ++index)
            {
                auto& rotor = static_cast<msr::airlib::RotorActuator&>(
                    model->getWrenchVertex(index));
                rotor.update();
                rotor.update();
            }
            initialMotorOutputsPending = false;
        }

        void AdvanceMotorResponse(double seconds)
        {
            actuation.AdvanceMotorResponse(seconds, maximumMotorSpeed);
        }

        /** @brief 현재 강체 자세를 읽어 충돌 상자의 월드 좌표를 갱신한다. */
        void RefreshCollisionGeometry()
        {
            collisionState = ToProjectState(kinematics.getState());
            for (std::size_t index = 0; index < collisionBoxes.size(); ++index)
            {
                worldCollisionBoxes[index] = ToWorldBox(
                    collisionState.positionMeters,
                    collisionState.orientation,
                    collisionBoxes[index]);
            }
        }
    };

    // AirSim 시계는 전역으로 보관된다. 월드는 자기 시계를 소유하고 호출 전에 다시 활성화한다.
    std::shared_ptr<msr::airlib::SteppableClock> clock =
        std::make_shared<msr::airlib::SteppableClock>(DroneModel::fixedStepSeconds);
    msr::airlib::FastPhysicsEngine physicsEngine{true};
    // 핸들 값은 고정하고, 벡터에는 현재 살아 있는 강체만 보관한다.
    std::vector<std::unique_ptr<Body>> bodies;
    std::unordered_map<std::uint32_t, std::size_t> bodyIndices;
    std::vector<std::pair<double, std::size_t>> bodyPositionsByX;
    std::vector<std::optional<ContactCandidate>> droneContacts;
    std::vector<Obstacle> obstacles;
    std::optional<terrain::TerrainGrid> terrain;
    std::uint64_t stepCount = 0;
    std::uint64_t nextBodyHandleValue = 0;

    Impl()
    {
        static_cast<void>(msr::airlib::ClockFactory::get(clock));
        physicsEngine.reset();
    }

    /**
     * @brief AirSim 엔진에 등록한 강체를 먼저 분리한다.
     * @details 엔진은 강체를 소유하지 않으므로, 빌린 주소를 지운 뒤 우리가 가진 강체를 파괴한다.
     */
    ~Impl()
    {
        physicsEngine.clear();
    }

    std::size_t GetBodyIndex(RigidBodyHandle handle) const
    {
        const auto found = bodyIndices.find(handle.value);
        if (found == bodyIndices.end())
            throw std::out_of_range("존재하지 않는 강체 번호입니다.");
        return found->second;
    }

    Body& GetBody(RigidBodyHandle handle)
    {
        return *bodies[GetBodyIndex(handle)];
    }

    const Body& GetBody(RigidBodyHandle handle) const
    {
        return *bodies[GetBodyIndex(handle)];
    }

    /**
     * @brief 각 강체 상태를 한 번 읽고 충돌 상자를 월드 좌표로 바꾼 뒤 X축으로 정렬한다.
     * @details 한 Tick의 모든 충돌 검사가 AirSim 적분 전 같은 위치와 자세를 사용한다.
     */
    double PrepareCollisionData()
    {
        if (bodyPositionsByX.size() != bodies.size())
        {
            bodyPositionsByX.clear();
            bodyPositionsByX.reserve(bodies.size());
            for (std::size_t index = 0; index < bodies.size(); ++index)
                bodyPositionsByX.emplace_back(0.0, index);
        }

        double largestCollisionRadius = 0.0;
        for (std::size_t index = 0; index < bodies.size(); ++index)
        {
            Body& body = *bodies[index];
            body.RefreshCollisionGeometry();
            largestCollisionRadius = std::max(
                largestCollisionRadius, body.collisionRadiusMeters);
        }

        // 드론의 X 좌표만 갱신하고, 순서가 바뀐 경우에만 다시 정렬한다.
        // 호버링처럼 서로 앞지르지 않는 동안 매 Tick 반복하던 정렬을 건너뛴다.
        for (auto& [positionX, bodyIndex] : bodyPositionsByX)
            positionX = bodies[bodyIndex]->collisionState.positionMeters.x;
        if (!std::is_sorted(bodyPositionsByX.begin(), bodyPositionsByX.end()))
            std::sort(bodyPositionsByX.begin(), bodyPositionsByX.end());
        return largestCollisionRadius;
    }

    /** @brief 후보 기체 두 대의 상자 접촉을 계산해 양쪽 기체에 기록한다. */
    void RecordDronePairContacts(std::size_t bodyIndex, std::size_t otherIndex)
    {
        const Body& body = *bodies[bodyIndex];
        const Body& otherBody = *bodies[otherIndex];
        if (!BroadphaseMayOverlap(body.collisionState.positionMeters,
                body.collisionRadiusMeters,
                otherBody.collisionState.positionMeters,
                otherBody.collisionRadiusMeters))
        {
            return;
        }

        for (const OrientedBox& ownBox : body.worldCollisionBoxes)
        {
            for (const OrientedBox& otherBox : otherBody.worldCollisionBoxes)
            {
                const auto contact = FindBoxContact(ownBox, otherBox);
                if (!contact)
                    continue;

                KeepDeeperContact(droneContacts[bodyIndex], *contact,
                    "drone", static_cast<int>(otherBody.handle.value + 1U));
                const BoxContact reverseContact{
                    -contact->normal,
                    SupportPoint(otherBox, contact->normal),
                    contact->penetrationMeters};
                KeepDeeperContact(droneContacts[otherIndex], reverseContact,
                    "drone", static_cast<int>(body.handle.value + 1U));
            }
        }
    }

    /** @brief 드론 한 쌍의 상자 충돌을 한 번 계산해 양쪽 기체에 저장한다. */
    void PrepareDroneContacts(double largestCollisionRadius)
    {
        droneContacts.clear();
        droneContacts.resize(bodies.size());

        // X 좌표로 정렬했으므로 앞쪽 기체만 훑으면 각 쌍을 한 번만 확인할 수 있다.
        // 검색 폭에는 가장 큰 충돌 반경을 써서 크기가 다른 기체도 후보에서 빠지지 않게 한다.
        for (std::size_t orderIndex = 0;
            orderIndex < bodyPositionsByX.size(); ++orderIndex)
        {
            const std::size_t bodyIndex = bodyPositionsByX[orderIndex].second;
            const Body& body = *bodies[bodyIndex];
            const glm::dvec3 center = body.collisionState.positionMeters;
            const double maximumCandidateX = center.x
                + body.collisionRadiusMeters + largestCollisionRadius;
            for (std::size_t candidateOrder = orderIndex + 1U;
                candidateOrder < bodyPositionsByX.size()
                    && bodyPositionsByX[candidateOrder].first <= maximumCandidateX;
                ++candidateOrder)
            {
                const std::size_t otherIndex = bodyPositionsByX[candidateOrder].second;
                RecordDronePairContacts(bodyIndex, otherIndex);
            }
        }
    }

    /** @brief 공통 지면 접촉 결과를 AirSim이 받는 좌표와 형식으로 바꾼다. */
    std::optional<msr::airlib::CollisionInfo> FindTerrainContact(
        const Body& body, std::uint64_t timestamp) const
    {
        const auto contact = collision::FindTerrainContact(
            body.worldCollisionBoxes, terrain ? &*terrain : nullptr);
        if (!contact)
            return std::nullopt;

        return msr::airlib::CollisionInfo{
            true,
            ToAirSimWorldVector(contact->normal),
            ToAirSimWorldVector(contact->impactPointMeters),
            ToAirSimWorldVector(body.collisionState.positionMeters),
            ToAirSimFloat(contact->penetrationMeters),
            timestamp,
            "ground",
            0};
    }

    /** @brief DEM 바깥으로 기체가 나가지 않도록 충돌 상자 전체를 경계 안에 둔다. */
    void ConstrainBodyToTerrain(Body& body)
    {
        if (!terrain)
            return;

        RigidBodyState state = ToProjectState(body.kinematics.getState());
        double minimumRelativeX = 0.0;
        double maximumRelativeX = 0.0;
        double minimumRelativeY = 0.0;
        double maximumRelativeY = 0.0;
        for (const RigidBodyBox& collisionBox : body.collisionBoxes)
        {
            const OrientedBox box = ToWorldBox(
                state.positionMeters, state.orientation, collisionBox);
            const glm::dvec3 relativeCenter = box.centerMeters - state.positionMeters;
            const glm::dvec3& halfExtents = box.axisAlignedHalfExtentsMeters;
            minimumRelativeX = std::min(minimumRelativeX, relativeCenter.x - halfExtents.x);
            maximumRelativeX = std::max(maximumRelativeX, relativeCenter.x + halfExtents.x);
            minimumRelativeY = std::min(minimumRelativeY, relativeCenter.y - halfExtents.y);
            maximumRelativeY = std::max(maximumRelativeY, relativeCenter.y + halfExtents.y);
        }

        const double halfWidth = terrain->HalfWidthMeters();
        const double halfDepth = terrain->HalfDepthMeters();
        double minimumX = -halfWidth - minimumRelativeX;
        double maximumX = halfWidth - maximumRelativeX;
        double minimumY = -halfDepth - minimumRelativeY;
        double maximumY = halfDepth - maximumRelativeY;
        if (minimumX > maximumX)
            minimumX = maximumX = (minimumX + maximumX) * 0.5;
        if (minimumY > maximumY)
            minimumY = maximumY = (minimumY + maximumY) * 0.5;
        const glm::dvec3 previousPosition = state.positionMeters;
        state.positionMeters.x = std::clamp(state.positionMeters.x, minimumX, maximumX);
        state.positionMeters.y = std::clamp(state.positionMeters.y, minimumY, maximumY);

        if (state.positionMeters.x != previousPosition.x
            && (previousPosition.x > maximumX
                ? state.linearVelocityMetersPerSecond.x > 0.0
                : state.linearVelocityMetersPerSecond.x < 0.0))
        {
            state.linearVelocityMetersPerSecond.x = 0.0;
        }
        if (state.positionMeters.y != previousPosition.y
            && (previousPosition.y > maximumY
                ? state.linearVelocityMetersPerSecond.y > 0.0
                : state.linearVelocityMetersPerSecond.y < 0.0))
        {
            state.linearVelocityMetersPerSecond.y = 0.0;
        }
        if (state.positionMeters == previousPosition)
            return;

        const msr::airlib::Kinematics::State constrainedState = ToAirSimState(state);
        body.kinematics.setState(constrainedState);
        body.model->setPose(constrainedState.pose);
        body.model->setTwist(constrainedState.twist);
    }

    /**
     * @brief AirSim이 지면에 고정된 순간에만 기체를 지면 위로 맞춘다.
     * @details 지면 고정 경로는 겹친 깊이를 위치에 반영하지 않으므로, 고정 전환 직후 가장 깊은 상자의 관통량을 보정한다.
     */
    void CorrectGroundPenetration(Body& body)
    {
        body.RefreshCollisionGeometry();

        const auto contact = FindTerrainContact(body, stepCount);
        if (!contact)
            return;

        const double penetrationMeters =
            static_cast<double>(contact->penetration_depth);
        if (!(penetrationMeters > 0.0))
            return;

        glm::dvec3 correctedPosition = body.collisionState.positionMeters;
        correctedPosition.z += penetrationMeters + 1.0e-4;
        body.model->setPose({ToAirSimWorldVector(correctedPosition),
            ToAirSimOrientation(body.collisionState.orientation)});
    }

    /** @brief 기체 상자와 고정 장애물 상자 중 가장 깊이 겹친 접촉을 찾는다. */
    std::optional<ContactCandidate> FindObstacleContact(const Body& body) const
    {
        const glm::dvec3 center = body.collisionState.positionMeters;
        // 바깥 반경으로 멀리 떨어진 장애물을 먼저 거른 뒤 상자끼리 정확히 검사한다.
        const double bodyRadius = body.collisionRadiusMeters;
        std::optional<ContactCandidate> deepestContact;
        double greatestPenetration = -1.0;
        for (std::size_t index = 0; index < obstacles.size(); ++index)
        {
            const Obstacle& obstacle = obstacles[index];
            if (!obstacle.outlineMeters.empty())
            {
                for (const OrientedBox& worldBox : body.worldCollisionBoxes)
                {
                    const glm::dvec3& halfExtents =
                        worldBox.axisAlignedHalfExtentsMeters;
                    const double horizontalRadius =
                        std::hypot(halfExtents.x, halfExtents.y);
                    std::optional<BoxContact> contact;
                    if (obstacle.footprintOnly)
                    {
                        const auto footprintContact = collision::FindFootprintContact(
                            obstacle.outlineMeters,
                            {worldBox.centerMeters.x, worldBox.centerMeters.y},
                            horizontalRadius);
                        if (footprintContact)
                        {
                            contact = BoxContact{
                                {footprintContact->normal.x,
                                    footprintContact->normal.y, 0.0},
                                {footprintContact->impactPointMeters.x,
                                    footprintContact->impactPointMeters.y,
                                    worldBox.centerMeters.z},
                                footprintContact->penetrationMeters};
                        }
                    }
                    else
                    {
                        const auto prismContact = collision::FindPrismContact(
                            obstacle.outlineMeters, obstacle.bottomMeters,
                            obstacle.topMeters, worldBox.centerMeters,
                            horizontalRadius, halfExtents.z);
                        if (prismContact)
                        {
                            contact = BoxContact{prismContact->normal,
                                prismContact->impactPointMeters,
                                prismContact->penetrationMeters};
                        }
                    }
                    if (!contact || contact->penetrationMeters <= greatestPenetration)
                        continue;
                    greatestPenetration = contact->penetrationMeters;
                    deepestContact = ContactCandidate{
                        *contact,
                        obstacle.footprintOnly ? "unknown-height-building" : "building",
                        static_cast<int>(index + 1)};
                }
                continue;
            }
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
                    glm::dvec3{0.0, 0.0, 1.0}},
                obstacle.halfExtentsMeters,
                glm::length(obstacle.halfExtentsMeters)};
            for (const OrientedBox& worldBox : body.worldCollisionBoxes)
            {
                const auto contact = FindBoxContact(worldBox, obstacleBox);
                if (!contact || contact->penetrationMeters <= greatestPenetration)
                    continue;

                greatestPenetration = contact->penetrationMeters;
                deepestContact = ContactCandidate{
                    *contact, "obstacle", static_cast<int>(index + 1)};
            }
        }
        return deepestContact;
    }

    // 단독 실행에서는 Unreal이 접촉 정보를 만들지 않으므로, 장면 접촉을 찾아 AirSim에 넘긴다.
    msr::airlib::CollisionInfo BuildAirSimContactInfo(
        const Body& body,
        const std::optional<ContactCandidate>& droneContact) const
    {
        const std::uint64_t timestamp = stepCount + 1;
        if (auto groundContact = FindTerrainContact(body, timestamp))
            return *groundContact;

        std::optional<ContactCandidate> deepestContact = FindObstacleContact(body);
        if (droneContact && (!deepestContact
            || droneContact->contact.penetrationMeters
                > deepestContact->contact.penetrationMeters))
        {
            deepestContact = droneContact;
        }

        if (!deepestContact)
            return {};

        const BoxContact& contact = deepestContact->contact;
        return {
            true,
            ToAirSimWorldVector(contact.normal),
            ToAirSimWorldVector(contact.impactPointMeters),
            ToAirSimWorldVector(body.collisionState.positionMeters),
            ToAirSimFloat(contact.penetrationMeters),
            timestamp,
            deepestContact->objectName,
            deepestContact->objectId};
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
    if (!collision::IsValidObstacleBox(centerMeters, halfExtentsMeters))
    {
        throw std::invalid_argument("고정 상자의 위치와 크기를 확인해 주세요.");
    }
    impl_->obstacles.push_back(Obstacle{
        .centerMeters = centerMeters,
        .halfExtentsMeters = glm::dvec3{halfExtentsMeters}});
}

void PhysicsWorld::AddStaticObstaclePrism(
    const std::vector<glm::dvec2>& outlineMeters,
    double bottomMeters, double topMeters)
{
    if (!collision::IsValidPrism(outlineMeters, bottomMeters, topMeters))
        throw std::invalid_argument("건물 외곽선과 높이 범위를 확인해 주세요.");

    impl_->obstacles.push_back(Obstacle{
        .outlineMeters = outlineMeters,
        .bottomMeters = bottomMeters,
        .topMeters = topMeters});
}

void PhysicsWorld::AddStaticObstacleFootprint(
    const std::vector<glm::dvec2>& outlineMeters)
{
    if (!collision::IsValidFootprint(outlineMeters))
        throw std::invalid_argument("구조물 외곽선 좌표를 확인해 주세요.");

    impl_->obstacles.push_back(Obstacle{
        .outlineMeters = outlineMeters,
        .footprintOnly = true});
}

RigidBodyHandle PhysicsWorld::CreateQuadrotor(
    const RigidBodyDefinition& definition,
    const QuadrotorParameters& quadrotor)
{
    if (!HasValidVehicleConfiguration(definition, quadrotor))
    {
        throw std::invalid_argument("드론의 물리 제원과 시작 자세를 확인해 주세요.");
    }

    if (!std::all_of(quadrotor.collisionBoxes.begin(), quadrotor.collisionBoxes.end(),
        HasValidCollisionBox))
    {
        throw std::invalid_argument("드론 충돌 모양의 크기와 위치를 확인해 주세요.");
    }

    if (impl_->nextBodyHandleValue > std::numeric_limits<std::uint32_t>::max())
        throw std::overflow_error("강체 식별값을 더 만들 수 없습니다.");

    static_cast<void>(msr::airlib::ClockFactory::get(impl_->clock));
    const RigidBodyHandle handle{
        static_cast<std::uint32_t>(impl_->nextBodyHandleValue)};
    auto body = std::make_unique<Impl::Body>(handle, definition, quadrotor);

    // 엔진에 주소를 빌려주기 전에 저장 공간을 확보해 등록 뒤 이동이 실패하지 않게 한다.
    impl_->bodies.reserve(impl_->bodies.size() + 1U);
    impl_->bodyIndices.reserve(impl_->bodyIndices.size() + 1U);
    const std::size_t bodyIndex = impl_->bodies.size();
    const auto indexEntry =
        impl_->bodyIndices.emplace(handle.value, bodyIndex).first;

    try
    {
        impl_->physicsEngine.insert(body->model.get());
    }
    catch (...)
    {
        impl_->physicsEngine.erase_remove(body->model.get());
        impl_->bodyIndices.erase(indexEntry);
        throw;
    }

    impl_->bodies.push_back(std::move(body));
    ++impl_->nextBodyHandleValue;
    return handle;
}

void PhysicsWorld::SetMotorTargets(
    RigidBodyHandle handle,
    const std::array<double, 4>& speeds)
{
    Impl::Body& body = impl_->GetBody(handle);
    if (!std::all_of(speeds.begin(), speeds.end(),
        [](double speed) { return std::isfinite(speed); }))
    {
        throw std::invalid_argument("로터 목표 속도는 유한한 값이어야 합니다.");
    }
    body.actuation.SetMotorTargets(speeds, body.maximumMotorSpeed);
}

void PhysicsWorld::SetRotorMotorFault(
    RigidBodyHandle handle, std::size_t rotorIndex, bool faulted)
{
    impl_->GetBody(handle).actuation.SetRotorMotorFault(rotorIndex, faulted);
}

void PhysicsWorld::SetInitialMotorSpeeds(
    RigidBodyHandle handle,
    const std::array<double, 4>& speeds)
{
    Impl::Body& body = impl_->GetBody(handle);
    if (body.updatedSinceReset)
        throw std::logic_error("초기 로터 속도는 첫 물리 Tick 전에만 바꿀 수 있습니다.");
    if (!std::all_of(speeds.begin(), speeds.end(), [&](double speed)
        {
            return std::isfinite(speed) && speed >= 0.0
                && speed <= body.maximumMotorSpeed;
        }))
    {
        throw std::invalid_argument("초기 로터 속도는 0부터 최대 속도 사이의 유한한 값이어야 합니다.");
    }
    body.actuation.SetInitialMotorSpeeds(speeds, body.maximumMotorSpeed);
    body.initialMotorOutputsPending = true;
}

void PhysicsWorld::SetWindVelocityMetersPerSecond(const glm::dvec3& velocity)
{
    if (!IsFinite(velocity))
        throw std::invalid_argument("바람 속도는 유한한 값이어야 합니다.");
    impl_->physicsEngine.setWind(ToAirSimWorldVector(velocity));
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
    const msr::airlib::Kinematics::State airSimState = ToAirSimState(state);
    static_cast<void>(msr::airlib::ClockFactory::get(impl_->clock));
    if (body.updatedSinceReset)
    {
        body.actuation.reset();
        body.model->reset();
        body.updatedSinceReset = false;
    }
    body.kinematics.setState(airSimState);
    body.model->setPose(airSimState.pose);
    body.model->setTwist(airSimState.twist);
    body.model->setCollisionInfo({});
    body.wasGrounded = false;
}

void PhysicsWorld::Step(double seconds, double* stepDurationMicroseconds)
{
    if (!std::isfinite(seconds) || !(seconds > 0.0))
        throw std::invalid_argument("물리 시간 간격을 확인해 주세요.");
    static_cast<void>(msr::airlib::ClockFactory::get(impl_->clock));
    static_cast<void>(impl_->clock->stepBy(seconds));
    ScopedDuration measurePhysicsStep(stepDurationMicroseconds);
    ++impl_->stepCount;
    // 적분 전 같은 기체 자세로 지형 경계와 드론 간 접촉을 준비한다.
    for (auto& body : impl_->bodies)
        impl_->ConstrainBodyToTerrain(*body);
    const double largestCollisionRadius = impl_->PrepareCollisionData();
    impl_->PrepareDroneContacts(largestCollisionRadius);
    // 접촉과 로터 입력을 각 기체에 전달한 다음 AirSim을 한 단계 진행한다.
    for (std::size_t index = 0; index < impl_->bodies.size(); ++index)
    {
        auto& body = impl_->bodies[index];
        body->environment.setPosition(body->kinematics.getPose().position);
        body->environment.update();
        body->model->setCollisionInfo(
            impl_->BuildAirSimContactInfo(*body, impl_->droneContacts[index]));
        if (body->initialMotorOutputsPending)
            body->InitializeMotorOutputs();
        body->AdvanceMotorResponse(seconds);
        body->ApplyMotorTargets();
        body->model->update();
        body->updatedSinceReset = true;
    }
    impl_->physicsEngine.update();
    // 지면에 고정된 순간에는 남아 있는 관통 깊이를 한 번 보정한다.
    for (auto& body : impl_->bodies)
    {
        impl_->ConstrainBodyToTerrain(*body);
        const bool isGrounded = body->model->isGrounded();
        if (isGrounded && !body->wasGrounded)
            impl_->CorrectGroundPenetration(*body);
        body->wasGrounded = isGrounded;
    }
}

bool PhysicsWorld::DestroyBody(RigidBodyHandle handle)
{
    const auto found = impl_->bodyIndices.find(handle.value);
    if (found == impl_->bodyIndices.end())
        return false;

    const std::size_t bodyIndex = found->second;
    impl_->physicsEngine.erase_remove(impl_->bodies[bodyIndex]->model.get());
    impl_->bodyIndices.erase(found);
    impl_->bodies.erase(impl_->bodies.begin()
        + static_cast<std::vector<std::unique_ptr<Impl::Body>>::difference_type>(bodyIndex));
    for (std::size_t index = bodyIndex; index < impl_->bodies.size(); ++index)
        impl_->bodyIndices.at(impl_->bodies[index]->handle.value) = index;
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
    if (!body.updatedSinceReset)
        return body.actuation.MotorSpeeds();

    std::array<double, 4> speeds{};
    for (std::size_t index = 0; index < speeds.size(); ++index)
    {
        speeds[index] = body.model->getRotorOutput(
            static_cast<msr::airlib::uint>(index)).speed;
    }
    return speeds;
}

std::array<double, 4> PhysicsWorld::MotorThrustsNewtons(
    RigidBodyHandle handle) const
{
    const Impl::Body& body = impl_->GetBody(handle);
    std::array<double, 4> thrusts{};
    const auto& parameters = body.parameters.getParams();
    const double airDensityRatio = body.environment.getState().air_density
        / parameters.rotor_params.air_density;
    for (std::size_t index = 0; index < thrusts.size(); ++index)
    {
        const auto output = body.model->getRotorOutput(
            static_cast<msr::airlib::uint>(index));
        thrusts[index] = static_cast<double>(output.thrust) * airDensityRatio;
    }
    return thrusts;
}

PhysicsDynamicsSample PhysicsWorld::DynamicsSample(RigidBodyHandle handle) const
{
    const Impl::Body& body = impl_->GetBody(handle);
    const auto& kinematics = body.kinematics.getState();
    const auto& gravity = body.environment.getState().gravity;
    return {
        ToProjectWorldVector(body.model->getWrench().force),
        ToProjectWorldVector(gravity),
        ToProjectWorldVector(kinematics.accelerations.linear)};
}
}
