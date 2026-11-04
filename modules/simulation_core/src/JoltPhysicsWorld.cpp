#include "simulation_core/JoltPhysicsWorld.h"
#include "simulation_core/CollisionValidation.h"
#include "simulation_core/DroneModel.h"
#include "geometry/VectorMath.h"
#include "simulation_core/QuadrotorModel.h"
#include "geometry/Polygon.h"
#include "terrain/TerrainGrid.h"

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Geometry/Triangle.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/MassProperties.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace dronesim::simulation
{
namespace
{
using JoltObjectLayer = JPH::ObjectLayer;

constexpr JoltObjectLayer nonMovingLayer = 0;
constexpr JoltObjectLayer movingLayer = 1;
constexpr JPH::uint objectLayerCount = 2;
constexpr JPH::uint broadPhaseLayerCount = 2;
constexpr JPH::uint maximumBodyCount = 65536;
constexpr JPH::uint maximumBodyPairCount = 65536;
constexpr JPH::uint maximumContactCount = 10240;
constexpr double gravityMetersPerSecondSquared = 9.81;

/** @brief Jolt의 전역 형식 등록을 프로세스에서 한 번만 관리한다. */
class JoltRuntime final
{
public:
    JoltRuntime()
    {
        JPH::RegisterDefaultAllocator();
        factory_ = std::make_unique<JPH::Factory>();
        JPH::Factory::sInstance = factory_.get();
        JPH::RegisterTypes();
    }

    ~JoltRuntime()
    {
        JPH::UnregisterTypes();
        JPH::Factory::sInstance = nullptr;
    }

    JoltRuntime(const JoltRuntime &) = delete;
    JoltRuntime &operator=(const JoltRuntime &) = delete;

private:
    std::unique_ptr<JPH::Factory> factory_;
};

JoltRuntime &EnsureJoltRuntime()
{
    static JoltRuntime runtime;
    return runtime;
}

bool IsValidOrientation(const glm::dquat &value)
{
    const double lengthSquared = glm::dot(value, value);
    return std::isfinite(value.w) && std::isfinite(value.x) && std::isfinite(value.y) &&
           std::isfinite(value.z) && std::isfinite(lengthSquared) && lengthSquared > 0.0;
}

float ToJoltFloat(double value)
{
    constexpr double maximum = std::numeric_limits<float>::max();
    if (!std::isfinite(value) || value < -maximum || value > maximum)
        throw std::invalid_argument("물리값이 Jolt에서 표현할 수 있는 범위를 벗어났습니다.");
    return static_cast<float>(value);
}

float ToJoltPositiveFloat(double value)
{
    const float converted = ToJoltFloat(value);
    if (!(converted > 0.0F))
        throw std::invalid_argument("양수 물리값이 Jolt 변환 뒤 0이 됩니다.");
    return converted;
}

JPH::Vec3 ToJoltVector(const glm::dvec3 &enu)
{
    // ENU의 북쪽 축을 Jolt의 -Z로 옮기면 오른손 좌표계를 유지할 수 있다.
    return {ToJoltFloat(enu.x), ToJoltFloat(enu.z), ToJoltFloat(-enu.y)};
}

JPH::Vec3 ToJoltHalfExtents(const glm::dvec3 &enuHalfExtents)
{
    // 반길이는 방향 부호가 필요 없으므로 ENU의 위·북 축 순서만 Jolt에 맞춘다.
    return {ToJoltPositiveFloat(enuHalfExtents.x), ToJoltPositiveFloat(enuHalfExtents.z),
            ToJoltPositiveFloat(enuHalfExtents.y)};
}

glm::dvec3 ToProjectVector(JPH::Vec3Arg jolt)
{
    return {jolt.GetX(), -jolt.GetZ(), jolt.GetY()};
}

const glm::dquat &ProjectToJoltRotation()
{
    // 벡터 변환 (동, 북, 위) → (동, 위, 남)은 X축 -90도 회전과 같다.
    static const glm::dquat rotation =
        glm::angleAxis(-glm::half_pi<double>(), glm::dvec3{1.0, 0.0, 0.0});
    return rotation;
}

JPH::Quat ToJoltRotation(const glm::dquat &enu)
{
    const glm::dquat normalized = glm::normalize(enu);
    // 좌표축 회전 C 아래에서 자세를 바꿀 때는 C q C⁻¹로 기저를 변환한다.
    const glm::dquat converted =
        ProjectToJoltRotation() * normalized * glm::conjugate(ProjectToJoltRotation());
    return {ToJoltFloat(converted.x), ToJoltFloat(converted.y), ToJoltFloat(converted.z),
            ToJoltFloat(converted.w)};
}

glm::dquat ToProjectRotation(JPH::QuatArg jolt)
{
    const glm::dquat converted{jolt.GetW(), jolt.GetX(), jolt.GetY(), jolt.GetZ()};
    // 위 변환을 되돌리기 위해 같은 식에 역방향 축 회전을 적용한다.
    return glm::normalize(glm::conjugate(ProjectToJoltRotation()) * converted *
                          ProjectToJoltRotation());
}

class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterface
{
public:
    // Jolt 물체 층 0은 고정물, 1은 움직이는 물체다. 아래 필터가 이 번호를 다시 사용한다.
    BroadPhaseLayers() : layers_{JPH::BroadPhaseLayer(0), JPH::BroadPhaseLayer(1)}
    {
    }

    JPH::uint GetNumBroadPhaseLayers() const override
    {
        return broadPhaseLayerCount;
    }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
    {
        return layers_.at(layer);
    }

private:
    std::array<JPH::BroadPhaseLayer, objectLayerCount> layers_;
};

// Jolt는 먼저 broad phase에서 충돌 후보를 거른 뒤, 아래 필터에서 실제 층 조합을 확인한다.
// 고정물끼리는 움직이지 않으므로 후보에서 빼고, 움직이는 물체는 고정물과 움직이는 물체 모두와
// 비교한다.
class ObjectVsBroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer objectLayer,
                       JPH::BroadPhaseLayer broadPhaseLayer) const override
    {
        if (objectLayer == nonMovingLayer)
            return broadPhaseLayer == JPH::BroadPhaseLayer(1);
        return objectLayer == movingLayer;
    }
};

// broad phase를 통과한 뒤에도 고정물끼리의 충돌은 만들지 않는다.
class ObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
{
public:
    bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override
    {
        return first == movingLayer || second == movingLayer;
    }
};

JPH::RefConst<JPH::Shape> MakeBoxShape(const glm::dvec3 &halfExtentsMeters)
{
    return new JPH::BoxShape(ToJoltHalfExtents(halfExtentsMeters));
}

JPH::RefConst<JPH::Shape> MakeQuadrotorShape(const QuadrotorParameters &parameters)
{
    JPH::StaticCompoundShapeSettings compound;
    // 여러 상자를 기체 중심 기준 위치에 붙여 본체와 다리의 충돌 모양을 하나의 강체로 묶는다.
    for (const QuadrotorCollisionBox &box : parameters.collisionBoxes)
    {
        if (!collision::IsValidObstacleBox(box.positionMeters, glm::dvec3{box.halfExtentsMeters}) ||
            !IsValidOrientation(box.orientation))
            throw std::invalid_argument("드론 충돌 상자의 크기와 자세를 확인해 주세요.");

        const JPH::RefConst<JPH::Shape> shape = MakeBoxShape(glm::dvec3{box.halfExtentsMeters});
        compound.AddShape(ToJoltVector(box.positionMeters), ToJoltRotation(box.orientation),
                          shape.GetPtr());
    }

    auto shapeResult = compound.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(shapeResult.GetError().c_str());
    return shapeResult.Get();
}

JPH::Vec3 ToJoltPrismVertex(const glm::dvec2 &point, double elevationMeters)
{
    return ToJoltVector({point.x, point.y, elevationMeters});
}

JPH::TriangleList MakePrismTriangles(const std::vector<glm::dvec2> &outline, double bottomMeters,
                                     double topMeters)
{
    JPH::TriangleList triangles;
    const auto caps = geometry::TriangulateSimplePolygon(outline);
    // 외곽선의 회전 방향에 따라 꼭짓점 순서를 뒤집어, 윗면·아랫면의 법선이 바깥을 향하게 한다.
    const bool counterClockwise = geometry::SignedArea(outline) > 0.0;
    for (const auto &cap : caps)
    {
        const auto [a, b, c] = cap;
        const JPH::Vec3 bottomA = ToJoltPrismVertex(outline[a], bottomMeters);
        const JPH::Vec3 bottomB = ToJoltPrismVertex(outline[b], bottomMeters);
        const JPH::Vec3 bottomC = ToJoltPrismVertex(outline[c], bottomMeters);
        const JPH::Vec3 topA = ToJoltPrismVertex(outline[a], topMeters);
        const JPH::Vec3 topB = ToJoltPrismVertex(outline[b], topMeters);
        const JPH::Vec3 topC = ToJoltPrismVertex(outline[c], topMeters);
        if (counterClockwise)
        {
            triangles.emplace_back(topA, topC, topB);
            triangles.emplace_back(bottomA, bottomB, bottomC);
        }
        else
        {
            triangles.emplace_back(topA, topB, topC);
            triangles.emplace_back(bottomA, bottomC, bottomB);
        }
    }

    for (std::size_t index = 0; index < outline.size(); ++index)
    {
        const std::size_t next = (index + 1) % outline.size();
        const JPH::Vec3 bottomA = ToJoltPrismVertex(outline[index], bottomMeters);
        const JPH::Vec3 bottomB = ToJoltPrismVertex(outline[next], bottomMeters);
        const JPH::Vec3 topA = ToJoltPrismVertex(outline[index], topMeters);
        const JPH::Vec3 topB = ToJoltPrismVertex(outline[next], topMeters);
        if (counterClockwise)
        {
            triangles.emplace_back(bottomA, bottomB, topB);
            triangles.emplace_back(bottomA, topB, topA);
        }
        else
        {
            triangles.emplace_back(bottomA, topB, bottomB);
            triangles.emplace_back(bottomA, topA, topB);
        }
    }
    return triangles;
}
} // namespace

struct JoltPhysicsWorld::Impl
{
    struct ForceAtPosition
    {
        /** @brief 장착점에 전달할 힘 [N] */
        glm::dvec3 forceNewtons{0.0};
        /** @brief ENU 월드 좌표의 힘 적용 위치 [m] */
        glm::dvec3 positionMeters{0.0};
    };

    struct QuadrotorForces
    {
        glm::dvec3 dragForceWorldNewtons{0.0};
        std::array<ForceAtPosition, QuadrotorParameters::rotorCount> rotorForces;
        glm::dvec3 reactionTorqueWorldNewtonMeters{0.0};
    };

    struct QuadrotorActuation
    {
        QuadrotorParameters parameters;
        std::array<double, QuadrotorParameters::rotorCount> targetSpeeds{};
        std::array<double, QuadrotorParameters::rotorCount> motorSpeeds{};
        bool hasStepped = false;

        /**
         * @brief 한 고정 Tick만큼 모터를 움직이고 기체에 작용할 힘과 토크를 계산한다.
         * @details 바람과 선속도는 ENU 월드 기준으로 받고, 기체 축에서 정의된 항력은
         *          기체 자세로 월드 좌표로 바꾼다. 여기서는 물리 엔진을 호출하지 않고
         *          계산 결과만 돌려줘서 기체 모델과 Jolt API의 책임을 나눈다.
         */
        [[nodiscard]] QuadrotorForces AdvanceAndCalculateForces(
            const glm::dvec3 &positionMeters, const glm::dquat &orientation,
            const glm::dvec3 &linearVelocityMetersPerSecond,
            const glm::dvec3 &windVelocityMetersPerSecond)
        {
            const double stepSeconds = DroneModel::fixedStepSeconds;
            const glm::dvec3 relativeAirflowBody =
                glm::inverse(orientation) *
                (linearVelocityMetersPerSecond - windVelocityMetersPerSecond);
            // 항력은 땅 기준 속도가 아니라 바람을 뺀 공기 상대 속도로 계산한다.
            const glm::dvec3 dragForceBody =
                CalculateQuadrotorDragForceBody(parameters, relativeAirflowBody);
            QuadrotorForces forces;
            forces.dragForceWorldNewtons = orientation * dragForceBody;

            motorSpeeds = AdvanceMotorSpeeds(parameters, motorSpeeds, targetSpeeds, stepSeconds);
            const auto rotorThrusts = CalculateRotorThrusts(parameters, motorSpeeds);
            for (std::size_t rotor = 0; rotor < motorSpeeds.size(); ++rotor)
            {
                const double thrust = rotorThrusts[rotor];
                const glm::dvec3 forceWorld = orientation * glm::dvec3{0.0, 0.0, thrust};
                // 힘을 회전축 중심이 아닌 로터 장착점에 가해 팔 길이에 따른 회전도 만든다.
                const glm::dvec3 rotorPositionMeters =
                    positionMeters + orientation * parameters.rotors[rotor].positionMeters;
                forces.rotorForces[rotor] = {forceWorld, rotorPositionMeters};
            }

            // 프로펠러가 공기를 돌리는 방향과 반대쪽으로 기체에 요 반작용 토크를 가한다.
            const double reactionTorqueBodyZ =
                CalculateRotorReactionTorqueBodyZ(parameters, rotorThrusts);
            forces.reactionTorqueWorldNewtonMeters =
                orientation * glm::dvec3{0.0, 0.0, reactionTorqueBodyZ};
            hasStepped = true;
            return forces;
        }
    };

    struct BodyRecord
    {
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
    std::optional<JoltRigidBodyHandle> terrainBody;
    glm::dvec3 windVelocityMetersPerSecond{0.0};
    std::uint32_t nextHandle = 1;

    /** @brief 로터 모델이 계산한 힘과 토크를 Jolt 강체에 적용한다. */
    void ApplyForces(JPH::BodyInterface &bodyInterface, JPH::BodyID bodyId,
                     const QuadrotorForces &forces)
    {
        bodyInterface.AddForce(bodyId, ToJoltVector(forces.dragForceWorldNewtons));
        for (const ForceAtPosition &rotorForce : forces.rotorForces)
        {
            bodyInterface.AddForce(bodyId, ToJoltVector(rotorForce.forceNewtons),
                                   ToJoltVector(rotorForce.positionMeters));
        }
        bodyInterface.AddTorque(
            bodyId, ToJoltVector(forces.reactionTorqueWorldNewtonMeters));
    }

    Impl() : runtime(EnsureJoltRuntime())
    {
        static_cast<void>(runtime);
        // 도시 지도의 정적 건물 수용을 위해 Jolt 권장 규모로 여유를 둔다.
        physicsSystem.Init(maximumBodyCount, 0, maximumBodyPairCount, maximumContactCount,
                           broadPhaseLayers, objectVsBroadPhaseFilter, objectLayerPairFilter);
        physicsSystem.SetGravity({0.0F, -static_cast<float>(gravityMetersPerSecondSquared), 0.0F});
    }

    ~Impl()
    {
        JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
        for (const auto &[handle, record] : bodies)
        {
            static_cast<void>(handle);
            bodyInterface.RemoveBody(record.id);
            bodyInterface.DestroyBody(record.id);
        }
    }

    JoltRigidBodyHandle AddBody(const JPH::BodyCreationSettings &settings,
                                JPH::EActivation activation, bool isStatic,
                                std::optional<QuadrotorActuation> quadrotor = std::nullopt)
    {
        if (nextHandle == 0)
            throw std::overflow_error("강체 식별값을 더 만들 수 없습니다.");

        JPH::BodyInterface &bodyInterface = physicsSystem.GetBodyInterface();
        const JPH::BodyID id = bodyInterface.CreateAndAddBody(settings, activation);
        if (id.IsInvalid())
            throw std::runtime_error("Jolt 물리 월드에 강체를 만들지 못했습니다.");

        const std::uint32_t handle = nextHandle++;
        try
        {
            bodies.emplace(handle, BodyRecord{id, isStatic, std::move(quadrotor)});
        }
        catch (...)
        {
            bodyInterface.RemoveBody(id);
            bodyInterface.DestroyBody(id);
            throw;
        }
        return {handle};
    }

    /** @brief 이미 만든 형상을 지정 위치의 고정 충돌체로 월드에 등록한다. */
    JoltRigidBodyHandle AddStaticShape(
        const JPH::RefConst<JPH::Shape> &shape,
        const glm::dvec3 &positionMeters)
    {
        const JPH::BodyCreationSettings settings(
            shape.GetPtr(), ToJoltVector(positionMeters), JPH::Quat::sIdentity(),
            JPH::EMotionType::Static, nonMovingLayer);
        return AddBody(settings, JPH::EActivation::DontActivate, true);
    }

    BodyRecord &GetBody(JoltRigidBodyHandle handle)
    {
        const auto found = bodies.find(handle.value);
        if (found == bodies.end())
            throw std::out_of_range("물리 월드에서 강체를 찾을 수 없습니다.");
        return found->second;
    }

    const BodyRecord &GetBody(JoltRigidBodyHandle handle) const
    {
        const auto found = bodies.find(handle.value);
        if (found == bodies.end())
            throw std::out_of_range("물리 월드에서 강체를 찾을 수 없습니다.");
        return found->second;
    }

    QuadrotorActuation &GetQuadrotorActuation(JoltRigidBodyHandle handle)
    {
        BodyRecord &body = GetBody(handle);
        if (!body.quadrotor)
            throw std::logic_error("로터 입력을 받는 드론 강체가 아닙니다.");
        return *body.quadrotor;
    }

    const QuadrotorActuation &GetQuadrotorActuation(JoltRigidBodyHandle handle) const
    {
        const BodyRecord &body = GetBody(handle);
        if (!body.quadrotor)
            throw std::logic_error("로터 입력을 받는 드론 강체가 아닙니다.");
        return *body.quadrotor;
    }
};

JoltPhysicsWorld::JoltPhysicsWorld() : impl_(std::make_unique<Impl>())
{
}

JoltPhysicsWorld::~JoltPhysicsWorld() = default;

JoltRigidBodyHandle JoltPhysicsWorld::AddStaticBox(const glm::dvec3 &centerMeters,
                                                   const glm::dvec3 &halfExtentsMeters)
{
    if (!collision::IsValidObstacleBox(centerMeters, halfExtentsMeters))
        throw std::invalid_argument("고정 상자의 위치와 크기를 확인해 주세요.");

    const auto shape = MakeBoxShape(halfExtentsMeters);
    return impl_->AddStaticShape(shape, centerMeters);
}

void JoltPhysicsWorld::AddStaticTriangleMesh(std::span<const glm::dvec3> verticesMeters,
                                             std::span<const std::uint32_t> triangleIndices)
{
    if (verticesMeters.size() < 3 || triangleIndices.empty() || triangleIndices.size() % 3 != 0 ||
        verticesMeters.size() > std::numeric_limits<JPH::uint32>::max())
    {
        throw std::invalid_argument("장면 충돌 메시의 정점과 삼각형 자료를 확인해 주세요.");
    }

    JPH::VertexList vertices;
    vertices.reserve(verticesMeters.size());
    for (const glm::dvec3 &vertex : verticesMeters)
    {
        if (!geometry::IsFinite(vertex))
            throw std::invalid_argument("장면 충돌 메시의 정점에 잘못된 값이 있습니다.");
        const JPH::Vec3 converted = ToJoltVector(vertex);
        vertices.emplace_back(converted.GetX(), converted.GetY(), converted.GetZ());
    }

    JPH::IndexedTriangleList triangles;
    triangles.reserve(triangleIndices.size() / 3);
    for (std::size_t index = 0; index < triangleIndices.size(); index += 3)
    {
        const std::uint32_t first = triangleIndices[index];
        const std::uint32_t second = triangleIndices[index + 1];
        const std::uint32_t third = triangleIndices[index + 2];
        if (first >= vertices.size() || second >= vertices.size() || third >= vertices.size())
        {
            throw std::invalid_argument("장면 충돌 메시의 삼각형 번호가 정점 범위를 벗어났습니다.");
        }
        triangles.emplace_back(first, second, third, 0);
    }

    JPH::MeshShapeSettings shapeSettings(std::move(vertices), std::move(triangles));
    auto shapeResult = shapeSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(shapeResult.GetError().c_str());
    const JPH::RefConst<JPH::Shape> shape = shapeResult.Get();
    static_cast<void>(impl_->AddStaticShape(shape, glm::dvec3{0.0}));
}

void JoltPhysicsWorld::SetTerrain(const dronesim::terrain::TerrainGrid &terrain)
{
    const std::size_t expectedSamples =
        static_cast<std::size_t>(terrain.sampleCount) * terrain.sampleCount;
    if (terrain.sampleCount < 3 || terrain.heightOffsetsMeters.size() != expectedSamples ||
        !std::isfinite(terrain.spacingXMeters) || terrain.spacingXMeters <= 0.0F ||
        !std::isfinite(terrain.spacingYMeters) || terrain.spacingYMeters <= 0.0F ||
        !std::isfinite(terrain.referenceElevationMeters) ||
        !std::all_of(
            terrain.heightOffsetsMeters.begin(), terrain.heightOffsetsMeters.end(),
            [](float height) { return std::isfinite(height); }))
    {
        throw std::invalid_argument("지형 격자와 높이 값을 확인해 주세요.");
    }

    // 격자 표본은 기준 고도에 대한 높이 오프셋이다. 형상의 원점을 기준 고도에 두고,
    // X/Z 시작점을 지도 중심에서 반 격자 폭만큼 옮겨 표본이 실제 월드 범위에 놓이게 한다.
    JPH::HeightFieldShapeSettings shapeSettings(terrain.heightOffsetsMeters.data(),
                                                {ToJoltFloat(-terrain.HalfWidthMeters()),
                                                 ToJoltFloat(terrain.referenceElevationMeters),
                                                 ToJoltFloat(-terrain.HalfDepthMeters())},
                                                {ToJoltPositiveFloat(terrain.spacingXMeters), 1.0F,
                                                 ToJoltPositiveFloat(terrain.spacingYMeters)},
                                                terrain.sampleCount);
    auto shapeResult = shapeSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(shapeResult.GetError().c_str());
    const JPH::RefConst<JPH::Shape> shape = shapeResult.Get();
    const JoltRigidBodyHandle newTerrain =
        impl_->AddStaticShape(shape, glm::dvec3{0.0});
    if (impl_->terrainBody)
        static_cast<void>(DestroyBody(*impl_->terrainBody));
    impl_->terrainBody = newTerrain;
}

void JoltPhysicsWorld::AddStaticPrism(const std::vector<glm::dvec2> &outlineMeters,
                                      double bottomMeters, double topMeters)
{
    if (!collision::IsValidPrism(outlineMeters, bottomMeters, topMeters))
        throw std::invalid_argument("구조물 외곽선과 높이를 확인해 주세요.");

    JPH::MeshShapeSettings shapeSettings(
        MakePrismTriangles(outlineMeters, bottomMeters, topMeters));
    auto shapeResult = shapeSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(shapeResult.GetError().c_str());
    const JPH::RefConst<JPH::Shape> shape = shapeResult.Get();
    static_cast<void>(impl_->AddStaticShape(shape, glm::dvec3{0.0}));
}

void JoltPhysicsWorld::SetWindVelocityMetersPerSecond(const glm::dvec3 &velocity)
{
    if (!geometry::IsFinite(velocity))
        throw std::invalid_argument("바람 속도는 유한한 값이어야 합니다.");
    impl_->windVelocityMetersPerSecond = velocity;
}

JoltRigidBodyHandle JoltPhysicsWorld::CreateDynamicBox(const JoltBoxDefinition &definition)
{
    if (!IsValidOrientation(definition.orientation) ||
        !collision::IsValidObstacleBox(definition.positionMeters, definition.halfExtentsMeters) ||
        !std::isfinite(definition.massKilograms) || definition.massKilograms <= 0.0 ||
        !std::isfinite(definition.friction) || definition.friction < 0.0 ||
        !std::isfinite(definition.restitution) || definition.restitution < 0.0 ||
        definition.restitution > 1.0)
    {
        throw std::invalid_argument("움직이는 상자의 모양과 물리값을 확인해 주세요.");
    }

    const auto shape = MakeBoxShape(definition.halfExtentsMeters);
    JPH::BodyCreationSettings settings(shape.GetPtr(), ToJoltVector(definition.positionMeters),
                                       ToJoltRotation(definition.orientation),
                                       JPH::EMotionType::Dynamic, movingLayer);
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    settings.mMassPropertiesOverride.mMass = ToJoltPositiveFloat(definition.massKilograms);
    settings.mFriction = ToJoltFloat(definition.friction);
    settings.mRestitution = ToJoltFloat(definition.restitution);
    return impl_->AddBody(settings, JPH::EActivation::Activate, false);
}

JoltRigidBodyHandle JoltPhysicsWorld::CreateQuadrotor(const JoltRigidBodyDefinition &body,
                                                      const QuadrotorParameters &parameters)
{
    if (!geometry::IsFinite(body.positionMeters) || !IsValidOrientation(body.orientation) ||
        !std::isfinite(body.friction) || body.friction < 0.0 || !std::isfinite(body.restitution) ||
        body.restitution < 0.0 || body.restitution > 1.0)
    {
        throw std::invalid_argument("드론의 시작 상태와 표면 물리값을 확인해 주세요.");
    }
    if (!parameters.HasValidPhysicalProperties())
    {
        throw std::invalid_argument("드론의 크기·질량·관성·로터 물리 제원을 확인해 주세요.");
    }
    if (!std::all_of(parameters.rotors.begin(), parameters.rotors.end(),
                     [](const RotorDefinition &rotor)
                     { return geometry::IsFinite(rotor.positionMeters); }))
    {
        throw std::invalid_argument("로터 위치를 확인해 주세요.");
    }

    const auto shape = MakeQuadrotorShape(parameters);
    JPH::BodyCreationSettings settings(shape.GetPtr(), ToJoltVector(body.positionMeters),
                                       ToJoltRotation(body.orientation), JPH::EMotionType::Dynamic,
                                       movingLayer);
    settings.mOverrideMassProperties = JPH::EOverrideMassProperties::MassAndInertiaProvided;
    settings.mMassPropertiesOverride.mMass = ToJoltPositiveFloat(parameters.massKilograms);
    // 주 관성값은 기체 중심축의 회전 저항이다. Jolt 축 순서에 맞춰 북·위 항목을 바꿔 넣는다.
    settings.mMassPropertiesOverride.mInertia = JPH::Mat44::sScale(
        JPH::Vec3(ToJoltPositiveFloat(parameters.principalInertiaKilogramMetersSquared.x),
                  ToJoltPositiveFloat(parameters.principalInertiaKilogramMetersSquared.z),
                  ToJoltPositiveFloat(parameters.principalInertiaKilogramMetersSquared.y)));
    settings.mFriction = ToJoltFloat(body.friction);
    settings.mRestitution = ToJoltFloat(body.restitution);

    Impl::QuadrotorActuation actuation;
    actuation.parameters = parameters;
    return impl_->AddBody(settings, JPH::EActivation::Activate, false, std::move(actuation));
}

void JoltPhysicsWorld::SetMotorTargets(JoltRigidBodyHandle handle,
                                       const std::array<double, 4> &speedRadiansPerSecond)
{
    auto &actuation = impl_->GetQuadrotorActuation(handle);
    const double maximumSpeed = actuation.parameters.maximumMotorSpeedRadiansPerSecond;
    for (std::size_t rotor = 0; rotor < speedRadiansPerSecond.size(); ++rotor)
    {
        if (!std::isfinite(speedRadiansPerSecond[rotor]))
            throw std::invalid_argument("로터 목표 속도는 유한한 값이어야 합니다.");
        actuation.targetSpeeds[rotor] =
            std::clamp(speedRadiansPerSecond[rotor], 0.0, maximumSpeed);
    }
}

void JoltPhysicsWorld::SetInitialMotorSpeeds(JoltRigidBodyHandle handle,
                                             const std::array<double, 4> &speedRadiansPerSecond)
{
    auto &actuation = impl_->GetQuadrotorActuation(handle);
    if (actuation.hasStepped)
        throw std::logic_error("초기 로터 속도는 첫 물리 단계 전에만 설정할 수 있습니다.");

    const double maximumSpeed = actuation.parameters.maximumMotorSpeedRadiansPerSecond;
    for (double speed : speedRadiansPerSecond)
    {
        if (!std::isfinite(speed) || speed < 0.0 || speed > maximumSpeed)
            throw std::invalid_argument("초기 로터 속도를 허용 범위 안에서 지정해 주세요.");
    }
    actuation.motorSpeeds = speedRadiansPerSecond;
    actuation.targetSpeeds = speedRadiansPerSecond;
}

std::array<double, 4>
JoltPhysicsWorld::MotorSpeedsRadiansPerSecond(JoltRigidBodyHandle handle) const
{
    return impl_->GetQuadrotorActuation(handle).motorSpeeds;
}

std::array<double, 4> JoltPhysicsWorld::MotorThrustsNewtons(JoltRigidBodyHandle handle) const
{
    const auto &actuation = impl_->GetQuadrotorActuation(handle);
    return CalculateRotorThrusts(actuation.parameters, actuation.motorSpeeds);
}

void JoltPhysicsWorld::ApplyForce(JoltRigidBodyHandle handle, const glm::dvec3 &forceNewtons)
{
    if (!geometry::IsFinite(forceNewtons))
        throw std::invalid_argument("강체에 가할 힘은 유한한 값이어야 합니다.");
    const auto &body = impl_->GetBody(handle);
    if (body.isStatic)
        throw std::logic_error("고정된 강체에는 힘을 가할 수 없습니다.");
    impl_->physicsSystem.GetBodyInterface().AddForce(body.id, ToJoltVector(forceNewtons));
}

void JoltPhysicsWorld::ApplyTorque(JoltRigidBodyHandle handle, const glm::dvec3 &torqueNewtonMeters)
{
    if (!geometry::IsFinite(torqueNewtonMeters))
        throw std::invalid_argument("강체에 가할 토크는 유한한 값이어야 합니다.");
    const auto &body = impl_->GetBody(handle);
    if (body.isStatic)
        throw std::logic_error("고정된 강체에는 토크를 가할 수 없습니다.");
    impl_->physicsSystem.GetBodyInterface().AddTorque(body.id, ToJoltVector(torqueNewtonMeters));
}

void JoltPhysicsWorld::SetState(JoltRigidBodyHandle handle, const JoltRigidBodyState &state)
{
    if (!geometry::IsFinite(state.positionMeters) || !IsValidOrientation(state.orientation) ||
        !geometry::IsFinite(state.linearVelocityMetersPerSecond) ||
        !geometry::IsFinite(state.angularVelocityRadiansPerSecond))
    {
        throw std::invalid_argument("강체 상태는 유한한 값과 올바른 자세여야 합니다.");
    }
    auto &body = impl_->GetBody(handle);
    if (body.isStatic)
        throw std::logic_error("고정된 강체의 상태는 바꿀 수 없습니다.");

    JPH::BodyInterface &bodyInterface = impl_->physicsSystem.GetBodyInterface();
    const glm::dquat orientation = glm::normalize(state.orientation);
    bodyInterface.SetPositionAndRotation(body.id, ToJoltVector(state.positionMeters),
                                         ToJoltRotation(orientation), JPH::EActivation::Activate);
    bodyInterface.SetLinearVelocity(body.id, ToJoltVector(state.linearVelocityMetersPerSecond));
    // 공개 상태는 각속도를 기체 축(FLU)으로 받는다. Jolt는 월드 축 값이 필요하므로
    // 자세 회전을 곱해 바꾼다. 읽어올 때는 반대 회전으로 같은 변환을 되돌린다.
    const glm::dvec3 angularVelocityWorld = orientation * state.angularVelocityRadiansPerSecond;
    bodyInterface.SetAngularVelocity(body.id, ToJoltVector(angularVelocityWorld));
    if (body.quadrotor)
    {
        body.quadrotor->targetSpeeds.fill(0.0);
        body.quadrotor->motorSpeeds.fill(0.0);
        body.quadrotor->hasStepped = false;
    }
}

void JoltPhysicsWorld::StepFixed(double *stepDurationMicroseconds)
{
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    if (stepDurationMicroseconds)
        *stepDurationMicroseconds = 0.0;

    JPH::BodyInterface &bodyInterface = impl_->physicsSystem.GetBodyInterface();
    // 적분 전 상태에서 바람·모터 힘을 계산해야 이번 Tick의 위치와 자세에 맞는 힘이 적용된다.
    for (auto &[handle, body] : impl_->bodies)
    {
        static_cast<void>(handle);
        if (!body.quadrotor)
            continue;

        auto &actuation = *body.quadrotor;
        JPH::RVec3 bodyPosition;
        JPH::Quat bodyRotation;
        bodyInterface.GetPositionAndRotation(body.id, bodyPosition, bodyRotation);
        const glm::dvec3 position = ToProjectVector(bodyPosition);
        const glm::dquat orientation = ToProjectRotation(bodyRotation);
        const JPH::Vec3 joltLinearVelocity = bodyInterface.GetLinearVelocity(body.id);
        const glm::dvec3 linearVelocity = ToProjectVector(joltLinearVelocity);
        const Impl::QuadrotorForces forces = actuation.AdvanceAndCalculateForces(
            position, orientation, linearVelocity,
            impl_->windVelocityMetersPerSecond);
        impl_->ApplyForces(bodyInterface, body.id, forces);
    }
    // 네 로터 힘을 모두 등록한 뒤 고정 간격 한 번만 적분해 기체 상태를 함께 갱신한다.
    impl_->physicsSystem.Update(static_cast<float>(DroneModel::fixedStepSeconds), 1,
                                &impl_->tempAllocator, &impl_->jobSystem);
    if (stepDurationMicroseconds)
    {
        *stepDurationMicroseconds =
            std::chrono::duration<double, std::micro>(Clock::now() - start).count();
    }
}

JoltRigidBodyState JoltPhysicsWorld::State(JoltRigidBodyHandle handle) const
{
    const auto &body = impl_->GetBody(handle);
    JPH::RVec3 position;
    JPH::Quat orientation;
    impl_->physicsSystem.GetBodyInterface().GetPositionAndRotation(body.id, position, orientation);
    JPH::Vec3 linearVelocity;
    JPH::Vec3 angularVelocity;
    impl_->physicsSystem.GetBodyInterface().GetLinearAndAngularVelocity(body.id, linearVelocity,
                                                                        angularVelocity);
    const glm::dquat projectOrientation = ToProjectRotation(orientation);
    // 위치·선속도는 월드 좌표로 반환하지만, 각속도는 DroneState와 맞춰 기체 축으로 돌린다.
    return {ToProjectVector(position), projectOrientation, ToProjectVector(linearVelocity),
            glm::inverse(projectOrientation) * ToProjectVector(angularVelocity)};
}

bool JoltPhysicsWorld::DestroyBody(JoltRigidBodyHandle handle)
{
    const auto found = impl_->bodies.find(handle.value);
    if (found == impl_->bodies.end())
        return false;

    JPH::BodyInterface &bodyInterface = impl_->physicsSystem.GetBodyInterface();
    bodyInterface.RemoveBody(found->second.id);
    bodyInterface.DestroyBody(found->second.id);
    impl_->bodies.erase(found);
    return true;
}
} // namespace dronesim::simulation
