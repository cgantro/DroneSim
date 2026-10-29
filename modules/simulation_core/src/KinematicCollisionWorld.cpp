#include "simulation_core/KinematicCollisionWorld.h"

#include "CollisionGeometry.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace aerosimlab::simulation
{
namespace
{
void ResolvePlanarContact(DroneState& state, DroneControlState& control,
    const glm::dvec2& outwardNormal, double penetrationMeters)
{
    const glm::dvec2 correction = outwardNormal * penetrationMeters;
    state.positionMeters.x += correction.x;
    state.positionMeters.y += correction.y;

    glm::dvec2 velocity{
        state.velocityMetersPerSecond.x, state.velocityMetersPerSecond.y};
    const double inwardSpeed = glm::dot(velocity, outwardNormal);
    if (inwardSpeed < 0.0)
        velocity -= inwardSpeed * outwardNormal;
    state.velocityMetersPerSecond.x = velocity.x;
    state.velocityMetersPerSecond.y = velocity.y;

    glm::dvec2 target{
        control.targetVelocityMetersPerSecond.x,
        control.targetVelocityMetersPerSecond.y};
    const double inwardTargetSpeed = glm::dot(target, outwardNormal);
    if (inwardTargetSpeed < 0.0)
        target -= inwardTargetSpeed * outwardNormal;
    control.targetVelocityMetersPerSecond.x = target.x;
    control.targetVelocityMetersPerSecond.y = target.y;
}

bool HasFiniteVector(const glm::dvec3& value) noexcept
{
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}

bool HasFiniteVector(const glm::vec3& value) noexcept
{
    return std::isfinite(value.x)
        && std::isfinite(value.y)
        && std::isfinite(value.z);
}
}

void KinematicCollisionWorld::SetTerrain(terrain::TerrainGrid terrain)
{
    terrain_ = std::move(terrain);
}

void KinematicCollisionWorld::AddStaticObstacleBox(
    const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters)
{
    if (!HasFiniteVector(centerMeters)
        || !HasFiniteVector(halfExtentsMeters)
        || halfExtentsMeters.x <= 0.0F || halfExtentsMeters.y <= 0.0F
        || halfExtentsMeters.z <= 0.0F)
    {
        throw std::invalid_argument("장애물 위치와 크기를 확인해 주세요.");
    }

    boxes_.push_back({centerMeters, glm::dvec3{halfExtentsMeters}});
}

void KinematicCollisionWorld::AddStaticObstaclePrism(
    const std::vector<glm::dvec2>& outlineMeters,
    double bottomMeters, double topMeters)
{
    if (!detail::IsValidFootprint(outlineMeters) || !std::isfinite(bottomMeters)
        || !std::isfinite(topMeters) || !(topMeters > bottomMeters))
    {
        throw std::invalid_argument("건물 외곽선과 높이 범위를 확인해 주세요.");
    }

    prisms_.push_back({outlineMeters, bottomMeters, topMeters});
}

void KinematicCollisionWorld::AddStaticObstacleFootprint(
    const std::vector<glm::dvec2>& outlineMeters)
{
    if (!detail::IsValidFootprint(outlineMeters))
        throw std::invalid_argument("구조물 외곽선 좌표를 확인해 주세요.");

    footprints_.push_back({outlineMeters});
}

void KinematicCollisionWorld::Resolve(DroneState& state,
    DroneControlState& control, const glm::dvec3& previousPositionMeters) const
{
    if (terrain_)
    {
        constexpr double droneHalfWidthMeters = 0.30;
        const double maximumCenterX = std::max(
            0.0, terrain_->HalfWidthMeters() - droneHalfWidthMeters);
        const double maximumCenterY = std::max(
            0.0, terrain_->HalfDepthMeters() - droneHalfWidthMeters);
        const auto clampAxis = [](
            double& position, double maximum, double& velocity, double& targetVelocity)
        {
            if (position > maximum)
            {
                position = maximum;
                if (velocity > 0.0)
                    velocity = 0.0;
                if (targetVelocity > 0.0)
                    targetVelocity = 0.0;
            }
            else if (position < -maximum)
            {
                position = -maximum;
                if (velocity < 0.0)
                    velocity = 0.0;
                if (targetVelocity < 0.0)
                    targetVelocity = 0.0;
            }
        };
        clampAxis(state.positionMeters.x, maximumCenterX,
            state.velocityMetersPerSecond.x,
            control.targetVelocityMetersPerSecond.x);
        clampAxis(state.positionMeters.y, maximumCenterY,
            state.velocityMetersPerSecond.y,
            control.targetVelocityMetersPerSecond.y);
    }

    ResolveTerrainContact(state, control);
    ResolveBoxContacts(state, control, previousPositionMeters);
    ResolvePrismContacts(state, control);
    ResolveFootprintContacts(state, control);
}

void KinematicCollisionWorld::ResolveTerrainContact(
    DroneState& state, DroneControlState& control) const
{
    if (!terrain_ || !terrain_->ContainsLocalPositionMeters(
            {state.positionMeters.x, state.positionMeters.y}))
        return;

    constexpr double droneHalfWidthMeters = 0.30;
    constexpr double droneHalfHeightMeters = 0.12;
    const glm::dvec2 dronePosition{state.positionMeters.x, state.positionMeters.y};
    control.groundElevationMeters = terrain_->HeightAtLocalMeters(dronePosition);

    double highestGroundMeters = control.groundElevationMeters;
    // 경사면에서도 동체 네 모서리가 지면 아래로 들어가지 않도록 높이를 확인한다.
    constexpr glm::dvec2 footprintOffsets[] = {
        {-droneHalfWidthMeters, -droneHalfWidthMeters},
        {-droneHalfWidthMeters, droneHalfWidthMeters},
        {droneHalfWidthMeters, -droneHalfWidthMeters},
        {droneHalfWidthMeters, droneHalfWidthMeters}};
    for (const glm::dvec2& offset : footprintOffsets)
    {
        const glm::dvec2 sample = dronePosition + offset;
        if (terrain_->ContainsLocalPositionMeters(sample))
            highestGroundMeters = std::max(highestGroundMeters,
                static_cast<double>(terrain_->HeightAtLocalMeters(sample)));
    }

    const double minimumCenterHeightMeters =
        highestGroundMeters + droneHalfHeightMeters;
    if (state.positionMeters.z >= minimumCenterHeightMeters)
        return;

    state.positionMeters.z = minimumCenterHeightMeters;
    if (state.velocityMetersPerSecond.z < 0.0)
        state.velocityMetersPerSecond.z = 0.0;
    if (control.targetVelocityMetersPerSecond.z < 0.0)
        control.targetVelocityMetersPerSecond.z = 0.0;
    if (state.mode == FlightMode::Landing)
    {
        // 지면 접촉으로 중심이 떠오른 착륙 기체는 멈춰 세워 착륙 완료로 바꾼다.
        state.velocityMetersPerSecond = glm::dvec3{0.0};
        control.targetVelocityMetersPerSecond = glm::dvec3{0.0};
        control.commandedYawRateRadiansPerSecond = 0.0;
        state.mode = FlightMode::Grounded;
    }
}

void KinematicCollisionWorld::ResolveBoxContacts(DroneState& state,
    DroneControlState& control, const glm::dvec3& previousPositionMeters) const
{
    constexpr glm::dvec3 droneHalfExtentsMeters{0.30, 0.30, 0.12};
    for (const Box& obstacle : boxes_)
    {
        const glm::dvec3 offset = state.positionMeters - obstacle.centerMeters;
        const glm::dvec3 overlap = obstacle.halfExtentsMeters
            + droneHalfExtentsMeters - glm::abs(offset);
        if (overlap.x <= 0.0 || overlap.y <= 0.0 || overlap.z <= 0.0)
            continue;

        glm::length_t axis = 0;
        if (overlap.y < overlap[axis])
            axis = 1;
        if (overlap.z < overlap[axis])
            axis = 2;

        const double previousSide = previousPositionMeters[axis]
            - obstacle.centerMeters[axis];
        const double currentSide = offset[axis];
        const double side = previousSide != 0.0
            ? std::copysign(1.0, previousSide)
            : (currentSide != 0.0 ? std::copysign(1.0, currentSide) : 1.0);
        state.positionMeters[axis] = obstacle.centerMeters[axis]
            + side * (obstacle.halfExtentsMeters[axis]
                + droneHalfExtentsMeters[axis]);

        if (state.velocityMetersPerSecond[axis] * side < 0.0)
            state.velocityMetersPerSecond[axis] = 0.0;
        if (control.targetVelocityMetersPerSecond[axis] * side < 0.0)
            control.targetVelocityMetersPerSecond[axis] = 0.0;
    }
}

void KinematicCollisionWorld::ResolvePrismContacts(
    DroneState& state, DroneControlState& control) const
{
    constexpr double droneRadiusMeters = 0.4242640687119285;
    constexpr double droneHalfHeightMeters = 0.12;
    for (const Prism& obstacle : prisms_)
    {
        const auto contact = detail::FindPrismContact(
            obstacle.outlineMeters, obstacle.bottomMeters, obstacle.topMeters,
            state.positionMeters, droneRadiusMeters, droneHalfHeightMeters);
        if (!contact)
            continue;

        if (std::abs(contact->normal.z) > 0.5)
        {
            state.positionMeters.z += contact->normal.z * contact->penetrationMeters;
            if (state.velocityMetersPerSecond.z * contact->normal.z < 0.0)
                state.velocityMetersPerSecond.z = 0.0;
            if (control.targetVelocityMetersPerSecond.z * contact->normal.z < 0.0)
                control.targetVelocityMetersPerSecond.z = 0.0;
            continue;
        }

        const glm::dvec2 normal{contact->normal.x, contact->normal.y};
        ResolvePlanarContact(state, control, normal, contact->penetrationMeters);
    }
}

void KinematicCollisionWorld::ResolveFootprintContacts(
    DroneState& state, DroneControlState& control) const
{
    constexpr double droneRadiusMeters = 0.4242640687119285;
    for (const Footprint& obstacle : footprints_)
    {
        const auto contact = detail::FindFootprintContact(
            obstacle.outlineMeters,
            {state.positionMeters.x, state.positionMeters.y}, droneRadiusMeters);
        if (contact)
            ResolvePlanarContact(state, control,
                contact->normal, contact->penetrationMeters);
    }
}
}
