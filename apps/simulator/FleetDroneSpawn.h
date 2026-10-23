#pragma once

#include "terrain/TerrainGrid.h"

#include <glm/vec3.hpp>

#include <cstddef>
#include <optional>

namespace aerosimlab::simulator::detail
{
/** @brief 지형 안에 놓을 수 있는 새 드론의 위치와 그 지점의 지면 높이다. */
struct FleetDroneSpawnPose
{
    glm::dvec3 positionMeters;
    double groundElevationMeters;
};

/** @brief 주 드론 주변의 정해진 자리에 새 드론을 놓을 수 있는지 계산한다. */
[[nodiscard]] std::optional<FleetDroneSpawnPose> FindFleetDroneSpawnPose(
    const terrain::TerrainGrid& terrain,
    const glm::dvec3& primaryPositionMeters,
    std::size_t slot);
}
