#pragma once

#include "terrain/TerrainGrid.h"

#include <filesystem>
#include <vector>

#include <glm/vec2.hpp>

namespace aerosimlab::terrain
{
/** @brief 물 영역의 바깥 경계와 섬처럼 물이 아닌 안쪽 경계를 보관한다. */
struct MapWaterBody
{
    std::vector<glm::dvec2> outerBoundaryMeters;
    std::vector<std::vector<glm::dvec2>> innerBoundariesMeters;
};

/** @brief OSM GeoJSON에서 지형 격자에 걸친 물 영역을 읽는다. */
[[nodiscard]] std::vector<MapWaterBody> LoadMapWaterBodies(const std::filesystem::path& path,
                                                           const TerrainGrid& terrain);

/** @brief 점이 물 영역 안에 있고 섬 경계 밖인지 확인한다. */
[[nodiscard]] bool ContainsWater(const MapWaterBody& water, const glm::dvec2& pointMeters) noexcept;
} // namespace aerosimlab::terrain
