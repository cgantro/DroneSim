#pragma once

#include "terrain/TerrainGrid.h"

#include <filesystem>
#include <string>
#include <vector>

#include <glm/vec2.hpp>

namespace aerosimlab::terrain
{
/** @brief OSM 교량 이름과 DEM 범위 안으로 자른 지면 투영 외곽선을 보관한다. */
struct MapBridgeFootprint
{
    std::string osmId;
    std::string name;
    std::vector<glm::dvec2> outlineMeters;
    std::vector<double> groundElevationsMeters;
};

/**
 * @brief OSM GeoJSON에서 지형 격자 안에 있는 교량 다각형을 읽는다.
 * @details DEM 경계에 걸친 외곽선은 격자 안쪽만 남긴다. 반환 높이는 지면 높이이며 교량 상판이나 충돌 형상을 뜻하지 않는다.
 */
[[nodiscard]] std::vector<MapBridgeFootprint> LoadMapBridgeFootprints(
    const std::filesystem::path& path, const TerrainGrid& terrain);
}
