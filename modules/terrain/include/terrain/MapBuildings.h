#pragma once

#include "terrain/TerrainGrid.h"

#include <filesystem>
#include <string>
#include <vector>

#include <glm/vec2.hpp>

namespace aerosimlab::terrain
{
/** @brief 건물 높이가 어디서 왔는지 구분한다. */
enum class BuildingHeightSource
{
    OsmHeightTag,
    EstimatedFromLevels
};

/** @brief OSM 건물 외곽선과 지면 높이, 화면 표현용 높이를 보관한다. */
struct MapBuilding
{
    std::string osmId;
    std::vector<glm::dvec2> outlineMeters;
    std::vector<double> groundElevationsMeters;
    double heightMeters = 0.0;
    BuildingHeightSource heightSource = BuildingHeightSource::OsmHeightTag;
};

/**
 * @brief OSM GeoJSON에서 지형 격자 안의 높이 정보가 있는 건물을 읽는다.
 * @details `height`가 없고 층수만 있으면 층당 3m를 곱해 화면 높이를 추정한다.
 *          이 추정값은 측량값이나 충돌 검증 자료가 아니다.
 */
[[nodiscard]] std::vector<MapBuilding> LoadMapBuildings(
    const std::filesystem::path& path, const TerrainGrid& terrain);
}
