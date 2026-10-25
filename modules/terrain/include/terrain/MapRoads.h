#pragma once

#include "terrain/TerrainGrid.h"

#include <filesystem>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

namespace aerosimlab::terrain
{
/** @brief 도로 폭을 OSM 값에서 읽었는지, 태그로 추정했는지 구분한다. */
enum class RoadWidthSource
{
    OsmWidthTag,
    EstimatedFromLanes,
    EstimatedFromRoadType
};

/** @brief OSM 도로 선형과 DEM 높이, 화면 표현용 폭을 보관한다. */
struct MapRoad
{
    std::string osmId;
    std::string name;
    std::vector<glm::dvec3> centerlineMeters;
    double widthMeters = 0.0;
    RoadWidthSource widthSource = RoadWidthSource::EstimatedFromRoadType;
    bool bridge = false;
};

/**
 * @brief OSM GeoJSON에서 지형 격자 안의 도로 선형을 읽는다.
 * @details 도로 폭이 없으면 차로 수 또는 도로 분류로 화면 폭을 추정한다.
 *          추정 폭과 DEM 높이는 시각화용이며, 도로·교량의 충돌 형상이나 상판 높이가 아니다.
 */
[[nodiscard]] std::vector<MapRoad> LoadMapRoads(
    const std::filesystem::path& path, const TerrainGrid& terrain);
}
