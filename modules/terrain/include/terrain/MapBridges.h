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

/** @brief 교량 하부에 그릴 임시 교각의 발자국과 높이다. */
struct MapBridgePier
{
    std::vector<glm::dvec2> outlineMeters;
    double bottomElevationMeters = 0.0;
    double topElevationMeters = 0.0;
};

/**
 * @brief OSM GeoJSON에서 지형 격자 안에 있는 교량 다각형을 읽는다.
 * @details DEM 경계에 걸친 외곽선은 격자 안쪽만 남긴다. 반환 높이는 지면 높이이며 교량 상판이나 충돌 형상을 뜻하지 않는다.
 */
[[nodiscard]] std::vector<MapBridgeFootprint> LoadMapBridgeFootprints(
    const std::filesystem::path& path, const TerrainGrid& terrain);

/**
 * @brief 교각 좌표 자료가 없는 교량에 장면 확인용 교각을 배치한다.
 * @details 45m 안팎 간격의 두 기둥 지지대를 만들며, 이 형상은 실측 교각 배치가 아니다.
 *          렌더링과 충돌 검사에서 같은 반환값을 사용한다.
 */
[[nodiscard]] std::vector<MapBridgePier> EstimateMapBridgePiers(
    const MapBridgeFootprint& bridge,
    const TerrainGrid& terrain,
    double deckBottomElevationMeters);
}
