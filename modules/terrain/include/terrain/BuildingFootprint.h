#pragma once

#include "terrain/TerrainGrid.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <glm/vec2.hpp>

namespace aerosimlab::terrain
{
/** @brief 지도 중심 기준 미터 좌표로 바꾼 OpenStreetMap 건물 외곽선이다. */
struct BuildingFootprint
{
    std::string osmId;
    std::string buildingType;
    std::vector<glm::dvec2> outlineMeters;
    std::optional<float> heightMeters;
};

/**
 * @brief GeoJSON에 저장한 건물 외곽선을 지형 격자와 같은 로컬 좌표로 읽는다.
 * @details 높이가 없는 건물에 임의의 높이를 넣지 않는다. 화면·충돌 쪽에서 필요한 경우 별도 가정을 정한다.
 */
[[nodiscard]] std::vector<BuildingFootprint> LoadBuildingFootprints(
    const std::filesystem::path& path, const TerrainGrid& terrain);
}
