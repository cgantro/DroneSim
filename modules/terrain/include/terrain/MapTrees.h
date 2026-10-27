#pragma once

#include "terrain/TerrainGrid.h"

#include <filesystem>
#include <vector>

#include <glm/vec2.hpp>

namespace aerosimlab::terrain
{
/** @brief 수치지도에 표시된 나무의 위치를 보관한다. 높이는 지형 격자에서 읽는다. */
struct MapTree
{
    glm::dvec2 positionMeters{0.0};
};

/** @brief 지도 자료에서 장면 범위 안의 나무 위치를 읽는다. */
[[nodiscard]] std::vector<MapTree> LoadMapTrees(
    const std::filesystem::path& path, const TerrainGrid& terrain);
}
