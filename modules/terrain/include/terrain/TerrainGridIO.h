#pragma once

#include "terrain/TerrainGrid.h"

#include <filesystem>

namespace dronesim::terrain
{
/** @brief 저장한 지형 격자를 읽는다. 파일 형식이나 높이 자료가 잘못되면 이유를 알린다. */
[[nodiscard]] TerrainGrid LoadTerrainGrid(const std::filesystem::path& path);

/** @brief ESRI ASCII 격자를 읽고 지정한 기준 높이를 뺀 지형 격자를 만든다. */
[[nodiscard]] TerrainGrid LoadAsciiTerrainGrid(
    const std::filesystem::path& path, float referenceElevationMeters);

/** @brief 지형 격자를 소수점 높이를 보존하는 저장 형식으로 기록한다. */
void SaveTerrainGrid(const TerrainGrid& terrain,
    const std::filesystem::path& path);
}
