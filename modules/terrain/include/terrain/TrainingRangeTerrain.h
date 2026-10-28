#pragma once

#include "terrain/TerrainGrid.h"

namespace aerosimlab::terrain
{
/** @brief 산업·산악 군집 시험장에 사용할 500m × 400m 높이 격자를 만든다. */
[[nodiscard]] TerrainGrid CreateTrainingRangeTerrain();
}
