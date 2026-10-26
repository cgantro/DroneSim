#pragma once

#include "simulation_core/EoCameraModel.h"

#include <string_view>

namespace aerosimlab::simulation
{
/**
 * @brief 점검면 조건을 담은 JSON 객체를 검사해 시뮬레이션 자료로 바꾼다.
 * @details 법선과 가로 방향은 서로 직각인 단위 벡터로 정리한다. 크기, 거리, 각도와 영상 크기가 잘못되면 예외를 던진다.
 */
[[nodiscard]] InspectionSurface ParseInspectionSurfaceJson(std::string_view jsonText);
}
