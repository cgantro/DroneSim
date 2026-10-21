#pragma once

#include <nlohmann/json_fwd.hpp>

#include <vector>

namespace aerosimlab::simulator
{
/** @brief 실행 시간 표본에서 p50·p95·p99와 표본 수를 계산한다. */
nlohmann::json SummarizeDurations(std::vector<double> durations);

#ifndef __EMSCRIPTEN__
/** @brief 화면 없이 1·2·4·8대 강체 모의를 재고 실행별 결과를 돌려준다. */
nlohmann::json MeasureDroneScaling();
#endif
}
