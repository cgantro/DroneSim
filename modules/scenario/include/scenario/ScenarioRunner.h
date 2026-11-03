#pragma once

#include "scenario/ScenarioResult.h"

#include <string_view>

namespace dronesim::scenario
{
/**
 * @brief JSON 시나리오를 실행하고 비행·통신·임무 결과를 돌려준다.
 * @details 형식 버전은 1이다. 드론 목록을 생략하면 기본 드론 한 대를 실행한다.
 * 잘못된 문법이나 빠진 필수 값은 std::invalid_argument로 알린다.
 */
[[nodiscard]] ScenarioResult RunScenario(std::string_view jsonText);
}
