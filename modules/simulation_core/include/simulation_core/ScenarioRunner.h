#pragma once

#include "simulation_core/DroneModel.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace aerosimlab::simulation
{
/**
 * @brief JSON 시나리오 실행의 최종 상태와 시험 결과를 돌려준다.
 * @details 통과 여부는 시나리오 명령이 모두 받아들여지고, 마지막 비행 상태와 위치가 예상값 범위에 들었는지로 정한다.
 */
struct ScenarioResult
{
    bool passed = false;
    std::uint64_t tickCount = 0;
    double simulationTimeSeconds = 0.0;
    DroneState finalState;
    std::vector<std::string> issues;
};

/**
 * @brief JSON 문자열에 적힌 명령을 고정 tick에 적용하고 예상 결과와 비교한다.
 * @details 형식 버전은 1이다. 시나리오에는 실행 tick 수, tick 순서대로 정렬한 명령, 예상 최종 상태가 필요하다.
 * 잘못된 문법이나 빠진 필수 값은 std::invalid_argument로 알린다.
 */
[[nodiscard]] ScenarioResult RunScenario(std::string_view jsonText);

/** @brief 비행 상태를 시나리오 결과에 기록할 짧은 이름으로 바꾼다. */
[[nodiscard]] std::string_view ToString(FlightMode mode) noexcept;
}
