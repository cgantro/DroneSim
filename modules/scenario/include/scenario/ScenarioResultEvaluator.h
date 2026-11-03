#pragma once

namespace dronesim::scenario
{
struct ScenarioResult;

namespace detail
{
struct ScenarioDefinition;

/** @brief 실행 결과를 기대 조건과 비교하고 최종 판정을 기록한다. */
void EvaluateExpectedResults(
    const ScenarioDefinition& definition, ScenarioResult& result);
}
}
