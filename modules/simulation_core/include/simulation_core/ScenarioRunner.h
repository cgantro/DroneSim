#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/EoCameraModel.h"

#include <cstdint>
#include <optional>
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
    CameraState finalCamera;
    std::vector<std::string> issues;
    std::vector<std::string> eventLog;
    /** @brief 기준을 요청한 시나리오에서 잰 최소 3차원 중심 간 거리다. */
    std::optional<double> minimumDroneSeparationMeters;
    /** @brief 시나리오에서 이름으로 지정한 드론의 마지막 상태다. */
    struct Drone
    {
        std::string name;
        DroneState finalState;
    };
    /** @brief 시나리오에 등장한 드론의 마지막 상태를 입력 순서대로 보관한다. */
    std::vector<Drone> drones;
};

/**
 * @brief JSON 문자열에 적힌 명령을 고정 tick에 적용하고 예상 결과와 비교한다.
 * @details 형식 버전은 1이다. 선택한 드론 목록이 있으면 이름별 위치와 명령을 시험하고, 없으면 기본 드론 한 대를 실행한다.
 * formation_links를 지정하면 한 드론이 다른 드론의 월드 위치를 따라가며, outage 구간에는 마지막 목표를 유지한다.
 * 잘못된 문법이나 빠진 필수 값은 std::invalid_argument로 알린다.
 */
[[nodiscard]] ScenarioResult RunScenario(std::string_view jsonText);

/** @brief 비행 상태를 시나리오 결과에 기록할 짧은 이름으로 바꾼다. */
[[nodiscard]] std::string_view ToString(FlightMode mode) noexcept;
}
