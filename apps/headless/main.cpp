#include "simulation_core/ScenarioRunner.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
constexpr std::string_view helpText =
    "사용법: aerosimlab_headless --scenario <JSON 파일>\n"
    "        aerosimlab_headless --help\n";
constexpr std::uintmax_t maximumScenarioFileBytes = 16 * 1024 * 1024;

nlohmann::json ToJson(const aerosimlab::simulation::ScenarioResult& result)
{
    const auto& state = result.finalState;
    return {
        {"passed", result.passed},
        {"tick_count", result.tickCount},
        {"simulation_time_seconds", result.simulationTimeSeconds},
        {"final_state", {
            {"mode", aerosimlab::simulation::ToString(state.mode)},
            {"position_meters", {state.positionMeters.x, state.positionMeters.y, state.positionMeters.z}},
            {"velocity_meters_per_second", {
                state.velocityMetersPerSecond.x,
                state.velocityMetersPerSecond.y,
                state.velocityMetersPerSecond.z}},
            {"yaw_radians", state.yawRadians}
        }},
        {"issues", result.issues}
    };
}
}

int main(int argc, char** argv)
{
    if (argc == 2 && std::string_view(argv[1]) == "--help")
    {
        std::cout << helpText;
        return 0;
    }
    if (argc != 3 || std::string_view(argv[1]) != "--scenario")
    {
        std::cerr << helpText;
        return 2;
    }

    try
    {
        const std::filesystem::path scenarioPath(argv[2]);
        const std::uintmax_t fileSize = std::filesystem::file_size(scenarioPath);
        if (fileSize > maximumScenarioFileBytes)
        {
            throw std::invalid_argument("시나리오 파일은 16 MiB를 넘을 수 없습니다.");
        }

        std::ifstream input(scenarioPath, std::ios::binary);
        if (!input)
        {
            throw std::runtime_error("시나리오 파일을 열 수 없습니다.");
        }
        const std::string jsonText{
            std::istreambuf_iterator<char>{input},
            std::istreambuf_iterator<char>{}};
        const auto result = aerosimlab::simulation::RunScenario(jsonText);
        std::cout << ToJson(result).dump(2) << '\n';
        return result.passed ? 0 : 1;
    }
    catch (const std::exception& error)
    {
        std::cerr << "시나리오 실행 실패: " << error.what() << '\n';
        return 2;
    }
}
