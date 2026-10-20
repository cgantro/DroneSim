#include "BenchmarkSupport.h"

#include <charconv>
#include <cstdint>
#include <iostream>
#include <string_view>

namespace
{
// --iterations 값은 양의 정수만 받으며, 옵션 전체를 숫자로 읽었는지도 확인합니다.
bool ParseIterations(std::string_view value, std::int64_t& iterations)
{
    const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), iterations);
    return error == std::errc{} && end == value.data() + value.size() && iterations > 0;
}

}

using namespace grasplink::benchmarks;

int main(int argc, char** argv)
{
    std::int64_t iterations = 0;
    int output = 1;
    for (int input = 1; input < argc; ++input)
    {
        constexpr std::string_view option = "--iterations=";
        const std::string_view argument(argv[input]);
        if (argument.compare(0, option.size(), option) == 0)
        {
            if (!ParseIterations(argument.substr(option.size()), iterations))
            {
                std::cerr << "--iterations에는 양의 정수를 입력해야 합니다.\n";
                return 2;
            }
            continue;
        }
        argv[output++] = argv[input];
    }
    argc = output;
    argv[argc] = nullptr;

    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv))
        return 2;

    // --iterations 옵션은 반복 비용이 낮은 기본 측정에 적용하고, 대량 사례 스트레스 측정은 등록된 반복 수를 유지합니다.
    auto* fk = benchmark::RegisterBenchmark("FK", Fk);
    auto* modelLoad = benchmark::RegisterBenchmark("URDF_GLB_ModelLoad", LoadUrdfAndGlb);
    auto* collisionBuild = benchmark::RegisterBenchmark("URDF_CollisionGeometryBuild", BuildUrdfCollisionGeometry);
    auto* ik = benchmark::RegisterBenchmark("LM_IK", BenchmarkLmIk);
    auto* planner = benchmark::RegisterBenchmark("LinearPathPlanner", LinearPathPlanning);
    auto* incrementalPlanner = benchmark::RegisterBenchmark(
        "IncrementalLinearPathPlanner", IncrementalLinearPathPlanning);
    benchmark::RegisterBenchmark("EndpointIkGeneratedReachable", EndpointIkGeneratedReachable)
        ->Arg(64)
        ->Arg(256);
    benchmark::RegisterBenchmark("MoveJJointPathValidation", MoveJJointPathValidation)
        ->Arg(64)
        ->Arg(256);
    benchmark::RegisterBenchmark("MoveJCompletion", MoveJCompletion)
        ->Arg(64)
        ->Arg(256);
    benchmark::RegisterBenchmark("JoltMoveJDirectPath", JoltMoveJDirectPath)
        ->Unit(benchmark::kNanosecond);
    benchmark::RegisterBenchmark("ControlPhysicsTick250Hz", ControlPhysicsTick250Hz)
        ->Unit(benchmark::kMicrosecond)
        ->UseManualTime();
    benchmark::RegisterBenchmark("RuntimeLinearPathStress", RuntimeLinearPathStress)
        ->Arg(64)
        ->Arg(256);
    benchmark::RegisterBenchmark("InteractiveLinearPathStress", InteractiveLinearPathStress)
        ->Arg(64)
        ->Iterations(1);
    benchmark::RegisterBenchmark("PosePlanEndpointStress", PosePlanEndpointStress)
        ->Arg(64)
        ->Arg(256)
        ->Iterations(1)
        ->UseRealTime();
    if (iterations > 0)
    {
        fk->Iterations(iterations);
        modelLoad->Iterations(iterations);
        collisionBuild->Iterations(iterations);
        ik->Iterations(iterations);
        planner->Iterations(iterations);
        incrementalPlanner->Iterations(iterations);
    }

    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
