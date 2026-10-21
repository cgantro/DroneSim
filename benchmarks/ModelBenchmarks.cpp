#include "BenchmarkSupport.h"

#include "model/RobotModel.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/planning/JointPathPlanner.h"
#include "simulation/robotics/RobotCollisionGeometryBuilder.h"

#include <cstdint>
#include <filesystem>

namespace grasplink::benchmarks
{
using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;
using namespace grasplink::robotics::models;
using namespace grasplink::robotics::models::hanwha;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;
// URDF에는 로봇 링크와 관절 구조가, GLB에는 화면에 표시할 메시가 들어 있습니다. 이 측정은 두 모델 파일을 함께 읽는 시간을 잽니다.
void LoadUrdfAndGlb(benchmark::State& state)
{
    for (auto _ : state)
    {
        auto loaded = grasplink::model::LoadRobotModel(
            std::filesystem::path{GRASPLINK_SOURCE_DIR} / "assets/robots/HCR12A");
        benchmark::DoNotOptimize(loaded.robot.links.data());
        benchmark::DoNotOptimize(loaded.visual.nodes.data());
    }
}

// 충돌 검사는 로봇 모델을 단순한 도형으로 바꿔 수행합니다. 이 측정은 URDF 모델에서 충돌용 도형을 만드는 시간을 잽니다.
void BuildUrdfCollisionGeometry(benchmark::State& state)
{
    const auto loaded = grasplink::model::LoadRobotModel(
        std::filesystem::path{GRASPLINK_SOURCE_DIR} / "assets/robots/HCR12A");
    std::size_t shapeCount = 0;
    for (auto _ : state)
    {
        const auto geometry = grasplink::simulation::robotics::collision_geometry::Build(loaded.robot);
        shapeCount += geometry.baseShapes.size();
        for (const auto& link : geometry.links)
            shapeCount += link.shapes.size();
        benchmark::DoNotOptimize(geometry.links.data());
    }
    state.SetItemsProcessed(static_cast<std::int64_t>(shapeCount));
    state.counters["shapes_per_build"] = static_cast<double>(shapeCount) / state.iterations();
}
}
