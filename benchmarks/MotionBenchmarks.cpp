#include "BenchmarkSupport.h"

#include "application/PickPlaceConfig.h"
#include "model/RobotModel.h"
#include "physics/PhysicsWorld.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/planning/JointPathPlanner.h"
#include "robotics/runtime/FixedControlLoop.h"
#include "scene/Scene.h"
#include "scene/TransformSystemModule.h"
#include "simulation/SimulationSceneBuilder.h"
#include "simulation/robotics/RobotEnvironmentCollisionGuard.h"
#include "simulation/robotics/RobotModelAdapter.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/robotics/RobotTransformAdapter.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <vector>

namespace grasplink::benchmarks
{
using namespace grasplink::robotics;
using namespace grasplink::robotics::kinematics;
using namespace grasplink::robotics::models;
using namespace grasplink::robotics::models::hanwha;
using namespace grasplink::robotics::planning;
using grasplink::robotics::backends::simulation::SimRobotController;

namespace
{
constexpr double kControlTickSeconds = 1.0 / 250.0;
constexpr double kControlTickBudgetNanoseconds = 4'000'000.0;
constexpr double kSlowMotionVelocityScale = 0.001;

struct ControlTickStages final
{
    double controlAndPoseNanoseconds = 0.0;
    double transformsBeforeCollisionNanoseconds = 0.0;
    double collisionCheckNanoseconds = 0.0;
    double physicsNanoseconds = 0.0;
    double transformsAfterPhysicsNanoseconds = 0.0;
};

class ControlPhysicsTickFixture final
{
public:
    enum class Mode
    {
        ControlTick,
        PosePlanning
    };

    explicit ControlPhysicsTickFixture(Mode mode = Mode::ControlTick)
        : model_(grasplink::model::RobotModelLoader::Load(
            std::filesystem::path{GRASPLINK_SOURCE_DIR} / "assets" / "robots" / "HCR12A"))
    {
        world_ = std::make_unique<flecs::world>();
        world_->import<grasplink::scene::TransformSystemModule>();
        scene_ = std::make_unique<grasplink::scene::Scene>(*world_);
        physicsWorld_ = std::make_unique<grasplink::physics::PhysicsWorld>();
        robotRoot_ = CreateModelHierarchy(model_.visual);
        modelAdapter_ = std::make_unique<grasplink::simulation::robotics::RobotModelAdapter>(model_.robot);
        controller_ = std::make_unique<SimRobotController>(modelAdapter_->Specification());

        const auto connection = controller_->Connect();
        if (!connection)
            throw std::runtime_error(connection.message);

        kinematics_ = std::make_unique<grasplink::robotics::kinematics::RobotKinematics>(
            modelAdapter_->Specification());
        visualAdapter_ = std::make_unique<grasplink::simulation::robotics::RobotTransformAdapter>(
            robotRoot_, modelAdapter_->Specification());
        physicsAdapter_ = std::make_unique<grasplink::simulation::RobotPhysicsAdapter>(
            *scene_, robotRoot_, model_.robot);
        CreateFloor();
        ApplyControllerPose();
        grasplink::scene::TransformSystemModule::UpdateWorldTransforms(*world_);

        physicsSystem_ = std::make_unique<grasplink::simulation::PhysicsSystemModule>(
            *world_, *physicsWorld_);
        collisionGuard_ = std::make_unique<grasplink::simulation::robotics::RobotEnvironmentCollisionGuard>(
            *controller_, *kinematics_, *visualAdapter_, *physicsAdapter_,
            *world_, *physicsWorld_, *physicsSystem_, robotRoot_);

        // 첫 tick에서 Jolt Body를 만들고 캐시를 준비해 초기화 비용을 측정에서 뺀다.
        AdvanceOneTick();
        if (mode == Mode::ControlTick)
            StartSlowJointMotion();
    }

    void AdvanceOneTick()
    {
        const std::size_t executedTicks = controlLoop_.Advance(kControlTickSeconds,
            [this](double fixedDeltaSeconds)
            {
                using Clock = std::chrono::steady_clock;
                auto stageStart = Clock::now();
                collisionGuard_->CaptureSafeJointPose();
                controller_->Update(fixedDeltaSeconds);
                ApplyControllerPose();
                auto stageEnd = Clock::now();
                lastStages_.controlAndPoseNanoseconds =
                    std::chrono::duration<double, std::nano>(stageEnd - stageStart).count();

                stageStart = stageEnd;
                grasplink::scene::TransformSystemModule::UpdateWorldTransforms(*world_);
                stageEnd = Clock::now();
                lastStages_.transformsBeforeCollisionNanoseconds =
                    std::chrono::duration<double, std::nano>(stageEnd - stageStart).count();

                stageStart = stageEnd;
                collisionGuard_->RestoreSafePoseIfOverlapping();
                stageEnd = Clock::now();
                lastStages_.collisionCheckNanoseconds =
                    std::chrono::duration<double, std::nano>(stageEnd - stageStart).count();

                stageStart = stageEnd;
                physicsSystem_->Step(fixedDeltaSeconds);
                stageEnd = Clock::now();
                lastStages_.physicsNanoseconds =
                    std::chrono::duration<double, std::nano>(stageEnd - stageStart).count();

                stageStart = stageEnd;
                grasplink::scene::TransformSystemModule::UpdateWorldTransforms(*world_);
                stageEnd = Clock::now();
                lastStages_.transformsAfterPhysicsNanoseconds =
                    std::chrono::duration<double, std::nano>(stageEnd - stageStart).count();
            });
        if (executedTicks != 1)
            throw std::logic_error("벤치마크는 4 ms마다 정확히 한 tick을 실행해야 합니다.");
    }

    [[nodiscard]] const grasplink::robotics::RobotState& State() const noexcept
    {
        return controller_->GetStateView();
    }

    [[nodiscard]] SimRobotController& Controller() noexcept
    {
        return *controller_;
    }

    [[nodiscard]] const ControlTickStages& LastStages() const noexcept
    {
        return lastStages_;
    }

private:
    grasplink::scene::Entity CreateModelHierarchy(const grasplink::model::ModelResource& visual)
    {
        if (!visual.rootNodeIndex || *visual.rootNodeIndex >= visual.nodes.size())
            throw std::runtime_error("벤치마크 모델에 유효한 시작 링크가 없습니다.");

        std::vector<grasplink::scene::Entity> entities;
        entities.reserve(visual.nodes.size());
        for (const auto& node : visual.nodes)
        {
            auto entity = scene_->CreateEntity(node.name);
            entity.SetLocalPosition(node.translation);
            entity.SetLocalRotation(node.rotation);
            entity.SetLocalScale(node.scale);
            entities.push_back(entity);
        }

        for (std::size_t index = 0; index < visual.nodes.size(); ++index)
        {
            if (visual.nodes[index].parentIndex)
                entities[index].SetParent(entities[*visual.nodes[index].parentIndex]);
        }
        return entities[*visual.rootNodeIndex];
    }

    void CreateFloor()
    {
        auto floor = scene_->CreateEntity("ControlPhysicsBenchFloor");
        grasplink::simulation::ConfigureStaticFloor(
            floor,
            grasplink::application::pick_place::config::floorHalfExtentsMeters,
            grasplink::application::pick_place::config::floorColliderCenterOffsetMeters);
    }

    void ApplyControllerPose()
    {
        const auto& pose = kinematics_->Update(controller_->GetStateView());
        visualAdapter_->Apply(pose);
        physicsAdapter_->Apply(pose);
    }

    void StartSlowJointMotion()
    {
        grasplink::robotics::JointMoveCommand command;
        command.targetPositions = controller_->GetStateView().jointPositions;
        command.targetPositions.front() = 0.25;
        command.velocityScale = kSlowMotionVelocityScale;
        const auto result = controller_->MoveJoint(command);
        if (!result)
            throw std::runtime_error("벤치마크 관절 이동을 시작하지 못했습니다: " + result.message);
        if (controller_->GetStateView().mode != grasplink::robotics::RobotMode::Moving)
            throw std::runtime_error("벤치마크 제어기가 이동 상태로 전환되지 않았습니다.");
    }

    // 뒤에 선언한 객체가 먼저 사라져 참조 관계를 유지한다.
    grasplink::model::LoadedRobotModel model_;
    std::unique_ptr<flecs::world> world_;
    std::unique_ptr<grasplink::scene::Scene> scene_;
    std::unique_ptr<grasplink::physics::PhysicsWorld> physicsWorld_;
    std::unique_ptr<grasplink::simulation::robotics::RobotModelAdapter> modelAdapter_;
    std::unique_ptr<SimRobotController> controller_;
    std::unique_ptr<grasplink::robotics::kinematics::RobotKinematics> kinematics_;
    grasplink::scene::Entity robotRoot_;
    std::unique_ptr<grasplink::simulation::robotics::RobotTransformAdapter> visualAdapter_;
    std::unique_ptr<grasplink::simulation::RobotPhysicsAdapter> physicsAdapter_;
    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> physicsSystem_;
    std::unique_ptr<grasplink::simulation::robotics::RobotEnvironmentCollisionGuard> collisionGuard_;
    grasplink::robotics::runtime::FixedControlLoop controlLoop_{kControlTickSeconds};
    ControlTickStages lastStages_;
};

double Percentile(std::vector<double>& sortedSamples, double percentile)
{
    const auto index = static_cast<std::size_t>(std::ceil(percentile * sortedSamples.size())) - 1;
    return sortedSamples[std::min(index, sortedSamples.size() - 1)];
}

template <typename ReadStage>
void AddStageCounters(benchmark::State& state, const std::vector<ControlTickStages>& samples,
    const char* name, ReadStage readStage)
{
    std::vector<double> stageSamples;
    stageSamples.reserve(samples.size());
    for (const ControlTickStages& sample : samples)
        stageSamples.push_back(readStage(sample));
    std::sort(stageSamples.begin(), stageSamples.end());
    state.counters[std::string{name} + "_p95_nanoseconds"] = Percentile(stageSamples, 0.95);
    state.counters[std::string{name} + "_max_nanoseconds"] = stageSamples.back();
}
}

// 관절 공간의 시작점과 끝점 사이가 관절 제한을 만족하는지 검사합니다. 실제 로봇 동작은 시뮬레이션하지 않습니다.
void MoveJJointPathValidation(benchmark::State& state)
{
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    std::uint64_t validPaths = 0;
    std::uint64_t endpointIkConverged = 0;
    std::uint64_t endpointIkIterations = 0;

    for (auto _ : state)
    {
        for (const auto& testCase : cases)
        {
            const auto validity = ValidateJointPath(
                kSeed, testCase.reachableTargetJoints, kHcr12a, {});
            validPaths += validity == JointStateInvalidity::None;
            endpointIkConverged += testCase.endpointIkConvergedFromStart;
            endpointIkIterations += testCase.endpointIkIterations;
            benchmark::DoNotOptimize(validity);
        }
    }

    const double totalCases = static_cast<double>(state.iterations() * cases.size());
    state.counters["fk_generated_reachable_endpoints"] = totalCases;
    state.counters["endpoint_ik_successes_from_start_seed"] = static_cast<double>(endpointIkConverged);
    state.counters["endpoint_ik_iterations_per_endpoint"] = totalCases > 0.0 ?
        static_cast<double>(endpointIkIterations) / totalCases : 0.0;
    state.counters["joint_limit_valid_movej_segments"] = static_cast<double>(validPaths);
    state.counters["joint_limit_valid_movej_segment_rate"] = totalCases > 0.0 ? static_cast<double>(validPaths) / totalCases : 0.0;
    state.counters["environment_collision_checks"] = 0;
    state.SetItemsProcessed(static_cast<std::int64_t>(totalCases));
}

// 각 MoveJ 목표를 SimRobotController에서 4 ms 고정 시간 간격으로 실행하고 완료, 거부, 오류, 시간 초과를 셉니다.
void MoveJCompletion(benchmark::State& state)
{
    const auto cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
    std::uint64_t completed = 0;
    std::uint64_t rejected = 0;
    std::uint64_t runtimeFaults = 0;
    std::uint64_t timeouts = 0;

    for (auto _ : state)
    {
        for (const auto& testCase : cases)
        {
            SimRobotController controller(kHcr12a);
            if (!controller.Connect())
            {
                state.SkipWithError("MoveJ 벤치마크에서 SimRobotController에 연결하지 못했습니다.");
                return;
            }
            const Result accepted = controller.MoveJoint({testCase.reachableTargetJoints, 1.0, 1.0});
            if (!accepted)
            {
                ++rejected;
                continue;
            }

            constexpr double fixedDeltaSeconds = 0.004;
            constexpr std::size_t maxTicks = 5000;
            std::size_t tick = 0;
            for (; tick < maxTicks && controller.GetStateView().mode == RobotMode::Moving; ++tick)
                controller.Update(fixedDeltaSeconds);
            if (controller.GetStateView().mode == RobotMode::Idle)
                ++completed;
            else if (controller.GetStateView().mode == RobotMode::Fault)
                ++runtimeFaults;
            else
                ++timeouts;
            benchmark::DoNotOptimize(controller.GetStateView().jointPositions.data());
        }
    }

    const double totalCases = static_cast<double>(state.iterations() * cases.size());
    state.counters["fk_generated_reachable_endpoints"] = totalCases;
    state.counters["movej_completed"] = static_cast<double>(completed);
    state.counters["movej_rejected"] = static_cast<double>(rejected);
    state.counters["movej_runtime_faults"] = static_cast<double>(runtimeFaults);
    state.counters["movej_timeouts"] = static_cast<double>(timeouts);
    state.counters["movej_completion_rate"] = totalCases > 0.0 ? static_cast<double>(completed) / totalCases : 0.0;
    state.counters["environment_collision_checks"] = 0;
    state.SetItemsProcessed(static_cast<std::int64_t>(totalCases));
}

// 단순 상자 프록시가 장애물을 비켜 가거나 가로지르는 관절 경로를 검사한다.
void JoltMoveJDirectPath(benchmark::State& state)
{
    using grasplink::physics::BodyDescription;
    using grasplink::physics::BodyMotionType;
    using grasplink::physics::BoxBodyDescription;
    using grasplink::physics::CollisionLayer;
    using grasplink::physics::PhysicsWorld;
    using grasplink::physics::Transform;

    const bool pathCrossesObstacle = state.range(0) != 0;
    const float clearanceOffsetMeters = pathCrossesObstacle ? 0.0F : 0.4F;

    PhysicsWorld world;
    BoxBodyDescription obstacle;
    obstacle.halfExtentsMeters = {0.10F, 0.15F, 0.10F};
    obstacle.transform.position = {0.0F, 0.5F, 0.0F};
    obstacle.motionType = BodyMotionType::Static;
    obstacle.collisionLayer = CollisionLayer::Environment;
    world.CreateBox(obstacle);

    BodyDescription proxyDescription;
    grasplink::physics::CollisionShapeDescription proxyShape;
    proxyShape.halfExtentsMeters = {0.035F, 0.06F, 0.035F};
    proxyDescription.shapes.push_back(proxyShape);
    proxyDescription.transform.position = {-0.20F, 0.5F, -0.20F + clearanceOffsetMeters};
    proxyDescription.motionType = BodyMotionType::Kinematic;
    proxyDescription.collisionLayer = CollisionLayer::Robot;
    const auto proxy = world.CreateBody(proxyDescription);

    JointVector start(kHcr12a.jointCount, 0.0);
    JointVector goal = start;
    start[0] = -1.0;
    start[1] = -1.0;
    goal[0] = 1.0;
    goal[1] = 1.0;

    std::uint64_t plannerRuns = 0;
    std::uint64_t accepted = 0;
    std::uint64_t rejected = 0;
    std::uint64_t joltQueries = 0;
    std::uint64_t plannerValidityChecks = 0;
    bool collisionDetected = false;
    for (auto _ : state)
    {
        StateValidityChecker checker = [&](const JointVector& joints)
        {
            ++joltQueries;
            Transform candidate;
            candidate.position = {
                static_cast<float>(joints[0] * 0.20),
                0.5F,
                static_cast<float>(joints[1] * 0.20) + clearanceOffsetMeters};
            collisionDetected = world.OverlapsEnvironmentAt(proxy, candidate);
            return collisionDetected
                ? JointStateInvalidity::EnvironmentCollision
                : JointStateInvalidity::None;
        };

        JointPathPlanningJob job;
        const Result begin = job.Begin(kHcr12a, start, goal, checker);
        if (!begin)
        {
            state.SkipWithError("Jolt 기반 MoveJ 계획기가 유효한 벤치마크 끝점을 거부했습니다.");
            return;
        }
        for (std::size_t work = 0; work < 10000 && job.GetState() == JointPathPlanningState::Running; ++work)
            job.Advance(1);

        const bool completed = job.GetState() == JointPathPlanningState::Completed;
        if (completed != !pathCrossesObstacle || collisionDetected != pathCrossesObstacle)
        {
            state.SkipWithError("Jolt MoveJ 벤치마크 경로가 예상한 충돌 결과와 다릅니다.");
            return;
        }

        ++plannerRuns;
        if (completed)
        {
            ++accepted;
            plannerValidityChecks += job.GetPlan().validityChecks;
        }
        else
            ++rejected;
        benchmark::DoNotOptimize(job.GetResult().code);
    }

    state.counters["jolt_direct_movej_runs"] = static_cast<double>(plannerRuns);
    state.counters["jolt_direct_movej_accepted"] = static_cast<double>(accepted);
    state.counters["jolt_direct_movej_rejected"] = static_cast<double>(rejected);
    state.counters["jolt_direct_movej_rejection_rate"] = plannerRuns > 0
        ? static_cast<double>(rejected) / static_cast<double>(plannerRuns) : 0.0;
    state.counters["jolt_environment_overlap_queries"] = static_cast<double>(joltQueries);
    state.counters["jolt_overlap_queries_per_run"] = plannerRuns > 0
        ? static_cast<double>(joltQueries) / static_cast<double>(plannerRuns) : 0.0;
    state.counters["validity_checks_per_accepted_run"] = accepted > 0
        ? static_cast<double>(plannerValidityChecks) / static_cast<double>(accepted) : 0.0;
    state.counters["path_crosses_obstacle"] = pathCrossesObstacle ? 1.0 : 0.0;
    state.SetItemsProcessed(static_cast<std::int64_t>(plannerRuns));
    state.SetLabel(pathCrossesObstacle ? "장애물 충돌 경로" : "장애물을 비켜 가는 경로");
}

// Viewer의 팔 제어, FK, 충돌 확인, Jolt Step, 계층 변환 갱신을 250 Hz tick으로 측정한다.
void ControlPhysicsTick250Hz(benchmark::State& state)
{
    std::unique_ptr<ControlPhysicsTickFixture> fixture;
    try
    {
        state.PauseTiming();
        fixture = std::make_unique<ControlPhysicsTickFixture>();
        std::vector<double> tickTimesNanoseconds;
        tickTimesNanoseconds.reserve(50'000);
        std::vector<ControlTickStages> stageSamples;
        stageSamples.reserve(50'000);
        state.ResumeTiming();

        for (auto _ : state)
        {
            const auto start = std::chrono::steady_clock::now();
            fixture->AdvanceOneTick();
            const auto end = std::chrono::steady_clock::now();
            const double elapsedNanoseconds = std::chrono::duration<double, std::nano>(end - start).count();
            state.SetIterationTime(elapsedNanoseconds * 1.0e-9);
            tickTimesNanoseconds.push_back(elapsedNanoseconds);
            stageSamples.push_back(fixture->LastStages());
            benchmark::DoNotOptimize(fixture->State().jointPositions.data());
            benchmark::ClobberMemory();
        }

        std::sort(tickTimesNanoseconds.begin(), tickTimesNanoseconds.end());
        const auto deadlineMisses = std::count_if(tickTimesNanoseconds.begin(), tickTimesNanoseconds.end(),
            [](double elapsed) { return elapsed > kControlTickBudgetNanoseconds; });
        state.counters["tick_budget_nanoseconds"] = kControlTickBudgetNanoseconds;
        state.counters["tick_samples"] = static_cast<double>(tickTimesNanoseconds.size());
        state.counters["tick_p50_nanoseconds"] = Percentile(tickTimesNanoseconds, 0.50);
        state.counters["tick_p95_nanoseconds"] = Percentile(tickTimesNanoseconds, 0.95);
        state.counters["tick_p99_nanoseconds"] = Percentile(tickTimesNanoseconds, 0.99);
        state.counters["tick_max_nanoseconds"] = tickTimesNanoseconds.back();
        state.counters["tick_deadline_misses"] = static_cast<double>(deadlineMisses);
        state.counters["tick_deadline_miss_rate"] = tickTimesNanoseconds.empty() ? 0.0
            : static_cast<double>(deadlineMisses) / tickTimesNanoseconds.size();
        AddStageCounters(state, stageSamples, "control_and_pose",
            [](const ControlTickStages& stages) { return stages.controlAndPoseNanoseconds; });
        AddStageCounters(state, stageSamples, "transform_before_collision",
            [](const ControlTickStages& stages) { return stages.transformsBeforeCollisionNanoseconds; });
        AddStageCounters(state, stageSamples, "collision_check",
            [](const ControlTickStages& stages) { return stages.collisionCheckNanoseconds; });
        AddStageCounters(state, stageSamples, "physics",
            [](const ControlTickStages& stages) { return stages.physicsNanoseconds; });
        AddStageCounters(state, stageSamples, "transform_after_physics",
            [](const ControlTickStages& stages) { return stages.transformsAfterPhysicsNanoseconds; });
        state.SetItemsProcessed(static_cast<std::int64_t>(tickTimesNanoseconds.size()));
    }
    catch (const std::exception& error)
    {
        state.SkipWithError(error.what());
    }
}

// 실제 URDF 충돌 형상과 Jolt 질의가 포함된 포즈 계획 비용을 측정한다.
void PosePlanWithRobotCollisions(benchmark::State& state)
{
    using Clock = std::chrono::steady_clock;
    constexpr std::size_t maximumPlanningFrames = 10000;
    constexpr auto planningFrameBudget = std::chrono::microseconds(1500);
    std::unique_ptr<ControlPhysicsTickFixture> fixture;
    std::vector<RuntimePathCase> cases;
    std::vector<double> planningFrameMicroseconds;
    try
    {
        state.PauseTiming();
        fixture = std::make_unique<ControlPhysicsTickFixture>(
            ControlPhysicsTickFixture::Mode::PosePlanning);
        cases = RuntimePathCases(static_cast<std::size_t>(state.range(0)));
        planningFrameMicroseconds.reserve(cases.size() * maximumPlanningFrames);
        state.ResumeTiming();

        std::uint64_t accepted = 0;
        std::uint64_t rejected = 0;
        std::uint64_t timedOut = 0;
        std::uint64_t beginRejected = 0;
        std::uint64_t environmentCollisionRejected = 0;
        std::uint64_t selfCollisionRejected = 0;
        std::uint64_t otherRejected = 0;
        for (auto _ : state)
        {
            const auto batchStart = Clock::now();
            for (const RuntimePathCase& testCase : cases)
            {
                auto& controller = fixture->Controller();
                const Result begin = controller.BeginPosePlanning(testCase.targetPose);
                if (!begin)
                {
                    ++beginRejected;
                    ++rejected;
                    continue;
                }

                std::size_t frames = 0;
                while (controller.IsMotionPlanning() && frames < maximumPlanningFrames)
                {
                    const auto frameStart = Clock::now();
                    const auto frameDeadline = frameStart + planningFrameBudget;
                    do
                    {
                        controller.AdvanceMotionPlanning(1);
                    } while (controller.IsMotionPlanning() && Clock::now() < frameDeadline);
                    planningFrameMicroseconds.push_back(std::chrono::duration<double, std::micro>(
                        Clock::now() - frameStart).count());
                    ++frames;
                }
                if (controller.IsMotionPlanning())
                {
                    ++timedOut;
                    controller.Stop();
                    continue;
                }

                const auto result = controller.TakeMotionPlanningResult();
                if (result && *result)
                    ++accepted;
                else
                {
                    ++rejected;
                    if (result && result->code == ErrorCode::EnvironmentContact)
                        ++environmentCollisionRejected;
                    else if (result && result->code == ErrorCode::SelfCollision)
                        ++selfCollisionRejected;
                    else
                        ++otherRejected;
                }
            }
            state.SetIterationTime(std::chrono::duration<double>(Clock::now() - batchStart).count());
        }

        state.counters["pose_plan_accepted"] = static_cast<double>(accepted);
        state.counters["pose_plan_rejected"] = static_cast<double>(rejected);
        state.counters["pose_plan_begin_rejected"] = static_cast<double>(beginRejected);
        state.counters["pose_plan_environment_collision_rejected"] =
            static_cast<double>(environmentCollisionRejected);
        state.counters["pose_plan_self_collision_rejected"] =
            static_cast<double>(selfCollisionRejected);
        state.counters["pose_plan_other_rejected"] = static_cast<double>(otherRejected);
        state.counters["pose_plan_timeouts"] = static_cast<double>(timedOut);
        const auto overBudgetFrames = std::count_if(
            planningFrameMicroseconds.begin(), planningFrameMicroseconds.end(),
            [planningFrameBudget](double elapsedMicroseconds)
            {
                return elapsedMicroseconds >
                    std::chrono::duration<double, std::micro>(planningFrameBudget).count();
            });
        std::sort(planningFrameMicroseconds.begin(), planningFrameMicroseconds.end());
        state.counters["planning_frame_p50_us"] = planningFrameMicroseconds.empty() ? 0.0
            : Percentile(planningFrameMicroseconds, 0.50);
        state.counters["planning_frame_p95_us"] = planningFrameMicroseconds.empty() ? 0.0
            : Percentile(planningFrameMicroseconds, 0.95);
        state.counters["planning_frame_p99_us"] = planningFrameMicroseconds.empty() ? 0.0
            : Percentile(planningFrameMicroseconds, 0.99);
        state.counters["planning_frame_over_1_5ms"] = static_cast<double>(overBudgetFrames);
        state.counters["collision_benchmark_mode_real_robot_guard"] = 1;
        state.SetItemsProcessed(static_cast<std::int64_t>(accepted + rejected + timedOut));
        state.SetLabel("HCR-12A URDF 충돌 형상과 Jolt 환경·자가 충돌 검사 포함");
    }
    catch (const std::exception& error)
    {
        state.SkipWithError(error.what());
    }
}
}
