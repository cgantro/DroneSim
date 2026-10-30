#pragma once

#include "simulation_core/QuadrotorModel.h"
#include "simulation_core/RigidBodyTypes.h"
#include "terrain/TerrainGrid.h"

#include <array>
#include <memory>
#include <vector>

namespace aerosimlab::simulation
{
/**
 * @brief SimulationCore가 사용하는 물리 기능을 제공한다.
 * @details 빌드 대상이 선택한 물리 엔진은 구현 파일에서만 다룬다. Flecs와 비행 제어기는
 * 이 경계를 통해 강체를 만들고, 모터 목표를 보내며, 계산된 상태를 읽는다.
 */
class SimulationPhysics
{
public:
    SimulationPhysics();
    ~SimulationPhysics();

    SimulationPhysics(const SimulationPhysics&) = delete;
    SimulationPhysics& operator=(const SimulationPhysics&) = delete;

    [[nodiscard]] RigidBodyHandle CreateQuadrotor(
        const RigidBodyDefinition& definition,
        const QuadrotorParameters& parameters);
    [[nodiscard]] bool DestroyBody(RigidBodyHandle body);
    void SetMotorTargets(
        RigidBodyHandle body, const std::array<double, 4>& speeds);
    void SetInitialMotorSpeeds(
        RigidBodyHandle body, const std::array<double, 4>& speeds);
    [[nodiscard]] std::array<double, 4> MotorSpeedsRadiansPerSecond(
        RigidBodyHandle body) const;
    void SetState(RigidBodyHandle body, const RigidBodyState& state);
    [[nodiscard]] RigidBodyState State(RigidBodyHandle body) const;
    void AddStaticObstacleBox(
        const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters);
    void AddStaticObstaclePrism(
        const std::vector<glm::dvec2>& outlineMeters,
        double bottomMeters, double topMeters);
    void SetTerrain(terrain::TerrainGrid terrain);
    void SetWindVelocityMetersPerSecond(const glm::dvec3& velocity);
    void StepFixed(double* stepDurationMicroseconds = nullptr);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
}
