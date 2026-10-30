#pragma once

#include "graphics/GlbModel.h"
#include "flight_planning/BuildingObstacle.h"
#include "simulation_core/SimulationCore.h"

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace aerosimlab::graphics
{
class Renderer;
class Shader;
}

namespace aerosimlab::simulator
{
/** @brief 도시 GLB를 그리고 메시 범위로 비행 충돌체와 정찰 지점을 준비한다. */
class TrainingRangeScene final
{
public:
    static constexpr double mapScale = 1.0;

    struct ReconPoint
    {
        std::string id;
        glm::dvec2 positionMeters{0.0};
    };

    /** @brief 도시 GLB의 바닥을 원점에 맞추고 구조물 충돌 상자를 등록한다. */
    void Load(const std::filesystem::path& assetRoot,
        aerosimlab::simulation::SimulationCore& simulation);

    /** @brief 준비한 시설물 모델을 현재 시점으로 그린다. */
    void Draw(aerosimlab::graphics::Renderer& renderer,
        aerosimlab::graphics::Shader& shader,
        const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition) const;

    [[nodiscard]] const std::vector<aerosimlab::flight_planning::BuildingObstacle>&
    BuildingObstacles() const noexcept;
    [[nodiscard]] const std::vector<ReconPoint>& ReconPoints() const noexcept;
    /** @brief 화면 바닥판의 동·북 방향 크기를 미터로 돌려준다. */
    [[nodiscard]] glm::dvec2 GroundSizeMeters() const noexcept;
    /** @brief M01 편대가 정찰을 시작하기 전에 모일 지점이다. */
    [[nodiscard]] glm::dvec2 FormationStagingPositionMeters() const noexcept;

private:
    std::unique_ptr<aerosimlab::graphics::GlbModel> cityModel_;
    glm::vec3 mapOffset_{0.0F};
    glm::mat4 mapTransform_{1.0F};
    glm::dvec2 groundSizeMeters_{0.0};
    std::vector<aerosimlab::flight_planning::BuildingObstacle> buildingObstacles_;
    std::vector<ReconPoint> reconPoints_;
};
}
