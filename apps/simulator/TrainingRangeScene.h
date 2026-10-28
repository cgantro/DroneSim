#pragma once

#include "graphics/GlbModel.h"
#include "simulation_core/SimulationCore.h"
#include "terrain/MapBuildings.h"
#include "terrain/TerrainGrid.h"

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
/** @brief 군집 시험장에 쓸 GLB 시설물과 충돌 상자를 준비한다. */
class TrainingRangeScene final
{
public:
    struct ReconPoint
    {
        std::string id;
        glm::dvec2 positionMeters{0.0};
    };

    /** @brief 지형 높이에 맞춰 모델·장애물·정찰 지점을 배치한다. */
    void Load(const std::filesystem::path& assetRoot,
        const aerosimlab::terrain::TerrainGrid& terrain,
        aerosimlab::simulation::SimulationCore& simulation);

    /** @brief 준비한 시설물 모델을 현재 시점으로 그린다. */
    void Draw(aerosimlab::graphics::Renderer& renderer,
        aerosimlab::graphics::Shader& shader,
        const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition) const;

    [[nodiscard]] const std::vector<aerosimlab::terrain::MapBuilding>& Obstacles() const noexcept;
    [[nodiscard]] const std::vector<ReconPoint>& ReconPoints() const noexcept;

private:
    struct ModelGroup
    {
        std::unique_ptr<aerosimlab::graphics::GlbModel> model;
    };

    std::vector<ModelGroup> models_;
    std::vector<aerosimlab::terrain::MapBuilding> obstacles_;
    std::vector<ReconPoint> reconPoints_;
};
}
