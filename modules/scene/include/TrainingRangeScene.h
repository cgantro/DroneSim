#pragma once

#include "graphics/GlbModel.h"
#include "flight_planning/BuildingObstacle.h"
#include "terrain/TerrainGrid.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace dronesim::graphics
{
class Renderer;
class Shader;
}

namespace dronesim::scene
{
/** @brief GLB 장면과 건물의 월드 좌표 충돌 자료를 준비하고 그린다. */
class TrainingRangeScene final
{
public:
    static constexpr double mapScale = 1.0;

    /** @brief 건물 충돌에 쓸 월드 좌표 삼각형 메시다. 정점 단위는 미터다. */
    struct BuildingCollisionMesh
    {
        std::vector<glm::dvec3> verticesMeters;
        std::vector<std::uint32_t> triangleIndices;
    };

    /** @brief 도시 GLB를 읽고 바닥을 원점에 맞춘 뒤 건물 충돌 자료를 만든다. */
    void Load(const std::filesystem::path& assetRoot);

    /** @brief 준비한 시설물 모델을 현재 시점으로 그린다. */
    void Draw(dronesim::graphics::Renderer& renderer,
        dronesim::graphics::Shader& shader,
        const glm::mat4& view, const glm::mat4& projection,
        const glm::vec3& cameraPosition) const;

    [[nodiscard]] const std::vector<dronesim::flight_planning::BuildingObstacle>&
    BuildingObstacles() const noexcept;
    /** @brief 물리 세계에 등록할 건물 삼각형 메시를 읽기 전용으로 돌려준다. */
    [[nodiscard]] const BuildingCollisionMesh& CollisionMesh() const noexcept;
    /** @brief 화면 바닥판의 동·북 방향 크기를 미터로 돌려준다. */
    [[nodiscard]] glm::dvec2 GroundSizeMeters() const noexcept;
    /** @brief 장면 바닥과 같은 크기의 지형 높이 격자를 돌려준다. */
    [[nodiscard]] const terrain::TerrainGrid& Terrain() const noexcept;

private:
    std::unique_ptr<dronesim::graphics::GlbModel> cityModel_;
    glm::vec3 mapOffset_{0.0F};
    glm::mat4 mapTransform_{1.0F};
    glm::dvec2 groundSizeMeters_{0.0};
    terrain::TerrainGrid terrain_;
    BuildingCollisionMesh collisionMesh_;
    std::vector<dronesim::flight_planning::BuildingObstacle> buildingObstacles_;
};
}
