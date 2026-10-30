#pragma once

#include <string>
#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace dronesim::flight_planning
{
/** @brief 훈련장 건물의 외곽선과 지면·높이를 보관한다. 좌표와 높이 단위는 미터다. */
struct BuildingObstacle
{
    std::string name;
    std::vector<glm::dvec2> outlineMeters;
    std::vector<double> groundElevationsMeters;
    double heightMeters = 0.0;
};

/** @brief 3차원 위치가 건물 외곽과 높이 범위 안에 있는지 확인한다. */
[[nodiscard]] bool ContainsBuildingVolume(
    const BuildingObstacle& building, const glm::dvec3& positionMeters) noexcept;
}
