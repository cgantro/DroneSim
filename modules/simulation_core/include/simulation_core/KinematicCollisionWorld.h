#pragma once

#include "simulation_core/DroneModel.h"
#include "terrain/TerrainGrid.h"

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <optional>
#include <vector>

namespace aerosimlab::simulation
{
/**
 * @brief 운동학 모드에서 지형과 고정 장애물 접촉을 처리한다.
 * @details 충돌 형상은 월드 ENU 좌표와 미터 단위로 받는다. 비행 위치를 전진시킨 뒤 이 객체가 겹침을 풀고 안쪽 속도를 제거한다.
 */
class KinematicCollisionWorld
{
public:
    /** @brief 지형 높이를 설정해 드론이 격자 바깥과 지면 아래로 나가지 않게 한다. */
    void SetTerrain(terrain::TerrainGrid terrain);

    /** @brief 축에 정렬된 상자 장애물을 추가한다. 반 크기는 모두 양수여야 한다. */
    void AddStaticObstacleBox(
        const glm::dvec3& centerMeters, const glm::vec3& halfExtentsMeters);

    /** @brief 다각형 외곽선을 높이로 돌출한 고정 장애물을 추가한다. */
    void AddStaticObstaclePrism(
        const std::vector<glm::dvec2>& outlineMeters,
        double bottomMeters, double topMeters);

    /** @brief 높이가 없는 다각형을 수평 비행 금지 구역으로 추가한다. */
    void AddStaticObstacleFootprint(
        const std::vector<glm::dvec2>& outlineMeters);

    /** @brief 운동학 한 단계 뒤의 위치와 속도를 지형·장애물에 맞춰 보정한다. */
    void Resolve(DroneState& state, DroneControlState& control,
        const glm::dvec3& previousPositionMeters) const;

private:
    struct Box
    {
        glm::dvec3 centerMeters{0.0};
        glm::dvec3 halfExtentsMeters{0.0};
    };

    struct Prism
    {
        std::vector<glm::dvec2> outlineMeters;
        double bottomMeters = 0.0;
        double topMeters = 0.0;
    };

    struct Footprint
    {
        std::vector<glm::dvec2> outlineMeters;
    };

    std::vector<Box> boxes_;
    std::vector<Prism> prisms_;
    std::vector<Footprint> footprints_;
    std::optional<terrain::TerrainGrid> terrain_;

    void ResolveTerrainContact(DroneState& state, DroneControlState& control) const;
    void ResolveBoxContacts(DroneState& state, DroneControlState& control,
        const glm::dvec3& previousPositionMeters) const;
    void ResolvePrismContacts(DroneState& state, DroneControlState& control) const;
    void ResolveFootprintContacts(DroneState& state, DroneControlState& control) const;
};
}
