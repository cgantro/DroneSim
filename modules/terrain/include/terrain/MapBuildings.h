#pragma once

#include "terrain/TerrainGrid.h"

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

namespace aerosimlab::terrain
{
inline constexpr double estimatedFootprintBuildingHeightMeters = 6.0;

/** @brief 건물 높이가 실측 태그, 층수 환산, 외곽선 임시값 중 어디서 왔는지 구분한다. */
enum class BuildingHeightSource
{
    OsmHeightTag,
    EstimatedFromLevels,
    EstimatedFromFootprint
};

/** @brief 건물 외곽선과 지면 높이, 높이 자료의 출처를 보관한다. */
struct MapBuilding
{
    std::string osmId;
    std::string name;
    std::vector<glm::dvec2> outlineMeters;
    std::vector<double> groundElevationsMeters;
    double heightMeters = 0.0;
    BuildingHeightSource heightSource = BuildingHeightSource::OsmHeightTag;
};

/** @brief 건물 외곽과 높이를 감싸는 지도 좌표계 기준 축 정렬 상자다. */
struct MapBuildingBoundsMeters
{
    glm::dvec3 centerMeters{0.0};
    glm::dvec3 halfExtentsMeters{0.0};
};

/** @brief OSM 건물 외곽선의 한 변에서 만든 수직 외벽 시험면이다. */
struct MapBuildingFacade
{
    glm::dvec3 centerMeters{0.0};
    glm::dvec3 outwardNormal{0.0};
    glm::dvec3 widthAxis{0.0};
    double widthMeters = 0.0;
    double heightMeters = 0.0;
};

/**
 * @brief OSM GeoJSON에서 지형 격자 안의 높이 정보가 있는 건물을 읽는다.
 * @details `height`가 없고 층수만 있으면 층당 3m를 곱해 화면 높이를 추정한다.
 *          이 추정값은 측량값이나 충돌 검증 자료가 아니다.
 */
[[nodiscard]] std::vector<MapBuilding> LoadMapBuildings(
    const std::filesystem::path& path, const TerrainGrid& terrain);

/** @brief 높이가 없는 건물에 임시 높이를 붙여 경로·충돌용 형상으로 읽는다. */
[[nodiscard]] std::vector<MapBuilding> LoadMapBuildingFootprints(
    const std::filesystem::path& path, const TerrainGrid& terrain);

/** @brief 건물 외곽의 동·북 범위와 지면·높이 범위에서 충돌용 상자를 만든다. */
[[nodiscard]] MapBuildingBoundsMeters GetMapBuildingBoundsMeters(
    const MapBuilding& building);

/** @brief 주어진 3차원 위치가 건물 외곽과 높이 범위 안에 있는지 확인한다. */
[[nodiscard]] bool ContainsMapBuildingVolume(
    const MapBuilding& building, const glm::dvec3& positionMeters) noexcept;

/** @brief 선분을 따라가며 건물 안을 벗어나는 첫 위치를 찾는다. */
[[nodiscard]] std::optional<glm::dvec3> FindFirstPointOutsideMapBuildings(
    const glm::dvec3& startMeters,
    const glm::dvec3& endMeters,
    const std::vector<MapBuilding>& buildings,
    double sampleSpacingMeters);

/** @brief 접근 위치에서 가장 가까운 실제 외곽선 변을 외벽 시험면으로 만든다. */
[[nodiscard]] MapBuildingFacade GetNearestMapBuildingFacade(
    const MapBuilding& building, const glm::dvec2& approachPositionMeters);

/** @brief 건물 외곽선의 각 변을 외벽 시험면으로 만든다. */
[[nodiscard]] std::vector<MapBuildingFacade> GetMapBuildingFacades(
    const MapBuilding& building);
}
