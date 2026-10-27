#include "AeroSimLabApp.h"
#include "ExecutableDirectory.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include "graphics/Camera.h"
#include "graphics/GlbModel.h"
#include "graphics/Mesh.h"
#ifndef __EMSCRIPTEN__
#include "graphics/MultisampleFramebuffer.h"
#endif
#include "graphics/Renderer.h"
#include "graphics/Texture2D.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
#ifndef __EMSCRIPTEN__
#include "SimulatorHud.h"
#include "media/RtpVideoStream.h"
#endif
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/InspectionSurfaceJson.h"
#include "simulation_core/ScopedDuration.h"
#include "terrain/MapBridges.h"
#include "terrain/MapBuildings.h"
#include "terrain/MapFlightRoute.h"
#include "terrain/MapRoads.h"
#include "terrain/MapWaterBodies.h"
#include "terrain/TerrainGrid.h"
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif
#ifndef __EMSCRIPTEN__
#include "vision/RgbFrameQuality.h"
#endif
#include <glm/gtc/matrix_transform.hpp>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <ctime>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace aerosimlab::simulator
{
namespace
{
#ifdef __EMSCRIPTEN__
AeroSimLabApp* activeWebApplication = nullptr;
#endif
void SetStartupDetail(std::string_view message)
{
#ifdef __EMSCRIPTEN__
    const std::string text(message);
    EM_ASM({ window.setSimulatorStartupStatus(UTF8ToString($0)); }, text.c_str());
#else
    static_cast<void>(message);
#endif
}

constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
// 탄천유수지 체육시설 주소 지점에 둔 시뮬레이션용 출발 위치다.
constexpr glm::dvec2 demoDroneStartMeters{-410.0, 457.0};
// GLB 원점은 장착 브래킷이고 렌즈 중심은 그 원점에서 앞·위로 떨어져 있다.
constexpr glm::vec3 cameraLensCenterInModelMeters{0.015F, 0.048F, 0.030F};
constexpr double bridgeDeckUndersideHeightMeters = 6.0;
constexpr double bridgeDeckThicknessMeters = 0.8;
constexpr double bridgeInspectionApproachBelowDeckMeters = 1.2;
// 나무 GLB 중 가장 높은 모델에 최대 배율을 적용한 높이다.
constexpr double tallestTreeModelHeightMeters = 25.5;
// 전방 카메라 시야가 수관에 막히지 않도록 통과 고도에 더하는 여유다.
constexpr double cameraClearanceAboveCanopyMeters = 14.0;

std::filesystem::path AssetPath(std::string_view relativePath)
{
    return detail::ExecutableDirectory() / relativePath;
}

glm::vec3 ToRenderPosition(const glm::dvec3& positionMeters)
{
    // 모의는 Z축을 위로 쓰고 화면은 Y축을 위로 쓴다. Y축 방향은 화면의 -Z로 옮겨 오른손 좌표계를 유지한다.
    return glm::vec3{
        static_cast<float>(positionMeters.x), static_cast<float>(positionMeters.z),
        static_cast<float>(-positionMeters.y)};
}

glm::vec3 ToRenderDirection(const glm::dvec3& direction)
{
    return glm::vec3{
        static_cast<float>(direction.x), static_cast<float>(direction.z),
        static_cast<float>(-direction.y)};
}

glm::dvec3 ReadPlanVector(const nlohmann::json& value, std::string_view field)
{
    if (!value.is_array() || value.size() != 3)
        throw std::runtime_error(std::string(field) + "은 숫자 세 개가 필요합니다.");
    glm::dvec3 result;
    for (int axis = 0; axis < 3; ++axis)
    {
        if (!value[axis].is_number())
            throw std::runtime_error(std::string(field) + "에 숫자가 아닌 값이 있습니다.");
        result[axis] = value[axis].get<double>();
        if (!std::isfinite(result[axis]))
            throw std::runtime_error(std::string(field) + "에 유효하지 않은 값이 있습니다.");
    }
    return result;
}

glm::quat ToRenderOrientation(const glm::dquat& orientation)
{
    constexpr float halfPi = 1.57079632679F;
    const glm::quat simulationOrientation{
        static_cast<float>(orientation.w),
        static_cast<float>(orientation.x),
        static_cast<float>(orientation.y),
        static_cast<float>(orientation.z)};
    const glm::quat simulationToRender = glm::angleAxis(
        -halfPi, glm::vec3{1.0F, 0.0F, 0.0F});
    return glm::normalize(
        simulationToRender * simulationOrientation * glm::inverse(simulationToRender));
}

glm::mat4 CubeTransform(const glm::vec3& position, const glm::vec3& size)
{
    return glm::translate(glm::mat4{1.0F}, position) * glm::scale(glm::mat4{1.0F}, size);
}

double Cross2D(const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c)
{
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

double PolygonArea(const std::vector<glm::dvec2>& outline)
{
    double twiceArea = 0.0;
    for (std::size_t i = 0; i < outline.size(); ++i)
    {
        const glm::dvec2& current = outline[i];
        const glm::dvec2& next = outline[(i + 1U) % outline.size()];
        twiceArea += current.x * next.y - next.x * current.y;
    }
    return twiceArea * 0.5;
}

bool IsInsideTriangle(const glm::dvec2& point,
    const glm::dvec2& a, const glm::dvec2& b, const glm::dvec2& c,
    double winding)
{
    constexpr double tolerance = 1.0e-9;
    return winding * Cross2D(a, b, point) >= -tolerance
        && winding * Cross2D(b, c, point) >= -tolerance
        && winding * Cross2D(c, a, point) >= -tolerance;
}

std::vector<std::array<std::uint32_t, 3>> TriangulateFootprint(
    const std::vector<glm::dvec2>& outline)
{
    const double area = PolygonArea(outline);
    if (std::abs(area) < 1.0e-8)
        return {};

    const double winding = area > 0.0 ? 1.0 : -1.0;
    std::vector<std::uint32_t> remaining;
    std::vector<std::array<std::uint32_t, 3>> triangles;
    remaining.reserve(outline.size());
    triangles.reserve(outline.size() - 2U);
    for (std::uint32_t i = 0; i < outline.size(); ++i)
        remaining.push_back(i);

    while (remaining.size() > 3U)
    {
        bool removedEar = false;
        for (std::size_t i = 0; i < remaining.size(); ++i)
        {
            const std::uint32_t previous = remaining[(i + remaining.size() - 1U)
                % remaining.size()];
            const std::uint32_t current = remaining[i];
            const std::uint32_t next = remaining[(i + 1U) % remaining.size()];
            if (winding * Cross2D(outline[previous], outline[current], outline[next])
                <= 1.0e-9)
                continue;

            bool containsVertex = false;
            for (const std::uint32_t candidate : remaining)
            {
                if (candidate == previous || candidate == current || candidate == next)
                    continue;
                if (IsInsideTriangle(outline[candidate], outline[previous],
                    outline[current], outline[next], winding))
                {
                    containsVertex = true;
                    break;
                }
            }
            if (containsVertex)
                continue;

            triangles.push_back({previous, current, next});
            remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(i));
            removedEar = true;
            break;
        }
        if (!removedEar)
            return {};
    }
    triangles.push_back({remaining[0], remaining[1], remaining[2]});
    return triangles;
}

std::unique_ptr<aerosimlab::graphics::Mesh> CreateBuildingMesh(
    const std::vector<aerosimlab::terrain::MapBuilding>& buildings,
    aerosimlab::terrain::BuildingHeightSource heightSource,
    bool createWalls)
{
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;
    for (const auto& building : buildings)
    {
        if (building.heightSource != heightSource)
            continue;
        const auto triangles = TriangulateFootprint(building.outlineMeters);
        if (triangles.empty())
            continue;

        // 충돌체와 같은 바닥·지붕 높이를 써 화면과 충돌 범위를 맞춘다.
        const auto buildingBounds =
            aerosimlab::terrain::GetMapBuildingBoundsMeters(building);
        const double bottomElevation = buildingBounds.centerMeters.z
            - buildingBounds.halfExtentsMeters.z;
        const double roofElevation = buildingBounds.centerMeters.z
            + buildingBounds.halfExtentsMeters.z;

        const double winding = PolygonArea(building.outlineMeters) > 0.0 ? 1.0 : -1.0;
        if (!createWalls)
        {
            const std::uint32_t roofStart = static_cast<std::uint32_t>(vertices.size());
            // 콘크리트 재질의 실제 크기인 2.1m에 맞춰 반복한다.
            constexpr double roofTextureTileSizeMeters = 2.1;
            for (std::size_t i = 0; i < building.outlineMeters.size(); ++i)
            {
                const auto& point = building.outlineMeters[i];
                vertices.push_back({ToRenderPosition({point.x, point.y, roofElevation}),
                    {0.0F, 1.0F, 0.0F},
                    {static_cast<float>(point.x / roofTextureTileSizeMeters),
                        static_cast<float>(-point.y / roofTextureTileSizeMeters)}});
            }
            for (const auto& triangle : triangles)
            {
                if (winding > 0.0)
                    indices.insert(indices.end(), {roofStart + triangle[0],
                        roofStart + triangle[2], roofStart + triangle[1]});
                else
                    indices.insert(indices.end(), {roofStart + triangle[0],
                        roofStart + triangle[1], roofStart + triangle[2]});
            }
        }

        if (!createWalls)
            continue;
        for (std::size_t i = 0; i < building.outlineMeters.size(); ++i)
        {
            const std::size_t next = (i + 1U) % building.outlineMeters.size();
            const auto& firstPoint = building.outlineMeters[i];
            const auto& secondPoint = building.outlineMeters[next];
            const glm::dvec2 edge = secondPoint - firstPoint;
            glm::dvec3 outward{edge.y, 0.0, edge.x};
            outward *= winding;
            const glm::vec3 normal = glm::normalize(ToRenderDirection(outward));
            const float edgeLength = static_cast<float>(glm::length(edge));
            const float wallHeight = static_cast<float>(roofElevation - bottomElevation);
            constexpr float facadeTileWidthMeters = 5.0F;
            constexpr float facadeTileHeightMeters = 3.0F;
            const auto bottomFirst = ToRenderPosition({firstPoint.x, firstPoint.y,
                bottomElevation});
            const auto bottomSecond = ToRenderPosition({secondPoint.x, secondPoint.y,
                bottomElevation});
            const auto topFirst = ToRenderPosition({firstPoint.x, firstPoint.y,
                roofElevation});
            const auto topSecond = ToRenderPosition({secondPoint.x, secondPoint.y,
                roofElevation});
            const std::uint32_t wallStart = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {
                {bottomFirst, normal, {0.0F, wallHeight / facadeTileHeightMeters}},
                {bottomSecond, normal, {edgeLength / facadeTileWidthMeters,
                    wallHeight / facadeTileHeightMeters}},
                {topSecond, normal, {edgeLength / facadeTileWidthMeters, 0.0F}},
                {topFirst, normal, {0.0F, 0.0F}}});
            indices.insert(indices.end(), {wallStart, wallStart + 1U, wallStart + 2U,
                wallStart, wallStart + 2U, wallStart + 3U});
        }
    }

    if (indices.empty())
        return {};
    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}

std::unique_ptr<aerosimlab::graphics::Mesh> CreateRoadMesh(
    const std::vector<aerosimlab::terrain::MapRoad>& roads,
    bool bridgeOnly)
{
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;
    for (const auto& road : roads)
    {
        if (road.bridge != bridgeOnly || road.centerlineMeters.size() < 2U)
            continue;

        for (std::size_t pointIndex = 1;
            pointIndex < road.centerlineMeters.size(); ++pointIndex)
        {
            const glm::dvec3& first = road.centerlineMeters[pointIndex - 1U];
            const glm::dvec3& second = road.centerlineMeters[pointIndex];
            const glm::dvec2 direction{second.x - first.x, second.y - first.y};
            const double length = glm::length(direction);
            if (!(length > 1.0e-6))
                continue;

            const glm::dvec2 halfWidth{-direction.y, direction.x};
            const glm::dvec2 lateralOffset = halfWidth
                * (road.widthMeters / (2.0 * length));
            constexpr double surfaceOffsetMeters = 0.15;
            const auto toSurface = [surfaceOffsetMeters](
                const glm::dvec3& center, const glm::dvec2& offset)
            {
                return ToRenderPosition({center.x + offset.x,
                    center.y + offset.y, center.z + surfaceOffsetMeters});
            };
            const glm::vec3 firstLeft = toSurface(first, lateralOffset);
            const glm::vec3 firstRight = toSurface(first, -lateralOffset);
            const glm::vec3 secondLeft = toSurface(second, lateralOffset);
            const glm::vec3 secondRight = toSurface(second, -lateralOffset);
            const glm::vec3 normal = glm::normalize(glm::cross(
                firstRight - firstLeft, secondRight - firstLeft));
            const auto firstVertex = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {
                {firstLeft, normal, {0.0F, 0.0F}},
                {firstRight, normal, {0.0F, 1.0F}},
                {secondRight, normal, {1.0F, 1.0F}},
                {secondLeft, normal, {1.0F, 0.0F}}});
            indices.insert(indices.end(), {firstVertex, firstVertex + 1U,
                firstVertex + 2U, firstVertex, firstVertex + 2U,
                firstVertex + 3U});
        }
    }

    if (indices.empty())
        return {};
    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}

enum class BridgeMeshPart
{
    DeckSurface,
    DeckStructure
};

/** @brief 교량 외곽선으로 영상이 이어지는 상판과 충돌 두께를 그린다. */
std::unique_ptr<aerosimlab::graphics::Mesh> CreateBridgeFootprintMesh(
    const std::vector<aerosimlab::terrain::MapBridgeFootprint>& bridges,
    const aerosimlab::terrain::TerrainGrid& terrain,
    BridgeMeshPart meshPart)
{
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;

    for (const auto& bridge : bridges)
    {
        if (bridge.outlineMeters.size() < 3U
            || bridge.outlineMeters.size() != bridge.groundElevationsMeters.size())
            continue;

        glm::dvec2 center{0.0};
        double groundElevation = 0.0;
        for (std::size_t index = 0; index < bridge.outlineMeters.size(); ++index)
        {
            center += bridge.outlineMeters[index];
            groundElevation += bridge.groundElevationsMeters[index];
        }
        center /= static_cast<double>(bridge.outlineMeters.size());
        groundElevation /= static_cast<double>(bridge.groundElevationsMeters.size());
        const double deckBottom = groundElevation + bridgeDeckUndersideHeightMeters;
        const double deckTop = deckBottom + bridgeDeckThicknessMeters;
        if (meshPart == BridgeMeshPart::DeckSurface)
        {
            const std::uint32_t centerIndex = static_cast<std::uint32_t>(vertices.size());
            const double halfWidth = terrain.HalfWidthMeters();
            const double halfDepth = terrain.HalfDepthMeters();
            const auto addSurfaceVertex = [&](const glm::dvec2& point)
            {
                const glm::vec2 uv{
                    static_cast<float>((point.x + halfWidth) / (2.0 * halfWidth)),
                    static_cast<float>((halfDepth - point.y) / (2.0 * halfDepth))};
                vertices.push_back({ToRenderPosition({point.x, point.y, deckTop}),
                    {0.0F, 1.0F, 0.0F}, uv});
            };
            addSurfaceVertex(center);
            for (const glm::dvec2& point : bridge.outlineMeters)
                addSurfaceVertex(point);
            for (std::uint32_t index = 0; index < bridge.outlineMeters.size(); ++index)
            {
                const std::uint32_t first = centerIndex + 1U + index;
                const std::uint32_t second = centerIndex + 1U
                    + (index + 1U)
                        % static_cast<std::uint32_t>(bridge.outlineMeters.size());
                indices.insert(indices.end(), {centerIndex, first, second});
            }
            continue;
        }

        const std::uint32_t undersideCenter = static_cast<std::uint32_t>(vertices.size());
        vertices.push_back({ToRenderPosition({center.x, center.y, deckBottom}),
            {0.0F, -1.0F, 0.0F}, {0.5F, 0.5F}});
        for (const glm::dvec2& point : bridge.outlineMeters)
            vertices.push_back({ToRenderPosition({point.x, point.y, deckBottom}),
                {0.0F, -1.0F, 0.0F},
                {static_cast<float>(point.x), static_cast<float>(point.y)}});
        for (std::uint32_t index = 0; index < bridge.outlineMeters.size(); ++index)
        {
            const std::uint32_t first = undersideCenter + 1U + index;
            const std::uint32_t second = undersideCenter + 1U
                + (index + 1U) % static_cast<std::uint32_t>(bridge.outlineMeters.size());
            indices.insert(indices.end(), {undersideCenter, second, first});
        }

        for (std::size_t index = 0; index < bridge.outlineMeters.size(); ++index)
        {
            const std::size_t next = (index + 1U) % bridge.outlineMeters.size();
            const glm::dvec2& first = bridge.outlineMeters[index];
            const glm::dvec2& second = bridge.outlineMeters[next];
            const glm::vec3 normal = glm::normalize(glm::vec3{
                static_cast<float>(second.y - first.y), 0.0F,
                static_cast<float>(second.x - first.x)});
            const std::uint32_t quadStart = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {
                {ToRenderPosition({first.x, first.y, deckBottom}), normal, {0.0F, 0.0F}},
                {ToRenderPosition({second.x, second.y, deckBottom}), normal, {1.0F, 0.0F}},
                {ToRenderPosition({second.x, second.y, deckTop}), normal, {1.0F, 1.0F}},
                {ToRenderPosition({first.x, first.y, deckTop}), normal, {0.0F, 1.0F}}});
            indices.insert(indices.end(), {quadStart, quadStart + 1U, quadStart + 2U,
                quadStart, quadStart + 2U, quadStart + 3U});
        }
    }

    if (indices.empty())
        return {};
    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}

/** @brief 같은 교각 발자국으로 화면에 보일 기둥 메시를 만든다. */
std::unique_ptr<aerosimlab::graphics::Mesh> CreateBridgePierMesh(
    const std::vector<aerosimlab::terrain::MapBridgePier>& piers)
{
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;
    for (const auto& pier : piers)
    {
        const auto triangles = TriangulateFootprint(pier.outlineMeters);
        if (triangles.empty())
            continue;

        const double winding = PolygonArea(pier.outlineMeters) > 0.0 ? 1.0 : -1.0;
        const std::uint32_t topStart = static_cast<std::uint32_t>(vertices.size());
        for (const auto& point : pier.outlineMeters)
        {
            vertices.push_back({ToRenderPosition(
                {point.x, point.y, pier.topElevationMeters}),
                {0.0F, 1.0F, 0.0F}, {static_cast<float>(point.x),
                    static_cast<float>(point.y)}});
        }
        for (const auto& triangle : triangles)
        {
            if (winding > 0.0)
                indices.insert(indices.end(), {topStart + triangle[0],
                    topStart + triangle[2], topStart + triangle[1]});
            else
                indices.insert(indices.end(), {topStart + triangle[0],
                    topStart + triangle[1], topStart + triangle[2]});
        }

        for (std::size_t index = 0; index < pier.outlineMeters.size(); ++index)
        {
            const std::size_t next = (index + 1U) % pier.outlineMeters.size();
            const glm::dvec2& first = pier.outlineMeters[index];
            const glm::dvec2& second = pier.outlineMeters[next];
            const glm::dvec2 edge = second - first;
            glm::dvec3 outward{edge.y, 0.0, edge.x};
            outward *= winding;
            const glm::vec3 normal = glm::normalize(ToRenderDirection(outward));
            const float edgeLength = static_cast<float>(glm::length(edge));
            const float pierHeight = static_cast<float>(
                pier.topElevationMeters - pier.bottomElevationMeters);
            constexpr float concreteTileWidthMeters = 5.0F;
            constexpr float concreteTileHeightMeters = 3.0F;
            const std::uint32_t wallStart = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {
                {ToRenderPosition({first.x, first.y, pier.bottomElevationMeters}),
                    normal, {0.0F, pierHeight / concreteTileHeightMeters}},
                {ToRenderPosition({second.x, second.y, pier.bottomElevationMeters}),
                    normal, {edgeLength / concreteTileWidthMeters,
                        pierHeight / concreteTileHeightMeters}},
                {ToRenderPosition({second.x, second.y, pier.topElevationMeters}),
                    normal, {edgeLength / concreteTileWidthMeters, 0.0F}},
                {ToRenderPosition({first.x, first.y, pier.topElevationMeters}),
                    normal, {0.0F, 0.0F}}});
            indices.insert(indices.end(), {wallStart, wallStart + 1U, wallStart + 2U,
                wallStart, wallStart + 2U, wallStart + 3U});
        }
    }

    if (indices.empty())
        return {};
    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}

std::unique_ptr<aerosimlab::graphics::Mesh> CreateTerrainMesh(
    const aerosimlab::terrain::TerrainGrid& terrain)
{
    const std::uint32_t count = terrain.sampleCount;
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;
    vertices.reserve(static_cast<std::size_t>(count) * count);
    indices.reserve(static_cast<std::size_t>(count - 1U) * (count - 1U) * 6U);

    for (std::uint32_t row = 0; row < count; ++row)
    {
        for (std::uint32_t column = 0; column < count; ++column)
        {
            const float east = (static_cast<float>(column) -
                static_cast<float>(count - 1U) * 0.5F) * terrain.spacingXMeters;
            // 격자 0행은 북쪽 끝이므로, 지도와 지형의 남북 방향을 맞춘다.
            const float north = (static_cast<float>(count - 1U) * 0.5F -
                static_cast<float>(row)) * terrain.spacingYMeters;
            // 높이 변화량으로 경사를 구해야 지형의 조명이 실제 굴곡을 따라간다.
            const std::uint32_t westColumn = column > 0U ? column - 1U : column;
            const std::uint32_t eastColumn = std::min(column + 1U, count - 1U);
            const std::uint32_t northRow = row > 0U ? row - 1U : row;
            const std::uint32_t southRow = std::min(row + 1U, count - 1U);
            const float eastDistance = static_cast<float>(eastColumn - westColumn)
                * terrain.spacingXMeters;
            const float southDistance = static_cast<float>(southRow - northRow)
                * terrain.spacingYMeters;
            const float eastSlope = (terrain.HeightAt(row, eastColumn)
                - terrain.HeightAt(row, westColumn)) / eastDistance;
            const float southSlope = (terrain.HeightAt(southRow, column)
                - terrain.HeightAt(northRow, column)) / southDistance;
            const glm::vec3 surfaceNormal = glm::normalize(
                glm::vec3{-eastSlope, 1.0F, -southSlope});
            vertices.push_back({
                {east, terrain.referenceElevationMeters + terrain.HeightAt(row, column), -north},
                surfaceNormal,
                {static_cast<float>(column) / static_cast<float>(count - 1U),
                    static_cast<float>(row) / static_cast<float>(count - 1U)}});
        }
    }

    for (std::uint32_t row = 0; row + 1U < count; ++row)
    {
        for (std::uint32_t column = 0; column + 1U < count; ++column)
        {
            const std::uint32_t northwest = row * count + column;
            const std::uint32_t northeast = northwest + 1U;
            const std::uint32_t southwest = northwest + count;
            const std::uint32_t southeast = southwest + 1U;
            indices.insert(indices.end(), {northwest, southwest, northeast,
                northeast, southwest, southeast});
        }
    }

    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}

/** @brief 물 안에 있는 지형 삼각형을 수면으로 덮어 지형과 틈이 생기지 않게 한다. */
std::unique_ptr<aerosimlab::graphics::Mesh> CreateWaterMesh(
    const std::vector<aerosimlab::terrain::MapWaterBody>& waterBodies,
    const aerosimlab::terrain::TerrainGrid& terrain)
{
    constexpr double surfaceOffsetMeters = 0.25;
    constexpr double waterBoundaryBucketSizeMeters = 1.0;
    const double halfX = terrain.HalfWidthMeters();
    const double halfY = terrain.HalfDepthMeters();
    const auto subdivisions = static_cast<std::uint32_t>(std::max(1.0, std::ceil(
        std::max(static_cast<double>(terrain.spacingXMeters),
            static_cast<double>(terrain.spacingYMeters)))));
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;

    struct WaterSurfacePoint
    {
        glm::dvec2 position;
        double elevation;
    };

    for (const auto& body : waterBodies)
    {
        if (body.outerBoundaryMeters.empty())
            continue;

        double minimumX = body.outerBoundaryMeters.front().x;
        double maximumX = minimumX;
        double minimumY = body.outerBoundaryMeters.front().y;
        double maximumY = minimumY;
        for (const glm::dvec2& point : body.outerBoundaryMeters)
        {
            minimumX = std::min(minimumX, point.x);
            maximumX = std::max(maximumX, point.x);
            minimumY = std::min(minimumY, point.y);
            maximumY = std::max(maximumY, point.y);
        }

        struct BoundarySegment
        {
            glm::dvec2 first;
            glm::dvec2 second;
        };
        const std::size_t bucketCount = static_cast<std::size_t>(
            std::ceil((maximumY - minimumY) / waterBoundaryBucketSizeMeters)) + 1U;
        std::vector<std::vector<BoundarySegment>> boundaryBuckets(bucketCount);
        const auto addRing = [&](const std::vector<glm::dvec2>& ring)
        {
            for (std::size_t index = 0U; index < ring.size(); ++index)
            {
                const glm::dvec2 first = ring[index];
                const glm::dvec2 second = ring[(index + 1U) % ring.size()];
                if (first.y == second.y)
                    continue;
                const std::size_t firstBucket = std::min(
                    static_cast<std::size_t>(std::floor(
                        (std::min(first.y, second.y) - minimumY)
                        / waterBoundaryBucketSizeMeters)),
                    boundaryBuckets.size() - 1U);
                const std::size_t lastBucket = std::min(
                    static_cast<std::size_t>(std::floor(
                        (std::max(first.y, second.y) - minimumY)
                        / waterBoundaryBucketSizeMeters)),
                    boundaryBuckets.size() - 1U);
                for (std::size_t bucket = firstBucket; bucket <= lastBucket; ++bucket)
                    boundaryBuckets[bucket].push_back({first, second});
            }
        };
        addRing(body.outerBoundaryMeters);
        for (const auto& inner : body.innerBoundariesMeters)
            addRing(inner);

        const auto containsWater = [&](const glm::dvec2& point)
        {
            if (point.x < minimumX || point.x > maximumX
                || point.y < minimumY || point.y > maximumY)
                return false;
            const std::size_t bucket = std::min(
                static_cast<std::size_t>(std::floor(
                    (point.y - minimumY) / waterBoundaryBucketSizeMeters)),
                boundaryBuckets.size() - 1U);
            bool inside = false;
            for (const auto& segment : boundaryBuckets[bucket])
            {
                if ((segment.first.y > point.y) == (segment.second.y > point.y))
                    continue;
                const double intersectionX = segment.first.x
                    + (point.y - segment.first.y)
                        * (segment.second.x - segment.first.x)
                        / (segment.second.y - segment.first.y);
                if (point.x < intersectionX)
                    inside = !inside;
            }
            return inside;
        };

        if (maximumX < -halfX || minimumX > halfX
            || maximumY < -halfY || minimumY > halfY)
            continue;

        const auto gridIndex = [](double position, double origin, double spacing,
                                   std::uint32_t sampleCount)
        {
            const double cell = std::floor((position - origin) / spacing);
            return static_cast<std::uint32_t>(std::clamp(
                cell, 0.0, static_cast<double>(sampleCount - 2U)));
        };
        const std::uint32_t firstColumn = gridIndex(
            minimumX, -halfX, terrain.spacingXMeters, terrain.sampleCount);
        const std::uint32_t lastColumn = gridIndex(
            maximumX, -halfX, terrain.spacingXMeters, terrain.sampleCount);
        const std::uint32_t firstRow = gridIndex(
            -maximumY, -halfY, terrain.spacingYMeters, terrain.sampleCount);
        const std::uint32_t lastRow = gridIndex(
            -minimumY, -halfY, terrain.spacingYMeters, terrain.sampleCount);

        for (std::uint32_t row = firstRow; row <= lastRow; ++row)
        {
            const double north = halfY - static_cast<double>(row) * terrain.spacingYMeters;
            const double south = north - terrain.spacingYMeters;
            for (std::uint32_t column = firstColumn; column <= lastColumn; ++column)
            {
                const double west = -halfX + static_cast<double>(column)
                    * terrain.spacingXMeters;
                const double east = west + terrain.spacingXMeters;
                const glm::dvec2 northWest{west, north};
                const glm::dvec2 northEast{east, north};
                const glm::dvec2 southWest{west, south};
                const glm::dvec2 southEast{east, south};
                const auto addTriangle = [&](const glm::dvec2& firstPoint,
                                             const glm::dvec2& secondPoint,
                                             const glm::dvec2& thirdPoint,
                                             std::uint32_t firstRow,
                                             std::uint32_t firstColumn,
                                             std::uint32_t secondRow,
                                             std::uint32_t secondColumn,
                                             std::uint32_t thirdRow,
                                             std::uint32_t thirdColumn)
                {
                    const auto makePoint = [&](const glm::dvec2& point,
                                               std::uint32_t vertexRow,
                                               std::uint32_t vertexColumn)
                    {
                        const double elevation = terrain.referenceElevationMeters
                            + terrain.HeightAt(vertexRow, vertexColumn)
                            + surfaceOffsetMeters;
                        return WaterSurfacePoint{point, elevation};
                    };
                    const std::array<WaterSurfacePoint, 3> triangle{
                        makePoint(firstPoint, firstRow, firstColumn),
                        makePoint(secondPoint, secondRow, secondColumn),
                        makePoint(thirdPoint, thirdRow, thirdColumn)};
                    const auto append = [&](const WaterSurfacePoint& first,
                                            const WaterSurfacePoint& second,
                                            const WaterSurfacePoint& third)
                    {
                        const glm::dvec2 center =
                            (first.position + second.position + third.position) / 3.0;
                        if (!containsWater(center))
                            return;
                        const std::uint32_t triangleStart =
                            static_cast<std::uint32_t>(vertices.size());
                        const auto addVertex = [&](const WaterSurfacePoint& point)
                        {
                            vertices.push_back({ToRenderPosition({point.position.x,
                                point.position.y, point.elevation}),
                                {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F}});
                        };
                        addVertex(first);
                        addVertex(second);
                        addVertex(third);
                        indices.insert(indices.end(), {triangleStart,
                            triangleStart + 1U, triangleStart + 2U});
                    };

                    const glm::dvec2 center =
                        (firstPoint + secondPoint + thirdPoint) / 3.0;
                    const bool firstInside = containsWater(firstPoint);
                    const bool secondInside = containsWater(secondPoint);
                    const bool thirdInside = containsWater(thirdPoint);
                    const bool centerInside = containsWater(center);
                    if (firstInside && secondInside && thirdInside && centerInside)
                    {
                        append(triangle[0], triangle[1], triangle[2]);
                        return;
                    }

                    const glm::dvec2 firstMidpoint = (firstPoint + secondPoint) * 0.5;
                    const glm::dvec2 secondMidpoint = (secondPoint + thirdPoint) * 0.5;
                    const glm::dvec2 thirdMidpoint = (thirdPoint + firstPoint) * 0.5;
                    if (!firstInside && !secondInside && !thirdInside && !centerInside
                        && !containsWater(firstMidpoint) && !containsWater(secondMidpoint)
                        && !containsWater(thirdMidpoint))
                        return;

                    const auto interpolate = [&](std::uint32_t eastSteps,
                                                  std::uint32_t southSteps)
                    {
                        const double firstWeight = 1.0
                            - static_cast<double>(eastSteps + southSteps) / subdivisions;
                        const double secondWeight =
                            static_cast<double>(eastSteps) / subdivisions;
                        const double thirdWeight =
                            static_cast<double>(southSteps) / subdivisions;
                        return WaterSurfacePoint{
                            triangle[0].position * firstWeight
                                + triangle[1].position * secondWeight
                                + triangle[2].position * thirdWeight,
                            triangle[0].elevation * firstWeight
                                + triangle[1].elevation * secondWeight
                                + triangle[2].elevation * thirdWeight};
                    };
                    for (std::uint32_t eastStep = 0U; eastStep < subdivisions; ++eastStep)
                    {
                        for (std::uint32_t southStep = 0U;
                             southStep + eastStep < subdivisions; ++southStep)
                        {
                            const WaterSurfacePoint first =
                                interpolate(eastStep, southStep);
                            const WaterSurfacePoint east =
                                interpolate(eastStep + 1U, southStep);
                            const WaterSurfacePoint south =
                                interpolate(eastStep, southStep + 1U);
                            append(first, east, south);

                            if (eastStep + southStep + 1U < subdivisions)
                            {
                                const WaterSurfacePoint eastSouth =
                                    interpolate(eastStep + 1U, southStep + 1U);
                                append(east, eastSouth, south);
                            }
                        }
                    }
                };

                // 지형 메시와 같은 대각선으로 나눠 수면과 지면이 서로 교차하지 않게 한다.
                addTriangle(northWest, southWest, northEast,
                    row, column, row + 1U, column, row, column + 1U);
                addTriangle(northEast, southWest, southEast,
                    row, column + 1U, row + 1U, column,
                    row + 1U, column + 1U);
            }
        }
    }

    if (indices.empty())
        return {};
    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}
}

AeroSimLabApp::AeroSimLabApp(Options options)
    : smokeTest_(options.smokeTest), demoMode_(options.demoMode),
      networkEnabled_(options.networkEnabled),
      benchmarkMode_(options.benchmarkMode || options.visibleBenchmarkMode),
      visibleBenchmarkMode_(options.visibleBenchmarkMode),
      groundControlCommandTimeoutTicks_(options.groundControlCommandTimeoutTicks),
      udpPort_(options.udpPort)
{
    if (groundControlCommandTimeoutTicks_ == 0)
        throw std::invalid_argument("지상통제 명령 유효기간은 1 Tick 이상이어야 합니다.");
    if (benchmarkMode_)
        simulation_.SetPhysicsStepTimingEnabled(true);
#ifndef __EMSCRIPTEN__
    visionAnalyzer_ = std::make_unique<aerosimlab::vision::RgbFrameAnalyzer>();
#else
    activeWebApplication = this;
#endif
}

AeroSimLabApp::~AeroSimLabApp()
{
#ifndef __EMSCRIPTEN__
    if (eoVideoStartWorker_.joinable())
        eoVideoStartWorker_.join();
    if (eoVideoSender_)
        eoVideoSender_->Stop();
#endif
#ifdef __EMSCRIPTEN__
    if (activeWebApplication == this)
        activeWebApplication = nullptr;
#endif
}

#ifdef __EMSCRIPTEN__
void AeroSimLabApp::QueueWebFlightCommand(
    aerosimlab::simulation::DroneCommand::Kind kind)
{
    externalControl_ = true;
    if (InspectionMissionIsActive())
        return;
    if (!commandRecording_.IsReplaying())
        pendingControllerAction_ = aerosimlab::simulation::DroneCommand{kind};
}

bool AeroSimLabApp::StartWebInspectionMission(int targetIndex)
{
    externalControl_ = true;
    if (targetIndex < -1
        || (targetIndex >= 0
            && static_cast<std::size_t>(targetIndex) >= inspectionTargets_.size()))
        return false;

    std::optional<std::size_t> selectedTarget;
    if (targetIndex >= 0)
        selectedTarget = static_cast<std::size_t>(targetIndex);
    return RequestInspectionMission(selectedTarget);
}

bool AeroSimLabApp::PauseWebInspectionMission()
{
    externalControl_ = true;
    return PauseInspectionMission();
}

bool AeroSimLabApp::ResumeWebInspectionMission()
{
    externalControl_ = true;
    return ResumeInspectionMission();
}

bool AeroSimLabApp::CancelWebInspectionMission()
{
    externalControl_ = true;
    return CancelInspectionMission();
}

void AeroSimLabApp::SetWebPilotAxes(
    const aerosimlab::manual_control::GamepadAxes& axes)
{
    externalControl_ = true;
    if (InspectionMissionIsActive())
    {
        webPilotAxes_.reset();
        pendingPilotCommand_.reset();
        return;
    }
    if (!commandRecording_.IsReplaying())
        webPilotAxes_ = axes;
}
#endif

void AeroSimLabApp::InitializeGraphics()
{
    InitializeWindow();
    InitializeRendererAndCameras();
    LoadScene();
    InitializeShaderAndHud();
}

#ifdef __EMSCRIPTEN__
bool AeroSimLabApp::InitializeGraphicsStep()
{
    switch (webStartupStep_++)
    {
    case 0:
        EM_ASM({ window.setSimulatorStartupStatus("화면을 준비하고 있습니다."); });
        InitializeWindow();
        return false;
    case 1:
        EM_ASM({ window.setSimulatorStartupStatus("렌더링 장치를 준비하고 있습니다."); });
        InitializeRendererAndCameras();
        return false;
    case 2:
        EM_ASM({ window.setSimulatorStartupStatus("지형 자료를 읽고 있습니다."); });
        LoadSceneModels();
        LoadTerrain();
        return false;
    case 3:
        EM_ASM({ window.setSimulatorStartupStatus("시설물 형상을 만들고 있습니다."); });
        LoadMapFeatures();
        return false;
    case 4:
        EM_ASM({ window.setSimulatorStartupStatus("비행 시작 위치를 준비하고 있습니다."); });
        InitializeStartingState();
        return false;
    case 5:
        EM_ASM({ window.setSimulatorStartupStatus("화면 구성을 마무리하고 있습니다."); });
        InitializeShaderAndHud();
        return false;
    default:
        return true;
    }
}
#endif

void AeroSimLabApp::InitializeWindow()
{
    window_ = std::make_unique<aerosimlab::graphics::Window>(
        aerosimlab::graphics::Window::Properties{windowWidth, windowHeight,
            "AeroSimLab | 드론 관찰 · EO 영상 | © OpenStreetMap contributors",
            !smokeTest_ && !benchmarkMode_,
            !smokeTest_ && (!benchmarkMode_ || visibleBenchmarkMode_)});
    window_->GetFramebufferSize(framebufferWidth_, framebufferHeight_);
}

void AeroSimLabApp::InitializeRendererAndCameras()
{
    renderer_ = std::make_unique<aerosimlab::graphics::Renderer>();
    renderer_->Init(framebufferWidth_, framebufferHeight_);
#ifndef __EMSCRIPTEN__
    sceneTarget_ = std::make_unique<aerosimlab::graphics::MultisampleFramebuffer>(
        framebufferWidth_, framebufferHeight_);
    cameraTarget_ = std::make_unique<aerosimlab::graphics::MultisampleFramebuffer>(
        framebufferWidth_, framebufferHeight_);
#endif
    observerCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{0.0F, 320.0F, 700.0F}, glm::vec3{0.0F, 18.0F, 0.0F}, 1.0F,
        45.0F, 0.1F, 2000.0F);
    eoCamera_ = std::make_unique<aerosimlab::graphics::Camera>(
        glm::vec3{0.0F}, glm::vec3{1.0F, 0.0F, 0.0F}, 1.0F,
        static_cast<float>(simulation_.Camera().model.verticalFieldOfViewDegrees),
        0.05F, 200.0F);
}

void AeroSimLabApp::InitializeShaderAndHud()
{
    shader_ = std::make_unique<aerosimlab::graphics::Shader>(
        AssetPath("shaders/Scene.glsl").string());
#ifndef __EMSCRIPTEN__
    hud_ = std::make_unique<SimulatorHud>(window_->GetNativeHandle());
#endif
}

void AeroSimLabApp::LoadScene()
{
    LoadSceneModels();
    LoadTerrain();
    LoadMapFeatures();
    InitializeStartingState();
}

void AeroSimLabApp::LoadSceneModels()
{
    cube_ = aerosimlab::graphics::Mesh::CreateCube(1.0F);
    droneModel_ = aerosimlab::graphics::GlbModel::Load(
        AssetPath("models/drones/x500/visual.glb").string());
    droneCameraModel_ = aerosimlab::graphics::GlbModel::Load(
        AssetPath("models/drones/x500/action-camera.glb").string());
    droneSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadFromFile(
        AssetPath("models/drones/x500/carbon-weave.ppm"));
    buildingFacadeTexture_ = aerosimlab::graphics::Texture2D::LoadFromFile(
        AssetPath("maps/tancheon/apartment-facade.png"), true);
    buildingRoofTexture_ = aerosimlab::graphics::Texture2D::LoadFromFile(
        AssetPath("maps/tancheon/roof-concrete.jpg"), true);
    constexpr std::array<const char*, 4> terrainTextureExtensions{
        "png", "jpg", "jpeg", "ppm"};
    for (const char* extension : terrainTextureExtensions)
    {
        const auto terrainTexturePath = AssetPath(
            "maps/tancheon/terrain." + std::string(extension));
        if (!std::filesystem::exists(terrainTexturePath))
            continue;

        terrainSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadFromFile(
            terrainTexturePath);
        break;
    }
}

void AeroSimLabApp::LoadTerrain()
{
    terrainGrid_ = aerosimlab::terrain::TerrainGrid::Load(
        AssetPath("maps/tancheon/terrain.aerotrn"));
    simulation_.SetTerrain(terrainGrid_);

    const auto terrainModelPath = AssetPath("maps/tancheon/terrain.glb");
    if (std::filesystem::exists(terrainModelPath))
        terrainMesh_ = aerosimlab::graphics::Mesh::LoadGlb(
            terrainModelPath.string());
    else
        terrainMesh_ = CreateTerrainMesh(terrainGrid_);
}

void AeroSimLabApp::LoadMapFeatures()
{
    SetStartupDetail("건물·교량·하천 윤곽을 읽고 있습니다.");
    routeBuildings_ = aerosimlab::terrain::LoadMapBuildingFootprints(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
    const auto& buildings = routeBuildings_;
    const auto bridges = aerosimlab::terrain::LoadMapBridgeFootprints(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
    const auto waterBodies = aerosimlab::terrain::LoadMapWaterBodies(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
    const auto trees = aerosimlab::terrain::LoadMapTrees(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
    constexpr std::array<std::string_view, 3> treeAssets{
        "models/environment/trees/mature-oak.glb",
        "models/environment/trees/mature-scots-pine.glb",
        "models/environment/trees/mature-sycamore.glb"};
    std::array<std::vector<glm::mat4>, 3> treeTransforms;
    cameraTransitAltitudeMeters_ = 0.0;
    for (const auto& tree : trees)
    {
        const auto east = static_cast<std::int64_t>(std::llround(tree.positionMeters.x * 10.0));
        const auto north = static_cast<std::int64_t>(std::llround(tree.positionMeters.y * 10.0));
        const std::uint64_t key = static_cast<std::uint64_t>(east) * 0x9E3779B185EBCA87ULL
            ^ static_cast<std::uint64_t>(north) * 0xC2B2AE3D27D4EB4FULL;
        const std::size_t species = static_cast<std::size_t>(key % treeTransforms.size());
        const float yaw = static_cast<float>((key >> 8U) % 6284U) * 0.001F;
        const float scale = 0.82F + static_cast<float>((key >> 20U) % 37U) * 0.01F;
        const float ground = static_cast<float>(terrainGrid_.HeightAtLocalMeters(
            tree.positionMeters));
        cameraTransitAltitudeMeters_ = std::max(cameraTransitAltitudeMeters_,
            static_cast<double>(ground) + tallestTreeModelHeightMeters
                + cameraClearanceAboveCanopyMeters);
        glm::mat4 transform = glm::translate(glm::mat4(1.0F), ToRenderPosition(
            {tree.positionMeters.x, tree.positionMeters.y, ground}));
        transform = glm::rotate(transform, yaw, glm::vec3{0.0F, 1.0F, 0.0F});
        transform = glm::scale(transform, glm::vec3{scale});
        treeTransforms[species].push_back(transform);
    }
    for (std::size_t species = 0; species < treeAssets.size(); ++species)
    {
        if (treeTransforms[species].empty())
            continue;
        treeModels_[species] = aerosimlab::graphics::GlbModel::Load(
            AssetPath(treeAssets[species]).string());
        treeModels_[species]->SetInstances(treeTransforms[species]);
    }
    std::vector<aerosimlab::terrain::MapBridgePier> bridgePiers;
    for (const auto& bridge : bridges)
    {
        if (bridge.outlineMeters.size() < 3U || bridge.groundElevationsMeters.empty())
            continue;

        double ground = 0.0;
        for (std::size_t index = 0; index < bridge.outlineMeters.size(); ++index)
        {
            ground += bridge.groundElevationsMeters[index];
        }
        ground /= static_cast<double>(bridge.groundElevationsMeters.size());
        // OSM에 교량 높이가 없어 지면에서 6m 위에 두께 0.8m의 임시 상판을 놓는다.
        simulation_.AddStaticObstaclePrism(
            bridge.outlineMeters, ground + bridgeDeckUndersideHeightMeters,
            ground + bridgeDeckUndersideHeightMeters + bridgeDeckThicknessMeters);
        auto estimatedPiers = aerosimlab::terrain::EstimateMapBridgePiers(
            bridge, terrainGrid_, ground + bridgeDeckUndersideHeightMeters);
        for (const auto& pier : estimatedPiers)
        {
            simulation_.AddStaticObstaclePrism(pier.outlineMeters,
                pier.bottomElevationMeters, pier.topElevationMeters);
        }
        bridgePiers.insert(bridgePiers.end(),
            estimatedPiers.begin(), estimatedPiers.end());
    }
    if (terrainSurfaceTexture_ == nullptr)
    {
        const auto roads = aerosimlab::terrain::LoadMapRoads(
            AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
        roadMesh_ = CreateRoadMesh(roads, false);
        bridgeMesh_ = CreateRoadMesh(roads, true);
    }
    bridgeFootprintMesh_ = CreateBridgeFootprintMesh(
        bridges, terrainGrid_, BridgeMeshPart::DeckSurface);
    bridgeStructureMesh_ = CreateBridgeFootprintMesh(
        bridges, terrainGrid_, BridgeMeshPart::DeckStructure);
    bridgePierMesh_ = CreateBridgePierMesh(bridgePiers);
    waterMesh_ = CreateWaterMesh(waterBodies, terrainGrid_);
    taggedBuildingMesh_ = CreateBuildingMesh(buildings,
        aerosimlab::terrain::BuildingHeightSource::OsmHeightTag, false);
    SetStartupDetail("높이가 확인된 건물의 벽을 만들고 있습니다.");
    taggedBuildingWallMesh_ = CreateBuildingMesh(buildings,
        aerosimlab::terrain::BuildingHeightSource::OsmHeightTag, true);
    SetStartupDetail("층수로 높이를 추정한 건물 지붕을 만들고 있습니다.");
    estimatedBuildingMesh_ = CreateBuildingMesh(buildings,
        aerosimlab::terrain::BuildingHeightSource::EstimatedFromLevels, false);
    SetStartupDetail("층수로 높이를 추정한 건물 벽을 만들고 있습니다.");
    estimatedBuildingWallMesh_ = CreateBuildingMesh(buildings,
        aerosimlab::terrain::BuildingHeightSource::EstimatedFromLevels, true);
    SetStartupDetail("높이를 알 수 없는 건물 외곽을 임시 높이로 표시합니다.");
    footprintBuildingMesh_ = CreateBuildingMesh(buildings,
        aerosimlab::terrain::BuildingHeightSource::EstimatedFromFootprint,
        false);
    footprintBuildingWallMesh_ = CreateBuildingMesh(buildings,
        aerosimlab::terrain::BuildingHeightSource::EstimatedFromFootprint,
        true);
    SetStartupDetail("건물 충돌 구역을 준비하고 있습니다.");
    inspectionOccluders_.reserve(buildings.size());
    std::unordered_map<std::string, const aerosimlab::terrain::MapBuilding*> buildingsById;
    for (const auto& building : buildings)
    {
        SetStartupDetail(std::string("건물 충돌 구역: ") + building.osmId);
        const auto bounds =
            aerosimlab::terrain::GetMapBuildingBoundsMeters(building);
        if (building.heightMeters > 0.0)
        {
            // 층수·외곽선에서 추정한 높이도 출처를 구분해 유한한 충돌 상자로 쓴다.
            simulation_.AddStaticObstaclePrism(building.outlineMeters,
                bounds.centerMeters.z - bounds.halfExtentsMeters.z,
                bounds.centerMeters.z + bounds.halfExtentsMeters.z);
        }
        else
        {
            simulation_.AddStaticObstacleFootprint(building.outlineMeters);
        }
        inspectionOccluders_.push_back({building.outlineMeters,
            bounds.centerMeters.z - bounds.halfExtentsMeters.z,
            bounds.centerMeters.z + bounds.halfExtentsMeters.z});
        if (building.heightSource
            == aerosimlab::terrain::BuildingHeightSource::OsmHeightTag)
            buildingsById.emplace(building.osmId, &building);
    }

    SetStartupDetail("시험용 정찰 위치와 경로를 읽고 있습니다.");
    std::ifstream planFile(AssetPath("maps/tancheon/demo-inspection-plan.json"));
    if (!planFile)
        throw std::runtime_error("시험용 시설물 점검 계획을 열지 못했습니다.");

    using Json = nlohmann::json;
    Json plan;
    try
    {
        planFile >> plan;
    }
    catch (const Json::exception&)
    {
        throw std::runtime_error("시험용 시설물 점검 계획의 JSON을 읽지 못했습니다.");
    }
    if (!plan.contains("targets") || !plan.at("targets").is_array()
        || plan.at("targets").empty())
        throw std::runtime_error("시험용 점검 계획에 구역이 없습니다.");

    const bool hasExplicitGeometry = std::any_of(
        plan.at("targets").begin(), plan.at("targets").end(),
        [](const Json& target) {
            return target.contains("inspection_surface")
                || target.contains("waypoints_meters");
        });
    if (hasExplicitGeometry
        && (!plan.contains("coordinate_frame")
            || plan.at("coordinate_frame") != "tancheon_local_enu"))
        throw std::runtime_error(
            "좌표로 지정한 점검면과 경유점에는 tancheon_local_enu 기준을 적어야 합니다.");

    std::unordered_set<std::string> zoneIds;
    for (const Json& target : plan.at("targets"))
    {
        const std::string facilityId = target.at("facility_id").get<std::string>();
        const std::string zoneId = target.at("zone_id").get<std::string>();
        std::string displayName = target.value("name", std::string{});
        if (!zoneIds.insert(zoneId).second)
            throw std::runtime_error("점검 계획에 같은 구역 ID가 두 번 있습니다: " + zoneId);

        aerosimlab::simulation::InspectionSurface surface;
        std::vector<glm::dvec3> routeWaypoints;
        glm::dvec3 approachPosition{0.0};
        std::string buildingOsmId;
        if (target.contains("waypoints_meters"))
        {
            routeWaypoints = aerosimlab::simulation::ParseInspectionWaypointsJson(
                target.at("waypoints_meters").dump());
            for (glm::dvec3& waypoint : routeWaypoints)
            {
                if (!terrainGrid_.ContainsLocalPositionMeters({waypoint.x, waypoint.y}))
                    throw std::runtime_error("점검 경유점이 지형 범위를 벗어났습니다: "
                        + zoneId);
                waypoint = terrainGrid_.ToSimulationPositionMeters(waypoint);
            }
        }
        if (target.contains("inspection_surface"))
        {
            surface = aerosimlab::simulation::ParseInspectionSurfaceJson(
                target.at("inspection_surface").dump());
            approachPosition = ReadPlanVector(
                target.at("approach_position_meters"), "approach_position_meters");
            surface.centerMeters = terrainGrid_.ToSimulationPositionMeters(
                surface.centerMeters);
            approachPosition = terrainGrid_.ToSimulationPositionMeters(approachPosition);
        }
        else
        {
            buildingOsmId =
                target.at("building_osm_id").get<std::string>();
            const auto building = buildingsById.find(buildingOsmId);
            if (building == buildingsById.end())
                throw std::runtime_error("점검 계획에서 지정한 건물 자료를 찾지 못했습니다: "
                    + buildingOsmId);
            if (displayName.empty())
                displayName = building->second->name;
            if (displayName.empty())
                displayName = "이름 없는 건물";

            const Json& capture = target.at("capture");
            const double approachDistance =
                target.at("approach_distance_meters").get<double>();
            const auto facade = aerosimlab::terrain::GetNearestMapBuildingFacade(
                *building->second, demoDroneStartMeters);
            if (facade.widthMeters < 3.0 || facade.heightMeters < 2.0
                || approachDistance <= 0.0)
                throw std::runtime_error(
                    "점검 계획의 외벽 시험면이나 접근 거리를 확인해 주세요: " + zoneId);

            approachPosition = facade.centerMeters
                + facade.outwardNormal * approachDistance;
            surface.centerMeters = facade.centerMeters;
            surface.normal = facade.outwardNormal;
            surface.widthAxis = facade.widthAxis;
            surface.widthMeters = std::min(facade.widthMeters,
                capture.at("width_meters").get<double>());
            surface.heightMeters = std::min(facade.heightMeters,
                capture.at("height_meters").get<double>());
            surface.minimumDistanceMeters =
                capture.at("minimum_distance_meters").get<double>();
            surface.maximumDistanceMeters =
                capture.at("maximum_distance_meters").get<double>();
            surface.maximumIncidenceAngleDegrees =
                capture.at("maximum_incidence_angle_degrees").get<double>();
            surface.minimumPixelWidth = capture.at("minimum_pixel_width").get<int>();
            surface.minimumPixelHeight = capture.at("minimum_pixel_height").get<int>();
        }

        if (!terrainGrid_.ContainsLocalPositionMeters(
            {approachPosition.x, approachPosition.y}))
            throw std::runtime_error("점검 구역 접근 위치가 지형 범위를 벗어났습니다: "
                + zoneId);
        const bool routeAutomaticallyPlanned = routeWaypoints.empty();
        inspectionTargets_.push_back({
            .facilityId = facilityId,
            .displayName = displayName.empty() ? "점검 대상" : displayName,
            .zoneId = zoneId,
            .buildingOsmId = std::move(buildingOsmId),
            .surface = surface,
            .routeWaypointsMeters = std::move(routeWaypoints),
            .approachPositionMeters = approachPosition,
            .routeAutomaticallyPlanned = routeAutomaticallyPlanned});
    }

    const auto mainBridge = std::find_if(bridges.begin(), bridges.end(),
        [](const auto& bridge) { return bridge.name == "탄천교"; });
    if (mainBridge != bridges.end())
    {
        glm::dvec2 bridgeCenter{0.0};
        double bridgeGround = 0.0;
        for (std::size_t index = 0; index < mainBridge->outlineMeters.size(); ++index)
        {
            bridgeCenter += mainBridge->outlineMeters[index];
            bridgeGround += mainBridge->groundElevationsMeters[index];
        }
        bridgeCenter /= static_cast<double>(mainBridge->outlineMeters.size());
        bridgeGround /= static_cast<double>(mainBridge->groundElevationsMeters.size());
        const double centerGround = terrainGrid_.HeightAtLocalMeters({0.0, 0.0});

        aerosimlab::simulation::InspectionSurface underside;
        underside.centerMeters = terrainGrid_.ToSimulationPositionMeters(
            {bridgeCenter.x, bridgeCenter.y,
                bridgeGround - centerGround + bridgeDeckUndersideHeightMeters});
        underside.normal = {0.0, 0.0, -1.0};
        underside.widthAxis = {1.0, 0.0, 0.0};
        underside.widthMeters = 14.0;
        underside.heightMeters = 2.0;
        underside.minimumDistanceMeters = 1.0;
        underside.maximumDistanceMeters = 8.0;
        underside.maximumIncidenceAngleDegrees = 75.0;
        inspectionTargets_.push_back({
            .facilityId = "osm-bridge-" + mainBridge->osmId,
            .displayName = "탄천교 하부",
            .zoneId = "bridge-underside",
            .surface = underside,
            .approachPositionMeters = terrainGrid_.ToSimulationPositionMeters(
                {bridgeCenter.x, bridgeCenter.y,
                    bridgeGround - centerGround + bridgeDeckUndersideHeightMeters
                        - bridgeInspectionApproachBelowDeckMeters}),
            .routeAutomaticallyPlanned = true});

        glm::dvec2 riverPosition = bridgeCenter;
        double nearestWaterDistance = std::numeric_limits<double>::max();
        for (const auto& water : waterBodies)
        {
            for (int northStep = -12; northStep <= 12; ++northStep)
            {
                for (int eastStep = -12; eastStep <= 12; ++eastStep)
                {
                    const glm::dvec2 point = bridgeCenter
                        + glm::dvec2{eastStep * 5.0, northStep * 5.0};
                    if (!aerosimlab::terrain::ContainsWater(water, point))
                        continue;
                    const double distance = glm::length(point - bridgeCenter);
                    if (distance < nearestWaterDistance)
                    {
                        nearestWaterDistance = distance;
                        riverPosition = point;
                    }
                }
            }
        }
        const double riverGround = terrainGrid_.HeightAtLocalMeters(riverPosition);
        aerosimlab::simulation::InspectionSurface riverSurface;
        riverSurface.centerMeters = terrainGrid_.ToSimulationPositionMeters(
            {riverPosition.x, riverPosition.y, riverGround - centerGround + 0.25});
        riverSurface.normal = {0.0, 0.0, 1.0};
        riverSurface.widthAxis = {1.0, 0.0, 0.0};
        riverSurface.widthMeters = 20.0;
        riverSurface.heightMeters = 8.0;
        riverSurface.minimumDistanceMeters = 2.0;
        riverSurface.maximumDistanceMeters = 15.0;
        riverSurface.maximumIncidenceAngleDegrees = 80.0;
        inspectionTargets_.push_back({
            .facilityId = "tancheon-waterway",
            .displayName = "탄천 물길",
            .zoneId = "river-channel",
            .surface = riverSurface,
            .approachPositionMeters = terrainGrid_.ToSimulationPositionMeters(
                {riverPosition.x, riverPosition.y, riverGround - centerGround + 7.0}),
            .routeAutomaticallyPlanned = true});
    }

    // 기본 임무는 아파트 외벽보다 탄천교와 물길을 먼저 지나도록 순서를 정한다.
    const auto targetPriority = [](const InspectionTarget& target)
    {
        if (target.zoneId == "bridge-underside")
            return 0;
        if (target.zoneId == "river-channel")
            return 1;
        return 2;
    };
    std::stable_sort(inspectionTargets_.begin(), inspectionTargets_.end(),
        [&targetPriority](const InspectionTarget& left, const InspectionTarget& right)
        {
            return targetPriority(left) < targetPriority(right);
        });

    glm::dvec3 routeStart{
        demoDroneStartMeters.x,
        demoDroneStartMeters.y,
        terrainGrid_.HeightAtLocalMeters(demoDroneStartMeters)
            + aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters};
    for (InspectionTarget& target : inspectionTargets_)
    {
        target.routeStartMeters = routeStart;
        if (target.routeAutomaticallyPlanned)
        {
            if (!target.buildingOsmId.empty())
            {
                const auto building = std::find_if(routeBuildings_.begin(),
                    routeBuildings_.end(), [&target](const auto& candidate)
                    {
                        return candidate.osmId == target.buildingOsmId;
                    });
                if (building == routeBuildings_.end())
                    throw std::runtime_error("점검 경로에 필요한 건물 외곽선을 찾지 못했습니다: "
                        + target.buildingOsmId);

                const double preferredDistance = glm::length(
                    target.approachPositionMeters - target.surface.centerMeters);
                const auto facadeRoute =
                    aerosimlab::terrain::PlanMapFlightRouteToFacade(
                        terrainGrid_, routeBuildings_, *building, routeStart,
                        preferredDistance, target.surface.maximumDistanceMeters,
                        2.0, 3.0, cameraTransitAltitudeMeters_);
                if (!facadeRoute)
                    throw std::runtime_error("점검 구역까지 도달할 수 있는 외벽 경로를 찾지 못했습니다: "
                        + target.displayName);

                target.surface.centerMeters = facadeRoute->facade.centerMeters;
                target.surface.normal = facadeRoute->facade.outwardNormal;
                target.surface.widthAxis = facadeRoute->facade.widthAxis;
                target.surface.widthMeters = std::min(target.surface.widthMeters,
                    facadeRoute->facade.widthMeters);
                target.surface.heightMeters = std::min(target.surface.heightMeters,
                    facadeRoute->facade.heightMeters);
                target.approachPositionMeters = facadeRoute->approachPositionMeters;
                target.routeWaypointsMeters.assign(facadeRoute->waypointsMeters.begin(),
                    facadeRoute->waypointsMeters.end() - 1);
            }
            else
            {
                std::optional<std::vector<glm::dvec3>> route;
                if (target.zoneId == "bridge-underside"
                    && mainBridge != bridges.end())
                {
                    double bridgeGround = 0.0;
                    for (const double elevation : mainBridge->groundElevationsMeters)
                        bridgeGround += elevation;
                    bridgeGround /= static_cast<double>(
                        mainBridge->groundElevationsMeters.size());
                    const double deckBottom =
                        bridgeGround + bridgeDeckUndersideHeightMeters;
                    route = aerosimlab::terrain::PlanMapFlightRouteUnderBridge(
                        terrainGrid_, routeBuildings_, *mainBridge, bridgePiers,
                        routeStart, target.approachPositionMeters, deckBottom,
                        deckBottom + bridgeDeckThicknessMeters, 3.0,
                        cameraTransitAltitudeMeters_);
                }
                else
                {
                    route = aerosimlab::terrain::PlanMapFlightRoute(
                        terrainGrid_, routeBuildings_, routeStart,
                        target.approachPositionMeters, 3.0,
                        cameraTransitAltitudeMeters_);
                }
                // 접근점이 이웃 건물과 너무 가까우면 촬영 가능한 거리 안에서 바깥으로 옮긴다.
                const double currentDistance = glm::length(
                    target.approachPositionMeters - target.surface.centerMeters);
                constexpr double approachStepMeters = 2.0;
                for (double distance = currentDistance + approachStepMeters;
                     !route && distance <= target.surface.maximumDistanceMeters;
                     distance += approachStepMeters)
                {
                    const glm::dvec3 candidate = target.surface.centerMeters
                        + target.surface.normal * distance;
                    route = aerosimlab::terrain::PlanMapFlightRoute(
                        terrainGrid_, routeBuildings_, routeStart, candidate, 3.0,
                        cameraTransitAltitudeMeters_);
                    if (route)
                        target.approachPositionMeters = candidate;
                }
                if (!route)
                    throw std::runtime_error(
                        "점검 구역까지 건물을 피해 갈 경로를 찾지 못했습니다: "
                        + target.displayName);
                target.routeWaypointsMeters.assign(route->begin(), route->end() - 1);
            }
        }
        routeStart = target.approachPositionMeters;
    }

    SetActiveInspectionTarget(0U);
#ifdef __EMSCRIPTEN__
    EM_ASM({
        if (window.clearDroneSimInspectionTargets)
            window.clearDroneSimInspectionTargets();
    });
    for (std::size_t index = 0; index < inspectionTargets_.size(); ++index)
    {
        const InspectionTarget& target = inspectionTargets_[index];
        EM_ASM({
            if (window.addDroneSimInspectionTarget)
                window.addDroneSimInspectionTarget(
                    $0, UTF8ToString($1), UTF8ToString($2), UTF8ToString($3));
        }, static_cast<int>(index), target.facilityId.c_str(),
            target.zoneId.c_str(), target.displayName.c_str());
    }
#endif
}

void AeroSimLabApp::InitializeStartingState()
{
    const double startElevation = terrainGrid_.HeightAtLocalMeters(demoDroneStartMeters);
    const glm::dvec2 toFirstTarget = glm::dvec2{
        inspectionTargets_.front().surface.centerMeters.x,
        inspectionTargets_.front().surface.centerMeters.y} - demoDroneStartMeters;
    const double initialYawRadians = std::atan2(toFirstTarget.y, toFirstTarget.x);
    simulation_.SetInitialPose({demoDroneStartMeters.x, demoDroneStartMeters.y,
        startElevation +
            aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters},
        initialYawRadians);
    fleetDroneIds_ = simulation_.DroneIds();
}

aerosimlab::simulation::CaptureAssessment AeroSimLabApp::AssessInspectionView() const
{
    return aerosimlab::simulation::EoCameraModel::AssessCapture(
        simulation_.Camera(), inspectionMission_.surface, {}, inspectionOccluders_);
}

void AeroSimLabApp::SetActiveInspectionTarget(std::size_t index)
{
    if (index >= inspectionTargets_.size())
        throw std::out_of_range("점검 구역 번호가 임무 범위를 벗어났습니다.");

    inspectionMission_.activeTargetIndex = index;
    const InspectionTarget& target = inspectionTargets_[index];
    inspectionMission_.facilityId = target.facilityId;
    inspectionMission_.targetDisplayName = target.displayName;
    inspectionMission_.zoneId = target.zoneId;
    inspectionMission_.surface = target.surface;
    inspectionMission_.routeWaypointIndex = 0;
    inspectionMission_.goalMeters = target.routeWaypointsMeters.empty()
        ? target.approachPositionMeters
        : target.routeWaypointsMeters.front();
}

void AeroSimLabApp::AdvanceSimulation(double elapsedSeconds)
{
    const std::size_t tickCount = fixedTickAccumulator_.Consume(elapsedSeconds);
    for (std::size_t tick = 0; tick < tickCount; ++tick)
    {
        double tickDurationMicroseconds = 0.0;
        const std::uint64_t previousSimulationTick = simulation_.TickCount();
        {
            aerosimlab::simulation::ScopedDuration measureTick(
                benchmarkMode_ ? &tickDurationMicroseconds : nullptr);
            AdvanceSimulationTick();
        }
        if (benchmarkMode_)
        {
            benchmarkMeasurements_.tickDurationsMicroseconds.push_back(
                tickDurationMicroseconds);
#ifndef __EMSCRIPTEN__
            if (simulation_.TickCount() != previousSimulationTick)
                benchmarkMeasurements_.physicsStepDurationsMicroseconds.push_back(
                    simulation_.LastPhysicsStepDurationMicroseconds());
#endif
        }
    }
}

void AeroSimLabApp::AdvanceSimulationTick()
{
    AdvanceAmbientWind();
    AdvanceFleetReconnaissance();
    if (demoMode_ && !externalControl_
        && inspectionMission_.stage == InspectionMissionStage::Ready)
        static_cast<void>(RequestInspectionMission());
    AdvanceInspectionMission();
    ExpireGroundControlCommand();
    const bool groundControlTickAdvanced = ProcessPendingGroundCommand();
    const bool fleetMissionActive = std::any_of(
        fleetReconnaissance_.begin(), fleetReconnaissance_.end(),
        [](const auto& entry)
        {
            return entry.second.status == "출발 준비"
                || entry.second.status == "이동 중"
                || entry.second.status == "착륙 중";
        });
    if (!groundControlTickAdvanced
        && groundControlCommandExpiresAtTick_.empty()
        && groundControlActive_ && !fleetMissionActive)
    {
        groundControlActive_ = false;
        window_->SetTitle(gamepadConnected_
            ? "AeroSimLab | 게임패드 연결됨"
            : "AeroSimLab | 드론 관찰 · EO 영상");
    }
    // 지상통제 명령이 이 Tick을 이미 진행했다면 재생·게임패드 입력을 더 적용하지 않는다.
    if (groundControlTickAdvanced)
        return;

    if (commandRecording_.IsReplaying())
    {
        AdvanceReplayTick();
        return;
    }

    AdvanceManualControlTick();
}

void AeroSimLabApp::AdvanceAmbientWind()
{
    constexpr std::uint32_t gustChangeIntervalTicks = 240U;
    constexpr double maximumWindMetersPerSecond = 0.5;
    constexpr double smoothingPerTick = 0.01;
    if (windGustTicksRemaining_ == 0U)
    {
        std::uniform_real_distribution<double> chooseWind(
            -maximumWindMetersPerSecond, maximumWindMetersPerSecond);
        windTargetMetersPerSecond_ = {
            chooseWind(windGenerator_), chooseWind(windGenerator_), 0.0};
        windGustTicksRemaining_ = gustChangeIntervalTicks;
    }
    --windGustTicksRemaining_;
    windVelocityMetersPerSecond_ += (windTargetMetersPerSecond_
        - windVelocityMetersPerSecond_) * smoothingPerTick;
    simulation_.SetWindVelocityMetersPerSecond(windVelocityMetersPerSecond_);
}

void AeroSimLabApp::AdvanceReplayTick()
{
    ApplyRecordedCommands();
    simulation_.AdvanceOneTick();
    commandRecording_.AdvanceReplayTick();
    if (commandRecording_.ReplayFinished())
    {
        commandRecording_.StopReplay();
        window_->SetTitle("AeroSimLab | 기록 재생 완료");
    }
}

void AeroSimLabApp::AdvanceManualControlTick()
{
    const bool inspectionMissionActive =
        inspectionMission_.stage == InspectionMissionStage::StartRequested
        || inspectionMission_.stage == InspectionMissionStage::Paused
        || inspectionMission_.stage == InspectionMissionStage::TakingOff
        || inspectionMission_.stage == InspectionMissionStage::Approaching
        || inspectionMission_.stage == InspectionMissionStage::AligningCamera
        || inspectionMission_.stage == InspectionMissionStage::Capturing
        || inspectionMission_.stage == InspectionMissionStage::Landing;
    if (inspectionMissionActive)
    {
        simulation_.AdvanceOneTick();
        commandRecording_.FinishTick();
        return;
    }

#ifdef __EMSCRIPTEN__
    if (webPilotAxes_)
    {
        const auto mode = simulation_.State().mode;
        if (mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying)
        {
            pendingPilotCommand_ = aerosimlab::manual_control::MapAxes(
                *webPilotAxes_, simulation_.State().yawRadians);
        }
        else
        {
            pendingPilotCommand_.reset();
        }
    }
#endif
    if (!groundControlActive_ && pendingControllerAction_)
    {
        static_cast<void>(ApplyAndRecordCommand(*pendingControllerAction_));
        pendingControllerAction_.reset();
    }
    if (!groundControlActive_ && pendingPilotCommand_)
    {
        const auto mode = simulation_.State().mode;
        if (mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying)
        {
            const aerosimlab::simulation::DroneCommand velocityCommand{
                aerosimlab::simulation::DroneCommand::Kind::SetVelocity,
                pendingPilotCommand_->velocityMetersPerSecond};
            const aerosimlab::simulation::DroneCommand yawCommand{
                aerosimlab::simulation::DroneCommand::Kind::SetYawRate,
                glm::dvec3{0.0},
                pendingPilotCommand_->yawRateRadiansPerSecond};
            static_cast<void>(ApplyAndRecordCommand(velocityCommand));
            static_cast<void>(ApplyAndRecordCommand(yawCommand));
        }
    }

    simulation_.AdvanceOneTick();
    commandRecording_.FinishTick();
}

aerosimlab::simulation::CommandResult AeroSimLabApp::ApplyAndRecordCommand(
    const aerosimlab::simulation::DroneCommand& command)
{
    return ApplyAndRecordCommand(simulation_.PrimaryDroneId(), command);
}

aerosimlab::simulation::CommandResult AeroSimLabApp::ApplyAndRecordCommand(
    aerosimlab::simulation::DroneId droneId,
    const aerosimlab::simulation::DroneCommand& command)
{
    const auto result = simulation_.Apply(droneId, command);
    if (result == aerosimlab::simulation::CommandResult::Accepted)
    {
        if (droneId == simulation_.PrimaryDroneId())
            commandRecording_.Record(command);
    }
    return result;
}

void AeroSimLabApp::RenderFrame()
{
#ifdef __EMSCRIPTEN__
    int width = 0;
    int height = 0;
    window_->GetFramebufferSize(width, height);
    if (width <= 0 || height <= 0)
        return;

    renderer_->Resize(width, height);
    const int observerWidth = width * 2 / 3;
    const int cameraWidth = width - observerWidth;
    const auto& camera = simulation_.Camera();
    const int cameraHeight = std::min(height,
        cameraWidth * static_cast<int>(camera.model.imageHeightPixels)
            / static_cast<int>(camera.model.imageWidthPixels));
    const int cameraBottom = (height - cameraHeight) / 2;
    const int cameraTop = height - cameraBottom - cameraHeight;
    UpdateObserverCamera(observerWidth, height);
    UpdateEoCamera(cameraWidth, cameraHeight);
    renderer_->BeginFrame();
    renderer_->SetSceneViewport(0, 0, observerWidth, height);
    DrawWorld(observerCamera_->GetViewMatrix(), observerCamera_->GetProjectionMatrix(),
        observerCamera_->GetPosition(), true);
    renderer_->SetSceneViewport(observerWidth, cameraBottom, cameraWidth, cameraHeight);
    DrawWorld(eoCamera_->GetViewMatrix(), eoCamera_->GetProjectionMatrix(),
        eoCamera_->GetPosition(), false, simulation_.PrimaryDroneId());
    renderer_->EndFrame();
    EM_ASM({
        if (window.copyDroneSimEoViewport) {
            window.copyDroneSimEoViewport($0, $1, $2, $3);
        }
    }, observerWidth, cameraTop, cameraWidth, cameraHeight);
#else
    if (hud_)
    {
        const bool groundControlConnected = lastGroundControlMessage_
            && std::chrono::steady_clock::now() - *lastGroundControlMessage_
                < std::chrono::seconds(2);
        const bool cameraActive = simulation_.Camera().status
            == aerosimlab::simulation::CameraStatus::Active;
        const auto& camera = simulation_.Camera();
        const glm::dvec3& dronePosition = simulation_.State().positionMeters;
        const glm::dvec3& capturePosition = inspectionTargets_[
            inspectionMission_.activeTargetIndex].approachPositionMeters;
        const double missionDistanceMeters = std::hypot(
            capturePosition.x - dronePosition.x,
            capturePosition.y - dronePosition.y);
        const bool missionCanStart = inspectionMission_.stage
                == InspectionMissionStage::Ready
            || inspectionMission_.stage == InspectionMissionStage::Saved
            || inspectionMission_.stage == InspectionMissionStage::GeometryOnly
            || inspectionMission_.stage == InspectionMissionStage::Failed;
        const auto captureAssessment = AssessInspectionView();
        HudRenderTimings hudTimings;
        hudTimings.collectGpuTimings = visibleBenchmarkMode_;
        hudTimings.frameIndex = static_cast<std::uint64_t>(renderedFrames_ + 1U);
        hud_->Render(simulation_, gamepadConnected_, groundControlConnected, cameraActive,
            camera.model.imageWidthPixels, camera.model.imageHeightPixels,
            missionDistanceMeters, inspectionMission_.targetDisplayName,
            InspectionMissionStatus(),
            missionCanStart,
            [this] { static_cast<void>(RequestInspectionMission()); }, captureAssessment,
            [this](int width, int height) { return RenderScenePanel(width, height); },
            [this] { return RenderCameraPanel(); },
            benchmarkMode_ ? &hudTimings : nullptr);
        frameMeasurements_.hudInterfaceBuildMicroseconds = hudTimings.interfaceBuildMicroseconds;
        frameMeasurements_.hudFinalizeMicroseconds = hudTimings.frameFinalizeMicroseconds;
        frameMeasurements_.hudBackendDrawMicroseconds = hudTimings.backendDrawMicroseconds;
        frameMeasurements_.hudDrawListCount = hudTimings.drawListCount;
        frameMeasurements_.hudDrawCommandCount = hudTimings.drawCommandCount;
        frameMeasurements_.hudVertexCount = hudTimings.vertexCount;
        frameMeasurements_.hudIndexCount = hudTimings.indexCount;
        frameMeasurements_.completedHudGpuTimingCount = std::min(
            hudTimings.completedGpuFrameCount, completedHudGpuTimings_.size());
        for (std::size_t index = 0;
             index < frameMeasurements_.completedHudGpuTimingCount; ++index)
        {
            completedHudGpuTimings_[index] = hudTimings.completedGpuFrames[index];
        }
    }
#endif
}

void AeroSimLabApp::UpdateObserverCamera(int width, int height)
{
    const auto& state = simulation_.State();
    const glm::vec3 dronePosition = ToRenderPosition(state.positionMeters);
    const float yaw = static_cast<float>(state.yawRadians);
    const glm::vec3 forward{std::cos(yaw), 0.0F, -std::sin(yaw)};
    const glm::vec3 up{0.0F, 1.0F, 0.0F};
    constexpr double followDistanceMeters = 1.15;
    constexpr double followHeightMeters = 0.7;
    glm::dvec3 observerPositionMeters = state.positionMeters
        - glm::dvec3{std::cos(state.yawRadians), std::sin(state.yawRadians), 0.0}
            * followDistanceMeters
        + glm::dvec3{0.0, 0.0, followHeightMeters};
    // 카메라가 건물 안에 놓이면 선분을 따라 기체 쪽의 첫 안전 위치로 옮긴다.
    if (const auto clearPosition = aerosimlab::terrain::FindFirstPointOutsideMapBuildings(
        observerPositionMeters, state.positionMeters, routeBuildings_, 0.5))
        observerPositionMeters = *clearPosition;
    observerCamera_->SetPosition(ToRenderPosition(observerPositionMeters));
    observerCamera_->SetTarget(dronePosition + forward * 0.55F + up * 0.12F);
    observerCamera_->SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    observerCamera_->SetFieldOfViewDegrees(50.0F);
}

void AeroSimLabApp::UpdateEoCamera(int width, int height)
{
    const auto& cameraState = simulation_.Camera();
    const glm::vec3 cameraPosition = ToRenderPosition(cameraState.positionMeters);
    const glm::dvec3 opticalAxis =
        aerosimlab::simulation::EoCameraModel::OpticalAxis(cameraState);
    const glm::dvec3 cameraUpAxis =
        aerosimlab::simulation::EoCameraModel::UpAxis(cameraState);
    const glm::vec3 viewDirection = ToRenderDirection(opticalAxis);
    const glm::vec3 cameraUp = ToRenderDirection(cameraUpAxis);
    eoCamera_->SetPosition(cameraPosition);
    eoCamera_->SetTarget(cameraPosition + viewDirection);
    eoCamera_->SetUp(cameraUp);
    eoCamera_->SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
    eoCamera_->SetFieldOfViewDegrees(
        static_cast<float>(cameraState.model.verticalFieldOfViewDegrees));
}

void AeroSimLabApp::DrawWorld(const glm::mat4& view, const glm::mat4& projection,
    const glm::vec3& cameraPosition, bool showMissionGuides,
    std::optional<aerosimlab::simulation::DroneId> hiddenDroneId)
{
    static const aerosimlab::graphics::Material orthophotoMaterial{
        glm::vec4{1.0F}, 0.0F, 0.92F};
    renderer_->SetAnimationTimeSeconds(static_cast<float>(simulation_.SimulationTimeSeconds()));
    const bool hasOrthophoto = terrainSurfaceTexture_ != nullptr;
    const auto& terrainMaterial = hasOrthophoto
        ? orthophotoMaterial : groundMaterial_;
    renderer_->Draw(glm::mat4{1.0F}, *terrainMesh_, terrainMaterial, *shader_,
        view, projection, cameraPosition, terrainSurfaceTexture_.get(), false,
        hasOrthophoto);
    if (waterMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *waterMesh_, waterMaterial_, *shader_,
            view, projection, cameraPosition);
    for (const auto& treeModel : treeModels_)
    {
        if (treeModel)
            treeModel->Draw(glm::mat4{1.0F}, *renderer_, *shader_, view,
                projection, cameraPosition);
    }
    // 정사영상에는 도로가 이미 찍혀 있어 같은 자리에 단색 메시를 덧그리지 않는다.
    if (roadMesh_ && !hasOrthophoto)
        renderer_->Draw(glm::mat4{1.0F}, *roadMesh_, roadMaterial_, *shader_,
            view, projection, cameraPosition);
    if (bridgeMesh_ && !hasOrthophoto)
        renderer_->Draw(glm::mat4{1.0F}, *bridgeMesh_, bridgeMaterial_, *shader_,
            view, projection, cameraPosition);
    if (bridgeStructureMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *bridgeStructureMesh_,
            bridgeMaterial_, *shader_, view, projection, cameraPosition);
    if (bridgePierMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *bridgePierMesh_,
            bridgePierMaterial_, *shader_, view, projection, cameraPosition);
    if (bridgeFootprintMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *bridgeFootprintMesh_, orthophotoMaterial,
            *shader_, view, projection, cameraPosition, terrainSurfaceTexture_.get());
    const auto drawBuildingRoof = [&](const auto& mesh)
    {
        if (mesh)
        {
            renderer_->Draw(glm::mat4{1.0F}, *mesh, buildingRoofMaterial_,
                *shader_, view, projection, cameraPosition, buildingRoofTexture_.get());
        }
    };
    drawBuildingRoof(taggedBuildingMesh_);
    if (taggedBuildingWallMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *taggedBuildingWallMesh_, buildingMaterial_,
            *shader_, view, projection, cameraPosition, buildingFacadeTexture_.get());
    drawBuildingRoof(estimatedBuildingMesh_);
    if (estimatedBuildingWallMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *estimatedBuildingWallMesh_,
            estimatedBuildingMaterial_, *shader_, view, projection, cameraPosition,
            buildingFacadeTexture_.get());
    drawBuildingRoof(footprintBuildingMesh_);
    if (footprintBuildingWallMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *footprintBuildingWallMesh_,
            estimatedBuildingMaterial_, *shader_, view, projection, cameraPosition,
            buildingFacadeTexture_.get());

    const auto drawRouteSegment = [this, &view, &projection, &cameraPosition](
        const glm::dvec3& startMeters, const glm::dvec3& endMeters)
    {
        const glm::vec3 start = ToRenderPosition(startMeters);
        const glm::vec3 end = ToRenderPosition(endMeters);
        const glm::vec3 direction = end - start;
        const float length = glm::length(direction);
        if (length < 0.1F)
            return;

        const glm::vec3 localY = direction / length;
        const glm::vec3 referenceAxis = std::abs(localY.y) < 0.99F
            ? glm::vec3{0.0F, 1.0F, 0.0F}
            : glm::vec3{1.0F, 0.0F, 0.0F};
        const glm::vec3 localX = glm::normalize(glm::cross(referenceAxis, localY));
        const glm::vec3 localZ = glm::cross(localX, localY);
        glm::mat4 orientation{1.0F};
        orientation[0] = glm::vec4{localX, 0.0F};
        orientation[1] = glm::vec4{localY, 0.0F};
        orientation[2] = glm::vec4{localZ, 0.0F};
        const glm::mat4 transform = glm::translate(
            glm::mat4{1.0F}, (start + end) * 0.5F)
            * orientation
            * glm::scale(glm::mat4{1.0F}, glm::vec3{0.04F, length, 0.04F});
        DrawCube(transform, cameraPosition, missionRouteMaterial_, view, projection);
    };

    if (showMissionGuides && InspectionMissionIsActive()
        && inspectionMission_.activeTargetIndex < inspectionTargets_.size())
    {
        const InspectionTarget& target =
            inspectionTargets_[inspectionMission_.activeTargetIndex];
        glm::dvec3 previous = target.routeStartMeters;
        for (const glm::dvec3& waypoint : target.routeWaypointsMeters)
        {
            drawRouteSegment(previous, waypoint);
            previous = waypoint;
        }
        drawRouteSegment(previous, target.approachPositionMeters);
    }

    if (showMissionGuides)
    {
        for (const auto& [droneId, mission] : fleetReconnaissance_)
        {
            if (mission.status != "출발 준비" && mission.status != "이동 중"
                && mission.status != "착륙 중")
                continue;
            glm::dvec3 previous = mission.startPositionMeters;
            for (const glm::dvec3& waypoint : mission.routeWaypointsMeters)
            {
                drawRouteSegment(previous, waypoint);
                previous = waypoint;
            }
            const glm::vec3 marker = ToRenderPosition(
                inspectionTargets_[mission.targetIndex].approachPositionMeters);
            DrawCube(glm::translate(glm::mat4{1.0F}, marker)
                * glm::scale(glm::mat4{1.0F}, glm::vec3{0.65F}),
                cameraPosition, missionRouteMaterial_, view, projection);
            static_cast<void>(droneId);
        }
    }

    for (const aerosimlab::simulation::DroneId id : fleetDroneIds_)
    {
        if (hiddenDroneId == id)
            continue;

        const auto& drone = simulation_.State(id);
        const glm::vec3 position = ToRenderPosition(drone.positionMeters);
        const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
            glm::mat4_cast(ToRenderOrientation(drone.orientation));
        droneModel_->Draw(heading, *renderer_, *shader_, view, projection,
            cameraPosition, &droneMaterial_, droneSurfaceTexture_.get(), true);
        const auto& cameraState = simulation_.Camera(id);
        const glm::vec3 cameraForward = glm::normalize(ToRenderDirection(
            aerosimlab::simulation::EoCameraModel::OpticalAxis(cameraState)));
        const glm::vec3 cameraUp = glm::normalize(ToRenderDirection(
            aerosimlab::simulation::EoCameraModel::UpAxis(cameraState)));
        // 이 GLB의 렌즈는 모델의 -Z 방향을 향한다. 렌즈가 비행 방향을 보도록
        // 카메라 모델의 뒤쪽(+Z)을 광축의 반대편에 맞춘다.
        const glm::vec3 cameraBackward = -cameraForward;
        const glm::vec3 cameraRight = glm::normalize(
            glm::cross(cameraForward, cameraUp));
        const glm::vec3 correctedCameraUp = glm::normalize(
            glm::cross(cameraBackward, cameraRight));
        glm::mat3 cameraOrientation{1.0F};
        cameraOrientation[0] = cameraRight;
        cameraOrientation[1] = correctedCameraUp;
        cameraOrientation[2] = cameraBackward;
        constexpr float cameraModelScale = 1.0F;
        const glm::mat4 cameraMount = glm::translate(glm::mat4{1.0F},
                ToRenderPosition(cameraState.positionMeters))
            * glm::mat4_cast(glm::quat_cast(cameraOrientation))
            * glm::translate(glm::mat4{1.0F}, -cameraLensCenterInModelMeters)
            * glm::scale(glm::mat4{1.0F}, glm::vec3{cameraModelScale});
        droneCameraModel_->Draw(cameraMount, *renderer_, *shader_, view, projection,
            cameraPosition);

        if (showMissionGuides && InspectionMissionIsActive()
            && cameraState.status == aerosimlab::simulation::CameraStatus::Active)
        {
            const glm::dvec3 cameraForward = glm::normalize(
                aerosimlab::simulation::EoCameraModel::OpticalAxis(cameraState));
            const glm::dvec3 cameraUp = glm::normalize(
                aerosimlab::simulation::EoCameraModel::UpAxis(cameraState));
            const glm::dvec3 cameraRight = glm::normalize(
                glm::cross(cameraForward, cameraUp));
            constexpr double viewDistanceMeters = 1.5;
            const double halfHeightMeters = viewDistanceMeters * std::tan(
                glm::radians(cameraState.model.verticalFieldOfViewDegrees * 0.5));
            const double halfWidthMeters = halfHeightMeters
                * cameraState.model.imageWidthPixels
                / static_cast<double>(cameraState.model.imageHeightPixels);
            const glm::dvec3 farCenter = cameraState.positionMeters
                + cameraForward * viewDistanceMeters;
            const std::array<glm::dvec3, 4> corners{
                farCenter - cameraRight * halfWidthMeters - cameraUp * halfHeightMeters,
                farCenter + cameraRight * halfWidthMeters - cameraUp * halfHeightMeters,
                farCenter + cameraRight * halfWidthMeters + cameraUp * halfHeightMeters,
                farCenter - cameraRight * halfWidthMeters + cameraUp * halfHeightMeters};

            for (std::size_t corner = 0; corner < corners.size(); ++corner)
            {
                drawRouteSegment(cameraState.positionMeters, corners[corner]);
                drawRouteSegment(corners[corner], corners[(corner + 1U) % corners.size()]);
            }
        }
    }
}

void AeroSimLabApp::DrawCube(const glm::mat4& transform, const glm::vec3& cameraPosition,
    const aerosimlab::graphics::Material& material,
    const glm::mat4& view, const glm::mat4& projection)
{
    renderer_->Draw(transform, *cube_, material, *shader_, view, projection, cameraPosition);
}

}

#ifdef __EMSCRIPTEN__
extern "C" EMSCRIPTEN_KEEPALIVE void DroneSimWebFlightCommand(int command)
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return;

    using Kind = aerosimlab::simulation::DroneCommand::Kind;
    switch (command)
    {
    case 0:
        aerosimlab::simulator::activeWebApplication->QueueWebFlightCommand(Kind::TakeOff);
        break;
    case 1:
        aerosimlab::simulator::activeWebApplication->QueueWebFlightCommand(Kind::Stop);
        break;
    case 2:
        aerosimlab::simulator::activeWebApplication->QueueWebFlightCommand(Kind::Land);
        break;
    default:
        break;
    }
}

extern "C" EMSCRIPTEN_KEEPALIVE void DroneSimWebPilotInput(
    double forward, double right, double vertical, double yaw)
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return;

    aerosimlab::simulator::activeWebApplication->SetWebPilotAxes({
        .forward = forward,
        .right = right,
        .vertical = vertical,
        .yaw = yaw});
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebStartInspectionMission(int targetIndex)
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->StartWebInspectionMission(targetIndex)
        ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebPauseInspectionMission()
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->PauseWebInspectionMission() ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebResumeInspectionMission()
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->ResumeWebInspectionMission() ? 1 : 0;
}

extern "C" EMSCRIPTEN_KEEPALIVE int DroneSimWebCancelInspectionMission()
{
    if (aerosimlab::simulator::activeWebApplication == nullptr)
        return 0;
    return aerosimlab::simulator::activeWebApplication
        ->CancelWebInspectionMission() ? 1 : 0;
}
#endif
