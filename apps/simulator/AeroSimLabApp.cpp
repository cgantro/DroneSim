#include "AeroSimLabApp.h"
#include "ExecutableDirectory.h"

#ifdef _WIN32
#include <Windows.h>
#endif

#include "graphics/Camera.h"
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
constexpr int windowWidth = 1280;
constexpr int windowHeight = 720;
// 하천 안에서 시작하지 않고 111동 외벽을 볼 수 있는 지상 지점이다.
constexpr glm::dvec2 demoDroneStartMeters{114.0, 100.0};

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
    aerosimlab::terrain::BuildingHeightSource heightSource)
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

        const std::uint32_t roofStart = static_cast<std::uint32_t>(vertices.size());
        const double winding = PolygonArea(building.outlineMeters) > 0.0 ? 1.0 : -1.0;
        for (std::size_t i = 0; i < building.outlineMeters.size(); ++i)
        {
            const auto& point = building.outlineMeters[i];
            const double roofHeight = building.groundElevationsMeters[i]
                + building.heightMeters;
            vertices.push_back({
                ToRenderPosition({point.x, point.y, roofHeight}),
                {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F}});
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

        for (std::size_t i = 0; i < building.outlineMeters.size(); ++i)
        {
            const std::size_t next = (i + 1U) % building.outlineMeters.size();
            const auto& firstPoint = building.outlineMeters[i];
            const auto& secondPoint = building.outlineMeters[next];
            const glm::dvec2 edge = secondPoint - firstPoint;
            glm::dvec3 outward{edge.y, 0.0, edge.x};
            outward *= winding;
            const glm::vec3 normal = glm::normalize(ToRenderDirection(outward));
            const auto bottomFirst = ToRenderPosition({firstPoint.x, firstPoint.y,
                building.groundElevationsMeters[i]});
            const auto bottomSecond = ToRenderPosition({secondPoint.x, secondPoint.y,
                building.groundElevationsMeters[next]});
            const auto topFirst = ToRenderPosition({firstPoint.x, firstPoint.y,
                building.groundElevationsMeters[i] + building.heightMeters});
            const auto topSecond = ToRenderPosition({secondPoint.x, secondPoint.y,
                building.groundElevationsMeters[next] + building.heightMeters});
            const std::uint32_t wallStart = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {
                {bottomFirst, normal, {0.0F, 0.0F}},
                {bottomSecond, normal, {0.0F, 0.0F}},
                {topSecond, normal, {0.0F, 1.0F}},
                {topFirst, normal, {0.0F, 1.0F}}});
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

/** @brief OSM 교량 외곽선을 DEM 지면 위에 얇은 선으로 표시한다. */
std::unique_ptr<aerosimlab::graphics::Mesh> CreateBridgeFootprintMesh(
    const std::vector<aerosimlab::terrain::MapBridgeFootprint>& bridges)
{
    constexpr double halfWidthMeters = 0.6;
    constexpr double surfaceOffsetMeters = 0.35;
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;

    for (const auto& bridge : bridges)
    {
        if (bridge.outlineMeters.size() < 3U
            || bridge.outlineMeters.size() != bridge.groundElevationsMeters.size())
            continue;

        for (std::size_t index = 0; index < bridge.outlineMeters.size(); ++index)
        {
            const std::size_t next = (index + 1U) % bridge.outlineMeters.size();
            const glm::dvec2& first = bridge.outlineMeters[index];
            const glm::dvec2& second = bridge.outlineMeters[next];
            const glm::dvec2 direction = second - first;
            const double length = glm::length(direction);
            if (!(length > 1.0e-6))
                continue;

            const glm::dvec2 side = glm::dvec2{-direction.y, direction.x}
                * (halfWidthMeters / length);
            const auto toBoundary = [&](const glm::dvec2& point,
                const glm::dvec2& offset, double groundElevation)
            {
                return ToRenderPosition({point.x + offset.x, point.y + offset.y,
                    groundElevation + surfaceOffsetMeters});
            };

            const std::uint32_t start = static_cast<std::uint32_t>(vertices.size());
            vertices.insert(vertices.end(), {
                {toBoundary(first, side, bridge.groundElevationsMeters[index]),
                    {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F}},
                {toBoundary(first, -side, bridge.groundElevationsMeters[index]),
                    {0.0F, 1.0F, 0.0F}, {0.0F, 1.0F}},
                {toBoundary(second, -side, bridge.groundElevationsMeters[next]),
                    {0.0F, 1.0F, 0.0F}, {1.0F, 1.0F}},
                {toBoundary(second, side, bridge.groundElevationsMeters[next]),
                    {0.0F, 1.0F, 0.0F}, {1.0F, 0.0F}}});
            indices.insert(indices.end(), {start, start + 1U, start + 2U,
                start, start + 2U, start + 3U});
        }
    }

    if (indices.empty())
        return {};
    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}

#ifdef __EMSCRIPTEN__
/**
 * @brief 웹 화면에서 빠르게 그릴 간단한 쿼드콥터 모형을 만든다.
 * @details 상세한 기체 GLB 대신 몸체·팔·모터·프로펠러를 상자 13개로 묶어 그린다.
 */
std::unique_ptr<aerosimlab::graphics::Mesh> CreateWebDroneMesh()
{
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;
    constexpr std::array<std::array<std::uint32_t, 4>, 6> faces{{
        {{0U, 1U, 2U, 3U}}, {{5U, 4U, 7U, 6U}},
        {{4U, 0U, 3U, 7U}}, {{1U, 5U, 6U, 2U}},
        {{3U, 2U, 6U, 7U}}, {{4U, 5U, 1U, 0U}}
    }};
    const std::array<glm::vec3, 8> corners{{
        {-0.5F, -0.5F, -0.5F}, {0.5F, -0.5F, -0.5F},
        {0.5F, 0.5F, -0.5F}, {-0.5F, 0.5F, -0.5F},
        {-0.5F, -0.5F, 0.5F}, {0.5F, -0.5F, 0.5F},
        {0.5F, 0.5F, 0.5F}, {-0.5F, 0.5F, 0.5F}
    }};
    const auto addBox = [&](const glm::vec3& center, const glm::vec3& size, float angle)
    {
        const glm::mat4 rotation = glm::rotate(
            glm::mat4{1.0F}, angle, glm::vec3{0.0F, 1.0F, 0.0F});
        const std::uint32_t first = static_cast<std::uint32_t>(vertices.size());
        for (const auto& face : faces)
        {
            const glm::vec3 a = corners[face[0]] * size;
            const glm::vec3 b = corners[face[1]] * size;
            const glm::vec3 c = corners[face[2]] * size;
            const glm::vec3 normal = glm::normalize(glm::cross(c - a, b - a));
            for (const std::uint32_t corner : face)
            {
                const glm::vec3 local = corners[corner] * size;
                vertices.push_back({
                    center + glm::vec3(rotation * glm::vec4{local, 0.0F}),
                    glm::normalize(glm::vec3(rotation * glm::vec4{normal, 0.0F})),
                    {0.0F, 0.0F}});
            }
        }
        for (std::uint32_t face = 0; face < 6U; ++face)
        {
            const std::uint32_t start = first + face * 4U;
            indices.insert(indices.end(), {start, start + 2U, start + 1U,
                start, start + 3U, start + 2U});
        }
    };

    addBox({}, {0.42F, 0.12F, 0.28F}, 0.0F);
    constexpr std::array<float, 2> sides{-1.0F, 1.0F};
    for (const float xSide : sides)
    {
        for (const float zSide : sides)
        {
            const float angle = std::atan2(zSide, xSide);
            addBox({xSide * 0.20F, 0.0F, zSide * 0.20F}, {0.52F, 0.035F, 0.035F}, angle);
            addBox({xSide * 0.38F, 0.0F, zSide * 0.38F}, {0.14F, 0.055F, 0.14F}, 0.0F);
            addBox({xSide * 0.38F, 0.04F, zSide * 0.38F}, {0.30F, 0.012F, 0.025F}, 0.0F);
        }
    }
    return std::make_unique<aerosimlab::graphics::Mesh>(
        vertices.data(), static_cast<std::uint32_t>(vertices.size()),
        indices.data(), static_cast<std::uint32_t>(indices.size()));
}
#endif

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
            const float south = (static_cast<float>(row) -
                static_cast<float>(count - 1U) * 0.5F) * terrain.spacingYMeters;
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
                {east, terrain.referenceElevationMeters + terrain.HeightAt(row, column), south},
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

/** @brief OSM 물 경계와 DEM 높이로 물 표면을 만든다. */
std::unique_ptr<aerosimlab::graphics::Mesh> CreateWaterMesh(
    const std::vector<aerosimlab::terrain::MapWaterBody>& waterBodies,
    const aerosimlab::terrain::TerrainGrid& terrain)
{
    constexpr double cellSizeMeters = 12.0;
    constexpr double surfaceOffsetMeters = 0.25;
    const double halfX = (static_cast<double>(terrain.sampleCount) - 1.0)
        * terrain.spacingXMeters * 0.5;
    const double halfY = (static_cast<double>(terrain.sampleCount) - 1.0)
        * terrain.spacingYMeters * 0.5;
    std::vector<aerosimlab::graphics::Vertex> vertices;
    std::vector<std::uint32_t> indices;

    for (double south = -halfY; south < halfY; south += cellSizeMeters)
    {
        const double north = std::min(south + cellSizeMeters, halfY);
        for (double west = -halfX; west < halfX; west += cellSizeMeters)
        {
            const double east = std::min(west + cellSizeMeters, halfX);
            const glm::dvec2 center{(west + east) * 0.5, (south + north) * 0.5};
            const bool isWater = std::any_of(waterBodies.begin(), waterBodies.end(),
                [&](const aerosimlab::terrain::MapWaterBody& body)
                {
                    return aerosimlab::terrain::ContainsWater(body, center);
                });
            if (!isWater)
                continue;

            const std::uint32_t first = static_cast<std::uint32_t>(vertices.size());
            const auto addCorner = [&](double x, double y, float u, float v)
            {
                const glm::dvec2 point{x, y};
                const double elevation = terrain.HeightAtLocalMeters(point)
                    + surfaceOffsetMeters;
                vertices.push_back({ToRenderPosition({x, y, elevation}),
                    {0.0F, 1.0F, 0.0F}, {u, v}});
            };
            addCorner(west, south, 0.0F, 0.0F);
            addCorner(west, north, 0.0F, 1.0F);
            addCorner(east, north, 1.0F, 1.0F);
            addCorner(east, south, 1.0F, 0.0F);
            indices.insert(indices.end(), {first, first + 1U, first + 2U,
                first, first + 2U, first + 3U});
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
#ifdef __EMSCRIPTEN__
    droneMesh_ = CreateWebDroneMesh();
#else
    droneMesh_ = aerosimlab::graphics::Mesh::LoadGlb(
        AssetPath("models/drones/x500/visual.glb").string());
#endif
#ifndef __EMSCRIPTEN__
    droneSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadPpm(
        AssetPath("models/drones/x500/carbon-weave.ppm"));
#endif
    const auto terrainTexturePath = AssetPath("maps/tancheon/terrain.ppm");
    if (std::filesystem::exists(terrainTexturePath))
        terrainSurfaceTexture_ = aerosimlab::graphics::Texture2D::LoadPpm(
            terrainTexturePath);
}

void AeroSimLabApp::LoadTerrain()
{
    terrainGrid_ = aerosimlab::terrain::TerrainGrid::Load(
        AssetPath("maps/tancheon/terrain.aerotrn"));
    simulation_.SetTerrain(terrainGrid_);
    terrainMesh_ = CreateTerrainMesh(terrainGrid_);
}

void AeroSimLabApp::LoadMapFeatures()
{
    routeBuildings_ = aerosimlab::terrain::LoadMapBuildingFootprints(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
    const auto& buildings = routeBuildings_;
    const auto roads = aerosimlab::terrain::LoadMapRoads(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
    const auto bridges = aerosimlab::terrain::LoadMapBridgeFootprints(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
    const auto waterBodies = aerosimlab::terrain::LoadMapWaterBodies(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
    roadMesh_ = CreateRoadMesh(roads, false);
    bridgeMesh_ = CreateRoadMesh(roads, true);
    bridgeFootprintMesh_ = CreateBridgeFootprintMesh(bridges);
    waterMesh_ = CreateWaterMesh(waterBodies, terrainGrid_);
    taggedBuildingMesh_ = CreateBuildingMesh(buildings,
        aerosimlab::terrain::BuildingHeightSource::OsmHeightTag);
    estimatedBuildingMesh_ = CreateBuildingMesh(buildings,
        aerosimlab::terrain::BuildingHeightSource::EstimatedFromLevels);
    inspectionOccluders_.reserve(buildings.size());
    std::unordered_map<std::string, const aerosimlab::terrain::MapBuilding*> buildingsById;
    for (const auto& building : buildings)
    {
        const auto bounds =
            aerosimlab::terrain::GetMapBuildingBoundsMeters(building);
        simulation_.AddStaticObstacleBox(
            bounds.centerMeters, glm::vec3{bounds.halfExtentsMeters});
        inspectionOccluders_.push_back({building.outlineMeters,
            bounds.centerMeters.z - bounds.halfExtentsMeters.z,
            bounds.centerMeters.z + bounds.halfExtentsMeters.z});
        if (building.heightSource
            == aerosimlab::terrain::BuildingHeightSource::OsmHeightTag)
            buildingsById.emplace(building.osmId, &building);
    }

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
        if (!zoneIds.insert(zoneId).second)
            throw std::runtime_error("점검 계획에 같은 구역 ID가 두 번 있습니다: " + zoneId);

        aerosimlab::simulation::InspectionSurface surface;
        std::vector<glm::dvec3> routeWaypoints;
        glm::dvec3 approachPosition{0.0};
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
            const std::string buildingId =
                target.at("building_osm_id").get<std::string>();
            const auto building = buildingsById.find(buildingId);
            if (building == buildingsById.end())
                throw std::runtime_error("점검 계획에서 지정한 건물 자료를 찾지 못했습니다: "
                    + buildingId);

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
            .zoneId = zoneId,
            .surface = surface,
            .routeWaypointsMeters = std::move(routeWaypoints),
            .approachPositionMeters = approachPosition,
            .routeAutomaticallyPlanned = routeAutomaticallyPlanned});
    }

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
            const auto route = aerosimlab::terrain::PlanMapFlightRoute(
                terrainGrid_, routeBuildings_, routeStart,
                target.approachPositionMeters);
            if (!route)
                throw std::runtime_error("점검 구역까지 건물을 피하는 경로를 찾지 못했습니다: "
                    + target.zoneId);
            target.routeWaypointsMeters.assign(route->begin(), route->end() - 1);
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
                    $0, UTF8ToString($1), UTF8ToString($2));
        }, static_cast<int>(index), target.facilityId.c_str(),
            target.zoneId.c_str());
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
    if (demoMode_ && !externalControl_
        && inspectionMission_.stage == InspectionMissionStage::Ready)
        static_cast<void>(RequestInspectionMission());
    AdvanceInspectionMission();
    ExpireGroundControlCommand();
    const bool groundControlTickAdvanced = ProcessPendingGroundCommand();
    if (!groundControlTickAdvanced
        && groundControlCommandExpiresAtTick_.empty()
        && groundControlActive_)
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
        eoCamera_->GetPosition(), false);
    renderer_->EndFrame();
    EM_ASM({
        if (window.setDroneSimEoViewport) {
            window.setDroneSimEoViewport($0, $1, $2, $3);
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
        const double missionDistanceMeters = std::hypot(
            inspectionMission_.goalMeters.x - dronePosition.x,
            inspectionMission_.goalMeters.y - dronePosition.y);
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
            missionDistanceMeters, inspectionMission_.facilityId, InspectionMissionStatus(),
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
    observerCamera_->SetPosition(dronePosition + glm::vec3{8.0F, 6.0F, 8.0F});
    observerCamera_->SetTarget(dronePosition + glm::vec3{0.0F, 0.4F, 0.0F});
    observerCamera_->SetAspectRatio(static_cast<float>(width) / static_cast<float>(height));
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
    const glm::vec3& cameraPosition, bool showMissionGuides)
{
    renderer_->Draw(glm::mat4{1.0F}, *terrainMesh_, groundMaterial_, *shader_,
        view, projection, cameraPosition, terrainSurfaceTexture_.get(), false,
        terrainSurfaceTexture_ == nullptr);
    if (waterMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *waterMesh_, waterMaterial_, *shader_,
            view, projection, cameraPosition);
    if (roadMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *roadMesh_, roadMaterial_, *shader_,
            view, projection, cameraPosition);
    if (bridgeMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *bridgeMesh_, bridgeMaterial_, *shader_,
            view, projection, cameraPosition);
    if (bridgeFootprintMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *bridgeFootprintMesh_,
            bridgeFootprintMaterial_, *shader_, view, projection, cameraPosition);
    if (taggedBuildingMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *taggedBuildingMesh_, buildingMaterial_,
            *shader_, view, projection, cameraPosition);
    if (estimatedBuildingMesh_)
        renderer_->Draw(glm::mat4{1.0F}, *estimatedBuildingMesh_,
            estimatedBuildingMaterial_, *shader_, view, projection, cameraPosition);

    if (showMissionGuides)
    {
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
            const glm::vec3 localX = glm::normalize(
                glm::cross(referenceAxis, localY));
            const glm::vec3 localZ = glm::cross(localX, localY);
            glm::mat4 orientation{1.0F};
            orientation[0] = glm::vec4{localX, 0.0F};
            orientation[1] = glm::vec4{localY, 0.0F};
            orientation[2] = glm::vec4{localZ, 0.0F};
            const glm::mat4 transform = glm::translate(
                glm::mat4{1.0F}, (start + end) * 0.5F)
                * orientation
                * glm::scale(glm::mat4{1.0F}, glm::vec3{0.10F, length, 0.10F});
            DrawCube(transform, cameraPosition, missionRouteMaterial_, view, projection);
        };

        for (const InspectionTarget& target : inspectionTargets_)
        {
            glm::dvec3 previous = target.routeStartMeters;
            for (const glm::dvec3& waypoint : target.routeWaypointsMeters)
            {
                drawRouteSegment(previous, waypoint);
                previous = waypoint;
            }
            drawRouteSegment(previous, target.approachPositionMeters);
        }
    }

    for (const aerosimlab::simulation::DroneId id : fleetDroneIds_)
    {
        const auto& drone = simulation_.State(id);
        const glm::vec3 position = ToRenderPosition(drone.positionMeters);
        const glm::mat4 heading = glm::translate(glm::mat4{1.0F}, position) *
            glm::mat4_cast(ToRenderOrientation(drone.orientation));
        renderer_->Draw(heading, *droneMesh_, droneMaterial_, *shader_,
            view, projection, cameraPosition,
#ifndef __EMSCRIPTEN__
            droneSurfaceTexture_.get(),
#else
            nullptr,
#endif
            true);
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
