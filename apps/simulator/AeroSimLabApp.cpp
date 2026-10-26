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
#endif
#include "simulation_core/QuadrotorFlightController.h"
#include "simulation_core/InspectionSurfaceJson.h"
#include "simulation_core/ScopedDuration.h"
#include "terrain/MapBridges.h"
#include "terrain/MapBuildings.h"
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
// 창이 멈춘 뒤 밀린 시간을 전부 따라잡느라 다음 화면까지 늦어지지 않게 한다.
constexpr std::size_t smokeTestFrameLimit = 8;
// 소프트웨어 렌더러에서도 네트워크 스모크의 모의가 화면 속도에 묶이지 않게 한다.
constexpr std::size_t networkSmokeSimulationBatchesPerFrame = 5;
constexpr glm::dvec2 demoDroneStartMeters{0.0, -9.0};
constexpr double benchmarkDurationSeconds = 5.0;

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
    for (std::size_t axis = 0; axis < 3; ++axis)
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
            vertices.push_back({
                {east, terrain.referenceElevationMeters + terrain.HeightAt(row, column), south},
                {0.0F, 1.0F, 0.0F},
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

void AeroSimLabApp::SetWebPilotAxes(
    const aerosimlab::manual_control::GamepadAxes& axes)
{
    externalControl_ = true;
    if (!commandRecording_.IsReplaying())
        webPilotAxes_ = axes;
}
#endif

int AeroSimLabApp::Run()
{
#ifndef __EMSCRIPTEN__
    if (networkEnabled_)
    {
        udpSocket_ = std::make_unique<aerosimlab::communication::UdpSocket>(udpPort_);
    }
#endif
#ifndef __EMSCRIPTEN__
    // 그래픽 자산을 불러오는 동안 도착한 UDP 요청도 소켓 수신 큐에 남긴다.
    InitializeGraphics();
#endif
    startTime_ = std::chrono::steady_clock::now();
#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop_arg([](void* application)
    {
        auto& app = *static_cast<AeroSimLabApp*>(application);
        try
        {
            if (!app.webGraphicsInitialized_)
            {
                app.webGraphicsInitialized_ = app.InitializeGraphicsStep();
                if (app.webGraphicsInitialized_)
                {
                    app.startTime_ = std::chrono::steady_clock::now();
                    app.webPreviousFrame_ = app.startTime_;
                    app.lastWebTelemetryUpdate_ = app.startTime_;
                }
                return;
            }

            if (app.window_->ShouldClose())
            {
                emscripten_cancel_main_loop();
                return;
            }
            app.RunFrame(app.webPreviousFrame_);
        }
        catch (const std::exception& error)
        {
            const std::string message = error.what();
            EM_ASM({
                window.showSimulatorStartupError(
                    "시뮬레이터를 시작하지 못했습니다. " + UTF8ToString($0));
            }, message.c_str());
            emscripten_cancel_main_loop();
        }
        catch (...)
        {
            EM_ASM({
                window.showSimulatorStartupError("시뮬레이터를 시작하지 못했습니다.");
            });
            emscripten_cancel_main_loop();
        }
    }, this, 0, false);
    return 0;
#else
    benchmarkCpuStart_ = std::clock();
    auto previousFrame = startTime_;

    if (benchmarkMode_)
    {
        while (!window_->ShouldClose()
            && std::chrono::steady_clock::now() - startTime_
                < std::chrono::duration<double>(benchmarkWarmupDurationSeconds))
        {
            RunFrame(previousFrame);
        }

        // 초기 셰이더·버퍼 생성 지연은 측정 구간에서 빼고, 워밍업 뒤 기준값부터 기록한다.
        benchmarkMeasurements_ = {};
        benchmarkStartRenderedFrames_ = renderedFrames_;
        benchmarkStartSimulationTicks_ = simulation_.TickCount();
        benchmarkCpuStart_ = std::clock();
        startTime_ = std::chrono::steady_clock::now();
        previousFrame = startTime_;
    }

    while (!window_->ShouldClose())
    {
        RunFrame(previousFrame);

        if (smokeTest_ && SmokeTestIsComplete())
            break;
        if (smokeTest_ && networkEnabled_
            && std::chrono::steady_clock::now() - startTime_ > std::chrono::seconds(30))
            return 4;
        if (benchmarkMode_ &&
            std::chrono::steady_clock::now() - startTime_ >= std::chrono::duration<double>(benchmarkDurationSeconds))
            break;
    }
#ifndef __EMSCRIPTEN__
    if (smokeTest_)
        DrainEoReadbacks();
#endif
    if (smokeTest_)
    {
        const int smokeTestResult = ValidateSmokeTestOutput();
        if (smokeTestResult != 0)
            return smokeTestResult;
    }
    if (benchmarkMode_)
    {
        const double elapsedSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - startTime_).count();
        WriteBenchmarkReport(elapsedSeconds);
    }
    return 0;
#endif
}

void AeroSimLabApp::RunFrame(std::chrono::steady_clock::time_point& previousFrame)
{
    const auto applicationFrameStart = benchmarkMode_
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    frameMeasurements_ = {};
    {
        aerosimlab::simulation::ScopedDuration measureEvents(
            benchmarkMode_ ? &frameMeasurements_.eventPollingMicroseconds : nullptr);
        window_->PollEvents();
    }
    {
        aerosimlab::simulation::ScopedDuration measureGroundControl(
            benchmarkMode_ ? &frameMeasurements_.groundControlPollingMicroseconds : nullptr);
        PollGroundControl();
    }
    {
        aerosimlab::simulation::ScopedDuration measureGamepad(
            benchmarkMode_ ? &frameMeasurements_.gamepadPollingMicroseconds : nullptr);
        PollGamepad();
    }
    const auto now = std::chrono::steady_clock::now();
    const double elapsedSeconds = std::chrono::duration<double>(now - previousFrame).count();
    // 화면 스모크는 프레임마다 한 Tick, 네트워크 스모크는 프레임마다 최대 따라잡기만큼 진행한다.
    double frameSeconds = elapsedSeconds;
    if (smokeTest_ && !benchmarkMode_)
    {
        frameSeconds = networkEnabled_
            ? aerosimlab::simulation::FixedTickAccumulator::maximumFrameSeconds
            : aerosimlab::simulation::FixedTickAccumulator::fixedStepSeconds;
    }
    previousFrame = now;

    const std::size_t simulationBatches = smokeTest_ && networkEnabled_
        && !benchmarkMode_
        ? networkSmokeSimulationBatchesPerFrame
        : 1U;
    {
        aerosimlab::simulation::ScopedDuration measureSimulation(
            benchmarkMode_ ? &frameMeasurements_.simulationUpdateMicroseconds : nullptr);
        for (std::size_t batch = 0; batch < simulationBatches; ++batch)
            AdvanceSimulation(frameSeconds);
    }
    eoReadbackHappenedThisFrame_ = false;
    {
        aerosimlab::simulation::ScopedDuration measureRenderFrame(
            benchmarkMode_ ? &frameMeasurements_.renderFrameMicroseconds : nullptr);
        {
            aerosimlab::simulation::ScopedDuration measureRender(
                benchmarkMode_ ? &frameMeasurements_.renderMicroseconds : nullptr);
            RenderFrame();
        }
        {
            aerosimlab::simulation::ScopedDuration measureBufferSwap(
                benchmarkMode_ ? &frameMeasurements_.bufferSwapMicroseconds : nullptr);
            window_->SwapBuffers();
        }
    }
    ++renderedFrames_;
#ifdef __EMSCRIPTEN__
    const auto telemetryTime = std::chrono::steady_clock::now();
    if (telemetryTime - lastWebTelemetryUpdate_ >= std::chrono::milliseconds(250))
    {
        const auto& state = simulation_.State();
        const std::string mode{aerosimlab::simulation::ToString(state.mode)};
        const double speedMetersPerSecond = glm::length(state.velocityMetersPerSecond);
        const bool cameraActive = simulation_.Camera().status
            == aerosimlab::simulation::CameraStatus::Active;
        const auto& camera = simulation_.Camera();
        const std::string captureAssessment{
            aerosimlab::simulation::ToString(AssessInspectionView())};
        const std::uint64_t simulationTick = simulation_.TickCount();
        EM_ASM({
            if (window.updateDroneSimGroundControl) {
                window.updateDroneSimGroundControl(
                    UTF8ToString($0), $1, $2, $3, $4, $5, UTF8ToString($6),
                    UTF8ToString($7), $8);
            }
        }, mode.c_str(), state.positionMeters.x, state.positionMeters.y,
            state.positionMeters.z, speedMetersPerSecond, cameraActive,
            captureAssessment.c_str(), InspectionMissionStatus().c_str(), simulationTick);
        EM_ASM({
            if (window.updateDroneSimInspectionMetadata) {
                window.updateDroneSimInspectionMetadata(
                    $0, $1, $2, $3, $4, $5, $6, UTF8ToString($7), UTF8ToString($8));
            }
        },
            camera.positionMeters.x, camera.positionMeters.y, camera.positionMeters.z,
            camera.orientation.x, camera.orientation.y, camera.orientation.z,
            camera.orientation.w, inspectionFacilityId_.c_str(), inspectionZoneId_.c_str());
        lastWebTelemetryUpdate_ = telemetryTime;
    }
    if (!webFirstFrameRendered_)
    {
        webFirstFrameRendered_ = true;
        EM_ASM({ window.completeSimulatorStartup(); });
    }
#endif
    if (benchmarkMode_)
    {
        const double applicationMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - applicationFrameStart).count();
        RecordBenchmarkFrame(applicationMicroseconds);
    }
}

bool AeroSimLabApp::SmokeTestIsComplete() const
{
    if (!networkEnabled_)
        return renderedFrames_ >= smokeTestFrameLimit;

    return smokeReplayStarted_
        && !commandRecording_.IsRecording()
        && !commandRecording_.IsReplaying()
        && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded;
}

int AeroSimLabApp::ValidateSmokeTestOutput() const
{
    const bool cameraFrameHasPixels = eoFrame_.IsValid()
        && std::any_of(eoFrame_.pixels.begin(), eoFrame_.pixels.end(), [](std::uint8_t value)
        {
            return value != 0;
        });
    if (!cameraFrameHasPixels)
        return 5;

    const auto& camera = simulation_.Camera();
    if (camera.status == aerosimlab::simulation::CameraStatus::Active
        && eoFrame_.simulationTick != camera.lastFrameTick)
    {
        std::cerr << "EO 영상 Tick이 카메라의 마지막 Tick과 다릅니다. 영상="
                  << eoFrame_.simulationTick << ", 카메라="
                  << camera.lastFrameTick << '\n';
        return 6;
    }

    return 0;
}

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
}

void AeroSimLabApp::LoadTerrain()
{
    terrainGrid_ = aerosimlab::terrain::TerrainGrid::Load(
        AssetPath("maps/tancheon/terrain.aerotrn"));
    terrainMesh_ = CreateTerrainMesh(terrainGrid_);
}

void AeroSimLabApp::LoadMapFeatures()
{
    const auto buildings = aerosimlab::terrain::LoadMapBuildings(
        AssetPath("maps/tancheon/site.geojson"), terrainGrid_);
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
#ifndef __EMSCRIPTEN__
        simulation_.AddStaticObstacleBox(
            bounds.centerMeters, glm::vec3{bounds.halfExtentsMeters});
#endif
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

    const bool hasExplicitSurface = std::any_of(
        plan.at("targets").begin(), plan.at("targets").end(),
        [](const Json& target) { return target.contains("inspection_surface"); });
    if (hasExplicitSurface
        && (!plan.contains("coordinate_frame")
            || plan.at("coordinate_frame") != "tancheon_local_enu"))
        throw std::runtime_error(
            "직접 지정한 점검면에는 tancheon_local_enu 좌표 기준을 적어야 합니다.");

    std::unordered_set<std::string> zoneIds;
    for (const Json& target : plan.at("targets"))
    {
        const std::string facilityId = target.at("facility_id").get<std::string>();
        const std::string zoneId = target.at("zone_id").get<std::string>();
        if (!zoneIds.insert(zoneId).second)
            throw std::runtime_error("점검 계획에 같은 구역 ID가 두 번 있습니다: " + zoneId);

        aerosimlab::simulation::InspectionSurface surface;
        glm::dvec3 approachPosition{0.0};
        if (target.contains("inspection_surface"))
        {
            surface = aerosimlab::simulation::ParseInspectionSurfaceJson(
                target.at("inspection_surface").dump());
            approachPosition = ReadPlanVector(
                target.at("approach_position_meters"), "approach_position_meters");
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
        inspectionTargets_.push_back({facilityId, zoneId, surface, approachPosition});
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
    simulation_.SetInitialPose({demoDroneStartMeters.x, demoDroneStartMeters.y,
        startElevation +
            aerosimlab::simulation::QuadrotorFlightController::groundedModelOriginHeightMeters});
    fleetDroneIds_ = simulation_.DroneIds();
#ifndef __EMSCRIPTEN__
    simulation_.SetTerrain(terrainGrid_);
#endif
}

aerosimlab::simulation::CaptureAssessment AeroSimLabApp::AssessInspectionView() const
{
    return aerosimlab::simulation::EoCameraModel::AssessCapture(
        simulation_.Camera(), inspectionSurface_, {}, inspectionOccluders_);
}

void AeroSimLabApp::SetActiveInspectionTarget(std::size_t index)
{
    if (index >= inspectionTargets_.size())
        throw std::out_of_range("점검 구역 번호가 임무 범위를 벗어났습니다.");

    activeInspectionTargetIndex_ = index;
    const InspectionTarget& target = inspectionTargets_[index];
    inspectionFacilityId_ = target.facilityId;
    inspectionZoneId_ = target.zoneId;
    inspectionSurface_ = target.surface;
    missionGoalMeters_ = target.approachPositionMeters;
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
        && inspectionMissionStage_ == InspectionMissionStage::Ready)
        static_cast<void>(RequestInspectionMission());
    AdvanceInspectionMission();
    ExpireGroundControlCommand();
    const bool groundControlTickAdvanced = ProcessPendingGroundCommand();
    if (!groundControlTickAdvanced
        && !groundControlCommandExpiresAtTick_
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
        inspectionMissionStage_ == InspectionMissionStage::StartRequested
        || inspectionMissionStage_ == InspectionMissionStage::TakingOff
        || inspectionMissionStage_ == InspectionMissionStage::Approaching
        || inspectionMissionStage_ == InspectionMissionStage::AligningCamera
        || inspectionMissionStage_ == InspectionMissionStage::Capturing
        || inspectionMissionStage_ == InspectionMissionStage::Landing;
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
    const auto result = simulation_.Apply(command);
    if (result == aerosimlab::simulation::CommandResult::Accepted)
        commandRecording_.Record(command);
    return result;
}

bool AeroSimLabApp::RequestInspectionMission(
    std::optional<std::size_t> selectedTargetIndex)
{
    if (inspectionMissionStage_ == InspectionMissionStage::StartRequested
        || inspectionMissionStage_ == InspectionMissionStage::TakingOff
        || inspectionMissionStage_ == InspectionMissionStage::Approaching
        || inspectionMissionStage_ == InspectionMissionStage::AligningCamera
        || inspectionMissionStage_ == InspectionMissionStage::Capturing
        || inspectionMissionStage_ == InspectionMissionStage::Landing)
    {
        inspectionMissionError_ = "점검 임무가 이미 진행 중입니다.";
        return false;
    }

    if (simulation_.State().mode != aerosimlab::simulation::FlightMode::Grounded)
    {
        inspectionMissionError_ = "기체가 지상에 있을 때 임무를 시작할 수 있습니다.";
        inspectionMissionStage_ = InspectionMissionStage::Failed;
        return false;
    }

    inspectionMissionError_.clear();
    inspectionMissionStartTick_ = 0;
    inspectionCaptureStartFrame_ = 0;
    inspectionCaptureReadyTick_ = 0;
    inspectionReadyFrameSaved_ = false;
    completedInspectionTargetCount_ = 0;
    if (selectedTargetIndex
        && *selectedTargetIndex >= inspectionTargets_.size())
    {
        inspectionMissionError_ = "선택한 점검 구역을 찾을 수 없습니다.";
        inspectionMissionStage_ = InspectionMissionStage::Failed;
        return false;
    }
    activeInspectionTargetIndex_ = selectedTargetIndex.value_or(0U);
    inspectionMissionEndTargetIndex_ = selectedTargetIndex
        ? *selectedTargetIndex + 1U
        : inspectionTargets_.size();
    inspectionMissionTargetCount_ = inspectionMissionEndTargetIndex_
        - activeInspectionTargetIndex_;
    SetActiveInspectionTarget(activeInspectionTargetIndex_);
    pendingControllerAction_.reset();
    pendingPilotCommand_.reset();
    inspectionMissionStage_ = InspectionMissionStage::StartRequested;
    return true;
}

void AeroSimLabApp::AdvanceInspectionMission()
{
    using Stage = InspectionMissionStage;
    using CommandKind = aerosimlab::simulation::DroneCommand::Kind;

    if (inspectionMissionStage_ == Stage::StartRequested)
    {
#ifndef __EMSCRIPTEN__
        aerosimlab::communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            aerosimlab::communication::VideoRecordingCommand::Start, recordingReply);
        if (!recordingReply.accepted)
        {
            FailInspectionMission(recordingReply.status);
            return;
        }
#endif
        const aerosimlab::simulation::DroneCommand takeoff{CommandKind::TakeOff};
        if (ApplyAndRecordCommand(takeoff)
            != aerosimlab::simulation::CommandResult::Accepted)
        {
            FailInspectionMission("이륙 명령을 적용하지 못했습니다.");
            return;
        }
        inspectionMissionStartTick_ = simulation_.TickCount();
        inspectionMissionStage_ = Stage::TakingOff;
    }

    const std::uint64_t elapsedTicks = simulation_.TickCount()
        - inspectionMissionStartTick_;
    if (inspectionMissionStage_ != Stage::Ready
        && inspectionMissionStage_ != Stage::Saved
        && inspectionMissionStage_ != Stage::GeometryOnly
        && inspectionMissionStage_ != Stage::Failed
        && elapsedTicks > 60U * 180U * inspectionMissionTargetCount_)
    {
        FailInspectionMission("임무 제한 시간 안에 구역 촬영을 마치지 못했습니다.");
        return;
    }

    if (inspectionMissionStage_ == Stage::TakingOff)
    {
        const auto mode = simulation_.State().mode;
        if (mode == aerosimlab::simulation::FlightMode::Hovering
            || mode == aerosimlab::simulation::FlightMode::Flying)
        {
            aerosimlab::simulation::DroneCommand waypoint;
            waypoint.kind = CommandKind::SetWaypoint;
            waypoint.vector = missionGoalMeters_;
            if (ApplyAndRecordCommand(waypoint)
                != aerosimlab::simulation::CommandResult::Accepted)
            {
                FailInspectionMission("점검 구역 이동 명령을 적용하지 못했습니다.");
                return;
            }
            inspectionMissionStage_ = Stage::Approaching;
        }
        return;
    }

    if (inspectionMissionStage_ == Stage::Approaching)
    {
        const double distanceMeters = glm::length(
            missionGoalMeters_ - simulation_.State().positionMeters);
        if (distanceMeters <= 2.0)
            inspectionMissionStage_ = Stage::AligningCamera;
        return;
    }

    if (inspectionMissionStage_ == Stage::AligningCamera)
    {
        const auto& state = simulation_.State();
        const glm::dvec2 direction{
            inspectionSurface_.centerMeters.x - state.positionMeters.x,
            inspectionSurface_.centerMeters.y - state.positionMeters.y};
        const double targetYawRadians = std::atan2(direction.y, direction.x);
        constexpr double twoPi = 6.28318530717958647692;
        const double yawErrorRadians = std::remainder(
            targetYawRadians - state.yawRadians, twoPi);
        if (std::abs(yawErrorRadians) <= 0.04)
        {
            const aerosimlab::simulation::DroneCommand stopYaw{
                CommandKind::SetYawRate, glm::dvec3{0.0}, 0.0};
            static_cast<void>(ApplyAndRecordCommand(stopYaw));
            inspectionCaptureStartFrame_ = simulation_.Camera().lastFrameTick;
            inspectionMissionStage_ = Stage::Capturing;
#ifdef __EMSCRIPTEN__
            EM_ASM({
                if (window.startDroneSimInspectionZoneRecording)
                    window.startDroneSimInspectionZoneRecording(
                        UTF8ToString($0), UTF8ToString($1), $2);
            }, inspectionFacilityId_.c_str(), inspectionZoneId_.c_str(),
                static_cast<double>(simulation_.TickCount()));
#endif
        }
        else
        {
            const aerosimlab::simulation::DroneCommand alignYaw{
                CommandKind::SetYawRate, glm::dvec3{0.0},
                std::clamp(yawErrorRadians * 2.0, -1.0, 1.0)};
            if (ApplyAndRecordCommand(alignYaw)
                != aerosimlab::simulation::CommandResult::Accepted)
            {
                FailInspectionMission("점검면을 향해 카메라 방향을 맞추지 못했습니다.");
                return;
            }
        }
        return;
    }

    if (inspectionMissionStage_ == Stage::Capturing)
    {
#ifdef __EMSCRIPTEN__
        const auto& camera = simulation_.Camera();
        if (camera.lastFrameTick > inspectionCaptureStartFrame_)
        {
            if (AssessInspectionView()
                == aerosimlab::simulation::CaptureAssessment::Ready)
            {
                if (!inspectionReadyFrameSaved_)
                    inspectionCaptureReadyTick_ = simulation_.TickCount();
                inspectionReadyFrameSaved_ = true;
            }
            else
            {
                inspectionReadyFrameSaved_ = false;
                inspectionCaptureReadyTick_ = 0;
            }
        }
        constexpr std::uint64_t captureHoldTicks = 60U;
        const bool captureHoldComplete = inspectionReadyFrameSaved_
            && simulation_.TickCount() - inspectionCaptureReadyTick_ >= captureHoldTicks;
#endif
#ifdef __EMSCRIPTEN__
        if (captureHoldComplete)
#else
        if (inspectionReadyFrameSaved_)
#endif
        {
#ifdef __EMSCRIPTEN__
            EM_ASM({
                if (window.recordDroneSimCapturedZone) {
                    window.recordDroneSimCapturedZone(
                        UTF8ToString($0), UTF8ToString($1), $2);
                }
            }, inspectionFacilityId_.c_str(), inspectionZoneId_.c_str(),
                static_cast<double>(camera.lastFrameTick));
#endif
            ++completedInspectionTargetCount_;
            if (activeInspectionTargetIndex_ + 1U < inspectionMissionEndTargetIndex_)
            {
                SetActiveInspectionTarget(activeInspectionTargetIndex_ + 1U);
                aerosimlab::simulation::DroneCommand waypoint;
                waypoint.kind = CommandKind::SetWaypoint;
                waypoint.vector = missionGoalMeters_;
                if (ApplyAndRecordCommand(waypoint)
                    != aerosimlab::simulation::CommandResult::Accepted)
                {
                    FailInspectionMission("다음 점검 구역 이동 명령을 적용하지 못했습니다.");
                    return;
                }
                inspectionCaptureStartFrame_ = 0;
                inspectionCaptureReadyTick_ = 0;
                inspectionReadyFrameSaved_ = false;
                inspectionMissionStage_ = Stage::Approaching;
                return;
            }

            const aerosimlab::simulation::DroneCommand land{CommandKind::Land};
            if (ApplyAndRecordCommand(land)
                != aerosimlab::simulation::CommandResult::Accepted)
            {
                FailInspectionMission("촬영 후 착륙 명령을 적용하지 못했습니다.");
                return;
            }
            inspectionMissionStage_ = Stage::Landing;
        }
        return;
    }

    if (inspectionMissionStage_ == Stage::Landing
        && simulation_.State().mode == aerosimlab::simulation::FlightMode::Grounded)
    {
#ifndef __EMSCRIPTEN__
        if (eoVideoRecorder_)
            eoVideoRecorder_->SetMissionOutcome(
                EoVideoRecorder::MissionOutcome::Completed);
        aerosimlab::communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            aerosimlab::communication::VideoRecordingCommand::Stop, recordingReply);
        if (!recordingReply.accepted)
        {
            FailInspectionMission(recordingReply.status);
            return;
        }
        inspectionMissionStage_ = Stage::Saved;
#else
        inspectionMissionStage_ = Stage::GeometryOnly;
#endif
    }
}

void AeroSimLabApp::FailInspectionMission(const std::string& reason)
{
    inspectionMissionError_ = reason;
    inspectionMissionStage_ = InspectionMissionStage::Failed;
    if (simulation_.State().mode != aerosimlab::simulation::FlightMode::Grounded)
    {
        const aerosimlab::simulation::DroneCommand land{
            aerosimlab::simulation::DroneCommand::Kind::Land};
        static_cast<void>(ApplyAndRecordCommand(land));
    }
#ifndef __EMSCRIPTEN__
    if (eoVideoRecorder_ && eoVideoRecorder_->IsRecording())
    {
        eoVideoRecorder_->SetMissionOutcome(
            EoVideoRecorder::MissionOutcome::Failed, reason);
        aerosimlab::communication::CommandReply recordingReply;
        ApplyVideoRecordingCommand(
            aerosimlab::communication::VideoRecordingCommand::Stop, recordingReply);
    }
#endif
}

std::string AeroSimLabApp::InspectionMissionStatus() const
{
    const std::size_t targetCount = inspectionMissionTargetCount_ == 0U
        ? inspectionTargets_.size()
        : inspectionMissionTargetCount_;
    const std::string progress = "구역 "
        + std::to_string(std::min(completedInspectionTargetCount_ + 1U,
            targetCount))
        + "/" + std::to_string(targetCount) + ": ";
    switch (inspectionMissionStage_)
    {
    case InspectionMissionStage::Ready: return "대기 중";
    case InspectionMissionStage::StartRequested: return progress + "시작 준비 중";
    case InspectionMissionStage::TakingOff: return progress + "이륙 중";
    case InspectionMissionStage::Approaching: return progress + "점검 구역으로 이동 중";
    case InspectionMissionStage::AligningCamera:
        return progress + "카메라를 점검면에 맞추는 중";
    case InspectionMissionStage::Capturing: return progress + "촬영 조건 확인 중";
    case InspectionMissionStage::Landing: return "착륙 중";
    case InspectionMissionStage::Saved:
        return "영상 저장 완료 (" + std::to_string(completedInspectionTargetCount_)
            + "개 구역)";
    case InspectionMissionStage::GeometryOnly:
        return "구도 확인 완료 (" + std::to_string(completedInspectionTargetCount_)
            + "개 구역)";
    case InspectionMissionStage::Failed: return "실패: " + inspectionMissionError_;
    }
    return "상태를 알 수 없습니다.";
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
    UpdateObserverCamera(observerWidth, height);
    UpdateEoCamera(cameraWidth, height);
    renderer_->BeginFrame();
    renderer_->SetSceneViewport(0, 0, observerWidth, height);
    DrawWorld(observerCamera_->GetViewMatrix(), observerCamera_->GetProjectionMatrix(),
        observerCamera_->GetPosition());
    renderer_->SetSceneViewport(observerWidth, 0, cameraWidth, height);
    DrawWorld(eoCamera_->GetViewMatrix(), eoCamera_->GetProjectionMatrix(),
        eoCamera_->GetPosition());
    renderer_->EndFrame();
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
            missionGoalMeters_.x - dronePosition.x,
            missionGoalMeters_.y - dronePosition.y);
        const bool missionCanStart = inspectionMissionStage_
                == InspectionMissionStage::Ready
            || inspectionMissionStage_ == InspectionMissionStage::Saved
            || inspectionMissionStage_ == InspectionMissionStage::GeometryOnly
            || inspectionMissionStage_ == InspectionMissionStage::Failed;
        const auto captureAssessment = AssessInspectionView();
        HudRenderTimings hudTimings;
        hudTimings.collectGpuTimings = visibleBenchmarkMode_;
        hudTimings.frameIndex = static_cast<std::uint64_t>(renderedFrames_ + 1U);
        hud_->Render(simulation_, gamepadConnected_, groundControlConnected, cameraActive,
            camera.model.imageWidthPixels, camera.model.imageHeightPixels,
            missionDistanceMeters, inspectionFacilityId_, InspectionMissionStatus(),
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
    const glm::vec3& cameraPosition)
{
    renderer_->Draw(glm::mat4{1.0F}, *terrainMesh_, groundMaterial_, *shader_,
        view, projection, cameraPosition);
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

    const glm::vec3 inspectionCenter = ToRenderPosition(inspectionSurface_.centerMeters);
    const glm::vec3 vertical{0.0F, 1.0F, 0.0F};
    const glm::vec3 surfaceNormal = glm::normalize(
        ToRenderDirection(inspectionSurface_.normal));
    const glm::vec3 surfaceWidth = glm::normalize(
        glm::cross(vertical, surfaceNormal));
    glm::mat4 surfaceOrientation{1.0F};
    surfaceOrientation[0] = glm::vec4{surfaceWidth, 0.0F};
    surfaceOrientation[1] = glm::vec4{vertical, 0.0F};
    surfaceOrientation[2] = glm::vec4{surfaceNormal, 0.0F};
    const glm::mat4 inspectionTransform =
        glm::translate(glm::mat4{1.0F}, inspectionCenter)
        * surfaceOrientation
        * glm::scale(glm::mat4{1.0F}, glm::vec3{
            static_cast<float>(inspectionSurface_.widthMeters),
            static_cast<float>(inspectionSurface_.heightMeters), 0.12F});
    renderer_->Draw(inspectionTransform, *cube_, missionMarkerMaterial_, *shader_,
        view, projection, cameraPosition);
    const glm::vec3 goal = ToRenderPosition(missionGoalMeters_);
    DrawCube(CubeTransform(goal + glm::vec3{0.0F, 0.09F, 0.0F},
        glm::vec3{2.4F, 0.18F, 2.4F}), cameraPosition,
        missionMarkerMaterial_, view, projection);
    DrawCube(CubeTransform(goal + glm::vec3{0.0F, 1.6F, 0.0F},
        glm::vec3{0.18F, 3.0F, 0.18F}), cameraPosition,
        missionMarkerMaterial_, view, projection);

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
#endif
