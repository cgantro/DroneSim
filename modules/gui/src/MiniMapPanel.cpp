#include "gui/MiniMapPanel.h"

#include <imgui.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace dronesim::gui
{
namespace
{
ImU32 BuildingHeightColor(double heightMeters, double maximumHeightMeters)
{
    const float ratio = maximumHeightMeters > 0.0
        ? static_cast<float>(std::clamp(heightMeters / maximumHeightMeters, 0.0, 1.0))
        : 0.0F;
    const ImVec4 low{0.20F, 0.61F, 0.76F, 1.0F};
    const ImVec4 high{0.95F, 0.48F, 0.19F, 1.0F};
    return ImGui::ColorConvertFloat4ToU32(ImVec4{
        low.x + (high.x - low.x) * ratio,
        low.y + (high.y - low.y) * ratio,
        low.z + (high.z - low.z) * ratio, 1.0F});
}

ImU32 RouteColor(std::size_t index)
{
    constexpr std::array<ImU32, 4> colors{
        IM_COL32(55, 220, 245, 255), IM_COL32(255, 225, 70, 255),
        IM_COL32(255, 105, 180, 255), IM_COL32(155, 245, 120, 255)};
    return colors[index % colors.size()];
}

void DrawBuildingMiniMap(
    std::span<const flight_planning::BuildingObstacle> buildings,
    std::span<const MiniMapDrone> drones,
    std::span<const MiniMapRoute> routes,
    const glm::dvec2& groundSizeMeters,
    double& zoom,
    glm::dvec2& centerMeters,
    ImDrawList* drawList, const ImVec2& sceneMinimum, const ImVec2& sceneMaximum)
{
    if (buildings.empty() && drones.empty() && routes.empty())
        return;

    const float panelWidth = sceneMaximum.x - sceneMinimum.x;
    const float panelHeight = sceneMaximum.y - sceneMinimum.y;
    if (panelWidth < 140.0F || panelHeight < 100.0F)
        return;

    const ImVec2 mapMinimum = sceneMinimum;
    const ImVec2 mapMaximum = sceneMaximum;
    drawList->AddRectFilled(mapMinimum, mapMaximum,
        IM_COL32(14, 20, 25, 255));
    drawList->AddRect(mapMinimum, mapMaximum,
        IM_COL32(210, 220, 225, 190), 0.0F, 0, 1.0F);
    drawList->AddText({mapMinimum.x + 10.0F, mapMinimum.y + 7.0F},
        IM_COL32(245, 245, 245, 255), "평면 지도 · 구조물 높이와 경로");
    drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 0.68F,
        {mapMinimum.x + 10.0F, mapMinimum.y + 25.0F},
        IM_COL32(185, 200, 205, 255),
        "A*: 5m 격자 → 장애물 회피 → 직선 구간으로 경로 축약");

    constexpr float headerHeight = 44.0F;
    constexpr float footerHeight = 25.0F;
    constexpr float sidePadding = 12.0F;
    const ImVec2 drawingMinimum{mapMinimum.x + sidePadding, mapMinimum.y + headerHeight};
    const ImVec2 drawingMaximum{mapMaximum.x - sidePadding, mapMaximum.y - footerHeight};
    const ImVec2 drawingSize{drawingMaximum.x - drawingMinimum.x,
        drawingMaximum.y - drawingMinimum.y};
    const double halfEast = std::max(groundSizeMeters.x * 0.5, 1.0);
    const double halfNorth = std::max(groundSizeMeters.y * 0.5, 1.0);
    double maximumHeightMeters = 0.0;
    for (const auto& building : buildings)
        maximumHeightMeters = std::max(maximumHeightMeters, building.heightMeters);

    // 동쪽은 오른쪽, 북쪽은 위쪽으로 놓아 실제 지도처럼 평면에 표시한다.
    const float baseScale = static_cast<float>(std::min(
        drawingSize.x / (2.0 * halfEast), drawingSize.y / (2.0 * halfNorth)));
    const ImVec2 mapCenter{(drawingMinimum.x + drawingMaximum.x) * 0.5F,
        (drawingMinimum.y + drawingMaximum.y) * 0.5F};
    const bool pointerOverMap = ImGui::IsMouseHoveringRect(drawingMinimum, drawingMaximum);
    ImGuiIO& io = ImGui::GetIO();
    if (pointerOverMap && io.MouseWheel != 0.0F)
    {
        const double oldZoom = zoom;
        const double newZoom = std::clamp(
            zoom * std::pow(1.18, static_cast<double>(io.MouseWheel)), 0.5, 8.0);
        const double screenEast = ImGui::GetMousePos().x - mapCenter.x;
        const double screenNorth = ImGui::GetMousePos().y - mapCenter.y;
        centerMeters.x += screenEast / baseScale * (1.0 / oldZoom - 1.0 / newZoom);
        centerMeters.y += screenNorth / baseScale * (1.0 / newZoom - 1.0 / oldZoom);
        zoom = newZoom;
    }
    if (pointerOverMap && ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
    {
        const ImVec2 drag = io.MouseDelta;
        centerMeters.x -= drag.x / (baseScale * zoom);
        centerMeters.y += drag.y / (baseScale * zoom);
    }
    const float scale = baseScale * static_cast<float>(zoom);
    const auto project = [&](const glm::dvec2& position)
    {
        return ImVec2{
            mapCenter.x + static_cast<float>(position.x - centerMeters.x) * scale,
            mapCenter.y - static_cast<float>(position.y - centerMeters.y) * scale};
    };

    drawList->PushClipRect(mapMinimum, mapMaximum, true);
    drawList->AddRectFilled(drawingMinimum, drawingMaximum,
        IM_COL32(25, 38, 42, 255));

    // 실제 탐색은 5m 칸에서 이뤄진다. 화면에는 다섯 칸마다 선을 그려 격자를 읽기 쉽게 한다.
    constexpr double gridLineSpacingMeters = 25.0;
    const double firstEastLine = std::ceil(-halfEast / gridLineSpacingMeters)
        * gridLineSpacingMeters;
    for (double east = firstEastLine; east <= halfEast; east += gridLineSpacingMeters)
    {
        const ImU32 color = std::abs(east) < 0.01
            ? IM_COL32(145, 160, 165, 150) : IM_COL32(95, 110, 115, 65);
        drawList->AddLine(project({east, -halfNorth}), project({east, halfNorth}),
            color, 1.0F);
    }
    const double firstNorthLine = std::ceil(-halfNorth / gridLineSpacingMeters)
        * gridLineSpacingMeters;
    for (double north = firstNorthLine; north <= halfNorth; north += gridLineSpacingMeters)
    {
        const ImU32 color = std::abs(north) < 0.01
            ? IM_COL32(145, 160, 165, 150) : IM_COL32(95, 110, 115, 65);
        drawList->AddLine(project({-halfEast, north}), project({halfEast, north}),
            color, 1.0F);
    }

    std::vector<std::size_t> drawOrder(buildings.size());
    for (std::size_t index = 0; index < drawOrder.size(); ++index)
        drawOrder[index] = index;
    std::sort(drawOrder.begin(), drawOrder.end(), [&](std::size_t left, std::size_t right)
    {
        return buildings[left].heightMeters < buildings[right].heightMeters;
    });

    for (const std::size_t index : drawOrder)
    {
        const auto& building = buildings[index];
        if (building.outlineMeters.size() < 3U)
            continue;

        std::vector<ImVec2> footprint;
        footprint.reserve(building.outlineMeters.size());
        for (const auto& point : building.outlineMeters)
            footprint.push_back(project(point));
        const ImU32 heightColor = BuildingHeightColor(
            building.heightMeters, maximumHeightMeters);
        drawList->AddConvexPolyFilled(footprint.data(),
            static_cast<int>(footprint.size()), heightColor);
        drawList->AddPolyline(footprint.data(), static_cast<int>(footprint.size()),
            IM_COL32(12, 19, 22, 255), ImDrawFlags_Closed, 1.0F);

        glm::dvec2 center{0.0};
        for (const auto& point : building.outlineMeters)
            center += point;
        center /= static_cast<double>(building.outlineMeters.size());
        const ImVec2 labelPosition = project(center);
        const std::string label = std::to_string(index + 1U);
        const ImVec2 labelSize = ImGui::CalcTextSize(label.c_str());
        drawList->AddText({labelPosition.x - labelSize.x * 0.5F,
            labelPosition.y - labelSize.y * 0.5F},
            IM_COL32(255, 255, 255, 255), label.c_str());

        ImGui::SetCursorScreenPos({labelPosition.x - 14.0F,
            labelPosition.y - 9.0F});
        const std::string itemId = "##구조물" + std::to_string(index + 1U);
        ImGui::InvisibleButton(itemId.c_str(), ImVec2{28.0F, 18.0F});
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("%s\n동 %.1f m · 북 %.1f m\n높이 %.1f m",
                building.name.c_str(), center.x, center.y, building.heightMeters);
        }
    }

    // 경로마다 색을 나눠 겹친 편대 경로도 구별하고, 점 번호로 경유 순서를 보여준다.
    for (const MiniMapRoute& route : routes)
    {
        if (route.waypointsMeters.empty())
            continue;
        const ImU32 color = RouteColor(static_cast<std::size_t>(route.droneId));
        std::vector<glm::dvec3> points;
        points.reserve(route.waypointsMeters.size() + 1U);
        points.push_back(route.startMeters);
        points.insert(points.end(), route.waypointsMeters.begin(),
            route.waypointsMeters.end());

        for (std::size_t index = 1; index < points.size(); ++index)
        {
            const ImVec2 first = project({points[index - 1U].x, points[index - 1U].y});
            const ImVec2 second = project({points[index].x, points[index].y});
            drawList->AddLine(first, second, IM_COL32(8, 12, 14, 230), 5.0F);
            drawList->AddLine(first, second, color, 2.5F);

            const ImVec2 direction{second.x - first.x, second.y - first.y};
            const float length = std::sqrt(direction.x * direction.x
                + direction.y * direction.y);
            if (length > 10.0F)
            {
                const ImVec2 unit{direction.x / length, direction.y / length};
                const ImVec2 side{-unit.y, unit.x};
                const ImVec2 tip{first.x + direction.x * 0.62F,
                    first.y + direction.y * 0.62F};
                const ImVec2 arrow[]{
                    tip,
                    ImVec2{tip.x - unit.x * 7.0F + side.x * 4.0F,
                        tip.y - unit.y * 7.0F + side.y * 4.0F},
                    ImVec2{tip.x - unit.x * 7.0F - side.x * 4.0F,
                        tip.y - unit.y * 7.0F - side.y * 4.0F}};
                drawList->AddTriangleFilled(arrow[0], arrow[1], arrow[2], color);
            }
        }

        for (std::size_t index = 0; index < points.size(); ++index)
        {
            const ImVec2 position = project({points[index].x, points[index].y});
            const bool isStart = index == 0U;
            const bool isGoal = index + 1U == points.size();
            const float radius = isStart || isGoal ? 5.5F : 4.0F;
            const ImU32 markerColor = isStart ? IM_COL32(80, 240, 125, 255)
                : (isGoal ? IM_COL32(255, 105, 90, 255) : color);
            drawList->AddCircleFilled(position, radius, markerColor);
            drawList->AddCircle(position, radius, IM_COL32(8, 12, 14, 255), 0, 1.3F);

            const std::string pointId = "##경로점_"
                + std::to_string(route.droneId) + "_" + std::to_string(index);
            ImGui::SetCursorScreenPos({position.x - 7.0F, position.y - 7.0F});
            ImGui::InvisibleButton(pointId.c_str(), ImVec2{14.0F, 14.0F});
            if (ImGui::IsItemHovered())
            {
                const std::string pointLabel = isStart ? "출발"
                    : (isGoal ? "목표" : "경유점 " + std::to_string(index));
                ImGui::SetTooltip("드론 %llu · %s\n동 %.1f m · 북 %.1f m · 고도 %.1f m",
                    static_cast<unsigned long long>(route.droneId), pointLabel.c_str(),
                    points[index].x, points[index].y, points[index].z);
            }
        }
    }

    // 각 드론은 방향을 가진 삼각형으로 표시하고, 경로선과 같은 색을 사용한다.
    for (const MiniMapDrone& drone : drones)
    {
        const ImVec2 position = project({drone.positionMeters.x, drone.positionMeters.y});
        const double screenHeading = -drone.yawRadians;
        const ImVec2 forward{static_cast<float>(std::cos(screenHeading)),
            static_cast<float>(std::sin(screenHeading))};
        const ImVec2 side{-forward.y, forward.x};
        constexpr float iconSize = 8.0F;
        const ImVec2 icon[]{
            ImVec2{position.x + forward.x * iconSize,
                position.y + forward.y * iconSize},
            ImVec2{position.x - forward.x * iconSize * 0.7F
                    + side.x * iconSize * 0.7F,
                position.y - forward.y * iconSize * 0.7F
                    + side.y * iconSize * 0.7F},
            ImVec2{position.x - forward.x * iconSize * 0.7F
                    - side.x * iconSize * 0.7F,
                position.y - forward.y * iconSize * 0.7F
                    - side.y * iconSize * 0.7F}};
        const ImU32 color = RouteColor(static_cast<std::size_t>(drone.droneId));
        drawList->AddTriangleFilled(icon[0], icon[1], icon[2], color);
        drawList->AddTriangle(icon[0], icon[1], icon[2],
            IM_COL32(8, 12, 14, 255), 1.5F);
        const std::string label = "D" + std::to_string(drone.droneId);
        drawList->AddText({position.x + 8.0F, position.y + 5.0F},
            IM_COL32(255, 255, 255, 255), label.c_str());

        const std::string itemId = "##드론_" + std::to_string(drone.droneId);
        ImGui::SetCursorScreenPos({position.x - 10.0F, position.y - 10.0F});
        ImGui::InvisibleButton(itemId.c_str(), ImVec2{20.0F, 20.0F});
        if (ImGui::IsItemHovered())
        {
            ImGui::SetTooltip("드론 %llu\n동 %.1f m · 북 %.1f m · 고도 %.1f m",
                static_cast<unsigned long long>(drone.droneId),
                drone.positionMeters.x, drone.positionMeters.y,
                drone.positionMeters.z);
        }
    }

    drawList->AddRect(drawingMinimum, drawingMaximum,
        IM_COL32(185, 200, 205, 180));
    drawList->AddText({drawingMaximum.x - 14.0F, drawingMinimum.y + 2.0F},
        IM_COL32(235, 240, 242, 255), "N");
    drawList->AddText({drawingMaximum.x - 14.0F, drawingMaximum.y - 16.0F},
        IM_COL32(235, 240, 242, 255), "E");
    const std::string heightLegend = "건물 높이 0–"
        + std::to_string(static_cast<int>(std::ceil(maximumHeightMeters)))
        + " m (청록→주황) · 드론=삼각형 · 경로=선";
    drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 0.72F,
        {mapMinimum.x + 10.0F, mapMaximum.y - 18.0F},
        IM_COL32(205, 215, 220, 255), heightLegend.c_str());
    drawList->PopClipRect();
}

void DrawMiniMapPanel(
    std::span<const flight_planning::BuildingObstacle> buildings,
    std::span<const MiniMapDrone> drones,
    std::span<const MiniMapRoute> routes,
    const glm::dvec2& groundSizeMeters,
    double& zoom,
    glm::dvec2& centerMeters)
{
    if (ImGui::Begin("지도##MapViewV5"))
    {
        ImGui::TextDisabled("휠 확대 · 가운데 드래그 이동 · 창 밖으로 끌어 분리");
        ImGui::SameLine();
        if (ImGui::SmallButton("전체 보기"))
        {
            zoom = 1.0;
            centerMeters = glm::dvec2{0.0};
        }
        const ImVec2 minimum = ImGui::GetCursorScreenPos();
        const ImVec2 available = ImGui::GetContentRegionAvail();
        const ImVec2 maximum{minimum.x + available.x, minimum.y + available.y};
        DrawBuildingMiniMap(buildings, drones, routes, groundSizeMeters,
            zoom, centerMeters,
            ImGui::GetWindowDrawList(), minimum, maximum);
    }
    ImGui::End();
}


}

void MiniMapPanel::Render(
    std::span<const flight_planning::BuildingObstacle> buildings,
    std::span<const MiniMapDrone> drones,
    std::span<const MiniMapRoute> routes,
    const glm::dvec2& groundSizeMeters)
{
    DrawMiniMapPanel(buildings, drones, routes, groundSizeMeters,
        zoom_, centerMeters_);
}
}
