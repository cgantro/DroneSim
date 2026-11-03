#include "gui/OperationsPanel.h"

#include "gui/GuiText.h"

#include <imgui.h>

#include <algorithm>

namespace dronesim::gui
{
void OperationsPanel::Render(const OperationsPanelFrame& frame)
{
    if (ImGui::Begin("운용##OperationsV5"))
    {
        const auto primary = std::find_if(frame.drones.begin(), frame.drones.end(),
            [&frame](const DroneStatusView& drone)
            {
                return drone.droneId == frame.primaryDroneId;
            });
        if (primary != frame.drones.end())
        {
            ImGui::Text("주 드론: %s", DroneLabel(primary->droneId).c_str());
            ImGui::Text("비행 상태: %s", simulation::ToString(primary->mode).data());
            ImGui::Text("위치 [m]: X %.2f   Y %.2f   높이 %.2f",
                primary->positionMeters.x, primary->positionMeters.y,
                primary->positionMeters.z);
        }
        ImGui::Text("모의 시간: %.2f초   진행 Tick: %llu",
            frame.simulationTimeSeconds,
            static_cast<unsigned long long>(frame.tickCount));

        ImGui::Separator();
        ImGui::Text("임무: %s", frame.completeFleetMissionStatus.c_str());
        if (frame.completeFleetMissionCanStart
            && ImGui::Button("편대 정찰 임무 시작"))
            frame.startCompleteFleetMission();
        else if (!frame.completeFleetMissionCanStart)
            ImGui::TextDisabled("모든 드론이 지상에 있고 다른 임무가 없어야 합니다.");
        if (frame.fleetReturnCanStart && ImGui::Button("출발 지점으로 복귀"))
            frame.startFleetReturn();

        if (frame.fleetNetworkStatus != nullptr)
            networkFaultPanel_.Render(*frame.fleetNetworkStatus,
                frame.setRadioEnabled);

        ImGui::Separator();
        ImGui::TextDisabled("조작: A 이륙 · 왼쪽 스틱 이동 · RT/LT 높이 · B 착륙");
        ImGui::Separator();
        ImGui::Text("게임패드: %s",
            frame.gamepadConnected ? "연결됨" : "연결 안 됨");
        ImGui::Separator();
        ImGui::Text("편대 상태 (%zu대)", frame.drones.size());
        for (const DroneStatusView& drone : frame.drones)
        {
            ImGui::Text("%s  %-16s  (%.1f, %.1f, %.1f) m",
                DroneLabel(drone.droneId).c_str(), simulation::ToString(drone.mode).data(),
                drone.positionMeters.x, drone.positionMeters.y,
                drone.positionMeters.z);
        }
    }
    ImGui::End();
}
}
