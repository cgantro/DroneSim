#include "gui/NetworkFaultPanel.h"
#include "gui/GuiText.h"

#include <imgui.h>

#include <algorithm>
#include <string>
#include <vector>

namespace dronesim::gui
{
namespace
{
const char* EventLabel(FleetNetworkStatus::EventKind kind)
{
    switch (kind)
    {
    case FleetNetworkStatus::EventKind::RadioDisabled: return "드론 라디오 차단";
    case FleetNetworkStatus::EventKind::RadioRestored: return "드론 라디오 복구";
    case FleetNetworkStatus::EventKind::MessageDelivered: return "메시지 전달";
    case FleetNetworkStatus::EventKind::PacketDropped: return "패킷 폐기";
    }
    return "통신 사건";
}
}

void NetworkFaultPanel::Render(const FleetNetworkStatus& status,
    const SetRadioEnabled& setRadioEnabled)
{
    ImGui::Separator();
    ImGui::Text("드론 간 UDP 통신: 직접 연결");

    if (!status.drones.empty())
    {
        // 드론 수가 바뀌어도 선택 인덱스가 범위를 벗어나지 않게 맞춘다.
        selectedDroneIndex_ = std::clamp(selectedDroneIndex_, 0,
            static_cast<int>(status.drones.size() - 1U));
        std::vector<std::string> names;
        std::vector<const char*> namePointers;
        names.reserve(status.drones.size());
        namePointers.reserve(status.drones.size());
        for (const auto& drone : status.drones)
            names.push_back(DroneLabel(drone.id));
        for (const std::string& name : names)
            namePointers.push_back(name.c_str());

        ImGui::SetNextItemWidth(-1.0F);
        ImGui::Combo("시험할 드론", &selectedDroneIndex_,
            namePointers.data(), static_cast<int>(namePointers.size()));

        const auto& drone = status.drones[
            static_cast<std::size_t>(selectedDroneIndex_)];
        ImGui::Text("소켓 포트: %u · 편대 연결: %s",
            drone.socketPort ? static_cast<unsigned int>(*drone.socketPort) : 0U,
            drone.isLeader ? "리더" : (drone.radioEnabled && drone.linkedToLeader
                ? "연결됨" : "끊김"));
        if (drone.radioEnabled)
        {
            if (ImGui::Button("이 드론 통신 차단"))
                setRadioEnabled(drone.id, false);
        }
        else if (ImGui::Button("이 드론 통신 복구"))
        {
            setRadioEnabled(drone.id, true);
        }
    }

    ImGui::Text("전달 %llu · 폐기 %llu",
        static_cast<unsigned long long>(status.deliveredMessages),
        static_cast<unsigned long long>(status.droppedMessages));
    const std::size_t visibleEvents = std::min<std::size_t>(
        status.recentEvents.size(), 4U);
    for (std::size_t offset = 0; offset < visibleEvents; ++offset)
    {
        const auto& event = status.recentEvents[
            status.recentEvents.size() - 1U - offset];
        const std::string sourceName = DroneLabel(event.source);
        const std::string destinationName = DroneLabel(event.destination);
        ImGui::Text("%.2f초  %s  %s → %s",
            static_cast<double>(event.tick) / 60.0,
            EventLabel(event.kind), sourceName.c_str(), destinationName.c_str());
    }
}
}
