#pragma once

#include "simulation_core/SimulationCore.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace dronesim::gui
{
/** @brief 통신 장애 패널이 표시하는 드론·메시지 상태다. */
struct FleetNetworkStatus
{
    enum class EventKind
    {
        RadioDisabled,
        RadioRestored,
        MessageDelivered,
        PacketDropped
    };

    struct Drone
    {
        simulation::DroneId id = 0;
        std::optional<std::uint16_t> socketPort;
        bool isLeader = false;
        bool radioEnabled = true;
        bool linkedToLeader = false;
    };

    struct RecentEvent
    {
        std::uint64_t tick = 0;
        EventKind kind = EventKind::PacketDropped;
        simulation::DroneId source = 0;
        simulation::DroneId destination = 0;
    };

    std::uint64_t deliveredMessages = 0;
    std::uint64_t droppedMessages = 0;
    std::vector<Drone> drones;
    std::vector<RecentEvent> recentEvents;
};

/** @brief 드론의 직접 통신을 끊고 복구하는 시험 조작을 표시한다. */
class NetworkFaultPanel
{
public:
    using SetRadioEnabled = std::function<void(simulation::DroneId, bool)>;

    void Render(const FleetNetworkStatus& status,
        const SetRadioEnabled& setRadioEnabled);

private:
    int selectedDroneIndex_ = 0;
};
}
