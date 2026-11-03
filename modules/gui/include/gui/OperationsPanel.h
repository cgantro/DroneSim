#pragma once

#include "gui/NetworkFaultPanel.h"

#include <functional>
#include <string>

namespace dronesim::gui
{
/** @brief 운용 패널에 표시할 상태와 사용자가 실행할 명령이다. */
struct OperationsPanelFrame
{
    const simulation::SimulationCore& simulation;
    bool gamepadConnected = false;
    bool fleetReturnCanStart = false;
    std::function<void()> startFleetReturn;
    std::string completeFleetMissionStatus;
    bool completeFleetMissionCanStart = false;
    std::function<void()> startCompleteFleetMission;
    const FleetNetworkStatus* fleetNetworkStatus = nullptr;
    NetworkFaultPanel::SetRadioEnabled setRadioEnabled;
};

/** @brief 비행 상태, 편대 목록, 임무 명령을 운용 패널에 표시한다. */
class OperationsPanel
{
public:
    void Render(const OperationsPanelFrame& frame);

private:
    NetworkFaultPanel networkFaultPanel_;
};
}
