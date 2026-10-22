#pragma once

#include "communication/Protocol.h"
#include "simulation_core/SimulationCore.h"

#include <vector>

struct GLFWwindow;

namespace aerosimlab::simulator
{
/** @brief 비행 상태와 편대 요약을 3D 화면 위에 표시한다. */
class SimulatorHud
{
public:
    explicit SimulatorHud(GLFWwindow* window);
    ~SimulatorHud();

    SimulatorHud(const SimulatorHud&) = delete;
    SimulatorHud& operator=(const SimulatorHud&) = delete;

    void Render(const simulation::SimulationCore& simulation,
        bool gamepadConnected, bool groundControlConnected);

private:
    bool initialized_ = false;
};
}
