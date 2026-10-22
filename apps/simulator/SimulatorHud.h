#pragma once

#include "communication/Protocol.h"
#include "simulation_core/SimulationCore.h"

#include <vector>
#include <cstdint>
#include <functional>
#include <string>

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
        bool gamepadConnected, bool groundControlConnected, bool cameraActive,
        int cameraImageWidth, int cameraImageHeight,
        const std::function<std::uint32_t(int, int)>& renderScene,
        const std::function<std::uint32_t()>& renderCamera);

private:
    bool initialized_ = false;
    std::string layoutFile_;
};
}
