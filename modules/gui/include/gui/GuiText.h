#pragma once

#include "simulation_core/SimulationCore.h"

#include <string>

namespace dronesim::gui
{
/** @brief 화면에 표시할 드론 이름을 식별값으로 만든다. */
[[nodiscard]] inline std::string DroneLabel(simulation::DroneId id)
{
    return "드론 " + std::to_string(id);
}
}
