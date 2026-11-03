#pragma once

#include "simulation_core/SimulationCore.h"

namespace dronesim::gui
{
/** @brief 한 화면 갱신에서 여러 패널이 함께 읽는 드론 상태다. */
struct DroneStatusView
{
    simulation::DroneId droneId = 0;
    simulation::FlightMode mode = simulation::FlightMode::Grounded;
    glm::dvec3 positionMeters{0.0};
    double yawRadians = 0.0;
};
}
