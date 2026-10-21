#include "simulation_core/EoCameraModel.h"
#include "simulation_core/DroneModel.h"

namespace aerosimlab::simulation
{
namespace
{
constexpr glm::dvec3 cameraMountPositionBodyMeters{0.35, 0.0, 0.12};
}

void EoCameraModel::AdvanceOneTick(CameraState& state, std::uint64_t tick) noexcept
{
    if (state.status != CameraStatus::Active)
        return;

    ++state.frameCount;
    state.lastFrameTick = tick;
}

void EoCameraModel::UpdatePose(CameraState& camera, const DroneState& drone) noexcept
{
    camera.positionMeters = drone.positionMeters
        + drone.orientation * cameraMountPositionBodyMeters;
    camera.orientation = drone.orientation;
}

void EoCameraModel::SetEnabled(CameraState& state, bool enabled) noexcept
{
    if (state.status == CameraStatus::Fault)
        return;

    state.status = enabled ? CameraStatus::Active : CameraStatus::Disabled;
}

void EoCameraModel::InjectFault(CameraState& state) noexcept
{
    state.status = CameraStatus::Fault;
}

void EoCameraModel::Recover(CameraState& state) noexcept
{
    state.status = CameraStatus::Active;
}

void EoCameraModel::Reset(CameraState& state) noexcept
{
    state = {};
}

std::string_view ToString(CameraStatus status) noexcept
{
    switch (status)
    {
    case CameraStatus::Active: return "active";
    case CameraStatus::Disabled: return "disabled";
    case CameraStatus::Fault: return "fault";
    }
    return "unknown";
}
}
