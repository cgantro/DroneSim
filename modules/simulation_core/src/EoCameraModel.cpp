#include "simulation_core/EoCameraModel.h"
#include "simulation_core/DroneModel.h"

#include <cmath>

namespace aerosimlab::simulation
{
namespace
{
// PX4 짐벌의 중립 관절과 센서 원점을 합친 기체 기준 위치다.
constexpr glm::dvec3 cameraMountPositionBodyMeters{0.10776, 0.0, -0.2259};
constexpr glm::dvec3 cameraOpticalAxisBody{1.0, 0.0, 0.0};
}

bool CameraModel::IsValid() const noexcept
{
    return imageWidthPixels > 0
        && imageWidthPixels <= maximumImageDimensionPixels
        && imageHeightPixels > 0
        && imageHeightPixels <= maximumImageDimensionPixels
        && std::isfinite(verticalFieldOfViewDegrees)
        && verticalFieldOfViewDegrees > 0.0
        && verticalFieldOfViewDegrees < 180.0;
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

glm::dvec3 EoCameraModel::OpticalAxis(const CameraState& camera) noexcept
{
    return glm::normalize(camera.orientation * glm::normalize(cameraOpticalAxisBody));
}

glm::dvec3 EoCameraModel::UpAxis(const CameraState& camera) noexcept
{
    return glm::normalize(camera.orientation * glm::dvec3{0.0, 0.0, 1.0});
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
    const CameraModel model = state.model;
    state = {};
    state.model = model;
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
