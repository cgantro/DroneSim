#include "simulation_core/EoCameraModel.h"
#include "simulation_core/DroneModel.h"

#include <cmath>

#include <glm/gtc/constants.hpp>

namespace dronesim::simulation
{
namespace
{
// PX4 짐벌의 중립 관절과 센서 원점을 합친 기체 기준 위치다.
constexpr glm::dvec3 cameraMountPositionBodyMeters{0.10776, 0.0, -0.2259};
constexpr glm::dvec3 cameraOpticalAxisBody{1.0, 0.0, 0.0};
}

bool CameraModel::IsValid() const noexcept
{
    // 화면 크기와 시야각은 카메라 투영 행렬을 만들 수 있는 범위인지 확인한다.
    return imageWidthPixels > 0
        && imageWidthPixels <= maximumImageDimensionPixels
        && imageHeightPixels > 0
        && imageHeightPixels <= maximumImageDimensionPixels
        && std::isfinite(verticalFieldOfViewDegrees)
        && verticalFieldOfViewDegrees > 0.0
        && verticalFieldOfViewDegrees < 180.0;
}

bool CameraGimbalAngles::IsValid() const noexcept
{
    // 실제 짐벌의 회전 한계는 모델별로 달라질 수 있어 여기서는 비정상 수만 거른다.
    return std::isfinite(yawDegrees)
        && std::isfinite(pitchUpDegrees)
        && std::isfinite(rollDegrees);
}

void EoCameraModel::AdvanceOneTick(CameraState& state, std::uint64_t tick) noexcept
{
    if (state.status != CameraStatus::Active)
        return;

    ++state.frameCount;
    // 프레임은 실제 렌더링이 아니라 모의 카메라의 갱신 주기를 기록한다.
    state.lastFrameTick = tick;
}

void EoCameraModel::UpdatePose(CameraState& camera, const DroneState& drone) noexcept
{
    // 렌즈 위치는 기체에 고정하고, 방향에는 Z-Y-X 순서의 짐벌 각도를 적용한다.
    camera.positionMeters = drone.positionMeters
        + drone.orientation * cameraMountPositionBodyMeters;
    camera.orientation = glm::normalize(
        drone.orientation * GimbalOrientation(camera.gimbalAngles));
}

glm::dquat EoCameraModel::GimbalOrientation(
    const CameraGimbalAngles& angles) noexcept
{
    constexpr double degreesToRadians = glm::pi<double>() / 180.0;
    const glm::dquat yaw = glm::angleAxis(
        -angles.yawDegrees * degreesToRadians, glm::dvec3{0.0, 0.0, 1.0});
    const glm::dquat roll = glm::angleAxis(
        -angles.rollDegrees * degreesToRadians, glm::dvec3{1.0, 0.0, 0.0});
    // PX4 짐벌 축 부호를 따르며, 양수 피치는 화면에서 위를 보도록 뒤집는다.
    const glm::dquat pitch = glm::angleAxis(
        -angles.pitchUpDegrees * degreesToRadians, glm::dvec3{0.0, 1.0, 0.0});
    return glm::normalize(yaw * roll * pitch);
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
