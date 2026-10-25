#include "simulation_core/EoCameraModel.h"
#include "simulation_core/DroneModel.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace aerosimlab::simulation
{
namespace
{
constexpr glm::dvec3 cameraMountPositionBodyMeters{0.35, 0.0, 0.12};
constexpr glm::dvec3 cameraOpticalAxisBody{25.0, 0.0, -2.5};
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

CaptureAssessment EoCameraModel::AssessCapture(
    const CameraState& camera, const InspectionSurface& surface) noexcept
{
    if (camera.status != CameraStatus::Active)
        return CaptureAssessment::CameraUnavailable;

    const glm::dvec3 toSurface = surface.centerMeters - camera.positionMeters;
    const double distanceMeters = glm::length(toSurface);
    if (distanceMeters < surface.minimumDistanceMeters)
        return CaptureAssessment::TooNear;
    if (distanceMeters > surface.maximumDistanceMeters)
        return CaptureAssessment::TooFar;

    const glm::dvec3 towardCamera = -toSurface / distanceMeters;
    const double maximumIncidenceRadians = glm::radians(
        surface.maximumIncidenceAngleDegrees);
    if (glm::dot(surface.normal, towardCamera)
        < std::cos(maximumIncidenceRadians))
    {
        return CaptureAssessment::SurfaceFacingAway;
    }

    const glm::dvec3 forward = OpticalAxis(camera);
    const glm::dvec3 cameraUp = UpAxis(camera);
    const glm::dvec3 right = glm::normalize(glm::cross(forward, cameraUp));
    const glm::dvec3 up = glm::normalize(glm::cross(right, forward));
    const double verticalFovRadians = glm::radians(camera.verticalFieldOfViewDegrees);
    const double tangentHalfVerticalFov = std::tan(verticalFovRadians * 0.5);
    const double tangentHalfHorizontalFov = tangentHalfVerticalFov
        * static_cast<double>(camera.imageWidthPixels)
        / static_cast<double>(camera.imageHeightPixels);
    const glm::dvec3 heightAxis = glm::normalize(
        glm::cross(surface.normal, surface.widthAxis));

    const std::array<glm::dvec3, 4> corners{
        surface.centerMeters - surface.widthAxis * (surface.widthMeters * 0.5)
            - heightAxis * (surface.heightMeters * 0.5),
        surface.centerMeters + surface.widthAxis * (surface.widthMeters * 0.5)
            - heightAxis * (surface.heightMeters * 0.5),
        surface.centerMeters - surface.widthAxis * (surface.widthMeters * 0.5)
            + heightAxis * (surface.heightMeters * 0.5),
        surface.centerMeters + surface.widthAxis * (surface.widthMeters * 0.5)
            + heightAxis * (surface.heightMeters * 0.5)};

    double leftmostPixel = static_cast<double>(camera.imageWidthPixels);
    double rightmostPixel = 0.0;
    double topmostPixel = static_cast<double>(camera.imageHeightPixels);
    double bottommostPixel = 0.0;
    for (const glm::dvec3& corner : corners)
    {
        const glm::dvec3 relativePosition = corner - camera.positionMeters;
        const double depthMeters = glm::dot(relativePosition, forward);
        if (depthMeters <= 0.0)
            return CaptureAssessment::OutsideFrame;

        const double horizontalPosition = glm::dot(relativePosition, right);
        const double verticalPosition = glm::dot(relativePosition, up);
        const double normalizedX = horizontalPosition
            / (depthMeters * tangentHalfHorizontalFov);
        const double normalizedY = verticalPosition
            / (depthMeters * tangentHalfVerticalFov);
        if (std::abs(normalizedX) > 1.0 || std::abs(normalizedY) > 1.0)
            return CaptureAssessment::OutsideFrame;

        const double pixelX = (normalizedX * 0.5 + 0.5)
            * static_cast<double>(camera.imageWidthPixels);
        const double pixelY = (0.5 - normalizedY * 0.5)
            * static_cast<double>(camera.imageHeightPixels);
        leftmostPixel = std::min(leftmostPixel, pixelX);
        rightmostPixel = std::max(rightmostPixel, pixelX);
        topmostPixel = std::min(topmostPixel, pixelY);
        bottommostPixel = std::max(bottommostPixel, pixelY);
    }

    if (rightmostPixel - leftmostPixel < surface.minimumPixelWidth
        || bottommostPixel - topmostPixel < surface.minimumPixelHeight)
    {
        return CaptureAssessment::ResolutionTooLow;
    }
    return CaptureAssessment::Ready;
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

std::string_view ToString(CaptureAssessment assessment) noexcept
{
    switch (assessment)
    {
    case CaptureAssessment::Ready: return "ready";
    case CaptureAssessment::CameraUnavailable: return "camera_unavailable";
    case CaptureAssessment::NoNewFrame: return "no_new_frame";
    case CaptureAssessment::TooNear: return "too_near";
    case CaptureAssessment::TooFar: return "too_far";
    case CaptureAssessment::SurfaceFacingAway: return "surface_facing_away";
    case CaptureAssessment::OutsideFrame: return "outside_frame";
    case CaptureAssessment::ResolutionTooLow: return "resolution_too_low";
    }
    return "unknown";
}
}
