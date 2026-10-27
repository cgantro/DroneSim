#include "simulation_core/EoCameraModel.h"
#include "simulation_core/DroneModel.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace aerosimlab::simulation
{
namespace
{
// 외장 카메라 렌즈를 기체 앞쪽 아래에 두고 비행 자세와 함께 움직이게 한다.
constexpr glm::dvec3 cameraMountPositionBodyMeters{0.12, 0.0, -0.10};
constexpr glm::dvec3 cameraOpticalAxisBody{25.0, 0.0, -2.5};

bool SegmentIntersectsBox(
    const glm::dvec3& start,
    const glm::dvec3& end,
    const InspectionOccluderBox& box) noexcept
{
    const glm::dvec3 direction = end - start;
    double entry = 0.0;
    double exit = 1.0;
    for (int axis = 0; axis < 3; ++axis)
    {
        const double minimum = box.centerMeters[axis] - box.halfExtentsMeters[axis];
        const double maximum = box.centerMeters[axis] + box.halfExtentsMeters[axis];
        if (std::abs(direction[axis]) < 1.0e-12)
        {
            if (start[axis] < minimum || start[axis] > maximum)
                return false;
            continue;
        }

        double nearDistance = (minimum - start[axis]) / direction[axis];
        double farDistance = (maximum - start[axis]) / direction[axis];
        if (nearDistance > farDistance)
            std::swap(nearDistance, farDistance);
        entry = std::max(entry, nearDistance);
        exit = std::min(exit, farDistance);
        if (entry > exit)
            return false;
    }
    return exit > 0.0 && entry < 1.0 - 1.0e-9;
}

double Cross(const glm::dvec2& first, const glm::dvec2& second) noexcept
{
    return first.x * second.y - first.y * second.x;
}

bool IsInsideFootprint(
    const glm::dvec2& point,
    std::span<const glm::dvec2> footprint) noexcept
{
    bool inside = false;
    for (std::size_t index = 0; index < footprint.size(); ++index)
    {
        const glm::dvec2& first = footprint[index];
        const glm::dvec2& second = footprint[(index + 1U) % footprint.size()];
        const glm::dvec2 edge = second - first;
        const glm::dvec2 offset = point - first;
        const double edgeLengthSquared = glm::dot(edge, edge);
        if (edgeLengthSquared > 1.0e-12
            && std::abs(Cross(edge, offset)) <= 1.0e-9
            && glm::dot(offset, edge) >= -1.0e-9
            && glm::dot(offset, edge) <= edgeLengthSquared + 1.0e-9)
            return true;

        const bool crossesHeight = (first.y > point.y) != (second.y > point.y);
        if (crossesHeight)
        {
            const double crossingX = first.x
                + (point.y - first.y) * (second.x - first.x)
                    / (second.y - first.y);
            if (point.x < crossingX)
                inside = !inside;
        }
    }
    return inside;
}

bool SegmentIntersectsPolygonPrism(
    const glm::dvec3& start,
    const glm::dvec3& end,
    const InspectionOccluderPolygon& occluder) noexcept
{
    const std::span<const glm::dvec2> footprint = occluder.footprintMeters;
    if (footprint.size() < 3U
        || !(occluder.topElevationMeters > occluder.bottomElevationMeters))
        return false;

    // 먼저 높이 범위에 들어오는 선분 구간만 남긴다.
    const double verticalChange = end.z - start.z;
    double firstAmount = 0.0;
    double lastAmount = 1.0;
    if (std::abs(verticalChange) > 1.0e-12)
    {
        double bottomAmount = (occluder.bottomElevationMeters - start.z)
            / verticalChange;
        double topAmount = (occluder.topElevationMeters - start.z)
            / verticalChange;
        if (bottomAmount > topAmount)
            std::swap(bottomAmount, topAmount);
        firstAmount = std::max(firstAmount, bottomAmount);
        lastAmount = std::min(lastAmount, topAmount);
    }
    else if (start.z < occluder.bottomElevationMeters
        || start.z > occluder.topElevationMeters)
    {
        return false;
    }
    // 점검면에 닿는 끝점 자체는 건물 내부 통과로 세지 않는다.
    lastAmount = std::min(lastAmount, 1.0 - 1.0e-9);
    if (firstAmount > lastAmount)
        return false;

    const glm::dvec3 direction = end - start;
    const glm::dvec3 middle = start + direction * ((firstAmount + lastAmount) * 0.5);
    if (IsInsideFootprint({middle.x, middle.y}, footprint))
        return true;

    const glm::dvec2 startHorizontal{start.x, start.y};
    const glm::dvec2 horizontalDirection{direction.x, direction.y};
    const double horizontalLengthSquared = glm::dot(
        horizontalDirection, horizontalDirection);
    if (horizontalLengthSquared <= 1.0e-12)
        return false;

    // 발자국의 각 변과 시선이 만나는지 직접 확인해 임시 메모리 할당을 피한다.
    for (std::size_t index = 0; index < footprint.size(); ++index)
    {
        const glm::dvec2& edgeStart = footprint[index];
        const glm::dvec2 edge = footprint[(index + 1U) % footprint.size()] - edgeStart;
        const glm::dvec2 offset = edgeStart - startHorizontal;
        const double denominator = Cross(horizontalDirection, edge);
        if (std::abs(denominator) > 1.0e-12)
        {
            const double pathAmount = Cross(offset, edge) / denominator;
            const double edgeAmount = Cross(offset, horizontalDirection) / denominator;
            if (pathAmount >= firstAmount && pathAmount <= lastAmount
                && edgeAmount >= 0.0 && edgeAmount <= 1.0)
                return true;
            continue;
        }

        if (std::abs(Cross(offset, horizontalDirection)) > 1.0e-9)
            continue;
        const double edgeStartAmount = glm::dot(offset, horizontalDirection)
            / horizontalLengthSquared;
        const double edgeEndAmount = glm::dot(edgeStart + edge - startHorizontal,
            horizontalDirection) / horizontalLengthSquared;
        const double overlapStart = std::max(firstAmount,
            std::min(edgeStartAmount, edgeEndAmount));
        const double overlapEnd = std::min(lastAmount,
            std::max(edgeStartAmount, edgeEndAmount));
        if (overlapStart <= overlapEnd)
            return true;
    }
    return false;
}
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

CaptureAssessment EoCameraModel::AssessCapture(
    const CameraState& camera, const InspectionSurface& surface,
    std::span<const InspectionOccluderBox> occluderBoxes,
    std::span<const InspectionOccluderPolygon> occluderPolygons) noexcept
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
    const double verticalFovRadians = glm::radians(
        camera.model.verticalFieldOfViewDegrees);
    const double tangentHalfVerticalFov = std::tan(verticalFovRadians * 0.5);
    const double tangentHalfHorizontalFov = tangentHalfVerticalFov
        * static_cast<double>(camera.model.imageWidthPixels)
        / static_cast<double>(camera.model.imageHeightPixels);
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
    std::array<glm::dvec3, 5> visibilitySamples{
        corners[0], corners[1], corners[2], corners[3], surface.centerMeters};

    double leftmostPixel = static_cast<double>(camera.model.imageWidthPixels);
    double rightmostPixel = 0.0;
    double topmostPixel = static_cast<double>(camera.model.imageHeightPixels);
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
            * static_cast<double>(camera.model.imageWidthPixels);
        const double pixelY = (0.5 - normalizedY * 0.5)
            * static_cast<double>(camera.model.imageHeightPixels);
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

    // 점검면의 네 모서리와 가운데를 잇는 시선이 상자에 닿으면 가림으로 처리한다.
    for (const glm::dvec3& sample : visibilitySamples)
    {
        for (const InspectionOccluderBox& box : occluderBoxes)
        {
            if (SegmentIntersectsBox(camera.positionMeters, sample, box))
                return CaptureAssessment::Occluded;
        }
        for (const InspectionOccluderPolygon& polygon : occluderPolygons)
        {
            if (SegmentIntersectsPolygonPrism(
                camera.positionMeters, sample, polygon))
                return CaptureAssessment::Occluded;
        }
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
    case CaptureAssessment::Occluded: return "occluded";
    }
    return "unknown";
}
}
