#include "simulation_core/InspectionSurfaceJson.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace aerosimlab::simulation
{
namespace
{
using Json = nlohmann::json;

const Json& Required(const Json& object, const char* key)
{
    const auto field = object.find(key);
    if (field == object.end())
        throw std::invalid_argument(std::string("필수 항목이 없습니다: ") + key);
    return *field;
}

double ReadFiniteNumber(const Json& value, const char* name)
{
    if (!value.is_number())
        throw std::invalid_argument(std::string("숫자가 필요합니다: ") + name);
    const double number = value.get<double>();
    if (!std::isfinite(number))
        throw std::invalid_argument(std::string("유효한 숫자가 필요합니다: ") + name);
    return number;
}

glm::dvec3 ReadVector(const Json& value, const char* name)
{
    if (!value.is_array() || value.size() != 3)
        throw std::invalid_argument(std::string("숫자 세 개가 필요합니다: ") + name);
    return {
        ReadFiniteNumber(value[0], name),
        ReadFiniteNumber(value[1], name),
        ReadFiniteNumber(value[2], name)};
}

std::uint64_t ReadUnsigned(const Json& value, const char* name)
{
    if ((!value.is_number_unsigned() && !value.is_number_integer())
        || (value.is_number_integer() && value.get<std::int64_t>() < 0))
        throw std::invalid_argument(std::string("0 이상의 정수가 필요합니다: ") + name);
    return value.get<std::uint64_t>();
}
}

InspectionSurface ParseInspectionSurfaceJson(std::string_view jsonText)
{
    const Json value = Json::parse(jsonText);
    if (!value.is_object())
        throw std::invalid_argument("inspection_surface는 JSON 객체여야 합니다.");

    InspectionSurface surface;
    surface.centerMeters = ReadVector(
        Required(value, "center_meters"), "inspection_surface.center_meters");
    surface.normal = ReadVector(
        Required(value, "normal"), "inspection_surface.normal");
    surface.widthAxis = ReadVector(
        Required(value, "width_axis"), "inspection_surface.width_axis");

    const double normalLength = glm::length(surface.normal);
    if (normalLength < 1.0e-9)
        throw std::invalid_argument("점검면 normal은 길이가 0일 수 없습니다.");
    surface.normal /= normalLength;
    surface.widthAxis -= surface.normal * glm::dot(surface.normal, surface.widthAxis);
    const double widthAxisLength = glm::length(surface.widthAxis);
    if (widthAxisLength < 1.0e-9)
        throw std::invalid_argument("점검면 width_axis는 normal과 평행할 수 없습니다.");
    surface.widthAxis /= widthAxisLength;

    surface.widthMeters = ReadFiniteNumber(
        Required(value, "width_meters"), "inspection_surface.width_meters");
    surface.heightMeters = ReadFiniteNumber(
        Required(value, "height_meters"), "inspection_surface.height_meters");
    surface.minimumDistanceMeters = ReadFiniteNumber(
        Required(value, "minimum_distance_meters"),
        "inspection_surface.minimum_distance_meters");
    surface.maximumDistanceMeters = ReadFiniteNumber(
        Required(value, "maximum_distance_meters"),
        "inspection_surface.maximum_distance_meters");
    surface.maximumIncidenceAngleDegrees = ReadFiniteNumber(
        Required(value, "maximum_incidence_angle_degrees"),
        "inspection_surface.maximum_incidence_angle_degrees");

    const std::uint64_t minimumPixelWidth = ReadUnsigned(
        Required(value, "minimum_pixel_width"), "inspection_surface.minimum_pixel_width");
    const std::uint64_t minimumPixelHeight = ReadUnsigned(
        Required(value, "minimum_pixel_height"), "inspection_surface.minimum_pixel_height");
    if (minimumPixelWidth == 0 || minimumPixelHeight == 0
        || minimumPixelWidth > static_cast<std::uint64_t>(std::numeric_limits<int>::max())
        || minimumPixelHeight > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("점검면 최소 영상 크기는 양의 정수여야 합니다.");
    surface.minimumPixelWidth = static_cast<int>(minimumPixelWidth);
    surface.minimumPixelHeight = static_cast<int>(minimumPixelHeight);

    if (surface.widthMeters <= 0.0 || surface.heightMeters <= 0.0
        || surface.minimumDistanceMeters <= 0.0
        || surface.maximumDistanceMeters <= surface.minimumDistanceMeters
        || surface.maximumIncidenceAngleDegrees < 0.0
        || surface.maximumIncidenceAngleDegrees >= 90.0)
        throw std::invalid_argument("점검면 크기·거리·각도 조건이 잘못되었습니다.");
    return surface;
}
}
