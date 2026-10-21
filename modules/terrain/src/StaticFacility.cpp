#include "terrain/StaticFacility.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace aerosimlab::terrain
{
namespace
{
template <typename Vector>
Vector ReadVector(const nlohmann::json& value, const char* label)
{
    if (!value.is_array() || value.size() != 3U)
        throw std::runtime_error(std::string("시설물 ") + label + "은 숫자 세 개여야 합니다.");

    Vector result;
    for (std::size_t axis = 0; axis < 3U; ++axis)
    {
        if (!value.at(axis).is_number())
            throw std::runtime_error(std::string("시설물 ") + label + "에 숫자가 아닌 값이 있습니다.");
        result[axis] = value.at(axis).get<typename Vector::value_type>();
        if (!std::isfinite(result[axis]))
            throw std::runtime_error(std::string("시설물 ") + label + "에 유한하지 않은 값이 있습니다.");
    }
    return result;
}
}

std::vector<StaticFacility> LoadStaticFacilities(const std::filesystem::path& path)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("시설물 자료를 열지 못했습니다: " + path.string());

    nlohmann::json document;
    try
    {
        input >> document;
    }
    catch (const nlohmann::json::exception& error)
    {
        throw std::runtime_error(std::string("시설물 자료를 읽지 못했습니다: ") + error.what());
    }

    if (!document.contains("objects") || !document.at("objects").is_array())
        throw std::runtime_error("시설물 자료에 객체 목록이 없습니다.");

    std::vector<StaticFacility> facilities;
    facilities.reserve(document.at("objects").size());
    for (const nlohmann::json& object : document.at("objects"))
    {
        StaticFacility facility;
        facility.id = object.value("id", std::string{});
        facility.type = object.value("type", std::string{});
        if (facility.id.empty() || facility.type.empty())
            throw std::runtime_error("시설물의 ID나 종류가 비어 있습니다.");

        facility.centerMeters = ReadVector<glm::dvec3>(object.at("center_m"), "중심 위치");
        facility.halfExtentsMeters = ReadVector<glm::vec3>(
            object.at("half_extents_m"), "충돌 상자 크기");
        if (!(facility.halfExtentsMeters.x > 0.0F)
            || !(facility.halfExtentsMeters.y > 0.0F)
            || !(facility.halfExtentsMeters.z > 0.0F))
        {
            throw std::runtime_error("시설물 충돌 상자의 반쪽 크기는 모두 0보다 커야 합니다.");
        }
        facilities.push_back(std::move(facility));
    }

    return facilities;
}
}
