#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <glm/vec3.hpp>

namespace aerosimlab::terrain
{
/** @brief 공공 3D 자료에서 가져온 시설물 하나와 충돌용 상자다. */
struct StaticFacility
{
    std::string id;
    std::string type;
    glm::dvec3 centerMeters{0.0};
    glm::vec3 halfExtentsMeters{0.0F};
};

/**
 * @brief 변환된 북한산 시설물 목록을 읽는다.
 * @details 중심과 크기는 시뮬레이션의 동쪽·북쪽·위쪽 좌표이며 미터 단위다.
 */
[[nodiscard]] std::vector<StaticFacility> LoadStaticFacilities(
    const std::filesystem::path& path);
}
