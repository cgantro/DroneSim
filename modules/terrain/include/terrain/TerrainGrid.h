#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

#include <glm/vec2.hpp>

namespace aerosimlab::terrain
{
/**
 * @brief 일정한 간격으로 높이를 기록한 정사각형 지형이다.
 * @details 높이는 기준 높이에서 얼마나 위나 아래에 있는지 미터로 저장한다. 첫 행은 북쪽, 첫 열은 서쪽이다.
 */
struct TerrainGrid
{
    std::uint32_t sampleCount = 0;
    float spacingXMeters = 0.0F;
    float spacingYMeters = 0.0F;
    float referenceElevationMeters = 0.0F;
    double centerLatitudeDegrees = 0.0;
    double centerLongitudeDegrees = 0.0;
    std::vector<float> heightOffsetsMeters;

    /** @brief 저장된 지형 격자를 읽는다. 파일 형식이 맞지 않으면 원인을 알려준다. */
    [[nodiscard]] static TerrainGrid Load(const std::filesystem::path& path);

    /** @brief 북쪽부터 센 행과 서쪽부터 센 열의 높이를 반환한다. */
    [[nodiscard]] float HeightAt(std::uint32_t row, std::uint32_t column) const;

    /** @brief 지도 중심 기준의 동쪽·북쪽 위치에서 격자 높이를 보간해 반환한다. */
    [[nodiscard]] float HeightAtLocalMeters(const glm::dvec2& positionMeters) const;

    /** @brief 위도·경도를 지형 중심 기준의 동쪽·북쪽 미터 좌표로 바꾼다. */
    [[nodiscard]] glm::dvec2 ToLocalMeters(
        double latitudeDegrees, double longitudeDegrees) const;
};
}
