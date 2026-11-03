#pragma once

#include <cstdint>
#include <vector>

#include <glm/vec2.hpp>

namespace dronesim::terrain
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
    std::vector<float> heightOffsetsMeters;

    /** @brief 지도 중심에서 서쪽·동쪽 경계까지의 거리를 미터로 반환한다. */
    [[nodiscard]] double HalfWidthMeters() const noexcept;

    /** @brief 지도 중심에서 남쪽·북쪽 경계까지의 거리를 미터로 반환한다. */
    [[nodiscard]] double HalfDepthMeters() const noexcept;

    /** @brief 기준 높이를 더한 절대 고도를 반환한다. 행은 북쪽부터, 열은 서쪽부터 센다. */
    [[nodiscard]] float HeightAt(std::uint32_t row, std::uint32_t column) const;

    /** @brief 동·북 방향의 미터 좌표가 지형 격자 안에 있는지 확인한다. */
    [[nodiscard]] bool ContainsLocalPositionMeters(
        const glm::dvec2& positionMeters) const noexcept;

    /** @brief 지도 중심 기준의 동쪽·북쪽 위치에서 격자 높이를 보간해 반환한다. */
    [[nodiscard]] float HeightAtLocalMeters(const glm::dvec2& positionMeters) const;
};
}
