#pragma once

#include <cstdint>
#include <vector>

namespace aerosimlab::sensor
{
/**
 * @brief 한 번의 시뮬레이션 Tick에서 얻은 EO 카메라 RGB 영상을 보관한다.
 * @details 픽셀은 행마다 빨강·초록·파랑 8비트 순서로 저장하고, 첫 행은 영상의 위쪽이다.
 * OpenGL이 읽어 오는 아래쪽 우선 행을 FlipVertical()로 바꾼 뒤 영상 분석 모듈에 전달한다.
 */
struct RgbFrame
{
    int width = 0;
    int height = 0;
    std::uint64_t simulationTick = 0;
    std::vector<std::uint8_t> pixels;

    /** @brief 크기와 픽셀 수가 서로 맞는 RGB 영상인지 확인한다. */
    [[nodiscard]] bool IsValid() const noexcept;

    /** @brief 아래쪽부터 시작하는 OpenGL 행 순서를 위쪽부터 시작하는 영상 순서로 바꾼다. */
    void FlipVertical();
};
}
