#pragma once

#include <cstdint>
#include <vector>

namespace aerosimlab::sensor
{
/** @brief 영상 픽셀을 읽을 때 기준으로 삼는 첫 행의 위치다. */
enum class RgbFrameRowOrder
{
    TopToBottom,
    BottomToTop
};

/**
 * @brief 한 번의 시뮬레이션 Tick에서 얻은 EO 카메라 RGB 영상을 보관한다.
 * @details 픽셀은 행마다 빨강·초록·파랑 8비트 순서로 저장한다. 행 방향은 rowOrder에 기록하며,
 * OpenGL에서 읽은 영상처럼 아래쪽 행부터 시작하는 데이터도 복사 없이 보관할 수 있다.
 */
struct RgbFrame
{
    int width = 0;
    int height = 0;
    std::uint64_t simulationTick = 0;
    std::vector<std::uint8_t> pixels;
    RgbFrameRowOrder rowOrder = RgbFrameRowOrder::TopToBottom;

    /** @brief 크기와 픽셀 수가 서로 맞는 RGB 영상인지 확인한다. */
    [[nodiscard]] bool IsValid() const noexcept;

    /** @brief 픽셀 행을 위아래로 뒤집고 rowOrder도 함께 바꾼다. */
    void FlipVertical();
};
}
