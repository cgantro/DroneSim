#pragma once

#include <opencv2/core/mat.hpp>

namespace aerosimlab::vision
{
/**
 * @brief RGB 영상의 평균 밝기와 대비를 간단히 나타낸다.
 * @details 두 값은 모두 0부터 255 사이의 8비트 채널 값 기준이다. 평균 밝기는 세 색 채널 평균의 평균이고,
 * 대비는 각 채널 픽셀 값의 표준편차 평균이다. 이 지표는 전체 품질 점수가 아니며 흐림이나 색 왜곡은 판단하지 않는다.
 */
struct RgbFrameQuality
{
    double meanBrightness = 0.0;
    double meanContrast = 0.0;
};

/**
 * @brief RGB 영상의 평균 밝기와 대비를 계산한다.
 * @param rgbImage 비어 있지 않은 8비트 3채널 RGB 영상.
 * @return 채널 평균과 표준편차를 각각 평균한 값.
 * @throws std::invalid_argument 영상이 비어 있거나 8비트 3채널 형식이 아닐 때.
 */
[[nodiscard]] RgbFrameQuality MeasureRgbFrameQuality(const cv::Mat& rgbImage);
}
