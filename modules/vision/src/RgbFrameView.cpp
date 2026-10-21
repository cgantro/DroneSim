#include "vision/RgbFrameView.h"

#include <opencv2/core.hpp>

#include <stdexcept>

namespace aerosimlab::vision
{
cv::Mat MakeRgbFrameView(sensor::RgbFrame& frame)
{
    if (!frame.IsValid())
        throw std::invalid_argument("RGB 영상 크기와 픽셀 수가 맞지 않습니다.");

    const auto rowBytes = static_cast<std::size_t>(frame.width) * 3U;
    return cv::Mat(frame.height, frame.width, CV_8UC3, frame.pixels.data(), rowBytes);
}
}
