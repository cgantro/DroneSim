#include "vision/RgbFrameQuality.h"

#include <opencv2/core.hpp>

#include <stdexcept>

namespace aerosimlab::vision
{
RgbFrameQuality MeasureRgbFrameQuality(const cv::Mat& rgbImage)
{
    if (rgbImage.empty() || rgbImage.type() != CV_8UC3)
        throw std::invalid_argument("RGB 영상은 비어 있지 않은 8비트 3채널 형식이어야 합니다.");

    cv::Scalar channelMeans;
    cv::Scalar channelStandardDeviations;
    cv::meanStdDev(rgbImage, channelMeans, channelStandardDeviations);

    constexpr double channelCount = 3.0;
    return {
        (channelMeans[0] + channelMeans[1] + channelMeans[2]) / channelCount,
        (channelStandardDeviations[0] + channelStandardDeviations[1]
            + channelStandardDeviations[2]) / channelCount
    };
}
}
