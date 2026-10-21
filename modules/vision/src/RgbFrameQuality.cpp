#include "vision/RgbFrameQuality.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <stdexcept>

namespace aerosimlab::vision
{
RgbFrameQuality RgbFrameAnalyzer::Analyze(const cv::Mat& rgbImage)
{
    if (rgbImage.empty() || rgbImage.type() != CV_8UC3)
        throw std::invalid_argument("RGB 영상은 비어 있지 않은 8비트 3채널 형식이어야 합니다.");

    cv::cvtColor(rgbImage, grayscaleImage_, cv::COLOR_RGB2GRAY);
    cv::Scalar mean;
    cv::Scalar standardDeviation;
    cv::meanStdDev(grayscaleImage_, mean, standardDeviation);

    cv::Laplacian(grayscaleImage_, laplacianImage_, CV_64F);
    cv::Scalar laplacianMean;
    cv::Scalar laplacianStandardDeviation;
    cv::meanStdDev(laplacianImage_, laplacianMean, laplacianStandardDeviation);

    return {mean[0], standardDeviation[0], laplacianStandardDeviation[0] * laplacianStandardDeviation[0]};
}
}
