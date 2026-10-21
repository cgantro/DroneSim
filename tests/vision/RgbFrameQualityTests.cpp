#include "vision/RgbFrameQuality.h"

#include <opencv2/core.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

namespace aerosimlab::vision
{
TEST(RgbFrameQuality, MeasuresMeanBrightnessAndContrastWithoutChangingPixels)
{
    cv::Mat image(1, 2, CV_8UC3);
    image.at<cv::Vec3b>(0, 0) = {0, 0, 0};
    image.at<cv::Vec3b>(0, 1) = {255, 255, 255};

    const RgbFrameQuality quality = MeasureRgbFrameQuality(image);

    EXPECT_DOUBLE_EQ(quality.meanBrightness, 127.5);
    EXPECT_DOUBLE_EQ(quality.meanContrast, 127.5);
    EXPECT_EQ(image.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 0));
    EXPECT_EQ(image.at<cv::Vec3b>(0, 1), cv::Vec3b(255, 255, 255));
    EXPECT_THROW(static_cast<void>(MeasureRgbFrameQuality(cv::Mat{})), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(MeasureRgbFrameQuality(cv::Mat(1, 1, CV_8UC1))), std::invalid_argument);
}
}
