#include "vision/RgbFrameQuality.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

#include <stdexcept>

namespace aerosimlab::vision
{
void Verify_RgbFrameQuality_MeasuresGrayscaleAndSharpnessWithoutChangingInput()
{
    cv::Mat image(2, 2, CV_8UC3);
    image.at<cv::Vec3b>(0, 0) = {0, 0, 0};
    image.at<cv::Vec3b>(0, 1) = {255, 255, 255};
    image.at<cv::Vec3b>(1, 0) = {255, 0, 0};
    image.at<cv::Vec3b>(1, 1) = {0, 255, 0};

    RgbFrameAnalyzer analyzer;
    const RgbFrameQuality quality = analyzer.Analyze(image);

    EXPECT_DOUBLE_EQ(quality.meanBrightness, 120.25);
    EXPECT_GT(quality.meanContrast, 0.0);
    EXPECT_GT(quality.sharpness, 0.0);

    cv::Mat verticallyFlippedImage;
    cv::flip(image, verticallyFlippedImage, 0);
    const RgbFrameQuality flippedQuality = analyzer.Analyze(verticallyFlippedImage);
    EXPECT_DOUBLE_EQ(flippedQuality.meanBrightness, quality.meanBrightness);
    EXPECT_DOUBLE_EQ(flippedQuality.meanContrast, quality.meanContrast);
    EXPECT_DOUBLE_EQ(flippedQuality.sharpness, quality.sharpness);

    EXPECT_EQ(image.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 0));
    EXPECT_EQ(image.at<cv::Vec3b>(0, 1), cv::Vec3b(255, 255, 255));
    EXPECT_THROW(static_cast<void>(analyzer.Analyze(cv::Mat{})), std::invalid_argument);
    EXPECT_THROW(static_cast<void>(analyzer.Analyze(cv::Mat(1, 1, CV_8UC1))), std::invalid_argument);

    const cv::Mat flatImage(4, 4, CV_8UC3, cv::Scalar{100, 100, 100});
    const RgbFrameQuality flatQuality = analyzer.Analyze(flatImage);
    EXPECT_DOUBLE_EQ(flatQuality.meanBrightness, 100.0);
    EXPECT_DOUBLE_EQ(flatQuality.meanContrast, 0.0);
    EXPECT_DOUBLE_EQ(flatQuality.sharpness, 0.0);
}

TEST(RgbFrameQualityTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_RgbFrameQuality_MeasuresGrayscaleAndSharpnessWithoutChangingInput());
}

}
