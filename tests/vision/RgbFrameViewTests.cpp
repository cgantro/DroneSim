#include "vision/RgbFrameView.h"

#include <gtest/gtest.h>

#include <stdexcept>

namespace aerosimlab::vision
{
TEST(RgbFrameView, UsesTheOriginalPixelsWithoutCopying)
{
    sensor::RgbFrame frame{2, 1, 3, {1, 2, 3, 4, 5, 6}};

    auto view = MakeRgbFrameView(frame);

    EXPECT_EQ(view.rows, frame.height);
    EXPECT_EQ(view.cols, frame.width);
    EXPECT_EQ(view.type(), CV_8UC3);
    EXPECT_EQ(view.data, frame.pixels.data());
    view.at<cv::Vec3b>(0, 1)[0] = 42;
    EXPECT_EQ(frame.pixels[3], 42);
}

TEST(RgbFrameView, RejectsInvalidFrames)
{
    sensor::RgbFrame frame{2, 1, 0, {1, 2, 3}};

    EXPECT_THROW(static_cast<void>(MakeRgbFrameView(frame)), std::invalid_argument);
}
}
