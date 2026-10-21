#include "sensor/RgbFrame.h"

#include <gtest/gtest.h>

namespace aerosimlab::sensor
{
TEST(RgbFrame, ChecksDimensionsAndPixelCount)
{
    RgbFrame frame{2, 1, 4, {1, 2, 3, 4, 5, 6}};
    EXPECT_TRUE(frame.IsValid());

    frame.pixels.pop_back();
    EXPECT_FALSE(frame.IsValid());
}

TEST(RgbFrame, ChangesOpenGlRowOrderToImageRowOrder)
{
    RgbFrame frame{1, 2, 7, {10, 11, 12, 20, 21, 22}};

    frame.FlipVertical();

    EXPECT_EQ(frame.pixels, (std::vector<std::uint8_t>{20, 21, 22, 10, 11, 12}));
    EXPECT_EQ(frame.simulationTick, 7U);
}
}
