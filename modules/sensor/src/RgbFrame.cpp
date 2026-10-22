#include "sensor/RgbFrame.h"

#include <algorithm>
#include <cstddef>
#include <limits>

namespace aerosimlab::sensor
{
bool RgbFrame::IsValid() const noexcept
{
    if (width <= 0 || height <= 0)
        return false;

    const auto frameWidth = static_cast<std::size_t>(width);
    const auto frameHeight = static_cast<std::size_t>(height);
    const auto maximumSize = std::numeric_limits<std::size_t>::max();
    if (frameWidth > maximumSize / 3U)
        return false;

    const auto rowBytes = frameWidth * 3U;
    if (frameHeight > maximumSize / rowBytes)
        return false;

    const auto expectedBytes = rowBytes * frameHeight;
    return pixels.size() == expectedBytes;
}

void RgbFrame::FlipVertical()
{
    if (!IsValid())
        return;

    const auto rowBytes = static_cast<std::size_t>(width) * 3U;
    for (int row = 0; row < height / 2; ++row)
    {
        auto top = pixels.begin() + static_cast<std::ptrdiff_t>(row) * static_cast<std::ptrdiff_t>(rowBytes);
        auto bottom = pixels.begin() + static_cast<std::ptrdiff_t>(height - row - 1)
            * static_cast<std::ptrdiff_t>(rowBytes);
        std::swap_ranges(top, top + static_cast<std::ptrdiff_t>(rowBytes), bottom);
    }

    rowOrder = rowOrder == RgbFrameRowOrder::TopToBottom
        ? RgbFrameRowOrder::BottomToTop
        : RgbFrameRowOrder::TopToBottom;
}
}
