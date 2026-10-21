#pragma once

#include "sensor/RgbFrame.h"

#include <opencv2/core/mat.hpp>

namespace aerosimlab::vision
{
/**
 * @brief 보관 중인 RGB 영상 픽셀을 복사 없이 OpenCV 영상으로 보여 준다.
 * @details 반환한 영상은 픽셀을 소유하지 않는다. 원본 프레임이 살아 있고
 * 크기나 저장 공간이 바뀌지 않는 동안에만 사용할 수 있다.
 * OpenCV 영상에서 픽셀을 바꾸면 원본 프레임도 함께 바뀐다.
 * 픽셀 채널은 빨강·초록·파랑 순서다. cv::Mat에는 색상 순서 정보가 없으므로,
 * 색상을 변환할 때는 RGB 입력에 맞는 변환을 선택해야 한다.
 * @param frame 화면 앱이 보관 중인 RGB 영상
 * @return 원본 픽셀을 그대로 가리키는 OpenCV 영상
 * @throws std::invalid_argument 영상 크기와 픽셀 수가 맞지 않을 때
 */
[[nodiscard]] cv::Mat MakeRgbFrameView(sensor::RgbFrame& frame);
}
