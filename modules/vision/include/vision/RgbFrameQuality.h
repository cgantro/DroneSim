#pragma once

#include <opencv2/core/mat.hpp>

namespace aerosimlab::vision
{
/**
 * @brief 회색 영상의 평균 밝기와 대비, 선명도 추정값을 나타낸다.
 * @details 밝기와 대비는 0부터 255 사이의 회색 픽셀 값 기준이다. 선명도는 라플라시안 응답의 분산이며,
 * 영상의 모서리와 노이즈가 많을수록 커질 수 있다. 따라서 세 값은 전체 품질 점수가 아니다.
 */
struct RgbFrameQuality
{
    double meanBrightness = 0.0;
    double meanContrast = 0.0;
    double sharpness = 0.0;
};

/**
 * @brief RGB 영상에서 재사용 가능한 품질 분석기를 제공한다.
 * @details 회색 변환과 라플라시안 계산에 필요한 작업 영상을 보관해 다음 프레임에서도 재사용한다.
 * 계산하는 세 지표는 픽셀 위치를 반환하지 않으므로, 위아래 행 순서가 바뀌어도 결과가 같다.
 * 작업 영상이 바뀌므로 한 인스턴스는 한 스레드에서 차례로 호출해야 한다.
 */
class RgbFrameAnalyzer
{
public:
    /**
     * @brief RGB 영상을 회색으로 바꾸고 기본 품질 지표를 계산한다.
     * @details 회색 변환은 빨강·초록·파랑의 밝기 차이를 반영한다. 라플라시안은 주변 픽셀과의 밝기 변화를
     * 강조하므로 그 분산으로 선명도를 가늠할 수 있지만, 노이즈나 장면의 세부 묘사도 결과를 키울 수 있다.
     * @param rgbImage 비어 있지 않은 8비트 3채널 RGB 영상. 호출 중에만 빌려 쓴다.
     * @return 회색 영상에서 계산한 평균 밝기·표준편차와 라플라시안 분산.
     * @throws std::invalid_argument 영상이 비어 있거나 8비트 3채널 형식이 아닐 때.
     */
    [[nodiscard]] RgbFrameQuality Analyze(const cv::Mat& rgbImage);

private:
    cv::Mat grayscaleImage_;
    cv::Mat laplacianImage_;
};
}
