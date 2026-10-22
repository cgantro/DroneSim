#pragma once

#include <cstdint>
#include <vector>

namespace aerosimlab::graphics
{

/**
 * @brief MSAA 색·깊이를 저장하고, 화면용 질감을 따로 만들어 장면 패널에 보여 준다.
 * @details Framebuffer는 그려진 색과 깊이를 저장할 GPU 대상이며 실제 창 화면과 다르다. 색은 픽셀마다 RGBA 채널 각각 8 bit, 카메라 앞뒤 비교용 깊이는 24 bit, stencil 표식은 8 bit로 저장한다.
 * Stencil은 그리기 중 특정 픽셀을 표시하거나 가려 이후 그릴 영역을 제한하는 값이다. 현재 Renderer는 이 표식을 사용하지 않는다.
 * MSAA는 한 픽셀 안 여러 지점에서 삼각형이 덮는지 검사해 가장자리가 차지하는 비율을 구하고 계단 모양을 줄인다.
 * ResolveToDefault()는 각 표본 색을 합쳐 Window가 표시할 한 픽셀로 복사한다. 창 출력 대상과 이 객체의 크기가 같아야 한다.
 * 생성·크기 변경·복사·삭제 때 GPU 작업을 실행하는 OpenGL 실행 환경(context)이 현재 스레드에서 활성화되어야 한다.
 */
class MultisampleFramebuffer final
{
public:
    /**
     * @brief 지정한 픽셀 크기와 픽셀마다 확인할 지점 수로 GPU 출력 대상을 만든다.
     * @param width 너비 [pixel]. 양수여야 한다.
     * @param height 높이 [pixel]. 양수여야 한다.
     * @param samples 픽셀당 확인할 지점 수. 기본값은 4이며 GPU 상한보다 크면 지원 가능한 최대값으로 낮춘다.
     * @throws std::invalid_argument 너비나 높이가 양수가 아닐 때.
     * @throws std::runtime_error 상한 적용 후 지점 수가 2보다 작거나 GPU 출력 대상 구성이 완성되지 않을 때.
     */
    MultisampleFramebuffer(int width, int height, int samples = 4);

    /** @brief 소유한 색·깊이 이미지와 출력 대상을 해제한다. 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다. */
    ~MultisampleFramebuffer();

    MultisampleFramebuffer(const MultisampleFramebuffer&) = delete;
    MultisampleFramebuffer& operator=(const MultisampleFramebuffer&) = delete;

    /** @brief 이 framebuffer에 색과 깊이를 쓰도록 GPU 출력 대상과 픽셀 영역(viewport)을 선택한다.
     * @details 현재 출력 대상과 화면 영역을 바꾸며, 이전 화면 영역은 저장하거나 복구하지 않는다.
     */
    void Bind() const;

    /**
     * @brief 색 이미지와 깊이·stencil 값을 저장할 영역을 새 픽셀 크기로 다시 만든다.
     * @param width 새 너비(pixel).
     * @param height 새 높이(pixel).
     * @details 너비나 높이가 0 이하이면 아무 작업도 하지 않는다. 크기가 같고 기존 출력 대상이 있으면 다시 만들지 않는다.
     * 크기가 바뀌면 기존 GPU 저장소를 지우고 새 크기의 색·카메라 깊이 저장소를 만든다. 새 구성이 완성되지 않으면 일부 저장소를 지우고 오류를 알린다.
     * 이 경우 새 크기는 보관되지만 출력 대상은 없으므로 유효한 크기로 다시 호출해 복구할 수 있다.
     * @throws std::runtime_error 재생성한 framebuffer가 불완전할 때.
     */
    void Resize(int width, int height);

    /**
     * @brief 여러 위치에서 얻은 색을 화면 출력 대상(0)의 픽셀 색 하나로 합쳐 복사한다.
     * @details 카메라 앞뒤 깊이와 stencil 값은 복사하지 않는다. 이 객체의 색을 읽어 화면 출력 대상으로 복사한 뒤 GPU 출력 선택은 0으로 바꾼다.
     * 화면에 그릴 영역은 바꾸지 않는다. 화면 대상과 크기를 맞춰야 하며 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다.
     */
    void ResolveToDefault() const;

    /** @brief 지정한 픽셀 사각형의 색만 창 출력 대상으로 복사한다. */
    void ResolveToDefault(int x, int y, int width, int height) const;

    /** @brief 여러 색 표본을 일반 색 텍스처에 합쳐 ImGui 패널에 표시할 수 있게 한다. */
    void ResolveToTexture() const;

    /** @brief 최근에 합성한 화면을 ImGui에서 사용할 OpenGL 텍스처 번호로 돌려준다. */
    [[nodiscard]] std::uint32_t GetTextureId() const { return m_ResolvedColorTexture; }

    /** @brief 합성된 화면의 RGB 픽셀을 아래 행부터 읽는다. */
    void ReadRgbPixels(std::vector<std::uint8_t>& pixels) const;

    /** @brief 생성 또는 마지막 크기 변경에 지정한 너비 [pixel]를 반환한다. */
    int GetWidth() const { return m_Width; }
    /** @brief 생성 또는 마지막 크기 변경에 지정한 높이 [pixel]를 반환한다. */
    int GetHeight() const { return m_Height; }
    /** @brief GPU 상한을 적용한 뒤 픽셀마다 검사하는 위치 수를 반환한다. */
    int GetSamples() const { return m_Samples; }

private:
    // 색과 카메라 깊이 저장소를 같은 크기·표본 수로 만들고 GPU가 함께 쓸 수 있는지 확인한다.
    void Create();

    // GPU 저장소를 해제하고 각 저장소 번호를 0으로 초기화한다.
    void Destroy();

private:
    std::uint32_t m_Framebuffer = 0;

    std::uint32_t m_ColorTexture = 0;

    std::uint32_t m_ResolvedFramebuffer = 0;
    std::uint32_t m_ResolvedColorTexture = 0;

    std::uint32_t m_DepthStencilBuffer = 0;

    // 색과 카메라 깊이 저장소의 현재 크기 [pixel].
    int m_Width = 0;
    int m_Height = 0;

    // 색과 카메라 깊이 저장소가 픽셀마다 공통으로 검사하는 위치 수.
    int m_Samples = 4;
};

} // namespace aerosimlab::graphics
