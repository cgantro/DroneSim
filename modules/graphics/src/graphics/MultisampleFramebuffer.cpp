#include "graphics/MultisampleFramebuffer.h"

#include "graphics/GlApi.h"

#include <algorithm>
#include <stdexcept>

namespace aerosimlab::graphics
{

MultisampleFramebuffer::MultisampleFramebuffer(int width, int height, int samples)
    : m_Width(width),
      m_Height(height),
      m_Samples(samples)
{
    if (width <= 0 || height <= 0)
        throw std::invalid_argument("MSAA framebuffer dimensions must be positive");
    // 요청한 표본 수가 GPU 상한을 넘으면 줄인다. 색상과 깊이 저장소는 같은 수의 표본을 사용해야 한다.
    GLint maxSamples = 1;
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
    m_Samples = std::min(m_Samples, static_cast<int>(maxSamples));

    if (m_Samples < 2)
        throw std::runtime_error("OpenGL multisampling is not supported");

    Create();
}

MultisampleFramebuffer::~MultisampleFramebuffer()
{
    Destroy();
}

void MultisampleFramebuffer::Create()
{
    // 색상 Texture와 깊이·stencil Renderbuffer를 같은 크기와 표본 수로 만들어 framebuffer에 연결한다.
    glGenFramebuffers(1, &m_Framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);

    glGenTextures(1, &m_ColorTexture);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, m_ColorTexture);
    glTexImage2DMultisample(
        GL_TEXTURE_2D_MULTISAMPLE,
        m_Samples,
        GL_RGBA8,
        m_Width,
        m_Height,
        GL_TRUE);

    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D_MULTISAMPLE,
        m_ColorTexture,
        0);

    glGenRenderbuffers(1, &m_DepthStencilBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_DepthStencilBuffer);
    glRenderbufferStorageMultisample(
        GL_RENDERBUFFER,
        m_Samples,
        GL_DEPTH24_STENCIL8,
        m_Width,
        m_Height);

    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER,
        GL_DEPTH_STENCIL_ATTACHMENT,
        GL_RENDERBUFFER,
        m_DepthStencilBuffer);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        // 사용할 수 없는 framebuffer를 남기지 않도록 지금까지 만든 GPU 객체를 해제한 뒤 생성 실패를 알린다.
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        Destroy();
        throw std::runtime_error("MSAA framebuffer is incomplete");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MultisampleFramebuffer::Destroy()
{
    if (m_DepthStencilBuffer != 0)
        glDeleteRenderbuffers(1, &m_DepthStencilBuffer);
    if (m_ColorTexture != 0)
        glDeleteTextures(1, &m_ColorTexture);
    if (m_Framebuffer != 0)
        glDeleteFramebuffers(1, &m_Framebuffer);

    m_DepthStencilBuffer = 0;
    m_ColorTexture = 0;
    m_Framebuffer = 0;
}

void MultisampleFramebuffer::Bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glViewport(0, 0, m_Width, m_Height);
}

void MultisampleFramebuffer::ResolveToDefault() const
{
    // 각 픽셀의 여러 색 표본을 합쳐 기본 framebuffer에 복사한다. 복사 전후 영역은 같은 픽셀 크기를 사용한다.
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_Framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    glBlitFramebuffer(
        0, 0, m_Width, m_Height,
        0, 0, m_Width, m_Height,
        GL_COLOR_BUFFER_BIT,
        GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MultisampleFramebuffer::ResolveToDefault(int x, int y, int width, int height) const
{
    if (width <= 0 || height <= 0)
        return;

    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_Framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
    glBlitFramebuffer(
        x, y, x + width, y + height,
        x, y, x + width, y + height,
        GL_COLOR_BUFFER_BIT,
        GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MultisampleFramebuffer::Resize(int width, int height)
{
    if (width <= 0 || height <= 0) return;
    if (width == m_Width && height == m_Height && m_Framebuffer != 0) return;

    m_Width = width;
    m_Height = height;

    // 화면 크기가 바뀌면 새 크기의 GPU 저장소가 필요하다. 기존 색상·깊이 저장소를 해제하고 새 크기로 만든다.
    Destroy();
    Create();
}

} // namespace aerosimlab::graphics
