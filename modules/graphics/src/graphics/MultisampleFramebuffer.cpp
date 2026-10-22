#include "graphics/MultisampleFramebuffer.h"

#include "graphics/GlApi.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>

namespace aerosimlab::graphics
{

struct MultisampleFramebuffer::PixelReadback
{
    struct Slot
    {
        GLuint buffer = 0;
        GLsync completion = nullptr;
        std::uint64_t simulationTick = 0;
        std::uint64_t order = 0;
        std::size_t byteCount = 0;
        bool pending = false;
    };

    std::array<Slot, 3> slots{};
    std::uint64_t nextOrder = 0;

    void Destroy() noexcept
    {
        for (Slot& slot : slots)
        {
            if (slot.completion != nullptr)
            {
                glDeleteSync(slot.completion);
                slot.completion = nullptr;
            }
            if (slot.buffer != 0)
            {
                glDeleteBuffers(1, &slot.buffer);
                slot.buffer = 0;
            }
            slot.pending = false;
            slot.byteCount = 0;
        }
    }
};

MultisampleFramebuffer::MultisampleFramebuffer(int width, int height, int samples)
    : m_Width(width),
      m_Height(height),
      m_Samples(samples),
      m_PixelReadback(std::make_unique<PixelReadback>())
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

    // ImGui는 일반 2D 텍스처를 표시하므로, MSAA 결과를 받을 화면도 준비한다.
    glGenFramebuffers(1, &m_ResolvedFramebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_ResolvedFramebuffer);
    glGenTextures(1, &m_ResolvedColorTexture);
    glBindTexture(GL_TEXTURE_2D, m_ResolvedColorTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_Width, m_Height, 0,
        GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D, m_ResolvedColorTexture, 0);

    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);

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

    const bool msaaComplete =
        glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, m_ResolvedFramebuffer);
    const bool resolvedComplete =
        glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    if (!msaaComplete || !resolvedComplete)
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
    m_PixelReadback->Destroy();
    if (m_ResolvedColorTexture != 0)
        glDeleteTextures(1, &m_ResolvedColorTexture);
    if (m_ResolvedFramebuffer != 0)
        glDeleteFramebuffers(1, &m_ResolvedFramebuffer);
    if (m_DepthStencilBuffer != 0)
        glDeleteRenderbuffers(1, &m_DepthStencilBuffer);
    if (m_ColorTexture != 0)
        glDeleteTextures(1, &m_ColorTexture);
    if (m_Framebuffer != 0)
        glDeleteFramebuffers(1, &m_Framebuffer);

    m_DepthStencilBuffer = 0;
    m_ColorTexture = 0;
    m_ResolvedColorTexture = 0;
    m_ResolvedFramebuffer = 0;
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

void MultisampleFramebuffer::ResolveToTexture() const
{
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_Framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, m_ResolvedFramebuffer);
    glBlitFramebuffer(0, 0, m_Width, m_Height,
        0, 0, m_Width, m_Height, GL_COLOR_BUFFER_BIT, GL_NEAREST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

bool MultisampleFramebuffer::QueueRgbReadback(std::uint64_t simulationTick)
{
    PixelReadback::Slot* available = nullptr;
    for (PixelReadback::Slot& slot : m_PixelReadback->slots)
    {
        if (!slot.pending)
        {
            available = &slot;
            break;
        }
    }
    if (available == nullptr)
        return false;

    if (available->buffer == 0)
        glGenBuffers(1, &available->buffer);

    const std::size_t byteCount =
        static_cast<std::size_t>(m_Width) * static_cast<std::size_t>(m_Height) * 3U;
    GLint previousPackBuffer = 0;
    GLint previousPackAlignment = 4;
    GLint previousReadFramebuffer = 0;
    GLint previousReadBuffer = GL_BACK;
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &previousPackBuffer);
    glGetIntegerv(GL_PACK_ALIGNMENT, &previousPackAlignment);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previousReadFramebuffer);
    glGetIntegerv(GL_READ_BUFFER, &previousReadBuffer);

    glBindBuffer(GL_PIXEL_PACK_BUFFER, available->buffer);
    if (available->byteCount != byteCount)
    {
        glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(byteCount),
            nullptr, GL_STREAM_READ);
        available->byteCount = byteCount;
    }

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_ResolvedFramebuffer);
    glReadBuffer(GL_COLOR_ATTACHMENT0);
    glReadPixels(0, 0, m_Width, m_Height, GL_RGB, GL_UNSIGNED_BYTE, nullptr);

    available->completion = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    const bool queued = available->completion != nullptr;
    if (queued)
    {
        available->simulationTick = simulationTick;
        available->order = m_PixelReadback->nextOrder++;
        available->pending = true;
    }
    else
    {
        // Fence를 만들지 못하면 다음 전송이 같은 버퍼를 덮기 전에 현재 복사를 마친다.
        glFinish();
    }

    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(previousReadFramebuffer));
    glReadBuffer(static_cast<GLenum>(previousReadBuffer));
    glPixelStorei(GL_PACK_ALIGNMENT, previousPackAlignment);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(previousPackBuffer));
    return queued;
}

bool MultisampleFramebuffer::TryReadRgbPixels(
    std::vector<std::uint8_t>& pixels, std::uint64_t& simulationTick)
{
    PixelReadback::Slot* oldest = nullptr;
    for (PixelReadback::Slot& slot : m_PixelReadback->slots)
    {
        if (slot.pending && (oldest == nullptr || slot.order < oldest->order))
            oldest = &slot;
    }
    if (oldest == nullptr)
        return false;

    const GLenum result = glClientWaitSync(
        oldest->completion, GL_SYNC_FLUSH_COMMANDS_BIT, 0);
    if (result == GL_TIMEOUT_EXPIRED)
        return false;
    if (result == GL_WAIT_FAILED)
    {
        glFinish();
        glDeleteSync(oldest->completion);
        oldest->completion = nullptr;
        oldest->pending = false;
        return false;
    }

    pixels.resize(oldest->byteCount);
    GLint previousPackBuffer = 0;
    glGetIntegerv(GL_PIXEL_PACK_BUFFER_BINDING, &previousPackBuffer);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, oldest->buffer);
    const void* source = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
        static_cast<GLsizeiptr>(oldest->byteCount), GL_MAP_READ_BIT);
    if (source != nullptr)
        std::memcpy(pixels.data(), source, oldest->byteCount);
    const GLboolean valid = source != nullptr
        ? glUnmapBuffer(GL_PIXEL_PACK_BUFFER) : GL_FALSE;
    glBindBuffer(GL_PIXEL_PACK_BUFFER, static_cast<GLuint>(previousPackBuffer));

    simulationTick = oldest->simulationTick;
    glDeleteSync(oldest->completion);
    oldest->completion = nullptr;
    oldest->pending = false;
    return valid == GL_TRUE;
}

void MultisampleFramebuffer::FinishRgbReadbacks()
{
    const bool hasPending = std::any_of(m_PixelReadback->slots.begin(),
        m_PixelReadback->slots.end(), [](const PixelReadback::Slot& slot)
        {
            return slot.pending;
        });
    if (hasPending)
        glFinish();
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
