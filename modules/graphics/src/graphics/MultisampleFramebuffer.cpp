#include "graphics/MultisampleFramebuffer.h"

#include "graphics/GlApi.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>
#include <stdexcept>

namespace aerosimlab::graphics
{

struct MultisampleFramebuffer::PixelReadback
{
    struct Slot
    {
        GLuint buffer = 0;
        const void* mappedPixels = nullptr;
        GLsync completion = nullptr;
        std::uint64_t simulationTick = 0;
        std::uint64_t order = 0;
        std::size_t byteCount = 0;
        bool pending = false;
    };

    std::array<Slot, 3> slots{};
    std::uint64_t nextOrder = 0;

    Slot* OldestPendingSlot()
    {
        Slot* oldest = nullptr;
        for (Slot& slot : slots)
        {
            if (slot.pending && (oldest == nullptr || slot.order < oldest->order))
                oldest = &slot;
        }
        return oldest;
    }

    Slot* AvailableSlot()
    {
        for (Slot& slot : slots)
        {
            if (!slot.pending)
                return &slot;
        }
        return nullptr;
    }

    void EnsureBufferSize(Slot& slot, std::size_t byteCount)
    {
        if (slot.byteCount == byteCount)
            return;

#ifndef __EMSCRIPTEN__
        if (slot.mappedPixels != nullptr)
        {
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
            slot.mappedPixels = nullptr;
            glDeleteBuffers(1, &slot.buffer);
            glGenBuffers(1, &slot.buffer);
            glBindBuffer(GL_PIXEL_PACK_BUFFER, slot.buffer);
        }

        if (GLAD_GL_ARB_buffer_storage && glad_glBufferStorage != nullptr)
        {
            // 지원되는 그래픽 환경에서는 PBO를 한 번만 매핑한다. 픽셀은 fence가 끝난 뒤에 읽는다.
            constexpr GLbitfield persistentReadFlags =
                GL_MAP_READ_BIT | GL_MAP_PERSISTENT_BIT | GL_MAP_COHERENT_BIT;
            glBufferStorage(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(byteCount),
                nullptr, persistentReadFlags);
            slot.mappedPixels = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
                static_cast<GLsizeiptr>(byteCount), persistentReadFlags);
            if (slot.mappedPixels == nullptr)
            {
                glDeleteBuffers(1, &slot.buffer);
                glGenBuffers(1, &slot.buffer);
                glBindBuffer(GL_PIXEL_PACK_BUFFER, slot.buffer);
                glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(byteCount),
                    nullptr, GL_STREAM_READ);
            }
        }
        else
#endif
        {
            glBufferData(GL_PIXEL_PACK_BUFFER, static_cast<GLsizeiptr>(byteCount),
                nullptr, GL_STREAM_READ);
        }
        slot.byteCount = byteCount;
    }

    bool SubmitReadback(
        Slot& slot, int width, int height, GLuint resolvedFramebuffer,
        std::uint64_t simulationTick, std::uint64_t& nextOrder,
        RgbReadbackEnqueueTimings* timings)
    {
        const auto readTargetSetupStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, resolvedFramebuffer);
        glReadBuffer(GL_COLOR_ATTACHMENT0);
        if (timings != nullptr)
        {
            timings->readTargetSetupMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - readTargetSetupStart).count();
        }

        const auto readPixelsStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        if (timings != nullptr)
        {
            timings->readPixelsCallMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - readPixelsStart).count();
        }

        const auto fenceCreationStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        slot.completion = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
        if (timings != nullptr)
        {
            timings->fenceCreationCallMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - fenceCreationStart).count();
        }
        const bool queued = slot.completion != nullptr;
        if (queued)
        {
            slot.simulationTick = simulationTick;
            slot.order = nextOrder++;
            slot.pending = true;
        }
        else
        {
            // Fence를 만들지 못하면 다음 전송이 같은 버퍼를 덮기 전에 현재 복사를 마친다.
            glFinish();
        }

        const auto stateRestoreStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        glReadBuffer(GL_BACK);
        glPixelStorei(GL_PACK_ALIGNMENT, 4);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        if (timings != nullptr)
        {
            timings->glStateRestoreMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - stateRestoreStart).count();
        }
        return queued;
    }

    bool WaitForCompletion(Slot& slot, RgbReadbackPollTimings* timings)
    {
        const auto waitStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        const GLenum result = glClientWaitSync(
            slot.completion, GL_SYNC_FLUSH_COMMANDS_BIT, 0);
        if (timings != nullptr)
        {
            timings->fenceWaitCallMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - waitStart).count();
        }
        if (result == GL_TIMEOUT_EXPIRED)
            return false;
        if (result != GL_WAIT_FAILED)
        {
            if (timings != nullptr)
                timings->pixelsReady = true;
            return true;
        }

        if (timings != nullptr)
            timings->fenceWaitFailed = true;
        const auto finishStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        glFinish();
        if (timings != nullptr)
        {
            timings->fallbackFinishMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - finishStart).count();
        }
        const auto cleanupStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        glDeleteSync(slot.completion);
        slot.completion = nullptr;
        slot.pending = false;
        if (timings != nullptr)
        {
            timings->cleanupMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - cleanupStart).count();
        }
        return false;
    }

    bool CopyCompletedPixels(
        Slot& slot, std::vector<std::uint8_t>& pixels,
        std::uint64_t& simulationTick, RgbReadbackPollTimings* timings)
    {
        const auto resizeStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        pixels.resize(slot.byteCount);
        if (timings != nullptr)
        {
            timings->pixelBufferResizeMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - resizeStart).count();
        }

        const auto bufferBindStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        glBindBuffer(GL_PIXEL_PACK_BUFFER, slot.buffer);
        if (timings != nullptr)
        {
            timings->pixelPackBufferBindMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - bufferBindStart).count();
        }

        const auto mapStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
#ifndef __EMSCRIPTEN__
        const void* source = slot.mappedPixels;
        if (source == nullptr)
#else
        const void* source = nullptr;
#endif
        {
            source = glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0,
                static_cast<GLsizeiptr>(slot.byteCount), GL_MAP_READ_BIT);
        }
        if (timings != nullptr)
        {
            timings->bufferMapCallMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - mapStart).count();
            timings->bufferMapped = source != nullptr;
        }

        const auto copyStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        if (source != nullptr)
            std::memcpy(pixels.data(), source, slot.byteCount);
        if (timings != nullptr && source != nullptr)
        {
            timings->pixelCopyMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - copyStart).count();
        }

        const auto unmapStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
#ifndef __EMSCRIPTEN__
        const GLboolean valid = slot.mappedPixels != nullptr
            ? GL_TRUE : (source != nullptr
                ? glUnmapBuffer(GL_PIXEL_PACK_BUFFER) : GL_FALSE);
#else
        const GLboolean valid = source != nullptr
            ? glUnmapBuffer(GL_PIXEL_PACK_BUFFER) : GL_FALSE;
#endif
        if (timings != nullptr && source != nullptr)
        {
            timings->bufferUnmapCallMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - unmapStart).count();
        }

        const auto cleanupStart = timings != nullptr
            ? std::chrono::steady_clock::now()
            : std::chrono::steady_clock::time_point{};
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        simulationTick = slot.simulationTick;
        glDeleteSync(slot.completion);
        slot.completion = nullptr;
        slot.pending = false;
        if (timings != nullptr)
        {
            timings->cleanupMicroseconds = std::chrono::duration<double, std::micro>(
                std::chrono::steady_clock::now() - cleanupStart).count();
        }
        return valid == GL_TRUE;
    }

    void Destroy() noexcept
    {
        const bool hasMappedReadbackInFlight = std::any_of(
            slots.begin(), slots.end(), [](const Slot& slot)
            {
                return slot.pending && slot.mappedPixels != nullptr;
            });
        if (hasMappedReadbackInFlight)
            glFinish();

        for (Slot& slot : slots)
        {
            if (slot.completion != nullptr)
            {
                glDeleteSync(slot.completion);
                slot.completion = nullptr;
            }
            if (slot.buffer != 0)
            {
#ifndef __EMSCRIPTEN__
                if (slot.mappedPixels != nullptr)
                {
                    glBindBuffer(GL_PIXEL_PACK_BUFFER, slot.buffer);
                    glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
                    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
                    slot.mappedPixels = nullptr;
                }
#endif
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

bool MultisampleFramebuffer::QueueRgbReadback(
    std::uint64_t simulationTick, RgbReadbackEnqueueTimings* timings)
{
    if (timings != nullptr)
        *timings = {};

    const auto slotSetupStart = timings != nullptr
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    PixelReadback::Slot* available = m_PixelReadback->AvailableSlot();
    if (available == nullptr)
        return false;

    if (available->buffer == 0)
        glGenBuffers(1, &available->buffer);
    if (timings != nullptr)
    {
        timings->slotAndBufferCreationMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - slotSetupStart).count();
    }

    const std::size_t byteCount =
        static_cast<std::size_t>(m_Width) * static_cast<std::size_t>(m_Height) * 3U;
    const auto bufferSetupStart = timings != nullptr
        ? std::chrono::steady_clock::now()
        : std::chrono::steady_clock::time_point{};
    glBindBuffer(GL_PIXEL_PACK_BUFFER, available->buffer);
    m_PixelReadback->EnsureBufferSize(*available, byteCount);
    if (timings != nullptr)
    {
        timings->bufferBindingAndAllocationMicroseconds = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - bufferSetupStart).count();
    }

    return m_PixelReadback->SubmitReadback(
        *available, m_Width, m_Height, m_ResolvedFramebuffer,
        simulationTick, m_PixelReadback->nextOrder, timings);
}
bool MultisampleFramebuffer::TryReadRgbPixels(
    std::vector<std::uint8_t>& pixels, std::uint64_t& simulationTick,
    RgbReadbackPollTimings* timings)
{
    if (timings != nullptr)
        *timings = {};

    PixelReadback::Slot* oldest = m_PixelReadback->OldestPendingSlot();
    if (oldest == nullptr || !m_PixelReadback->WaitForCompletion(*oldest, timings))
        return false;

    return m_PixelReadback->CopyCompletedPixels(
        *oldest, pixels, simulationTick, timings);
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
