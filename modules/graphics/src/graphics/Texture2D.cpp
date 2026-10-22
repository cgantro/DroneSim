#include "graphics/Texture2D.h"

#include "graphics/GlApi.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

namespace aerosimlab::graphics
{

namespace
{

/**
 * @brief 비스듬히 보이는 텍스처의 흐려짐을 줄입니다.
 * @details 지원하지 않는 그래픽 장치에서는 기본 필터 설정을 그대로 씁니다.
 */
void SetAnisotropicFilteringIfAvailable()
{
#ifndef __EMSCRIPTEN__
    constexpr GLenum textureMaxAnisotropy = 0x84FE;
    constexpr GLenum maxTextureMaxAnisotropy = 0x84FF;
    constexpr char extensionName[] = "GL_EXT_texture_filter_anisotropic";

    GLint extensionCount = 0;
    glGetIntegerv(GL_NUM_EXTENSIONS, &extensionCount);
    bool isSupported = false;
    for (GLint index = 0; index < extensionCount; ++index)
    {
        const auto* extension = reinterpret_cast<const char*>(
            glGetStringi(GL_EXTENSIONS, static_cast<GLuint>(index)));
        if (extension != nullptr && std::strcmp(extension, extensionName) == 0)
        {
            isSupported = true;
            break;
        }
    }

    if (!isSupported)
        return;

    GLfloat maximumAnisotropy = 1.0F;
    glGetFloatv(maxTextureMaxAnisotropy, &maximumAnisotropy);
    glTexParameterf(GL_TEXTURE_2D, textureMaxAnisotropy,
        std::min(maximumAnisotropy, 8.0F));
#endif
}

} // namespace

std::unique_ptr<Texture2D> Texture2D::LoadPpm(const std::filesystem::path& path)
{
    std::ifstream image(path, std::ios::binary);
    std::string format;
    std::uint32_t width = 0U;
    std::uint32_t height = 0U;
    std::uint32_t maximumValue = 0U;
    if (!(image >> format >> width >> height >> maximumValue)
        || format != "P6" || width == 0U || height == 0U || maximumValue != 255U)
        throw std::runtime_error("PPM 이미지 머리 정보를 읽지 못했습니다: " + path.string());

    const int separator = image.get();
    if (separator != '\n' && separator != '\r' && separator != ' ' && separator != '\t')
        throw std::runtime_error("PPM 이미지의 픽셀 시작 위치가 올바르지 않습니다: " + path.string());
    if (separator == '\r' && image.peek() == '\n')
        image.get();

    constexpr std::size_t bytesPerPixel = 3U;
    const std::size_t maximumDimension = std::numeric_limits<std::size_t>::max();
    if (static_cast<std::size_t>(width) > maximumDimension / height
        || static_cast<std::size_t>(width) * height > maximumDimension / bytesPerPixel)
        throw std::runtime_error("PPM 이미지 크기가 너무 큽니다: " + path.string());

    const std::size_t pixelByteCount = static_cast<std::size_t>(width) * height * bytesPerPixel;
    if (pixelByteCount > static_cast<std::size_t>(std::numeric_limits<std::streamsize>::max()))
        throw std::runtime_error("PPM 이미지 크기가 너무 큽니다: " + path.string());
    std::vector<std::uint8_t> pixels(pixelByteCount);
    image.read(reinterpret_cast<char*>(pixels.data()), static_cast<std::streamsize>(pixelByteCount));
    if (image.gcount() != static_cast<std::streamsize>(pixelByteCount))
        throw std::runtime_error("PPM 이미지 픽셀 자료가 중간에 끝났습니다: " + path.string());

    return std::make_unique<Texture2D>(width, height, pixels);
}

Texture2D::Texture2D(std::uint32_t width, std::uint32_t height,
    const std::vector<std::uint8_t>& pixels)
{
    constexpr std::size_t bytesPerPixel = 3U;
    const std::size_t widthSize = width;
    const std::size_t heightSize = height;
    const std::size_t maximumSize = std::numeric_limits<std::size_t>::max();
    if (width == 0U || height == 0U
        || width > static_cast<std::uint32_t>(std::numeric_limits<GLsizei>::max())
        || height > static_cast<std::uint32_t>(std::numeric_limits<GLsizei>::max())
        || widthSize > maximumSize / heightSize
        || widthSize * heightSize > maximumSize / bytesPerPixel
        || widthSize * heightSize * bytesPerPixel != pixels.size())
        throw std::invalid_argument("GPU 이미지 크기와 RGB 자료가 맞지 않습니다.");

    glGenTextures(1, &rendererId_);
    glBindTexture(GL_TEXTURE_2D, rendererId_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    SetAnisotropicFilteringIfAvailable();

    GLint previousUnpackAlignment = 4;
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &previousUnpackAlignment);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB8,
        static_cast<GLsizei>(width), static_cast<GLsizei>(height),
        0, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    glGenerateMipmap(GL_TEXTURE_2D);
    glPixelStorei(GL_UNPACK_ALIGNMENT, previousUnpackAlignment);
    glBindTexture(GL_TEXTURE_2D, 0);
}

Texture2D::~Texture2D()
{
    if (rendererId_ != 0U)
        glDeleteTextures(1, &rendererId_);
}

void Texture2D::Bind() const
{
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, rendererId_);
}

void Texture2D::Unbind() const
{
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, 0);
}

} // namespace aerosimlab::graphics
