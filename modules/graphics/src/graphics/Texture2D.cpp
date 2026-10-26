#include "graphics/Texture2D.h"

#include "graphics/GlApi.h"

#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <algorithm>
#include <cctype>
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

/** @brief RGB 픽셀 자료를 검사하고 OpenGL 텍스처로 올린다. */
GLuint UploadRgbImage(std::uint32_t width, std::uint32_t height,
    const std::uint8_t* pixels, std::size_t pixelByteCount)
{
    constexpr std::size_t bytesPerPixel = 3U;
    const std::size_t widthSize = width;
    const std::size_t heightSize = height;
    const std::size_t maximumSize = std::numeric_limits<std::size_t>::max();
    if (pixels == nullptr || width == 0U || height == 0U
        || width > static_cast<std::uint32_t>(std::numeric_limits<GLsizei>::max())
        || height > static_cast<std::uint32_t>(std::numeric_limits<GLsizei>::max())
        || widthSize > maximumSize / heightSize
        || widthSize * heightSize > maximumSize / bytesPerPixel
        || widthSize * heightSize * bytesPerPixel != pixelByteCount)
        throw std::invalid_argument("GPU 이미지 크기와 RGB 자료가 맞지 않습니다.");

    GLuint rendererId = 0U;
    glGenTextures(1, &rendererId);
    glBindTexture(GL_TEXTURE_2D, rendererId);
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
        0, GL_RGB, GL_UNSIGNED_BYTE, pixels);
    glGenerateMipmap(GL_TEXTURE_2D);
    glPixelStorei(GL_UNPACK_ALIGNMENT, previousUnpackAlignment);
    glBindTexture(GL_TEXTURE_2D, 0);
    return rendererId;
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

    return std::unique_ptr<Texture2D>(
        new Texture2D(width, height, pixels.data(), pixels.size()));
}

std::unique_ptr<Texture2D> Texture2D::LoadFromFile(const std::filesystem::path& path)
{
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
        [](unsigned char value) { return static_cast<char>(std::tolower(value)); });

    if (extension == ".ppm")
        return LoadPpm(path);
    if (extension != ".png" && extension != ".jpg" && extension != ".jpeg")
        throw std::runtime_error("지원하는 이미지 형식은 PNG, JPEG, PPM입니다: "
            + path.string());

    int width = 0;
    int height = 0;
    using StbiImage = std::unique_ptr<stbi_uc, decltype(&stbi_image_free)>;
    StbiImage pixels(stbi_load(path.string().c_str(), &width, &height,
        nullptr, STBI_rgb), &stbi_image_free);
    if (!pixels || width <= 0 || height <= 0)
    {
        const char* reason = stbi_failure_reason();
        throw std::runtime_error("이미지를 읽지 못했습니다: " + path.string()
            + (reason == nullptr ? std::string{} : " (" + std::string(reason) + ")"));
    }

    const auto imageWidth = static_cast<std::uint32_t>(width);
    const auto imageHeight = static_cast<std::uint32_t>(height);
    constexpr std::size_t bytesPerPixel = 3U;
    const std::size_t widthSize = imageWidth;
    const std::size_t heightSize = imageHeight;
    const std::size_t maximumSize = std::numeric_limits<std::size_t>::max();
    if (widthSize > maximumSize / heightSize
        || widthSize * heightSize > maximumSize / bytesPerPixel)
        throw std::runtime_error("이미지 크기가 너무 큽니다: " + path.string());

    const std::size_t pixelByteCount = widthSize * heightSize * bytesPerPixel;
    return std::unique_ptr<Texture2D>(new Texture2D(
        imageWidth, imageHeight, pixels.get(), pixelByteCount));
}

Texture2D::Texture2D(std::uint32_t width, std::uint32_t height,
    const std::vector<std::uint8_t>& pixels)
    : Texture2D(width, height, pixels.data(), pixels.size())
{
}

Texture2D::Texture2D(std::uint32_t width, std::uint32_t height,
    const std::uint8_t* pixels, std::size_t pixelByteCount)
    : rendererId_(UploadRgbImage(width, height, pixels, pixelByteCount))
{
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
