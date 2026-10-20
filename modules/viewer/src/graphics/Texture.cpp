#include "graphics/Texture.h"

#include "graphics/GlApi.h"

#include <stdexcept>

namespace grasplink::graphics
{

Texture::Texture() = default;

Texture::~Texture()
{
    // OpenGL 객체를 삭제할 때도 context가 필요하다. 이 Texture를 소유한 코드가 context보다 먼저 파괴되도록 수명을 맞춘다.
    if (m_RendererID != 0)
        glDeleteTextures(1, &m_RendererID);
}

void Texture::Bind(std::uint32_t slot) const
{
    // 지정한 texture unit을 활성화하고 이 Texture를 해당 target에 연결한다. Shader의 sampler uniform에 unit 번호를 넣는 일은 호출자가 따로 한다.
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(m_Target, m_RendererID);
}

std::shared_ptr<Texture> Texture::Create2D(
    int width,
    int height,
    int channels,
    const unsigned char* pixels,
    bool srgb)
{
    // 잘못된 이미지 입력은 OpenGL 상태를 바꾸기 전에 거부한다. 그래야 일부만 생성된 Texture ID가 남지 않는다.
    if (width <= 0 || height <= 0 || pixels == nullptr)
        throw std::runtime_error("Invalid texture data");

    // format은 CPU 픽셀 배열의 채널 순서를 나타낸다. internalFormat은 GPU가 저장할 채널 형식과 색 변환 방식을 정한다.
    GLenum format = GL_RGB;
    GLenum internalFormat = GL_RGB8;

    switch (channels)
    {
    case 1:
        format = GL_RED;
        internalFormat = GL_R8;
        break;
    case 3:
        format = GL_RGB;
        internalFormat = srgb ? GL_SRGB8 : GL_RGB8;
        break;
    case 4:
        format = GL_RGBA;
        internalFormat = srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8;
        break;
    default:
        throw std::runtime_error("Unsupported texture channel count");
    }

    auto texture = std::make_shared<Texture>();
    texture->m_Target = GL_TEXTURE_2D;

    GLint previousTexture = 0;
    GLint previousUnpackAlignment = 4;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &previousTexture);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &previousUnpackAlignment);

    glGenTextures(1, &texture->m_RendererID);
    glBindTexture(GL_TEXTURE_2D, texture->m_RendererID);

    // Texture가 화면에서 작게 보일 때 미리 만든 mip 단계의 색을 선형 보간해 깜빡임과 들쭉날쭉한 무늬를 줄인다.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // CPU 배열은 행 끝 padding 없이 연속 저장되어 있다. 기본 4-byte 정렬 대신 1-byte 정렬로 읽어 각 행을 정확히 해석한다.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        internalFormat,
        width,
        height,
        0,
        format,
        GL_UNSIGNED_BYTE,
        pixels);

    // 축소된 이미지 단계를 미리 만들어 멀리 있거나 작게 보이는 표면에서 Texture 무늬가 깜빡이는 현상을 줄인다.
    glGenerateMipmap(GL_TEXTURE_2D);

    // 업로드 중 바꾼 설정은 OpenGL context 전체에 남으므로 호출 전 값으로 복구한다.
    glPixelStorei(GL_UNPACK_ALIGNMENT, previousUnpackAlignment);
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(previousTexture));

    // 이 경로는 glTF에 저장된 sampler 설정을 읽지 않고 Texture 반복 방식과 필터를 고정값으로 사용한다.
    return texture;
}

} // namespace grasplink::graphics
