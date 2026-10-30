#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <vector>

namespace dronesim::graphics
{

/** @brief RGB 이미지 한 장을 GPU에 올려 장면 표면에 입힌다. */
class Texture2D final
{
public:
    Texture2D(std::uint32_t width, std::uint32_t height,
        const std::vector<std::uint8_t>& pixels);

    /** @brief 프로젝트의 간단한 PPM 이미지 하나를 GPU 텍스처로 읽는다. */
    [[nodiscard]] static std::unique_ptr<Texture2D> LoadPpm(
        const std::filesystem::path& path, bool repeat = false);

    /** @brief PPM, PNG, JPEG 이미지를 읽어 GPU 텍스처로 올린다. */
    [[nodiscard]] static std::unique_ptr<Texture2D> LoadFromFile(
        const std::filesystem::path& path, bool repeat = false);

    /** @brief 메모리에 담긴 PNG 또는 JPEG를 GPU 텍스처로 읽는다. */
    [[nodiscard]] static std::unique_ptr<Texture2D> LoadFromMemory(
        std::span<const std::uint8_t> encodedImage, bool repeat = false);

    ~Texture2D();

    Texture2D(const Texture2D&) = delete;
    Texture2D& operator=(const Texture2D&) = delete;

    /** @brief 장면을 그릴 때 사용하는 첫 번째 이미지 슬롯에서 이 텍스처를 선택한다. */
    void Bind() const;
    /** @brief 첫 번째 이미지 슬롯에서 텍스처 선택을 해제한다. */
    void Unbind() const;

private:
    Texture2D(std::uint32_t width, std::uint32_t height,
        const std::uint8_t* pixels, std::size_t pixelByteCount, bool repeat = false);

    std::uint32_t rendererId_ = 0U;
};

} // namespace dronesim::graphics
