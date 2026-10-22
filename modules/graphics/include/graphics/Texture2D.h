#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

namespace aerosimlab::graphics
{

/** @brief RGB 이미지 한 장을 GPU에 올려 장면 표면에 입힌다. */
class Texture2D final
{
public:
    Texture2D(std::uint32_t width, std::uint32_t height,
        const std::vector<std::uint8_t>& pixels);

    /** @brief 프로젝트의 간단한 PPM 이미지 하나를 GPU 텍스처로 읽는다. */
    [[nodiscard]] static std::unique_ptr<Texture2D> LoadPpm(
        const std::filesystem::path& path);

    ~Texture2D();

    Texture2D(const Texture2D&) = delete;
    Texture2D& operator=(const Texture2D&) = delete;

    /** @brief 장면을 그릴 때 사용하는 첫 번째 이미지 슬롯에서 이 텍스처를 선택한다. */
    void Bind() const;
    /** @brief 첫 번째 이미지 슬롯에서 텍스처 선택을 해제한다. */
    void Unbind() const;

private:
    std::uint32_t rendererId_ = 0U;
};

} // namespace aerosimlab::graphics
