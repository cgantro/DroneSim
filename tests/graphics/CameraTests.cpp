#include "graphics/Camera.h"
#include "graphics/GlApi.h"
#include "graphics/GlbModel.h"
#include "graphics/MultisampleFramebuffer.h"
#include "graphics/Renderer.h"
#include "graphics/Shader.h"
#include "graphics/Window.h"
#include "graphics/GlbData.h"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace dronesim::graphics
{
namespace
{
void Verify_TextureCoordinatesPreserveGlTfValues()
{
    using Json = glb_detail::Json;
    std::vector<std::uint8_t> binary(20U);
    const float values[]{0.0F, 0.0F, 0.0F, 0.25F, 0.75F};
    std::memcpy(binary.data(), values, sizeof(values));
    const Json document = {
        {"bufferViews", Json::array({
            {{"buffer", 0}, {"byteOffset", 0}, {"byteLength", 12}},
            {{"buffer", 0}, {"byteOffset", 12}, {"byteLength", 8}}
        })},
        {"accessors", Json::array({
            {{"bufferView", 0}, {"componentType", 5126}, {"count", 1}, {"type", "VEC3"}},
            {{"bufferView", 1}, {"componentType", 5126}, {"count", 1}, {"type", "VEC2"}}
        })}
    };
    const Json primitive = {{"attributes", {{"POSITION", 0}, {"TEXCOORD_0", 1}}}};

    const auto vertices = glb_detail::ReadVertices(document, primitive, binary);

    ASSERT_EQ(vertices.size(), 1U);
    EXPECT_FLOAT_EQ(vertices[0].textureCoordinate.x, 0.25F);
    EXPECT_FLOAT_EQ(vertices[0].textureCoordinate.y, 0.75F);
}

void AppendUint32(std::vector<std::uint8_t>& bytes, std::uint32_t value)
{
    bytes.push_back(static_cast<std::uint8_t>(value));
    bytes.push_back(static_cast<std::uint8_t>(value >> 8U));
    bytes.push_back(static_cast<std::uint8_t>(value >> 16U));
    bytes.push_back(static_cast<std::uint8_t>(value >> 24U));
}

void AppendFloat(std::vector<std::uint8_t>& bytes, float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    AppendUint32(bytes, bits);
}

void AppendAligned(std::vector<std::uint8_t>& bytes)
{
    while (bytes.size() % 4U != 0U)
        bytes.push_back(0U);
}

std::vector<std::uint8_t> CreateTexturedTriangleGlb()
{
    using Json = glb_detail::Json;

    std::vector<std::uint8_t> binary;
    constexpr std::array<float, 9> positions{
        -0.5F, -0.5F, 0.0F, 0.5F, -0.5F, 0.0F, 0.0F, 0.5F, 0.0F};
    constexpr std::array<float, 9> normals{
        0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F};
    constexpr std::array<float, 6> textureCoordinates{
        0.0F, 0.0F, 1.0F, 0.0F, 0.5F, 1.0F};
    for (const float value : positions)
        AppendFloat(binary, value);
    for (const float value : normals)
        AppendFloat(binary, value);
    for (const float value : textureCoordinates)
        AppendFloat(binary, value);

    for (const std::uint16_t index : {0U, 1U, 2U})
    {
        binary.push_back(static_cast<std::uint8_t>(index));
        binary.push_back(static_cast<std::uint8_t>(index >> 8U));
    }
    AppendAligned(binary);
    const std::size_t imageOffset = binary.size();

    // 1x1 RGB PNG fixture; embedding it exercises the same bufferView path as GLB exporters.
    constexpr std::array<std::uint8_t, 69> png{
        0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A,
        0x00, 0x00, 0x00, 0x0D, 0x49, 0x48, 0x44, 0x52,
        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
        0x08, 0x02, 0x00, 0x00, 0x00, 0x90, 0x77, 0x53,
        0xDE, 0x00, 0x00, 0x00, 0x0C, 0x49, 0x44, 0x41,
        0x54, 0x78, 0x9C, 0x63, 0xF8, 0xCF, 0xC0, 0x00,
        0x00, 0x03, 0x01, 0x01, 0x00, 0xC9, 0xFE, 0x92,
        0xEF, 0x00, 0x00, 0x00, 0x00, 0x49, 0x45, 0x4E,
        0x44, 0xAE, 0x42, 0x60, 0x82};
    binary.insert(binary.end(), png.begin(), png.end());
    const std::size_t binaryLength = binary.size();
    AppendAligned(binary);

    const Json document = Json::parse(R"json(
        {
          "asset": {"version": "2.0"},
          "scene": 0,
          "scenes": [{"nodes": [0]}],
          "nodes": [{"mesh": 0}],
          "meshes": [{"primitives": [{
            "attributes": {"POSITION": 0, "NORMAL": 1, "TEXCOORD_0": 2},
            "indices": 3,
            "material": 0
          }]}],
          "materials": [{"pbrMetallicRoughness": {
            "baseColorFactor": [0.8, 0.7, 0.6, 1.0],
            "metallicFactor": 0.2,
            "roughnessFactor": 0.7,
            "baseColorTexture": {"index": 0}
          }}],
          "textures": [{"source": 0, "sampler": 0}],
          "samplers": [{"magFilter": 9729, "minFilter": 9987}],
          "images": [{"bufferView": 4, "mimeType": "image/png"}],
          "buffers": [{"byteLength": 173}],
          "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": 36},
            {"buffer": 0, "byteOffset": 36, "byteLength": 36},
            {"buffer": 0, "byteOffset": 72, "byteLength": 24},
            {"buffer": 0, "byteOffset": 96, "byteLength": 6},
            {"buffer": 0, "byteOffset": 104, "byteLength": 69}
          ],
          "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": 3, "type": "VEC3"},
            {"bufferView": 1, "componentType": 5126, "count": 3, "type": "VEC3"},
            {"bufferView": 2, "componentType": 5126, "count": 3, "type": "VEC2"},
            {"bufferView": 3, "componentType": 5123, "count": 3, "type": "SCALAR"}
          ]
        }
    )json");
    EXPECT_EQ(imageOffset, 104U);
    EXPECT_EQ(binaryLength, 173U);

    std::string json = document.dump();
    while (json.size() % 4U != 0U)
        json.push_back(' ');

    std::vector<std::uint8_t> glb;
    const std::uint32_t totalLength = static_cast<std::uint32_t>(
        12U + 8U + json.size() + 8U + binary.size());
    AppendUint32(glb, 0x46546C67U);
    AppendUint32(glb, 2U);
    AppendUint32(glb, totalLength);
    AppendUint32(glb, static_cast<std::uint32_t>(json.size()));
    AppendUint32(glb, 0x4E4F534AU);
    glb.insert(glb.end(), json.begin(), json.end());
    AppendUint32(glb, static_cast<std::uint32_t>(binary.size()));
    AppendUint32(glb, 0x004E4942U);
    glb.insert(glb.end(), binary.begin(), binary.end());
    return glb;
}

class TemporaryGlbFile final
{
public:
    TemporaryGlbFile()
    {
        const auto uniqueId = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path()
            / ("dronesim-textured-glb-" + std::to_string(uniqueId) + ".glb");
        const std::vector<std::uint8_t> bytes = CreateTexturedTriangleGlb();
        std::ofstream file(path_, std::ios::binary);
        if (!file || !file.write(reinterpret_cast<const char*>(bytes.data()),
                static_cast<std::streamsize>(bytes.size())))
            throw std::runtime_error("임시 GLB 시험 자료를 저장하지 못했습니다.");
    }

    ~TemporaryGlbFile()
    {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    [[nodiscard]] const std::filesystem::path& Path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

void Verify_GlbModelDrawsEmbeddedBaseColorPngWithoutOpenGlErrors()
{
    TemporaryGlbFile fixture;
    Window::Properties properties;
    properties.width = 64;
    properties.height = 64;
    properties.title = "GLB import test";
    properties.visible = false;
    properties.vsync = false;
    Window window(properties);
    Renderer renderer;
    renderer.Init(properties.width, properties.height);
    MultisampleFramebuffer target(properties.width, properties.height);
    Shader shader((std::filesystem::path{DRONESIM_SOURCE_DIR}
        / "assets/shaders/Scene.glsl").string());
    Camera camera{{0.0F, 0.0F, 1.0F}, {0.0F, 0.0F, 0.0F}, 1.0F};

    {
        const auto model = GlbModel::Load(fixture.Path().string());
        ASSERT_NE(model, nullptr);
        EXPECT_EQ(model->PartCount(), 1U);

        renderer.BeginView(target);
        model->Draw(glm::mat4{1.0F}, renderer, shader,
            camera.GetViewMatrix(), camera.GetProjectionMatrix(), camera.GetPosition());
        renderer.EndView(target);

        EXPECT_EQ(glGetError(), GL_NO_ERROR);
    }
}
}

void Verify_GraphicsCamera_UsesSuppliedUpDirectionForRoll()
{
    Camera camera{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}, 1.0F};
    const glm::vec4 worldUp{0.0F, 1.0F, 0.0F, 0.0F};

    const glm::vec4 neutralViewUp = camera.GetViewMatrix() * worldUp;
    EXPECT_NEAR(neutralViewUp.y, 1.0F, 1.0e-6F);

    camera.SetUp({0.0F, 0.0F, 1.0F});
    const glm::vec4 rolledViewUp = camera.GetViewMatrix() * worldUp;
    EXPECT_NEAR(rolledViewUp.x, -1.0F, 1.0e-6F);
    EXPECT_NEAR(rolledViewUp.y, 0.0F, 1.0e-6F);
}

TEST(CameraTests, CoreScenarios)
{
    EXPECT_NO_FATAL_FAILURE(Verify_GraphicsCamera_UsesSuppliedUpDirectionForRoll());
    EXPECT_NO_FATAL_FAILURE(Verify_TextureCoordinatesPreserveGlTfValues());
    EXPECT_NO_FATAL_FAILURE(Verify_GlbModelDrawsEmbeddedBaseColorPngWithoutOpenGlErrors());
}

}
