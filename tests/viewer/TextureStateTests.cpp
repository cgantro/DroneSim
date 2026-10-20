#include "graphics/GlApi.h"
#include "graphics/IndexBuffer.h"
#include "graphics/Mesh.h"
#include "graphics/Shader.h"
#include "graphics/Texture.h"
#include "graphics/Window.h"
#include "model/ModelResource.h"
#include "rendering/assets/AssetManager.h"
#include "rendering/assets/PrefabFactory.h"
#include "scene/Scene.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
PFNGLGETSHADERIVPROC originalGetShaderiv = nullptr;

void APIENTRY ReturnEmptyShaderLog(GLuint shader, GLenum name, GLint* value)
{
    if (name == GL_INFO_LOG_LENGTH)
    {
        *value = 0;
        return;
    }
    originalGetShaderiv(shader, name, value);
}
}

TEST(GraphicsResource, InvalidUploadDataDoesNotPolluteAssetCacheOrOpenGlState)
{
    using grasplink::model::MaterialData;
    using grasplink::model::MeshData;
    using grasplink::model::ModelResource;
    using grasplink::model::TextureData;
    using grasplink::rendering::AssetManager;
    using grasplink::graphics::Texture;
    using grasplink::graphics::Window;

    Window window{Window::Properties{32, 32, "Texture state test", false, false}};

    const grasplink::model::Vertex vertex{};
    constexpr std::array<std::uint32_t, 3> triangleIndices{0U, 0U, 0U};
    GLuint sentinelVertexArray = 0;
    glGenVertexArrays(1, &sentinelVertexArray);
    glBindVertexArray(sentinelVertexArray);
    GLuint sentinelIndexBuffer = 0;
    glGenBuffers(1, &sentinelIndexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sentinelIndexBuffer);
    GLint previousVertexArray = 0;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previousVertexArray);
    GLint previousIndexBuffer = 0;
    glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &previousIndexBuffer);
    EXPECT_THROW((grasplink::graphics::Mesh{
        &vertex, std::numeric_limits<std::uint32_t>::max(),
        triangleIndices.data(), static_cast<std::uint32_t>(triangleIndices.size())}),
        std::runtime_error);
    EXPECT_THROW((grasplink::graphics::IndexBuffer{
        triangleIndices.data(), std::numeric_limits<std::uint32_t>::max()}),
        std::runtime_error);
    GLint currentVertexArray = 0;
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &currentVertexArray);
    EXPECT_EQ(currentVertexArray, previousVertexArray);
    GLint currentIndexBuffer = 0;
    glGetIntegerv(GL_ELEMENT_ARRAY_BUFFER_BINDING, &currentIndexBuffer);
    EXPECT_EQ(currentIndexBuffer, previousIndexBuffer);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    glDeleteBuffers(1, &sentinelIndexBuffer);
    glDeleteVertexArrays(1, &sentinelVertexArray);

    ModelResource malformedModel;
    TextureData shortPixelBuffer;
    shortPixelBuffer.width = 2;
    shortPixelBuffer.height = 2;
    shortPixelBuffer.channels = 4;
    shortPixelBuffer.pixels = {24, 128, 220};
    malformedModel.textures.push_back(std::move(shortPixelBuffer));
    AssetManager assets;
    EXPECT_THROW(assets.UploadModel(malformedModel), std::runtime_error);

    ModelResource invalidMeshModel;
    MaterialData material;
    material.uniqueID = grasplink::model::ResourceID{42U};
    invalidMeshModel.materials.push_back(material);
    MeshData mesh;
    mesh.uniqueID = grasplink::model::ResourceID{43U};
    invalidMeshModel.meshes.push_back(mesh);
    EXPECT_THROW(assets.UploadModel(invalidMeshModel), std::runtime_error);
    EXPECT_EQ(assets.GetMaterial(material.uniqueID), nullptr);

    ModelResource invalidIndexModel;
    MeshData invalidIndexMesh;
    invalidIndexMesh.uniqueID = grasplink::model::ResourceID{44U};
    invalidIndexMesh.vertices.resize(3U);
    invalidIndexMesh.indices = {0U, 1U, 3U};
    invalidIndexModel.meshes.push_back(invalidIndexMesh);
    EXPECT_THROW(assets.UploadModel(invalidIndexModel), std::runtime_error);
    EXPECT_EQ(assets.GetMesh(invalidIndexMesh.uniqueID), nullptr);

    ModelResource incompleteTriangleModel;
    MeshData incompleteTriangleMesh;
    incompleteTriangleMesh.uniqueID = grasplink::model::ResourceID{45U};
    incompleteTriangleMesh.vertices.resize(3U);
    incompleteTriangleMesh.indices = {0U, 1U};
    incompleteTriangleModel.meshes.push_back(incompleteTriangleMesh);
    EXPECT_THROW(assets.UploadModel(incompleteTriangleModel), std::runtime_error);
    EXPECT_EQ(assets.GetMesh(incompleteTriangleMesh.uniqueID), nullptr);

    GLint previousActiveTexture = 0;
    GLint previousUnpackAlignment = 0;
    glGetIntegerv(GL_ACTIVE_TEXTURE, &previousActiveTexture);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &previousUnpackAlignment);

    glActiveTexture(GL_TEXTURE3);
    GLuint sentinelTexture = 0;
    glGenTextures(1, &sentinelTexture);
    glBindTexture(GL_TEXTURE_2D, sentinelTexture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 8);

    constexpr std::array<unsigned char, 3> pixel{24, 128, 220};
    const auto texture = Texture::Create2D(1, 1, 3, pixel.data(), false);

    GLint boundTexture = 0;
    GLint unpackAlignment = 0;
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &boundTexture);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &unpackAlignment);
    EXPECT_EQ(boundTexture, static_cast<GLint>(sentinelTexture));
    EXPECT_EQ(unpackAlignment, 8);
    EXPECT_NE(texture, nullptr);

    glBindTexture(GL_TEXTURE_2D, 0);
    glDeleteTextures(1, &sentinelTexture);
    glPixelStorei(GL_UNPACK_ALIGNMENT, previousUnpackAlignment);
    glActiveTexture(static_cast<GLenum>(previousActiveTexture));
    EXPECT_EQ(glGetError(), GL_NO_ERROR);
}

TEST(GraphicsResource, InvalidShaderSourceFailsCleanly)
{
    using grasplink::graphics::Shader;
    using grasplink::graphics::Window;

    Window window{Window::Properties{32, 32, "Shader failure test", false, false}};
    const std::string vertex =
        "#version 330 core\nvoid main(){gl_Position=vec4(0.0,0.0,0.0,1.0);}";
    const std::string fragment =
        "#version 330 core\nthis is not valid GLSL;";

    originalGetShaderiv = glad_glGetShaderiv;
    glad_glGetShaderiv = ReturnEmptyShaderLog;
    EXPECT_THROW(Shader::CreateFromSource("invalid", vertex, fragment), std::runtime_error);
    glad_glGetShaderiv = originalGetShaderiv;
    originalGetShaderiv = nullptr;
    EXPECT_EQ(glGetError(), GL_NO_ERROR);
}

TEST(GraphicsResource, ModelWithoutSceneRootIsRejectedBeforeSceneMutation)
{
    flecs::world world;
    grasplink::scene::Scene scene{world};
    grasplink::rendering::AssetManager assets;
    grasplink::model::ModelResource model;
    const grasplink::scene::Entity sceneRoot{scene.GetSceneRoot()};
    const std::size_t childCountBefore = sceneRoot.GetChildren().size();

    try
    {
        grasplink::rendering::prefab_factory::CreateModel(scene, model, assets, {});
        FAIL() << "A model without a scene root cannot create a prefab";
    }
    catch (const std::runtime_error& error)
    {
        EXPECT_STREQ("PrefabFactory requires a valid scene root", error.what());
    }
    EXPECT_EQ(sceneRoot.GetChildren().size(), childCountBefore);
}
