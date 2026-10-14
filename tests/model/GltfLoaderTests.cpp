#include "model/GltfLoader.h"
#include <gtest/gtest.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
using grasplink::model::GltfLoader;
using grasplink::model::NodeData;

using Bytes = std::vector<unsigned char>;

void AppendUInt32(Bytes& bytes, std::uint32_t value)
{
    for (unsigned int shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<unsigned char>((value >> shift) & 0xffU));
}

Bytes TriangleBytes()
{
    // 가장 작은 POSITION 자료로 float 3개짜리 3D 점 세 개(총 36 bytes)를 만든다.
    // 뒤의 여유 공간으로 점 데이터 끝과 glTF buffer view 끝을 따로 검사한다.
    Bytes bytes(64, 0U);
    const float positions[]{0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F};
    std::memcpy(bytes.data(), positions, sizeof(positions));
    return bytes;
}

// JSON 설명과 바이너리(BIN) 데이터를 따로 만들어 TinyGLTF 파싱부터 실제 로더 처리까지 확인한다. POSITION 정점값은 모델 자체의 기준 좌표다.
std::string TriangleJson(
    const std::string& accessor = R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
    const std::string& views = R"({"buffer":0,"byteOffset":0,"byteLength":36})",
    const std::string& nodes = R"({"mesh":0})")
{
    return R"({"asset":{"version":"2.0"},"buffers":[{"byteLength":64}],"bufferViews":[)" + views +
        R"(],"accessors":[)" + accessor + R"(],"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],"nodes":[)" +
        nodes + R"(],"scenes":[{"nodes":[0]}],"scene":0})";
}

/**
 * @brief 임시 경로에 직접 만든 최소 GLB와 잘못된 GLB 입력을 제공하는 fixture.
 * @details JSON과 BIN chunk는 GLB 파일 안에서 JSON 설명과 정점 자료를 담는 두 구역이다. Fixture는 각 구역을 4-byte 경계에 맞추고 전체 파일 길이를 헤더에 기록한다.
 * 잘못된 입력이 예상한 이유로 거부되는지 확인하고, 임시 파일과 디렉터리는 시험 객체가 끝날 때 제거한다.
 */
class Fixtures
{
public:
    Fixtures()
    {
        directory_ = std::filesystem::temp_directory_path() /
            ("grasplink-gltf-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(directory_);
    }

    ~Fixtures()
    {
        std::error_code error;
        std::filesystem::remove_all(directory_, error);
    }

    std::filesystem::path Write(const std::string& name, std::string json, Bytes binary = TriangleBytes())
    {
        // GLB chunk 시작 주소가 4 bytes 경계에 오도록 JSON에는 공백을, BIN에는 0을 채우고, 그 다음 little-endian 형식의 GLB header를 기록한다.
        while (json.size() % 4U != 0U) json.push_back(' ');
        while (binary.size() % 4U != 0U) binary.push_back(0U);
        Bytes bytes;
        AppendUInt32(bytes, 0x46546c67U);
        AppendUInt32(bytes, 2U);
        AppendUInt32(bytes, static_cast<std::uint32_t>(12U + 8U + json.size() + 8U + binary.size()));
        AppendUInt32(bytes, static_cast<std::uint32_t>(json.size()));
        AppendUInt32(bytes, 0x4e4f534aU);
        bytes.insert(bytes.end(), json.begin(), json.end());
        AppendUInt32(bytes, static_cast<std::uint32_t>(binary.size()));
        AppendUInt32(bytes, 0x004e4942U);
        bytes.insert(bytes.end(), binary.begin(), binary.end());
        const auto path = directory_ / (name + ".glb");
        std::ofstream file(path, std::ios::binary);
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        if (!file) throw std::runtime_error("write GLB fixture: " + name);
        return path;
    }

    void Reject(const std::string& name, const std::string& json, const std::string& diagnostic,
        Bytes binary = TriangleBytes())
    {
        // 잘못된 시험 자료는 로더가 거부하고 예상한 오류 설명까지 반환한 경우에만 의도한 실패 사례로 인정한다.
        const auto path = Write(name, json, std::move(binary));
        std::string errorMessage;
        try { GltfLoader::LoadGLB(path); }
        catch (const std::runtime_error& error) { errorMessage = error.what(); }
        ASSERT_FALSE(errorMessage.empty()) << name + " must be rejected";
        EXPECT_NE(errorMessage.find(diagnostic), std::string::npos)
            << name + " rejected for unexpected reason: " + errorMessage;
    }

private:
    std::filesystem::path directory_;
};

void CheckRepositoryAssets()
{
    // 저장소에 있는 로봇과 바닥 GLB 파일이 Mesh를 읽고 하나의 최상위 node에서 시작하는 계층으로 구성되는지 확인한다.
    for (const char* filename : {"HCR12A_2F-85.glb", "plane.glb"})
    {
        const auto resource = GltfLoader::LoadGLB(std::filesystem::path(GRASPLINK_TEST_ASSET_DIR) / filename);
        ASSERT_TRUE((!resource.meshes.empty() && !resource.nodes.empty())) << std::string(filename) + " has model data";
        ASSERT_TRUE((resource.rootNodeIndex >= 0 && resource.rootNodeIndex < static_cast<int>(resource.nodes.size()))) << std::string(filename) + " has a valid root";
        ASSERT_TRUE((resource.nodes[static_cast<std::size_t>(resource.rootNodeIndex)].parentIndex == -1)) << std::string(filename) + " root has no parent";
        for (const auto& mesh : resource.meshes)
        {
            ASSERT_TRUE((!mesh.vertices.empty() && !mesh.indices.empty())) << std::string(filename) + " has geometry";
            for (const auto index : mesh.indices)
                ASSERT_TRUE((index < mesh.vertices.size())) << std::string(filename) + " index is inside mesh";
        }
    }
}

void CheckAccessors(Fixtures& fixtures)
{
    // 정점 좌표가 연달아 저장되거나 다른 정점 속성과 섞여 저장되어도 각 FLOAT 위치를 정확히 읽는지 확인한다.
    // 자료 구역 밖 접근, 크기 overflow, 지원하지 않는 sparse 자료, 정점 0개, NaN 좌표는 오류로 거부해야 한다.
    const auto valid = GltfLoader::LoadGLB(fixtures.Write("triangle", TriangleJson()));
    ASSERT_TRUE((valid.meshes.front().vertices.size() == 3 && valid.meshes.front().indices ==
        std::vector<std::uint32_t>{0U, 1U, 2U})) << "packed FLOAT positions and generated indices";
    EXPECT_NEAR((valid.meshes.front().vertices[1].position.x), (1.0), (0.0)) << "second packed position";

    Bytes interleaved(64, 0U);
    const float positions[]{0.0F, 0.0F, 0.0F, 99.0F, 1.0F, 0.0F, 0.0F, 99.0F, 0.0F, 1.0F, 0.0F};
    std::memcpy(interleaved.data(), positions, sizeof(positions));
    const auto strided = GltfLoader::LoadGLB(fixtures.Write("interleaved", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteLength":44,"byteStride":16})"), interleaved));
    EXPECT_NEAR((strided.meshes.front().vertices[1].position.x), (1.0), (0.0)) << "interleaved position skips padding";
    EXPECT_NEAR((strided.meshes.front().vertices[2].position.y), (1.0), (0.0)) << "interleaved final position";

    const std::string sparseViews = R"({"buffer":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":1},{"buffer":0,"byteOffset":40,"byteLength":12})";
    const std::string sparse = R"("componentType":5126,"count":3,"type":"VEC3","sparse":{"count":1,"indices":{"bufferView":1,"componentType":5121},"values":{"bufferView":2}})";
    fixtures.Reject("sparse-with-base", TriangleJson("{\"bufferView\":0," + sparse + "}", sparseViews), "Sparse glTF accessors");
    fixtures.Reject("sparse-without-base", TriangleJson("{" + sparse + "}", sparseViews), "Sparse glTF accessors");
    fixtures.Reject("view-outside-buffer", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteOffset":60,"byteLength":36})"), "bufferView exceeds buffer size");
    fixtures.Reject("view-offset-overflow", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteOffset":18446744073709551612,"byteLength":36})"), "bufferView exceeds buffer size");
    fixtures.Reject("accessor-crosses-view", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteLength":24})"), "accessor exceeds bufferView");
    fixtures.Reject("accessor-offset-outside-view", TriangleJson(
        R"({"bufferView":0,"byteOffset":40,"componentType":5126,"count":3,"type":"VEC3"})"), "accessor offset exceeds bufferView");
    fixtures.Reject("accessor-range-overflow", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":18446744073709551615,"type":"VEC3"})"), "accessor range overflows");
    fixtures.Reject("stride-smaller-than-element", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteLength":36,"byteStride":8})"), "Invalid glTF accessor stride");
    // 요소 개수가 0이면 buffer 끝을 읽어 보지 않고 Mesh가 비어 있다는 오류로 입력을 거부해야 한다.
    fixtures.Reject("zero-count-at-buffer-end", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":0,"type":"VEC3"})",
        R"({"buffer":0,"byteOffset":64,"byteLength":0})"), "zero vertices");
    auto nonfinite = TriangleBytes();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(nonfinite.data(), &nan, sizeof(nan));
    fixtures.Reject("nonfinite-position", TriangleJson(), "non-finite value", nonfinite);
}

void CheckTransformsAndHierarchy(Fixtures& fixtures)
{
    // 각 모델 node의 matrix는 위치·회전·크기로 분해할 수 있어야 한다. 원근 변환, 회전과 크기로 나눌 수 없는 기울어진 행렬, 잘못된 크기나 quaternion, 부모 순환, 부모가 둘인 연결은 로더가 거부해야 한다.
    const std::string accessor = R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})";
    const std::string view = R"({"buffer":0,"byteLength":36})";
    const auto translated = GltfLoader::LoadGLB(fixtures.Write("valid-matrix", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[2,0,0,0,0,3,0,0,0,0,4,0,5,6,7,1]})")));
    EXPECT_NEAR((translated.nodes.front().translation.x), (5.0), (1e-5)) << "matrix local translation";
    EXPECT_NEAR((translated.nodes.front().scale.z), (4.0), (1e-5)) << "matrix local scale";
    fixtures.Reject("matrix-skew", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[1,0,0,0,0.25,1,0,0,0,0,1,0,0,0,0,1]})"), "non-degenerate TRS only");
    fixtures.Reject("matrix-perspective", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[1,0,0,0.25,0,1,0,0,0,0,1,0,0,0,0,1]})"), "Perspective node matrices");
    fixtures.Reject("matrix-degenerate", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]})"), "decompose glTF node matrix");
    fixtures.Reject("matrix-float-overflow", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[1e300,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]})"), "non-finite value");
    fixtures.Reject("trs-zero-scale", TriangleJson(accessor, view,
        R"({"mesh":0,"scale":[1,0,1]})"), "scale must be non-degenerate");
    fixtures.Reject("trs-zero-quaternion", TriangleJson(accessor, view,
        R"({"mesh":0,"rotation":[0,0,0,0]})"), "rotation quaternion is zero");
    fixtures.Reject("trs-translation-overflow", TriangleJson(accessor, view,
        R"({"mesh":0,"translation":[1e300,0,0]})"), "non-finite value");
    fixtures.Reject("trs-scale-overflow", TriangleJson(accessor, view,
        R"({"mesh":0,"scale":[1e300,1,1]})"), "non-finite value");
    fixtures.Reject("trs-rotation-overflow", TriangleJson(accessor, view,
        R"({"mesh":0,"rotation":[1e300,0,0,1]})"), "non-finite value");
    fixtures.Reject("node-self-cycle", TriangleJson(accessor, view,
        R"({"mesh":0,"children":[0]})"), "hierarchy contains a cycle");
    fixtures.Reject("node-two-cycle", TriangleJson(accessor, view,
        R"({"children":[1]},{"mesh":0,"children":[0]})"), "hierarchy contains a cycle");
    fixtures.Reject("node-multiple-parents", TriangleJson(accessor, view,
        R"({"children":[2]},{"children":[2]},{"mesh":0})"), "multiple parents");
    fixtures.Reject("node-disconnected-from-scene", TriangleJson(accessor, view,
        R"({"mesh":0},{"mesh":0})"), "single tree under the scene root");
}

/**
 * @brief glTF의 xyzw 순서로 Local TRS quaternion fixture를 만든다.
 * @details glTF는 quaternion을 (x,y,z,w) 순서로 기록하지만 GLM 생성자는 (w,x,y,z)를 받는다. 이 helper는 GLB 입력 형식의 순서로 직접 기록한다.
 * float 정밀도를 보존하므로 JSON 출력 과정의 오차가 위아래 기울기가 90도에 가까운 회전 검사를 가리지 않는다.
 */
std::string QuaternionTrsNode(const glm::vec3& position, const glm::quat& rotation, const glm::vec3& scale)
{
    std::ostringstream json;
    json << std::setprecision(std::numeric_limits<float>::max_digits10)
         << "{\"mesh\":0,\"translation\":[" << position.x << ',' << position.y << ',' << position.z
         << "],\"rotation\":[" << rotation.x << ',' << rotation.y << ',' << rotation.z << ',' << rotation.w
         << "],\"scale\":[" << scale.x << ',' << scale.y << ',' << scale.z << "]}";
    return json.str();
}

/** @brief GLM의 열 벡터 행렬을 glTF가 요구하는 열 우선 배열 순서로 시험 자료에 기록한다. */
std::string MatrixNode(const glm::mat4& matrix)
{
    std::ostringstream json;
    json << std::setprecision(std::numeric_limits<float>::max_digits10) << "{\"mesh\":0,\"matrix\":[";
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
        {
            if (column != 0 || row != 0) json << ',';
            json << matrix[column][row];
        }
    json << "]}";
    return json.str();
}

void VerifyNodeQuaternion(const NodeData& node, const glm::quat& expected, const std::string& label)
{
    EXPECT_NEAR((glm::length(node.rotation)), (1.0), (1e-5)) << label + " unit quaternion";
    // quaternion의 부호를 뒤집어도 같은 회전이다. 행렬을 quaternion으로 분해할 때 라이브러리가 어느 부호를 고르는지에 시험이 좌우되지 않게 한다.
    EXPECT_NEAR((std::abs(glm::dot(node.rotation, glm::normalize(expected)))), (1.0), (1e-5)) << label + " quaternion direction";
}

void VerifyNodeMatrix(const NodeData& node, const glm::mat4& expected, const std::string& label)
{
    const glm::mat4 actual = glm::translate(glm::mat4(1.0F), node.translation) *
        glm::mat4_cast(node.rotation) * glm::scale(glm::mat4(1.0F), node.scale);
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            EXPECT_NEAR((actual[column][row]), (expected[column][row]), (1e-5)) << label + " [" + std::to_string(column) + "][" + std::to_string(row) + "]";
}

/**
 * @brief GLB TRS와 matrix 입력이 quaternion 방향과 Local 행렬을 보존하는지 확인한다.
 * @details 기대 회전은 각 축의 회전을 quaternion끼리 직접 곱해 만든다. 세 개의 각도로 되돌리지 않고 기울기가 90도보다 조금 작거나 큰 복합 회전도 방향이 유지되는지 확인한다.
 * GLB의 xyzw 순서, 성분 부호만 반대인 같은 방향, 길이가 1이 아닌 입력도 함께 검사한다.
 */
void CheckQuaternionTransforms(Fixtures& fixtures)
{
    const std::string accessor = R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})";
    const std::string view = R"({"buffer":0,"byteLength":36})";
    const glm::vec3 position{0.8F, -1.2F, 0.4F};
    const glm::vec3 scale{1.5F, 0.7F, 2.0F};
    const glm::quat ordered = glm::normalize(glm::quat{0.9F, 0.1F, -0.2F, 0.3F});
    const auto orderedResource = GltfLoader::LoadGLB(fixtures.Write("quaternion-xyzw",
        TriangleJson(accessor, view, QuaternionTrsNode(position, ordered, scale))));
    const glm::quat actualOrdered = orderedResource.nodes.front().rotation;
    EXPECT_NEAR((actualOrdered.w), (ordered.w), (1e-6)) << "glTF xyzw maps w to quaternion scalar";
    EXPECT_NEAR((actualOrdered.x), (ordered.x), (1e-6)) << "glTF xyzw maps x";
    EXPECT_NEAR((actualOrdered.y), (ordered.y), (1e-6)) << "glTF xyzw maps y";
    EXPECT_NEAR((actualOrdered.z), (ordered.z), (1e-6)) << "glTF xyzw maps z";

    const auto defaults = GltfLoader::LoadGLB(fixtures.Write("quaternion-default", TriangleJson()));
    VerifyNodeQuaternion(defaults.nodes.front(), glm::quat{1.0F, 0.0F, 0.0F, 0.0F}, "default rotation");

    const float halfPi = std::acos(-1.0F) * 0.5F;
    int fixtureIndex = 0;
    for (const float pitch : {halfPi - 1.0e-5F, halfPi, halfPi + 1.0e-5F})
    {
        const glm::quat expectedRotation = glm::normalize(
            glm::angleAxis(-0.8F, glm::vec3{0.0F, 0.0F, 1.0F}) *
            glm::angleAxis(pitch, glm::vec3{0.0F, 1.0F, 0.0F}) *
            glm::angleAxis(0.4F, glm::vec3{1.0F, 0.0F, 0.0F}));
        const glm::mat4 expectedMatrix = glm::translate(glm::mat4(1.0F), position) *
            glm::mat4_cast(expectedRotation) * glm::scale(glm::mat4(1.0F), scale);
        for (const float multiplier : {1.0F, -1.0F, 7.0F})
        {
            const std::string name = "quaternion-trs-" + std::to_string(fixtureIndex++);
            const auto resource = GltfLoader::LoadGLB(fixtures.Write(name, TriangleJson(accessor, view,
                QuaternionTrsNode(position, expectedRotation * multiplier, scale))));
            VerifyNodeQuaternion(resource.nodes.front(), expectedRotation, name);
            VerifyNodeMatrix(resource.nodes.front(), expectedMatrix, name + " Local TRS");
        }
        const std::string name = "quaternion-matrix-" + std::to_string(fixtureIndex++);
        const auto resource = GltfLoader::LoadGLB(fixtures.Write(name,
            TriangleJson(accessor, view, MatrixNode(expectedMatrix))));
        VerifyNodeQuaternion(resource.nodes.front(), expectedRotation, name);
        VerifyNodeMatrix(resource.nodes.front(), expectedMatrix, name + " decomposed Local TRS");
    }
}
}

TEST(GltfLoader, RepositoryAssets)
{
    CheckRepositoryAssets();
}

TEST(GltfLoader, Accessors)
{
    Fixtures fixtures;
    CheckAccessors(fixtures);
}

TEST(GltfLoader, TransformsAndHierarchy)
{
    Fixtures fixtures;
    CheckTransformsAndHierarchy(fixtures);
}

TEST(GltfLoader, QuaternionTransforms)
{
    Fixtures fixtures;
    CheckQuaternionTransforms(fixtures);
}
