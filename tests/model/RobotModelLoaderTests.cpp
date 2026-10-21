#include "model/RobotModel.h"
#include "model/GltfLoader.h"

#include <gtest/gtest.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
constexpr std::size_t glbJsonLengthOffset = 12U;
constexpr std::size_t glbJsonChunkStart = 20U;

std::filesystem::path WriteModel(const std::string& body)
{
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto path = std::filesystem::temp_directory_path() /
        ("grasplink-robot-" + std::to_string(stamp) + ".urdf");
    std::ofstream output(path);
    output << "<robot name=\"test\">" << body << "</robot>";
    output.close();
    return path;
}

class TemporaryUrdf
{
public:
    explicit TemporaryUrdf(std::string body) : path_(WriteModel(body)) {}
    ~TemporaryUrdf() { std::error_code ignored; std::filesystem::remove(path_, ignored); }
    const std::filesystem::path& Path() const { return path_; }

private:
    std::filesystem::path path_;
};

class TemporaryGlb
{
public:
    explicit TemporaryGlb(const std::filesystem::path& sourcePath)
    {
        std::ifstream input(sourcePath, std::ios::binary);
        std::string bytes{std::istreambuf_iterator<char>{input}, {}};
        if (input.bad() || bytes.size() < glbJsonChunkStart)
            throw std::runtime_error("Could not read the GLB test asset");

        const auto byteAt = [&](std::size_t offset)
        {
            return static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[offset]));
        };
        const std::uint32_t jsonLength = byteAt(glbJsonLengthOffset) |
            (byteAt(glbJsonLengthOffset + 1U) << 8U) |
            (byteAt(glbJsonLengthOffset + 2U) << 16U) |
            (byteAt(glbJsonLengthOffset + 3U) << 24U);
        if (jsonLength > bytes.size() - glbJsonChunkStart)
            throw std::runtime_error("The GLB test asset has an invalid JSON chunk");

        constexpr std::string_view original = "\"byteLength\":96";
        const std::string_view json{bytes.data() + glbJsonChunkStart, jsonLength};
        const std::size_t field = json.find(original);
        if (field == std::string_view::npos)
            throw std::runtime_error("The GLB test asset changed unexpectedly");

        // 정점 자료는 그대로 두고 BufferView를 1바이트 짧게 선언해 범위 검사를 확인한다.
        bytes[glbJsonChunkStart + field + original.size() - 1] = '5';
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path() /
            ("grasplink-invalid-accessor-" + std::to_string(stamp) + ".glb");
        std::ofstream output(path_, std::ios::binary);
        output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        if (!output)
            throw std::runtime_error("Could not write the malformed GLB test asset");
    }

    ~TemporaryGlb()
    {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }

    const std::filesystem::path& Path() const { return path_; }

private:
    std::filesystem::path path_;
};

class TemporaryRobotDirectory
{
public:
    explicit TemporaryRobotDirectory(const std::filesystem::path& source)
    {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path() /
            ("grasplink-duplicate-node-" + std::to_string(stamp));
        std::filesystem::create_directory(path_);
        std::filesystem::copy_file(source / "robot.urdf", path_ / "robot.urdf");

        std::ifstream input(source / "robot.glb", std::ios::binary);
        std::string bytes{std::istreambuf_iterator<char>{input}, {}};
        const std::string rootNode = "\"name\":\"RobotRoot\"";
        const std::size_t nodeName = bytes.find(rootNode);
        if (input.bad() || nodeName == std::string::npos)
            throw std::runtime_error("The axis test model has no expected GLB root node");
        bytes.replace(nodeName, rootNode.size(), "\"name\":\"base_link\"");

        std::ofstream output(path_ / "robot.glb", std::ios::binary);
        output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        if (!output)
            throw std::runtime_error("Could not write the temporary robot model");
    }

    ~TemporaryRobotDirectory()
    {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }

    [[nodiscard]] const std::filesystem::path& Path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

std::string Link(const std::string& name)
{
    if (name == "base_link" || name == "tool0")
        return name == "base_link"
            ? "<link name=\"base_link\"><collision><geometry><box size=\"0.2 0.2 0.2\"/>"
                "</geometry></collision></link>"
            : "<link name=\"tool0\"/>";
    return "<link name=\"" + name + "\"><collision><geometry>"
        "<box size=\"0.1 0.1 0.2\"/></geometry></collision></link>";
}

std::string RevoluteJoint(const std::string& name, const std::string& parent,
    const std::string& child)
{
    return "<joint name=\"" + name + "\" type=\"revolute\"><parent link=\"" + parent +
        "\"/><child link=\"" + child + "\"/><axis xyz=\"0 0 1\"/>"
        "<limit lower=\" -2 \" upper=\" 2 \" velocity=\" 1 \"/></joint>";
}

std::string SerialArm(std::size_t count)
{
    std::string xml = Link("base_link");
    for (std::size_t i = 0; i < count; ++i)
        xml += Link("link" + std::to_string(i));
    xml += Link("tool0");
    std::string parent = "base_link";
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::string child = "link" + std::to_string(i);
        xml += RevoluteJoint("joint" + std::to_string(i), parent, child);
        parent = child;
    }
    xml += "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"" + parent +
        "\"/><child link=\"tool0\"/></joint>";
    return xml;
}
}

TEST(RobotModelLoader, LoadsDifferentJointCountsAndBundledRobot)
{
    for (const std::size_t count : {4U, 6U, 7U})
    {
        TemporaryUrdf file{SerialArm(count)};
        const auto model = grasplink::model::RobotModelLoader::LoadUrdf(file.Path());
        EXPECT_EQ(model.activeJoints.size(), count);
        EXPECT_EQ(model.links[model.baseLink].name, "base_link");
        EXPECT_EQ(model.links[model.toolLink].name, "tool0");

        const auto testModel = grasplink::model::RobotModelLoader::Load(
            std::filesystem::path(GRASPLINK_SOURCE_DIR) / "tests/assets/robots" /
                ("axis" + std::to_string(count)));
        EXPECT_EQ(testModel.robot.activeJoints.size(), count);
        ASSERT_TRUE(testModel.visual.rootNodeIndex.has_value());
        EXPECT_FALSE(testModel.visual.nodes.empty());
        EXPECT_FALSE(testModel.visual.meshes.empty());
        for (const auto& joint : testModel.robot.joints)
            EXPECT_EQ(joint.visualNodeIndex.has_value(),
                joint.type != grasplink::model::RobotJointType::Fixed);
    }

    TemporaryRobotDirectory duplicateUnrelatedName{
        std::filesystem::path(GRASPLINK_SOURCE_DIR) / "tests/assets/robots/axis4"};
    const auto duplicateNameModel = grasplink::model::RobotModelLoader::Load(
        duplicateUnrelatedName.Path());
    EXPECT_EQ(duplicateNameModel.robot.activeJoints.size(), 4U);

    TemporaryUrdf file{"<link name=\"base_link\"><collision><geometry><box size=\"0.2 0.2 0.2\"/>"
        "</geometry></collision></link><link name=\"arm\"><collision><origin xyz=\"0 0 0.1\"/>"
        "<geometry><sphere radius=\"0.05\"/></geometry></collision><collision><geometry>"
        "<cylinder radius=\"0.02\" length=\"0.1\"/></geometry></collision></link>"
        "<link name=\"tool0\"/><joint name=\"joint\" type=\"revolute\"><parent link=\"base_link\"/>"
        "<child link=\"arm\"/><axis xyz=\"0 0 1\"/><limit lower=\"-1\" upper=\"1\" velocity=\"1\"/>"
        "</joint><joint name=\"fixed\" type=\"fixed\"><parent link=\"arm\"/><child link=\"tool0\"/>"
        "</joint>"};
    const auto model = grasplink::model::RobotModelLoader::LoadUrdf(file.Path());
    const auto& collisions = model.links[model.joints[model.activeJoints.front()].childLink].collisionGeometry;
    ASSERT_EQ(collisions.size(), 2U);
    EXPECT_EQ(collisions[0].type, grasplink::model::RobotCollisionType::Sphere);
    EXPECT_DOUBLE_EQ(collisions[0].positionMeters.z, 0.1);
    EXPECT_EQ(collisions[1].type, grasplink::model::RobotCollisionType::Cylinder);

    const auto loaded = grasplink::model::RobotModelLoader::Load(
        std::filesystem::path(GRASPLINK_SOURCE_DIR) / "assets/robots/HCR12A");
    EXPECT_EQ(loaded.robot.activeJoints.size(), 6U);
    EXPECT_FALSE(loaded.visual.nodes.empty());
    ASSERT_FALSE(loaded.visual.meshes.empty());
    const auto& firstMesh = loaded.visual.meshes.front();
    EXPECT_FALSE(firstMesh.vertices.empty());
    EXPECT_FALSE(firstMesh.indices.empty());
    ASSERT_FALSE(firstMesh.subMeshes.empty());
    EXPECT_EQ(firstMesh.subMeshes.front().indexStart, 0U);
    EXPECT_GT(firstMesh.subMeshes.front().indexCount, 0U);
    ASSERT_TRUE(loaded.visual.rootNodeIndex.has_value());
    ASSERT_LT(*loaded.visual.rootNodeIndex, loaded.visual.nodes.size());
    EXPECT_FALSE(loaded.visual.nodes[*loaded.visual.rootNodeIndex].parentIndex.has_value());
    EXPECT_FALSE(loaded.robot.links[loaded.robot.baseLink].visualNodeIndex.has_value());
    EXPECT_FALSE(loaded.robot.links[loaded.robot.toolLink].visualNodeIndex.has_value());
    for (const auto& joint : loaded.robot.joints)
        EXPECT_EQ(joint.visualNodeIndex.has_value(), joint.type != grasplink::model::RobotJointType::Fixed);
}

TEST(RobotModelLoader, ReadsSupportedJointTypesAndRejectsMalformedModels)
{
    std::string xml = Link("base_link") + Link("slide") + Link("turn") + Link("tool0");
    xml += "<joint name=\"linear\" type=\"prismatic\"><parent link=\"base_link\"/>"
        "<child link=\"slide\"/><axis xyz=\"0 1 0\"/>"
        "<limit lower=\"0\" upper=\"0.4\" velocity=\"0.2\"/></joint>";
    xml += "<joint name=\"spin\" type=\"continuous\"><parent link=\"slide\"/>"
        "<child link=\"turn\"/><axis xyz=\"0 0 2\"/>"
        "<limit velocity=\"1.5\"/></joint>";
    xml += "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"turn\"/>"
        "<child link=\"tool0\"/></joint>";

    TemporaryUrdf file{xml};
    const auto model = grasplink::model::RobotModelLoader::LoadUrdf(file.Path());
    ASSERT_EQ(model.activeJoints.size(), 2U);
    EXPECT_EQ(model.joints[model.activeJoints[0]].type, grasplink::model::RobotJointType::Prismatic);
    EXPECT_EQ(model.joints[model.activeJoints[1]].type, grasplink::model::RobotJointType::Continuous);
    EXPECT_TRUE(std::isinf(model.joints[model.activeJoints[1]].lowerLimit));
    EXPECT_TRUE(std::isinf(model.joints[model.activeJoints[1]].upperLimit));
    EXPECT_NEAR(glm::length(model.joints[model.activeJoints[1]].axis), 1.0, 1e-12);

    const std::string missingAxisXml = Link("base_link") + Link("arm") + Link("tool0") +
        "<joint name=\"arm_joint\" type=\"revolute\"><parent link=\"base_link\"/>"
        "<child link=\"arm\"/><limit lower=\"-1\" upper=\"1\" velocity=\"1\"/></joint>" +
        "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"arm\"/>"
        "<child link=\"tool0\"/></joint>";
    const auto expectRejected = [](const std::string& xml)
    {
        TemporaryUrdf file{xml};
        EXPECT_THROW(grasplink::model::RobotModelLoader::LoadUrdf(file.Path()), std::runtime_error);
    };
    const std::string emptyNumericAttributeXml = Link("base_link") + Link("arm") + Link("tool0") +
        "<joint name=\"arm_joint\" type=\"revolute\"><parent link=\"base_link\"/>"
        "<child link=\"arm\"/><axis xyz=\"0 0 1\"/>"
        "<limit lower=\"   \" upper=\"1\" velocity=\"1\"/></joint>" +
        "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"arm\"/>"
        "<child link=\"tool0\"/></joint>";
    expectRejected(emptyNumericAttributeXml);
    expectRejected(Link("base_link") + Link("base_link") + Link("tool0"));
    expectRejected(Link("base_link") + Link("tool0") +
        "<joint name=\"broken\" type=\"fixed\"><parent link=\"base_link\"/>"
        "<child link=\"missing\"/></joint>");
    expectRejected(missingAxisXml);

    const std::string zeroAxisXml = Link("base_link") + Link("arm") + Link("tool0") +
        "<joint name=\"arm_joint\" type=\"revolute\"><parent link=\"base_link\"/>"
        "<child link=\"arm\"/><axis xyz=\"0 0 0\"/>"
        "<limit lower=\"-1\" upper=\"1\" velocity=\"1\"/></joint>" +
        "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"arm\"/>"
        "<child link=\"tool0\"/></joint>";
    expectRejected(zeroAxisXml);

    const std::string reversedLimitXml = Link("base_link") + Link("arm") + Link("tool0") +
        "<joint name=\"arm_joint\" type=\"revolute\"><parent link=\"base_link\"/>"
        "<child link=\"arm\"/><axis xyz=\"0 0 1\"/>"
        "<limit lower=\"1\" upper=\"-1\" velocity=\"1\"/></joint>" +
        "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"arm\"/>"
        "<child link=\"tool0\"/></joint>";
    expectRejected(reversedLimitXml);

    const std::string zeroCollisionDimensionXml =
        "<link name=\"base_link\"><collision><geometry><box size=\"0 1 1\"/>"
        "</geometry></collision></link>" + Link("arm") + Link("tool0") +
        "<joint name=\"arm_joint\" type=\"revolute\"><parent link=\"base_link\"/>"
        "<child link=\"arm\"/><axis xyz=\"0 0 1\"/>"
        "<limit lower=\"-1\" upper=\"1\" velocity=\"1\"/></joint>" +
        "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"arm\"/>"
        "<child link=\"tool0\"/></joint>";
    expectRejected(zeroCollisionDimensionXml);

    const std::string disconnectedLinkXml = Link("base_link") + Link("arm") +
        Link("tool0") + Link("orphan") + RevoluteJoint("arm_joint", "base_link", "arm") +
        "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"arm\"/>"
        "<child link=\"tool0\"/></joint>";
    expectRejected(disconnectedLinkXml);

    const std::string branchedLinkXml = Link("base_link") + Link("arm") +
        Link("side_link") + Link("tool0") + RevoluteJoint("arm_joint", "base_link", "arm") +
        "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"arm\"/>"
        "<child link=\"tool0\"/></joint><joint name=\"side_fixed\" type=\"fixed\">"
        "<parent link=\"arm\"/><child link=\"side_link\"/></joint>";
    expectRejected(branchedLinkXml);

    const std::string cycleXml = Link("base_link") + Link("arm") + Link("tool0") +
        RevoluteJoint("arm_joint", "base_link", "arm") +
        RevoluteJoint("return_joint", "arm", "base_link") +
        "<joint name=\"tool_fixed\" type=\"fixed\"><parent link=\"base_link\"/>"
        "<child link=\"tool0\"/></joint>";
    expectRejected(cycleXml);

    const auto modelPath = std::filesystem::path(GRASPLINK_SOURCE_DIR) /
        "tests/assets/robots/axis4/robot.glb";
    TemporaryGlb malformedModel{modelPath};
    EXPECT_THROW(grasplink::model::GltfLoader::LoadGLB(malformedModel.Path()), std::runtime_error);
}
