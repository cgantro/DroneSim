#include "model/RobotModel.h"

#include "model/GltfLoader.h"

#include <tinyxml2.h>

#include <charconv>
#include <cctype>
#include <cmath>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace grasplink::model
{
namespace
{
using tinyxml2::XMLElement;
using LinkIndexByName = std::unordered_map<std::string, std::size_t>;
constexpr double kMinimumJointAxisLength = 1.0e-12;

// 모든 로봇 폴더가 따르는 파일 이름과 팔 경로의 기준 링크 이름이다.
namespace robot_model_format
{
inline constexpr char urdfFile[] = "robot.urdf";
inline constexpr char glbFile[] = "robot.glb";
inline constexpr char baseLink[] = "base_link";
inline constexpr char toolLink[] = "tool0";
}

[[noreturn]] void Invalid(const std::filesystem::path& path, const std::string& message)
{
    throw std::runtime_error("Robot model '" + path.string() + "': " + message);
}

const char* RequiredAttribute(const XMLElement& element, const char* name,
    const std::filesystem::path& path)
{
    const char* value = element.Attribute(name);
    if (value == nullptr || *value == '\0')
        Invalid(path, std::string("URDF attribute '" ) + name + "' is missing");
    return value;
}

// URDF는 점을 소수점으로 쓰며, double 범위를 벗어난 숫자는 0으로 바꾸지 않고 거부한다.
double Number(const char* value, const std::filesystem::path& path, const std::string& field)
{
    if (value == nullptr)
        Invalid(path, field + " is missing");

    std::string_view text{value};
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.front())))
        text.remove_prefix(1);
    while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back())))
        text.remove_suffix(1);
    if (!text.empty() && text.front() == '+')
        text.remove_prefix(1);

    double result = 0.0;
    const auto [end, error] = std::from_chars(
        text.data(), text.data() + text.size(), result, std::chars_format::general);
    if (error != std::errc{} || end != text.data() + text.size() || !std::isfinite(result))
        Invalid(path, field + " must be a finite number");
    return result;
}

glm::dvec3 Vector3(const char* value, const std::filesystem::path& path,
    const std::string& field)
{
    if (value == nullptr)
        return glm::dvec3{0.0};
    std::istringstream input(value);
    input.imbue(std::locale::classic());
    glm::dvec3 result;
    if (!(input >> result.x >> result.y >> result.z))
        Invalid(path, field + " must contain three numbers");
    std::string extra;
    if (input >> extra || !std::isfinite(result.x) || !std::isfinite(result.y) ||
        !std::isfinite(result.z))
        Invalid(path, field + " must contain exactly three finite numbers");
    return result;
}

glm::dquat RotationFromRpy(const glm::dvec3& rpy)
{
    // URDF의 RPY는 roll, pitch, yaw 순서다. 각 회전을 x·y·z축에 적용하므로 합성 순서는 z·y·x다.
    const glm::dquat x{std::cos(rpy.x * 0.5), std::sin(rpy.x * 0.5), 0.0, 0.0};
    const glm::dquat y{std::cos(rpy.y * 0.5), 0.0, std::sin(rpy.y * 0.5), 0.0};
    const glm::dquat z{std::cos(rpy.z * 0.5), 0.0, 0.0, std::sin(rpy.z * 0.5)};
    return glm::normalize(z * y * x);
}

bool HasValidCollisionDimensions(const RobotCollisionGeometry& geometry)
{
    const auto& dimensions = geometry.dimensionsMeters;
    if (!std::isfinite(dimensions.x) || !std::isfinite(dimensions.y) ||
        !std::isfinite(dimensions.z) || dimensions.x <= 0.0)
        return false;

    switch (geometry.type)
    {
    case RobotCollisionType::Box:
        return dimensions.y > 0.0 && dimensions.z > 0.0;
    case RobotCollisionType::Sphere:
        return true;
    case RobotCollisionType::Cylinder:
        return dimensions.y > 0.0;
    }
    return false;
}

std::size_t FindLink(const LinkIndexByName& linkIndices, const std::string& name,
    const std::filesystem::path& path)
{
    const auto found = linkIndices.find(name);
    if (found == linkIndices.end())
        Invalid(path, "joint refers to unknown link '" + name + "'");
    return found->second;
}

bool IsDescendant(const ModelResource& visual, std::size_t node, std::size_t ancestor)
{
    while (node < visual.nodes.size())
    {
        if (node == ancestor)
            return true;
        const auto parent = visual.nodes[node].parentIndex;
        if (!parent)
            break;
        node = *parent;
    }
    return false;
}

void ConnectVisualTree(RobotModel& robot, const ModelResource& visual,
    const std::filesystem::path& path)
{
    std::unordered_map<std::string, std::size_t> nodeByName;
    std::unordered_set<std::string> duplicateNodeNames;
    for (std::size_t i = 0; i < visual.nodes.size(); ++i)
    {
        const auto& node = visual.nodes[i];
        if (!nodeByName.emplace(node.name, i).second)
            duplicateNodeNames.insert(node.name);
    }

    // 중복 이름은 로봇 링크나 관절을 찾을 때만 문제가 된다. 그 외 GLB 부품 이름은 겹쳐도 된다.
    const auto findNode = [&](const std::string& name, const char* missingMessage)
    {
        if (duplicateNodeNames.find(name) != duplicateNodeNames.end())
            Invalid(path, "GLB node name is ambiguous for URDF item '" + name + "'");
        const auto found = nodeByName.find(name);
        if (found == nodeByName.end())
            Invalid(path, std::string{missingMessage} + name + "'");
        return found->second;
    };

    for (auto& link : robot.links)
    {
        if (link.name == robot_model_format::baseLink || link.name == robot_model_format::toolLink)
            continue;
        link.visualNodeIndex = findNode(link.name, "GLB has no node for URDF link '");
    }

    for (auto& joint : robot.joints)
    {
        if (joint.type == RobotJointType::Fixed)
            continue;
        joint.visualNodeIndex = findNode(joint.name, "GLB has no pivot node for URDF joint '");

        const auto& parentNode = robot.links[joint.parentLink].visualNodeIndex;
        const auto& childNode = robot.links[joint.childLink].visualNodeIndex;
        if (parentNode && visual.nodes[*joint.visualNodeIndex].parentIndex != parentNode)
            Invalid(path, "GLB joint '" + joint.name + "' must be a direct child of its parent link");
        if (childNode && !IsDescendant(visual, *childNode, *joint.visualNodeIndex))
            Invalid(path, "GLB child link '" + robot.links[joint.childLink].name +
                "' must be below its joint pivot '" + joint.name + "'");
    }
}

struct LinkTree
{
    std::vector<std::optional<std::size_t>> parentJoint;
    std::vector<std::vector<std::size_t>> childJoints;
};

LinkTree BuildLinkTree(const RobotModel& model, const std::filesystem::path& path)
{
    LinkTree tree{
        std::vector<std::optional<std::size_t>>(model.links.size()),
        std::vector<std::vector<std::size_t>>(model.links.size())};
    for (std::size_t i = 0; i < model.joints.size(); ++i)
    {
        // 각 자식 링크는 부모 관절을 하나만 가져야 한다. 역방향 탐색과 하위 링크 순회를 위해 양쪽 색인을 함께 만든다.
        const auto parent = model.joints[i].parentLink;
        const auto child = model.joints[i].childLink;
        if (tree.parentJoint[child])
            Invalid(path, "a link cannot have more than one parent joint");
        tree.parentJoint[child] = i;
        tree.childJoints[parent].push_back(i);
    }
    return tree;
}

void ValidateLinksReachableFromBase(const RobotModel& model, const LinkTree& tree,
    const std::filesystem::path& path)
{
    // base_link부터 관절 트리를 한 번 순회해 순환과 연결되지 않은 링크를 함께 찾는다.
    std::vector<bool> reached(model.links.size(), false);
    std::vector<std::size_t> pending{model.baseLink};
    reached[model.baseLink] = true;
    for (std::size_t next = 0; next < pending.size(); ++next)
    {
        for (const std::size_t jointIndex : tree.childJoints[pending[next]])
        {
            const std::size_t child = model.joints[jointIndex].childLink;
            if (reached[child])
                Invalid(path, "the link tree contains a cycle");
            reached[child] = true;
            pending.push_back(child);
        }
    }
    for (std::size_t linkIndex = 0; linkIndex < reached.size(); ++linkIndex)
        if (!reached[linkIndex])
            Invalid(path, "link '" + model.links[linkIndex].name + "' is not connected to base_link");
}

std::vector<std::size_t> FindActiveJoints(const RobotModel& model, const LinkTree& tree,
    const std::filesystem::path& path)
{
    // tool0에서 부모 쪽으로 거슬러 올라가므로, 제어 순서에 맞게 뒤집기 전까지는 관절 번호를 역순으로 모은다.
    std::vector<std::size_t> reversePath;
    std::vector<bool> linksOnArmPath(model.links.size(), false);
    std::size_t current = model.toolLink;
    linksOnArmPath[current] = true;
    while (current != model.baseLink)
    {
        const auto parent = tree.parentJoint[current];
        if (!parent)
            Invalid(path, "tool0 is not connected to base_link");
        const auto& joint = model.joints[*parent];
        if (joint.type != RobotJointType::Fixed)
            reversePath.push_back(*parent);
        current = joint.parentLink;
        linksOnArmPath[current] = true;
    }

    for (std::size_t linkIndex = 0; linkIndex < linksOnArmPath.size(); ++linkIndex)
        if (!linksOnArmPath[linkIndex])
            Invalid(path, "link '" + model.links[linkIndex].name +
                "' is outside the base_link to tool0 path; branching is not supported");

    if (reversePath.empty())
        Invalid(path, "the base_link to tool0 path has no movable joints");
    return {reversePath.rbegin(), reversePath.rend()};
}

void FindActiveChain(RobotModel& model, const LinkIndexByName& linkIndices,
    const std::filesystem::path& path)
{
    model.baseLink = FindLink(linkIndices, robot_model_format::baseLink, path);
    model.toolLink = FindLink(linkIndices, robot_model_format::toolLink, path);
    if (model.baseLink == model.toolLink)
        Invalid(path, "base_link and tool0 must be different links");

    const LinkTree tree = BuildLinkTree(model, path);
    ValidateLinksReachableFromBase(model, tree, path);
    model.activeJoints = FindActiveJoints(model, tree, path);
}

void ReadCollisionShape(const XMLElement& shape, const std::string& linkName,
    const std::filesystem::path& path, RobotCollisionGeometry& result)
{
    const std::string shapeType = shape.Name();
    if (shapeType == "box")
    {
        result.type = RobotCollisionType::Box;
        result.dimensionsMeters = Vector3(RequiredAttribute(shape, "size", path), path, "box size");
    }
    else if (shapeType == "sphere")
    {
        result.type = RobotCollisionType::Sphere;
        result.dimensionsMeters = glm::dvec3{Number(shape.Attribute("radius"), path, "sphere radius")};
    }
    else if (shapeType == "cylinder")
    {
        result.type = RobotCollisionType::Cylinder;
        result.dimensionsMeters = {
            Number(shape.Attribute("radius"), path, "cylinder radius"),
            Number(shape.Attribute("length"), path, "cylinder length"), 0.0};
    }
    else
        Invalid(path, "link '" + linkName + "' collision must use box, sphere, or cylinder geometry");

    if (!HasValidCollisionDimensions(result))
        Invalid(path, "link '" + linkName + "' collision dimensions must be positive");
}

RobotCollisionGeometry ReadCollisionGeometry(
    const XMLElement& collision, const std::string& linkName, const std::filesystem::path& path)
{
    RobotCollisionGeometry result;
    if (const XMLElement* origin = collision.FirstChildElement("origin"))
    {
        result.positionMeters = Vector3(origin->Attribute("xyz"), path, "collision origin xyz");
        result.rotation = RotationFromRpy(Vector3(origin->Attribute("rpy"), path, "collision origin rpy"));
    }

    const XMLElement* geometry = collision.FirstChildElement("geometry");
    if (geometry == nullptr)
        Invalid(path, "link '" + linkName + "' collision has no geometry");
    const XMLElement* shape = geometry->FirstChildElement();
    if (shape == nullptr || shape->NextSiblingElement() != nullptr)
        Invalid(path, "link '" + linkName + "' collision must contain exactly one shape");

    // 세 모양의 치수는 모두 미터로 읽어 충돌 형상 계산에 전달합니다.
    ReadCollisionShape(*shape, linkName, path, result);
    return result;
}

RobotLink ReadLink(const XMLElement& element, const std::filesystem::path& path)
{
    RobotLink link;
    link.name = RequiredAttribute(element, "name", path);
    for (const XMLElement* collision = element.FirstChildElement("collision"); collision != nullptr;
        collision = collision->NextSiblingElement("collision"))
        link.collisionGeometry.push_back(ReadCollisionGeometry(*collision, link.name, path));

    if (link.name != robot_model_format::toolLink && link.collisionGeometry.empty())
        Invalid(path, "link '" + link.name + "' has no collision geometry");
    return link;
}

RobotJointType ReadJointType(const std::string& type, const std::string& name,
    const std::filesystem::path& path)
{
    if (type == "fixed") return RobotJointType::Fixed;
    if (type == "revolute") return RobotJointType::Revolute;
    if (type == "continuous") return RobotJointType::Continuous;
    if (type == "prismatic") return RobotJointType::Prismatic;
    Invalid(path, "joint '" + name + "' uses unsupported type '" + type + "'");
}

void ReadJointAxisAndLimits(
    RobotJoint& joint, const XMLElement& element, const std::filesystem::path& path)
{
    const XMLElement* axis = element.FirstChildElement("axis");
    if (axis == nullptr || axis->Attribute("xyz") == nullptr)
        Invalid(path, "joint '" + joint.name + "' must define its axis");
    joint.axis = Vector3(axis->Attribute("xyz"), path, "joint axis");
    const double axisLength = glm::length(joint.axis);
    if (!std::isfinite(axisLength) || axisLength <= kMinimumJointAxisLength)
        Invalid(path, "joint '" + joint.name + "' has a zero axis");
    joint.axis /= axisLength;

    const XMLElement* limit = element.FirstChildElement("limit");
    if (limit == nullptr)
        Invalid(path, "joint '" + joint.name + "' has no limits");
    joint.maximumVelocity = Number(limit->Attribute("velocity"), path,
        "joint '" + joint.name + "' velocity limit");
    if (joint.maximumVelocity <= 0.0)
        Invalid(path, "joint '" + joint.name + "' velocity limit must be positive");

    if (joint.type == RobotJointType::Continuous)
    {
        joint.lowerLimit = -std::numeric_limits<double>::infinity();
        joint.upperLimit = std::numeric_limits<double>::infinity();
        return;
    }

    joint.lowerLimit = Number(limit->Attribute("lower"), path,
        "joint '" + joint.name + "' lower limit");
    joint.upperLimit = Number(limit->Attribute("upper"), path,
        "joint '" + joint.name + "' upper limit");
    if (joint.lowerLimit > joint.upperLimit)
        Invalid(path, "joint '" + joint.name + "' lower limit exceeds its upper limit");
}

RobotJoint ReadJoint(const XMLElement& element, const LinkIndexByName& linkIndices,
    const std::filesystem::path& path)
{
    RobotJoint joint;
    joint.name = RequiredAttribute(element, "name", path);
    joint.type = ReadJointType(RequiredAttribute(element, "type", path), joint.name, path);

    const XMLElement* parent = element.FirstChildElement("parent");
    const XMLElement* child = element.FirstChildElement("child");
    if (parent == nullptr || child == nullptr)
        Invalid(path, "joint '" + joint.name + "' must name a parent and child link");
    joint.parentLink = FindLink(linkIndices, RequiredAttribute(*parent, "link", path), path);
    joint.childLink = FindLink(linkIndices, RequiredAttribute(*child, "link", path), path);

    if (const XMLElement* origin = element.FirstChildElement("origin"))
    {
        joint.originPositionMeters = Vector3(origin->Attribute("xyz"), path, "joint origin xyz");
        joint.originRotation = RotationFromRpy(Vector3(origin->Attribute("rpy"), path, "joint origin rpy"));
    }
    if (joint.type != RobotJointType::Fixed)
        ReadJointAxisAndLimits(joint, element, path);
    return joint;
}

void ReadLinks(const XMLElement& root, RobotModel& model, LinkIndexByName& linkIndices,
    const std::filesystem::path& path)
{
    // URDF 관절은 링크를 이름으로 가리키지만 FK와 충돌 계산은 링크 번호를 사용합니다.
    // 그래서 관절을 읽기 전에 링크를 등록하고, 이름에서 번호를 찾을 수 있게 보관합니다.
    for (const XMLElement* element = root.FirstChildElement("link"); element != nullptr;
        element = element->NextSiblingElement("link"))
    {
        RobotLink link = ReadLink(*element, path);
        if (!linkIndices.emplace(link.name, model.links.size()).second)
            Invalid(path, "duplicate link name '" + link.name + "'");
        model.links.push_back(std::move(link));
    }
    if (model.links.empty())
        Invalid(path, "the robot has no links");
}

void ReadJoints(const XMLElement& root, RobotModel& model, const LinkIndexByName& linkIndices,
    const std::filesystem::path& path)
{
    std::unordered_set<std::string> jointNames;
    // 앞서 만든 이름과 번호의 대응표로 각 관절의 parent와 child 링크를 연결합니다.
    for (const XMLElement* element = root.FirstChildElement("joint"); element != nullptr;
        element = element->NextSiblingElement("joint"))
    {
        RobotJoint joint = ReadJoint(*element, linkIndices, path);
        if (!jointNames.insert(joint.name).second)
            Invalid(path, "duplicate joint name '" + joint.name + "'");
        model.joints.push_back(std::move(joint));
    }
    if (model.joints.empty())
        Invalid(path, "the robot has no joints");
}
}

LoadedRobotModel LoadRobotModel(const std::filesystem::path& directory)
{
    LoadedRobotModel loaded;
    loaded.directory = std::filesystem::absolute(directory).lexically_normal();
    const auto urdfPath = loaded.directory / robot_model_format::urdfFile;
    const auto glbPath = loaded.directory / robot_model_format::glbFile;
    if (!std::filesystem::is_regular_file(urdfPath))
        Invalid(urdfPath, "robot.urdf was not found");
    if (!std::filesystem::is_regular_file(glbPath))
        Invalid(glbPath, "robot.glb was not found");

    loaded.robot = LoadRobotUrdf(urdfPath);
    loaded.visual = LoadGlb(glbPath);
    ConnectVisualTree(loaded.robot, loaded.visual, loaded.directory);
    return loaded;
}

RobotModel LoadRobotUrdf(const std::filesystem::path& path)
{
    tinyxml2::XMLDocument document;
    if (document.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS)
        Invalid(path, document.ErrorStr() == nullptr ? "cannot read URDF" : document.ErrorStr());

    const XMLElement* root = document.FirstChildElement("robot");
    if (root == nullptr)
        Invalid(path, "the document must have a <robot> root");

    RobotModel model;
    LinkIndexByName linkIndices;
    model.name = root->Attribute("name") == nullptr ? path.parent_path().filename().string()
        : root->Attribute("name");

    // 관절은 링크 이름을 참조하므로 링크를 모두 등록한 뒤에 읽습니다.
    ReadLinks(*root, model, linkIndices, path);
    ReadJoints(*root, model, linkIndices, path);

    // 전체 관절 중 base_link에서 tool0까지 실제로 이어지는 경로를 찾아 제어 순서로 저장합니다.
    FindActiveChain(model, linkIndices, path);
    return model;
}

} // namespace grasplink::model
