#include "simulation/robotics/RobotCollisionGeometryBuilder.h"
#include "simulation/robotics/RobotModelTransforms.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <utility>

namespace grasplink::simulation::robotics::collision_geometry
{
namespace
{
using grasplink::model::RobotCollisionGeometry;
using grasplink::model::RobotCollisionType;
using grasplink::model::RobotLink;
using Shape = grasplink::physics::CollisionShapeDescription;

struct CollisionGeometryLimits final
{
    inline static constexpr double halfExtentScale = 0.5;
    inline static constexpr int cylinderSegments = 16;
    inline static constexpr int sphereLongitudeSegments = 16;
    inline static constexpr int sphereLatitudeSegments = 8;
    inline static constexpr double pi = 3.14159265358979323846;
};

struct CollisionGeometryErrorMessages final
{
    inline static constexpr char invalidBaseOrToolIndex[] =
        "Robot collision geometry: base link index is out of range";
    inline static constexpr char missingBaseGeometry[] =
        "Robot collision geometry: base_link has no URDF collision geometry";
    inline static constexpr char invalidActiveJointIndex[] =
        "Robot collision geometry: active joint index is out of range";
    inline static constexpr char disconnectedToolLink[] =
        "Robot collision geometry: tool link is not connected to base_link";
    inline static constexpr char collisionLinkOutsideArmPath[] =
        "Robot collision geometry: collision links must belong to the single arm path";
    inline static constexpr char collisionLinkNotAttached[] =
        "Robot collision geometry: link is not attached to the arm or base";
    inline static constexpr char missingArmGeometry[] =
        "Robot collision geometry: arm links have no URDF collision geometry";
    inline static constexpr char unsupportedCollisionType[] =
        "Robot collision geometry: collision shape type is unsupported";
};

struct LinkJointLookup final
{
    std::vector<std::optional<std::size_t>> parentJointByLink;
    std::vector<std::optional<std::size_t>> activeJointByLink;
};

grasplink::physics::Transform ToPhysicsTransform(const RobotCollisionGeometry& source)
{
    grasplink::physics::Transform result;
    result.position = glm::vec3(source.positionMeters);
    result.rotation = glm::quat(source.rotation);
    return result;
}

Shape BuildBox(const RobotCollisionGeometry& source)
{
    Shape result;
    result.localTransform = ToPhysicsTransform(source);
    result.type = grasplink::physics::CollisionShapeType::Box;
    result.halfExtentsMeters = glm::vec3(
        source.dimensionsMeters * CollisionGeometryLimits::halfExtentScale);
    return result;
}

Shape BuildCylinder(const RobotCollisionGeometry& source)
{
    Shape result;
    result.localTransform = ToPhysicsTransform(source);
    result.type = grasplink::physics::CollisionShapeType::ConvexHull;
    constexpr int segments = CollisionGeometryLimits::cylinderSegments;
    constexpr double pi = CollisionGeometryLimits::pi;
    const double radius = source.dimensionsMeters.x;
    const double halfLength = source.dimensionsMeters.y * 0.5;
    result.pointsMeters.reserve(segments * 2);
    for (int side = 0; side < segments; ++side)
    {
        const double angle = 2.0 * pi * side / segments;
        const float x = static_cast<float>(radius * std::cos(angle));
        const float y = static_cast<float>(radius * std::sin(angle));
        result.pointsMeters.emplace_back(x, y, static_cast<float>(-halfLength));
        result.pointsMeters.emplace_back(x, y, static_cast<float>(halfLength));
    }
    return result;
}

Shape BuildSphere(const RobotCollisionGeometry& source)
{
    Shape result;
    result.localTransform = ToPhysicsTransform(source);
    result.type = grasplink::physics::CollisionShapeType::ConvexHull;
    constexpr int segments = CollisionGeometryLimits::sphereLongitudeSegments;
    constexpr int latitudeSegments = CollisionGeometryLimits::sphereLatitudeSegments;
    constexpr double pi = CollisionGeometryLimits::pi;
    const double radius = source.dimensionsMeters.x;
    result.pointsMeters.reserve((latitudeSegments - 1) * segments + 2);
    result.pointsMeters.emplace_back(0.0F, 0.0F, static_cast<float>(-radius));
    for (int latitude = 1; latitude < latitudeSegments; ++latitude)
    {
        const double elevation = -pi * 0.5 + pi * latitude / latitudeSegments;
        const double ringRadius = radius * std::cos(elevation);
        const double z = radius * std::sin(elevation);
        for (int side = 0; side < segments; ++side)
        {
            const double angle = 2.0 * pi * side / segments;
            result.pointsMeters.emplace_back(
                static_cast<float>(ringRadius * std::cos(angle)),
                static_cast<float>(ringRadius * std::sin(angle)),
                static_cast<float>(z));
        }
    }
    result.pointsMeters.emplace_back(0.0F, 0.0F, static_cast<float>(radius));
    return result;
}

Shape BuildShape(const RobotCollisionGeometry& source)
{
    switch (source.type)
    {
    case RobotCollisionType::Box:
        return BuildBox(source);
    case RobotCollisionType::Cylinder:
        return BuildCylinder(source);
    case RobotCollisionType::Sphere:
        return BuildSphere(source);
    }

    throw std::invalid_argument(CollisionGeometryErrorMessages::unsupportedCollisionType);
}

std::vector<Shape> BuildLinkShapes(const RobotLink& link,
    const grasplink::robotics::models::Pose3& linkPoseInAnchor = {})
{
    std::vector<Shape> result;
    result.reserve(link.collisionGeometry.size());
    if (link.collisionGeometry.empty())
        return result;

    const glm::mat4 linkTransform = PoseMatrix(linkPoseInAnchor);
    for (const RobotCollisionGeometry& geometry : link.collisionGeometry)
    {
        Shape shape = BuildShape(geometry);
        const glm::vec3 shapePosition = shape.localTransform.position;
        const glm::quat shapeRotation = shape.localTransform.rotation;
        const glm::mat4 shapeTransform = glm::translate(glm::mat4{1.0F}, shapePosition) *
            glm::mat4_cast(shapeRotation);
        const glm::mat4 combined = linkTransform * shapeTransform;
        shape.localTransform.position = glm::vec3{combined[3]};
        shape.localTransform.rotation = glm::normalize(glm::quat_cast(glm::mat3{combined}));
        result.push_back(std::move(shape));
    }
    return result;
}

LinkJointLookup BuildJointLookup(const grasplink::model::RobotModel& model)
{
    LinkJointLookup lookup{
        std::vector<std::optional<std::size_t>>(model.links.size()),
        std::vector<std::optional<std::size_t>>(model.links.size())};

    for (std::size_t jointIndex = 0; jointIndex < model.joints.size(); ++jointIndex)
        lookup.parentJointByLink[model.joints[jointIndex].childLink] = jointIndex;

    for (std::size_t activeIndex = 0; activeIndex < model.activeJoints.size(); ++activeIndex)
    {
        const std::size_t jointIndex = model.activeJoints[activeIndex];
        if (jointIndex >= model.joints.size())
            throw std::invalid_argument(CollisionGeometryErrorMessages::invalidActiveJointIndex);
        lookup.activeJointByLink[model.joints[jointIndex].childLink] = activeIndex;
    }

    return lookup;
}

std::vector<std::size_t> BuildArmLinkOrder(const grasplink::model::RobotModel& model,
    const LinkJointLookup& lookup)
{
    std::vector<std::size_t> linkOrder;
    for (std::size_t linkIndex = model.toolLink;;)
    {
        linkOrder.push_back(linkIndex);
        if (linkIndex == model.baseLink)
            break;

        const auto& parentJoint = lookup.parentJointByLink[linkIndex];
        if (!parentJoint)
            throw std::invalid_argument(CollisionGeometryErrorMessages::disconnectedToolLink);
        linkIndex = model.joints[*parentJoint].parentLink;
    }
    std::reverse(linkOrder.begin(), linkOrder.end());
    return linkOrder;
}

void ValidateCollisionLinksOnArmPath(const grasplink::model::RobotModel& model,
    const std::vector<std::size_t>& linkOrder)
{
    std::vector<bool> isOnArmPath(model.links.size(), false);
    for (const std::size_t linkIndex : linkOrder)
        isOnArmPath[linkIndex] = true;

    for (std::size_t linkIndex = 0; linkIndex < model.links.size(); ++linkIndex)
    {
        if (!isOnArmPath[linkIndex] && !model.links[linkIndex].collisionGeometry.empty())
            throw std::invalid_argument(
                CollisionGeometryErrorMessages::collisionLinkOutsideArmPath);
    }
}

std::pair<std::size_t, std::optional<std::size_t>> FindCollisionAnchor(
    const grasplink::model::RobotModel& model, const LinkJointLookup& lookup,
    std::size_t linkIndex)
{
    std::size_t anchorLink = linkIndex;
    while (anchorLink != model.baseLink)
    {
        if (lookup.activeJointByLink[anchorLink])
            return {anchorLink, lookup.activeJointByLink[anchorLink]};

        const auto& parentJoint = lookup.parentJointByLink[anchorLink];
        if (!parentJoint)
            break;
        anchorLink = model.joints[*parentJoint].parentLink;
    }

    if (anchorLink != model.baseLink)
        throw std::invalid_argument(CollisionGeometryErrorMessages::collisionLinkNotAttached);
    return {anchorLink, std::nullopt};
}

void AddLinkCollisionGeometry(const grasplink::model::RobotModel& model,
    const std::vector<grasplink::robotics::models::Pose3>& bindPoses,
    const LinkJointLookup& lookup, std::size_t linkIndex, Result& result)
{
    const auto [anchorLink, activeJoint] = FindCollisionAnchor(model, lookup, linkIndex);
    if (!activeJoint)
    {
        auto shapes = BuildLinkShapes(model.links[linkIndex],
            RelativeRobotPose(bindPoses[model.baseLink], bindPoses[linkIndex]));
        result.baseShapes.insert(result.baseShapes.end(),
            std::make_move_iterator(shapes.begin()), std::make_move_iterator(shapes.end()));
        return;
    }

    const auto localPose = RelativeRobotPose(bindPoses[anchorLink], bindPoses[linkIndex]);
    result.links.push_back({model.links[linkIndex].name, *activeJoint, localPose,
        BuildLinkShapes(model.links[linkIndex])});
}

}

Result Build(
    const grasplink::model::RobotModel& model)
{
    if (model.baseLink >= model.links.size() || model.toolLink >= model.links.size())
        throw std::invalid_argument(CollisionGeometryErrorMessages::invalidBaseOrToolIndex);
    const auto bindPoses = grasplink::simulation::robotics::BuildRobotLinkBindPoses(model);

    Result result;
    result.baseShapes = BuildLinkShapes(model.links[model.baseLink]);
    if (result.baseShapes.empty())
        throw std::invalid_argument(CollisionGeometryErrorMessages::missingBaseGeometry);

    const LinkJointLookup jointLookup = BuildJointLookup(model);
    const std::vector<std::size_t> linkOrder = BuildArmLinkOrder(model, jointLookup);
    ValidateCollisionLinksOnArmPath(model, linkOrder);

    result.links.reserve(linkOrder.size() - 1);
    for (const std::size_t linkIndex : linkOrder)
    {
        if (linkIndex == model.baseLink || model.links[linkIndex].collisionGeometry.empty())
            continue;

        AddLinkCollisionGeometry(model, bindPoses, jointLookup, linkIndex, result);
    }

    if (result.links.empty())
        throw std::invalid_argument(CollisionGeometryErrorMessages::missingArmGeometry);
    return result;
}
}
