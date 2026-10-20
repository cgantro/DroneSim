#include "simulation/robotics/GripperTransformAdapter.h"

#include "scene/TransformComponents.h"
#include "scene/TransformSystemModule.h"

#include <glm/gtx/quaternion.hpp>

#include <cmath>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace grasplink::simulation::robotics
{
using grasplink::scene::Rotation;
using grasplink::scene::Entity;

namespace
{
struct GripperTransformLimits final
{
    inline static constexpr float unitScale = 1.0F;
    inline static constexpr float scaleTolerance = 1.0e-4F;
    inline static constexpr std::size_t initialParentPathCapacity = 16;
};

struct GripperTransformErrorMessages final
{
    inline static constexpr char nonFiniteBindTransform[] =
        "GripperTransformAdapter: non-finite bind transform";
    inline static constexpr char nonUnitScale[] =
        "GripperTransformAdapter: gripper hierarchy requires unit scale";
    inline static constexpr char invalidRoot[] =
        "GripperTransformAdapter: invalid Gripper root";
    inline static constexpr char emptySpecification[] =
        "GripperTransformAdapter: empty joint specification";
    inline static constexpr char invalidJointName[] =
        "GripperTransformAdapter: empty or duplicate specification joint name";
    inline static constexpr char duplicateGlbJointNamePrefix[] =
        "GripperTransformAdapter: duplicate GLB joint name: ";
    inline static constexpr char jointNotFoundPrefix[] =
        "GripperTransformAdapter: joint not found: ";
    inline static constexpr char jointOutsideRootPrefix[] =
        "GripperTransformAdapter: joint is outside the Gripper root: ";
    inline static constexpr char invalidBindRotationPrefix[] =
        "GripperTransformAdapter: invalid bind rotation: ";
    inline static constexpr char targetOutsideHierarchy[] =
        "GripperTransformAdapter: target is outside the Gripper hierarchy";
    inline static constexpr char removedScene[] =
        "GripperTransformAdapter: Gripper Scene has been removed";
    inline static constexpr char changedHierarchy[] =
        "GripperTransformAdapter: Gripper hierarchy changed or was removed";
    inline static constexpr char invalidJointPose[] =
        "GripperTransformAdapter: joint pose count or master angle is invalid";
    inline static constexpr char nonFiniteJointPose[] =
        "GripperTransformAdapter: non-finite joint pose";
    inline static constexpr char invalidDeltaRotation[] =
        "GripperTransformAdapter: zero or invalid delta rotation";
};

bool IsFinite(const glm::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsFinite(const glm::quat& value)
{
    return std::isfinite(value.w) && std::isfinite(value.x) &&
        std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsUnitScale(const glm::vec3& value)
{
    return std::abs(value.x - GripperTransformLimits::unitScale) <=
            GripperTransformLimits::scaleTolerance &&
        std::abs(value.y - GripperTransformLimits::unitScale) <=
            GripperTransformLimits::scaleTolerance &&
        std::abs(value.z - GripperTransformLimits::unitScale) <=
            GripperTransformLimits::scaleTolerance;
}

bool IsFinite(const ::grasplink::robotics::models::QuaternionWxyz& value)
{
    return std::isfinite(value.w) && std::isfinite(value.x) &&
        std::isfinite(value.y) && std::isfinite(value.z);
}

glm::dquat ToDoubleQuaternion(const ::grasplink::robotics::models::QuaternionWxyz& value)
{
    return glm::dquat{value.w, value.x, value.y, value.z};
}

::grasplink::robotics::models::QuaternionWxyz ToModelQuaternion(const glm::dquat& value)
{
    return {value.w, value.x, value.y, value.z};
}

void ValidateLocalTransform(const Entity& entity)
{
    const glm::vec3 position = entity.GetLocalPosition();
    const glm::quat rotation = entity.GetLocalRotation();
    const glm::vec3 scale = entity.GetLocalScale();
    if (!IsFinite(position) || !IsFinite(rotation) || !IsFinite(scale))
        throw std::invalid_argument(GripperTransformErrorMessages::nonFiniteBindTransform);
    // 일반 setter를 거치지 않고 ECS에서 직접 기록한 값도 검사한다. 영 quaternion은 방향을 나타낼 수 없으므로 bind 계층 연결을 거부한다.
    (void)Rotation{rotation};
    if (!IsUnitScale(scale))
        throw std::invalid_argument(GripperTransformErrorMessages::nonUnitScale);
}

bool IsSameHierarchy(const Entity& joint, const std::vector<Entity>& ancestry)
{
    if (ancestry.empty() || !joint.IsValid())
        return false;
    if (ancestry.front() != joint)
        return false;

    for (std::size_t i = 0; i < ancestry.size(); ++i)
    {
        if (!ancestry[i].IsValid())
            return false;
        if (i + 1 < ancestry.size() && ancestry[i].GetParent() != ancestry[i + 1])
            return false;
    }
    return true;
}
}

GripperTransformAdapter::GripperTransformAdapter(
    const Entity& gripperRoot,
    const ::grasplink::robotics::models::GripperSpecification& specification)
    : gripperRoot_(gripperRoot)
{
    if (!gripperRoot_)
        throw std::runtime_error(GripperTransformErrorMessages::invalidRoot);
    if (specification.joints == nullptr || specification.jointCount == 0)
        throw std::invalid_argument(GripperTransformErrorMessages::emptySpecification);

    ValidateLocalTransform(gripperRoot_);

    // 모델 사양에 있는 관절 이름만 검색 대상으로 삼는다. 메시와 link처럼 관절이 아닌 authored node는 저장된 변환을 그대로 유지한다.
    std::unordered_set<std::string> wantedNames;
    wantedNames.reserve(specification.jointCount);
    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const std::string name(specification.joints[i].name);
        if (name.empty() || !wantedNames.emplace(name).second)
            throw std::invalid_argument(GripperTransformErrorMessages::invalidJointName);
    }

    std::unordered_map<std::string, Entity> foundJoints;
    std::function<void(const Entity&)> collect = [&](const Entity& parent)
    {
        for (const Entity& child : parent.GetChildren())
        {
            const char* rawName = child.GetHandle().name().c_str();
            const std::string name = rawName == nullptr ? std::string{} : std::string(rawName);
            if (wantedNames.count(name) != 0 && !foundJoints.emplace(name, child).second)
                throw std::runtime_error(
                    GripperTransformErrorMessages::duplicateGlbJointNamePrefix + name);
            collect(child);
        }
    };
    collect(gripperRoot_);

    joints_.reserve(specification.jointCount);
    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const std::string name(specification.joints[i].name);
        const auto found = foundJoints.find(name);
        if (found == foundJoints.end())
            throw std::runtime_error(GripperTransformErrorMessages::jointNotFoundPrefix + name);

        JointBinding binding;
        binding.entity = found->second;
        for (Entity current = binding.entity; current; current = current.GetParent())
        {
            ValidateLocalTransform(current);
            binding.ancestry.push_back(current);
            if (current == gripperRoot_)
                break;
        }
        if (binding.ancestry.empty() || binding.ancestry.back() != gripperRoot_)
            throw std::runtime_error(
                GripperTransformErrorMessages::jointOutsideRootPrefix + name);

        // 그리퍼의 장착 위치와 방향은 root에 남기고, 각 관절은 GLB 기준 회전에 개폐 회전을 더한다.
        const glm::dquat rawBindRotation{binding.entity.GetLocalRotation()};
        const double rawBindLengthSquared = glm::dot(rawBindRotation, rawBindRotation);
        if (!std::isfinite(rawBindLengthSquared) || rawBindLengthSquared <= 0.0)
            throw std::invalid_argument(
                GripperTransformErrorMessages::invalidBindRotationPrefix + name);
        const glm::dquat bindRotation = glm::normalize(rawBindRotation);
        binding.bindRotation = ToModelQuaternion(bindRotation);
        joints_.push_back(std::move(binding));
    }
    preparedRotations_.resize(joints_.size());
    parentPathScratch_.reserve(GripperTransformLimits::initialParentPathCapacity);
}

void GripperTransformAdapter::Apply(
    const ::grasplink::robotics::kinematics::GripperKinematicState& state)
{
    PrepareRotations(state);
    for (std::size_t i = 0; i < joints_.size(); ++i)
        joints_[i].entity.SetLocalRotation(preparedRotations_[i]);
}

void GripperTransformAdapter::BuildCandidateWorldTransforms(
    const ::grasplink::robotics::kinematics::GripperKinematicState& state,
    const glm::mat4& gripperRootWorldTransform,
    const std::vector<Entity>& targets,
    std::vector<glm::mat4>& output)
{
    PrepareRotations(state);
    output.clear();
    output.reserve(targets.size());
    auto& parentPath = parentPathScratch_;
    for (const Entity& target : targets)
    {
        parentPath.clear();
        Entity current = target;
        while (current && current != gripperRoot_)
        {
            parentPath.push_back(current);
            current = current.GetParent();
        }
        if (!current)
            throw std::runtime_error(GripperTransformErrorMessages::targetOutsideHierarchy);

        glm::mat4 worldTransform = gripperRootWorldTransform;
        for (auto iterator = parentPath.rbegin(); iterator != parentPath.rend(); ++iterator)
        {
            const Entity& entity = *iterator;
            ValidateLocalTransform(entity);
            glm::quat localRotation = entity.GetLocalRotation();
            for (std::size_t jointIndex = 0; jointIndex < joints_.size(); ++jointIndex)
            {
                if (joints_[jointIndex].entity == entity)
                {
                    localRotation = preparedRotations_[jointIndex];
                    break;
                }
            }
            worldTransform *= grasplink::scene::TransformSystemModule::ComposeLocalMatrix(
                entity.GetLocalPosition(), localRotation, entity.GetLocalScale());
        }
        output.push_back(worldTransform);
    }
}

void GripperTransformAdapter::PrepareRotations(
    const ::grasplink::robotics::kinematics::GripperKinematicState& state)
{
    if (!std::isfinite(state.masterAngleRadians) ||
        state.jointAnglesRadians.size() != joints_.size() ||
        state.jointLocalRotations.size() != joints_.size())
        throw std::invalid_argument(GripperTransformErrorMessages::invalidJointPose);
    if (!gripperRoot_.IsValid())
        throw std::runtime_error(GripperTransformErrorMessages::removedScene);

    for (const JointBinding& binding : joints_)
        if (!IsSameHierarchy(binding.entity, binding.ancestry))
            throw std::runtime_error(GripperTransformErrorMessages::changedHierarchy);

    for (std::size_t i = 0; i < joints_.size(); ++i)
    {
        if (!std::isfinite(state.jointAnglesRadians[i]) || !IsFinite(state.jointLocalRotations[i]))
            throw std::invalid_argument(GripperTransformErrorMessages::nonFiniteJointPose);

        const glm::dquat rawDelta = ToDoubleQuaternion(state.jointLocalRotations[i]);
        const double rawDeltaLengthSquared = glm::dot(rawDelta, rawDelta);
        if (!std::isfinite(rawDeltaLengthSquared) || rawDeltaLengthSquared <= 0.0)
            throw std::invalid_argument(GripperTransformErrorMessages::invalidDeltaRotation);
        const glm::dquat delta = glm::normalize(rawDelta);
        const glm::dquat bind = glm::normalize(ToDoubleQuaternion(joints_[i].bindRotation));
        const glm::dquat result = glm::normalize(bind * delta);
        preparedRotations_[i] = Rotation{glm::quat{result}};
    }
}
} // namespace grasplink::simulation::robotics
