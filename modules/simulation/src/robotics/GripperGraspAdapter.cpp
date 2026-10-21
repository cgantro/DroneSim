#include "simulation/robotics/GripperGraspAdapter.h"

#include "physics/PhysicsWorld.h"
#include "simulation/components/RobotCollisionProxy.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include <algorithm>
#include <vector>

namespace grasplink::simulation
{
using grasplink::scene::Entity;

namespace
{
// 손끝 표면의 실제 이격만 열기 차단으로 판단하고 수치 잡음을 무시한다.
constexpr float kOpeningBlockMotionEpsilonMeters = 1.0e-7F;
// 두 접촉 법선이 충분히 반대일 때에만 물체를 양쪽에서 끼운 것으로 본다.
constexpr float kOpposingFingerNormalDotThreshold = -0.25F;

bool SameBody(physics::PhysicsBodyHandle first, physics::PhysicsBodyHandle second)
{
    return first.IsValid() && second.IsValid() && first.value == second.value && first.worldToken == second.worldToken;
}
}

GripperGraspAdapter::GripperGraspAdapter(physics::PhysicsWorld& world, PhysicsSystemModule& system,
    robotics::backends::simulation::SimGripperController& controller)
    : world_(world), system_(system), controller_(controller), releaseRevision_(controller.GetReleaseRevision())
{
}

GripperGraspAdapter::~GripperGraspAdapter()
{
    Release();
}

bool GripperGraspAdapter::Bind(const Entity& robotRoot)
{
    Release();
    anchor_ = {};
    left_ = {};
    right_ = {};
    if (!robotRoot)
        return false;
    bool uniqueBindings = true;
    std::vector<Entity> pending{robotRoot};
    while (!pending.empty())
    {
        Entity entity = pending.back();
        pending.pop_back();
        if (entity.Has<GripperCollisionProxy>())
        {
            switch (entity.Get<GripperCollisionProxy>().part)
            {
            case GripperCollisionPart::Body:
                uniqueBindings = uniqueBindings && !anchor_;
                anchor_ = entity;
                break;
            case GripperCollisionPart::LeftFingerTip:
                uniqueBindings = uniqueBindings && !left_;
                left_ = entity;
                break;
            case GripperCollisionPart::RightFingerTip:
                uniqueBindings = uniqueBindings && !right_;
                right_ = entity;
                break;
            case GripperCollisionPart::Other:
                break;
            }
        }
        const auto children = entity.GetChildren();
        pending.insert(pending.end(), children.begin(), children.end());
    }
    releaseRevision_ = controller_.GetReleaseRevision();
    disarmed_ = false;
    previousTipsValid_ = false;
    if (!uniqueBindings || !anchor_ || !left_ || !right_)
    {
        anchor_ = {};
        left_ = {};
        right_ = {};
        return false;
    }
    return true;
}

void GripperGraspAdapter::BeforePhysicsStep()
{
    RefreshBindingAndReleaseState();
}

void GripperGraspAdapter::RefreshBindingAndReleaseState()
{
    const auto feedback = controller_.GetState();
    const std::uint64_t revision = controller_.GetReleaseRevision();
    const bool releaseRequested = revision != releaseRevision_ || !feedback.valid || !feedback.activated ||
        controller_.IsOpeningRequested();
    releaseRevision_ = revision;
    if (releaseRequested)
    {
        Release();
        return;
    }
    if (constraint_.IsValid() && (!world_.IsConstraintValid(constraint_) || !anchor_ || !left_ || !right_ ||
        !SameBody(system_.GetBodyHandle(anchor_.GetHandle()), heldAnchor_) ||
        !SameBody(system_.GetBodyHandle(left_.GetHandle()), heldLeft_) ||
        !SameBody(system_.GetBodyHandle(right_.GetHandle()), heldRight_)))
        Release();
}

void GripperGraspAdapter::AfterPhysicsStep()
{
    // 이 조회는 매 틱 수행한다. ECS 설정 변경으로 proxy의 Body가 교체돼도 이전 핸들로 새 Body를 파지하지 않는다.
    RefreshBindingAndReleaseState();
    state_.leftContact = false;
    state_.rightContact = false;
    const auto feedback = controller_.GetState();
    if (!feedback.valid || !feedback.activated || !anchor_ || !left_ || !right_)
        return;
    const auto anchor = system_.GetBodyHandle(anchor_.GetHandle());
    const auto left = system_.GetBodyHandle(left_.GetHandle());
    const auto right = system_.GetBodyHandle(right_.GetHandle());
    if (!anchor.IsValid() || !left.IsValid() || !right.IsValid())
        return;

    const auto currentLeft = world_.GetBodyTransform(left);
    const auto currentRight = world_.GetBodyTransform(right);
    const bool closing = controller_.IsClosingRequested();
    const bool opening = controller_.IsOpeningRequested();
    bool openingBlocked = false;

    leftContactCandidates_.clear();
    rightContactCandidates_.clear();
    world_.CopyContactsTo(contactSnapshots_);
    for (const auto& contact : contactSnapshots_)
    {
        // Jolt는 가까워지는 물체를 미리 보고할 수 있다. 음수 겹침 깊이는 실제 접촉이 아니므로 파지 후보에서 제외한다.
        if (contact.penetrationMeters < 0.0F)
            continue;
        auto collect = [&](physics::PhysicsBodyHandle finger, bool& touched,
            std::vector<ContactCandidate>& objects)
        {
            physics::PhysicsBodyHandle other;
            glm::vec3 normal;
            if (SameBody(contact.first, finger)) { other = contact.second; normal = contact.normal; }
            else if (SameBody(contact.second, finger)) { other = contact.first; normal = -contact.normal; }
            else return;
            touched = true;
            if (opening && previousTipsValid_)
            {
                // 열기 직후 남은 닫힘 접촉은 손끝이 물체에서 멀어지면 정지 이유가 아니다.
                // 접촉점을 현재 Body 원점 기준 좌표로 바꾼 뒤 이전 자세에 적용해 회전으로 움직인 표면까지 비교한다.
                const bool isLeft = SameBody(finger, left);
                const auto& current = isLeft ? currentLeft : currentRight;
                const auto& previous = isLeft ? previousLeft_ : previousRight_;
                const glm::vec3 localPoint = glm::inverse(current.rotation) * (contact.point - current.position);
                const glm::vec3 previousPoint = previous.position + previous.rotation * localPoint;
                openingBlocked = openingBlocked ||
                    glm::dot(contact.point - previousPoint, normal) > kOpeningBlockMotionEpsilonMeters;
            }
            // 바닥 같은 Static 환경 접촉은 정지에만 쓰고 파지할 수 있는 Dynamic 물체만 목록에 모은다.
            if (world_.GetBodyMotionType(other) == physics::BodyMotionType::Dynamic)
                objects.push_back({other, normal});
        };
        collect(left, state_.leftContact, leftContactCandidates_);
        collect(right, state_.rightContact, rightContactCandidates_);
    }

    previousLeft_ = currentLeft;
    previousRight_ = currentRight;
    previousTipsValid_ = true;
    physics::PhysicsBodyHandle graspCandidate;
    for (const auto& candidate : leftContactCandidates_)
        if (std::any_of(
                rightContactCandidates_.begin(), rightContactCandidates_.end(),
                [&](const auto& rightCandidate)
                {
                    return SameBody(candidate.object, rightCandidate.object) &&
                        glm::dot(candidate.fingerToObject, rightCandidate.fingerToObject) <=
                            kOpposingFingerNormalDotThreshold;
                }))
        {
            graspCandidate = candidate.object;
            break;
        }

    // 한쪽만 먼저 닿았다고 닫힘을 멈추면 반대 손가락이 물체를 끼울 기회가 사라진다. 같은 물체를 양쪽에서 끼웠거나 열 때 접촉이 움직임을 막은 경우에만 멈춘다.
    if ((closing && graspCandidate.IsValid()) || openingBlocked)
        controller_.ApplyContactFeedback(opening);
    if (disarmed_ && controller_.GetCommandRevision() != releasedCommandRevision_)
        disarmed_ = false;
    if (!closing || disarmed_ || world_.IsConstraintValid(constraint_))
        return;

    // 접촉 normal은 손끝에서 물체를 향한다. 두 normal의 내적이 -0.25 이하면 서로 반대쪽에서 물체를 끼운 것으로 본다.
    // 이 기준은 약 104도 이상의 방향 차이를 요구하며 실제 마찰이나 파지 힘을 계산하지 않는다.
    if (!graspCandidate.IsValid())
        return;
    constraint_ = world_.CreateFixedConstraint(anchor, graspCandidate);
    heldAnchor_ = anchor;
    heldLeft_ = left;
    heldRight_ = right;
    state_.grasped = true;
    state_.object = graspCandidate;
}

void GripperGraspAdapter::Release()
{
    world_.DestroyConstraint(constraint_);
    constraint_ = {};
    heldAnchor_ = {};
    heldLeft_ = {};
    heldRight_ = {};
    releasedCommandRevision_ = controller_.GetCommandRevision();
    disarmed_ = true;
    state_ = {};
}

GripperGraspState GripperGraspAdapter::GetState() const
{
    auto result = state_;
    if (!world_.IsConstraintValid(constraint_))
    {
        result.grasped = false;
        result.object = {};
    }
    return result;
}
}
