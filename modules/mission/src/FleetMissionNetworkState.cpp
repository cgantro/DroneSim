#include "FleetMission.h"

#include <algorithm>
#include <utility>

#include <glm/geometric.hpp>

namespace dronesim::mission
{
namespace
{
/**
 * @brief 시작 Tick 이후 지정한 간격이 흘렀는지 확인한다.
 * @details 시뮬레이션을 초기화하면 현재 Tick이 과거 기록보다 작아질 수 있다.
 * 이때 부호 없는 정수를 먼저 빼면 큰 값으로 넘치므로, 순서를 확인한 뒤 차이를 구한다.
 */
bool HasTickIntervalElapsed(
    std::uint64_t currentTick,
    std::uint64_t previousTick,
    std::uint64_t intervalTicks) noexcept
{
    return currentTick >= previousTick
        && currentTick - previousTick >= intervalTicks;
}
}

std::uint32_t FleetRouteDeliveryState::NextSequence() const noexcept
{
    // 새 명령은 이미 적용된 경로와 아직 확인을 기다리는 경로보다 앞선 번호를 쓴다.
    return std::max(appliedSequence_, pendingSequence_) + 1U;
}

void FleetRouteDeliveryState::Queue(
    std::uint32_t sequence,
    std::string payload,
    std::uint64_t currentTick)
{
    pendingSequence_ = sequence;
    pendingPayload_ = std::move(payload);
    transmissionCount_ = 1U;
    sentAtTick_ = currentTick;
    lastTransmissionTick_ = currentTick;
}

void FleetRouteDeliveryState::RecordApplied(std::uint32_t sequence) noexcept
{
    appliedSequence_ = sequence;
}

bool FleetRouteDeliveryState::Acknowledge(std::uint32_t sequence) noexcept
{
    // 예전 경로나 아직 적용되지 않은 경로의 응답으로 현재 재전송 대기를 지우지 않는다.
    if (pendingSequence_ != sequence || appliedSequence_ != sequence)
        return false;

    ClearPending();
    return true;
}

void FleetRouteDeliveryState::ClearPending() noexcept
{
    pendingSequence_ = 0U;
    pendingPayload_.clear();
    transmissionCount_ = 0U;
}

bool FleetRouteDeliveryState::CanRetry(
    std::uint64_t currentTick,
    std::uint64_t retryIntervalTicks,
    std::uint32_t maximumTransmissions) const noexcept
{
    // 모의 시간이 초기화됐거나 Tick이 아직 간격만큼 흐르지 않았다면 재전송하지 않는다.
    return HasPendingTransmission()
        && transmissionCount_ < maximumTransmissions
        && HasTickIntervalElapsed(
            currentTick, lastTransmissionTick_, retryIntervalTicks);
}

void FleetRouteDeliveryState::RecordRetry(std::uint64_t currentTick) noexcept
{
    lastTransmissionTick_ = currentTick;
    ++transmissionCount_;
}
void FormationNetworkState::Reset() noexcept
{
    nextMessageSequence_ = 1U;
    sentFollowerTargets_.clear();
    lastTargetSentAtTick_.clear();
    lastAppliedSequence_.clear();
    lastNetworkNodeUpdateTick_.reset();
}
bool FormationNetworkState::IsNewAppliedSequence(
    simulation::DroneId droneId,
    std::uint32_t sequence) const noexcept
{
    const auto previous = lastAppliedSequence_.find(droneId);
    return previous == lastAppliedSequence_.end() || sequence > previous->second;
}

void FormationNetworkState::RecordAppliedSequence(
    simulation::DroneId droneId,
    std::uint32_t sequence)
{
    lastAppliedSequence_[droneId] = sequence;
}

bool FormationNetworkState::ShouldSendFollowerTarget(
    simulation::DroneId droneId,
    const glm::dvec3& targetMeters,
    std::uint64_t currentTick,
    double changeThresholdMeters,
    std::uint64_t refreshIntervalTicks) const noexcept
{
    // 목표가 충분히 바뀌면 즉시 보내고, 그대로라면 일정 Tick마다 현재 목표를 다시 알린다.
    const auto previousTarget = sentFollowerTargets_.find(droneId);
    if (previousTarget == sentFollowerTargets_.end()
        || glm::distance(previousTarget->second, targetMeters) >= changeThresholdMeters)
        return true;

    const auto lastSent = lastTargetSentAtTick_.find(droneId);
    return lastSent == lastTargetSentAtTick_.end()
        || HasTickIntervalElapsed(currentTick, lastSent->second,
            refreshIntervalTicks);
}

void FormationNetworkState::RecordFollowerTargetSent(
    simulation::DroneId droneId,
    const glm::dvec3& targetMeters,
    std::uint64_t currentTick)
{
    sentFollowerTargets_[droneId] = targetMeters;
    lastTargetSentAtTick_[droneId] = currentTick;
}

bool FormationNetworkState::ShouldRefreshNetworkNodes(
    std::uint64_t currentTick,
    std::uint64_t refreshIntervalTicks) const noexcept
{
    // 모의 Tick이 초기화되면 이전 시각과 비교할 수 없으므로 새 기준 시각이 올 때까지 기다린다.
    return !lastNetworkNodeUpdateTick_
        || HasTickIntervalElapsed(
            currentTick, *lastNetworkNodeUpdateTick_, refreshIntervalTicks);
}

void FormationNetworkState::RecordNetworkNodesUpdated(std::uint64_t currentTick) noexcept
{
    lastNetworkNodeUpdateTick_ = currentTick;
}

}
