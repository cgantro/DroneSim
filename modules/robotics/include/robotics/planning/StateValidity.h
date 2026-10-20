#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <functional>

namespace grasplink::robotics::planning
{

/**
 * @brief 관절 자세를 사용할 수 없는 이유를 구분한다.
 * @details 관절 수와 관절 한계를 먼저 검사한 뒤, 등록된 검사 함수가 바닥·장애물이나 로봇 링크 사이의 충돌을 확인한다.
 * 각 실패 이유를 나누어 반환하므로 Controller는 환경 충돌과 로봇 자체 충돌을 서로 다른 결과로 보고할 수 있다.
 */
enum class JointStateInvalidity : std::uint8_t
{
    None, // 관절 값이 유효하고, 등록된 충돌 검사가 있다면 자세를 허용했다.
    JointCountMismatch, // 제공한 관절 수가 로봇 사양과 다르다.
    NonFinitePosition, // 관절각에 NaN 또는 무한대가 포함되어 있다.
    JointLimitViolation, // 하나 이상의 관절각이 허용 범위를 벗어났다.
    PathSamplingLimitExceeded, // 충돌 검사에 필요한 경로 표본 수가 정책 한도를 넘었다.
    EnvironmentCollision, // 로봇이나 그리퍼가 바닥 또는 작업대 같은 고정 환경과 겹친다.
    SelfCollision, // 로봇 링크끼리 겹친다.
    AttachedObjectCollision // 로봇에 붙은 물체가 환경과 겹친다.
};

/** @brief 관절 자세 하나가 계획에 사용 가능한지 확인하는 함수 형식이다. */
using StateValidityChecker = std::function<JointStateInvalidity(const JointVector&)>;

/**
 * @brief 관절 수, 숫자 값, 관절 한계와 등록된 충돌 검사를 순서대로 확인한다.
 * @param specification 검사할 관절 수와 허용 범위를 제공하는 로봇 사양이다.
 * @param joints 사양에 적힌 순서의 관절 위치다. 회전 관절은 [rad], 직선 관절은 [m]다.
 * @param stateValidityChecker 관절 값 검증 후 충돌을 확인하는 선택 함수다.
 */
[[nodiscard]] JointStateInvalidity ValidateJointState(
    const models::RobotSpecification& specification,
    const JointVector& joints,
    const StateValidityChecker& stateValidityChecker);

/** @brief 관절 자세의 실패 이유를 Controller가 반환할 공통 결과로 바꾼다. */
[[nodiscard]] Result MapJointStateInvalidity(JointStateInvalidity invalidity);

} // namespace grasplink::robotics::planning
