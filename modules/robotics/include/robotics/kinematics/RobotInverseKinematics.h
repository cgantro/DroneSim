#pragma once

#include "robotics/kinematics/RobotKinematics.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace grasplink::robotics::kinematics
{
namespace detail
{
inline constexpr std::size_t taskSpaceDimension = 6;
}

/**
 * @brief IK 반복 계산이 목표 자세를 찾았는지와 찾지 못한 이유를 구분한다.
 * @details Unreachable은 링크 길이로 계산한 보수적 거리 상한 밖에 있는 목표만 뜻한다.
 * 상한 안에 있다는 사실만으로 실제로 도달 가능하다고 판단하지 않는다.
 * JointLimitReached는 관절 경계에 막혀 오차를 더 줄이지 못한 경우이고 DidNotConverge는 국소 계산의 반복 한도 또는 정체다.
 * 두 실패 모두 다른 시작 관절각에서 해를 찾을 가능성을 배제하지 않는다.
 */
enum class IkStatus { Success, InvalidInput, MissingToolFrame, Unreachable, JointLimitReached, DidNotConverge };

/** @brief 반복 계산이 끝난 원인을 나타낸다. */
enum class IkTerminationReason
{
    None,
    Converged,
    InvalidInput,
    MissingToolFrame,
    ConservativeReachExceeded,
    ActiveJointLimit,
    Stagnation,
    IterationLimit,
    Cancelled,
    NumericalFailure
};

/**
 * @brief IK가 맞출 TCP 목표 조건을 지정한다.
 * @details PositionOnly는 위치 오차만 줄이고 방향은 제한하지 않는다. Pose는 위치와 방향을 함께 맞춘다.
 * 계산기는 지정된 옵션을 그대로 따른다. 기본 옵션 함수는 관절 수를 보고 시작 모드를 고를 수 있다.
 */
enum class IkTaskMode
{
    PositionOnly,
    Pose
};

/**
 * @brief 위치와 방향의 허용 오차 및 Levenberg–Marquardt 반복 계산 설정을 정한다.
 * @details damping은 팔을 펴서 일부 방향으로 움직일 수 없는 특이 자세에서도 작은 수로 나누어 관절 변화가 폭증하는 것을 줄이는 양수 계수다.
 * 이 값에서 시작해 오차 감소가 확인되면 최소 1%까지 낮추고 감소하지 않으면 높여 안정성과 작은 잔여 오차를 함께 다룬다.
 * orientationWeightMetersPerRadian은 회전 오차 [rad]를 위치 오차 [m]와 함께 최소화하기 위한 길이 비율이며 실제 공구 길이나 속도 제한은 아니다.
 * 수렴은 위치와 방향의 개별 허용 오차를 모두 만족해야 하며 반복 계산만으로 항상 해를 찾는다고 보장하지 않는다.
 * 관절이 한계에 닿아 바깥으로 향하는 계산값을 내면 해당 관절을 그 반복에서 고정하고 나머지 관절로 다시 계산한다.
 * 정체 한도는 작은 관절 변화와 작은 오차 감소가 연속될 때 계산을 멈추며, 오차가 허용 범위 안에 든 성공 해는 중단하지 않는다.
 */
struct IkOptions
{
    /// 기본값 Pose는 위치와 방향을 모두 맞춘다. 위치만 요청하려면 PositionOnly를 지정한다.
    IkTaskMode taskMode = IkTaskMode::Pose;
    std::size_t maxIterations = 200;
    /// 실패한 해의 야코비안 진단값도 계산할지 정한다. 경로 계획처럼 진단값이 필요 없는 계산에서는 false로 할 수 있다.
    bool collectFailureJacobianDiagnostics = true;
    double positionToleranceMeters = 1e-5;
    double orientationToleranceRadians = 1e-4;
    double damping = 1e-3;
    double orientationWeightMetersPerRadian = 0.3;
    /// 한 번에 바꿀 회전 관절의 최대 크기 [rad]다.
    double maxJointStep = 0.2;
    /// 한 번에 움직일 직선 관절의 최대 거리 [m]다.
    double maxPrismaticStepMeters = 0.05;
    /// 허용 오차에 못 미친 채 작은 변화만 반복할 때 종료할 연속 반복 횟수다.
    std::size_t stagnationIterationLimit = 8;
    /// 반복 중 오차 감소율이 이 값 이하일 때 작은 변화로 볼 상대 기준이다.
    double stagnationRelativeCostTolerance = 1e-8;
    /// 관절 변화가 이 값 이하일 때 작은 변화로 볼 크기다. 회전 관절은 [rad], 직선 관절은 [m]다.
    double stagnationJointStepTolerance = 1e-5;
    /// 직선 관절의 작은 변화 기준 [m]다.
    double stagnationPrismaticStepToleranceMeters = 1e-6;
};

/**
 * @brief 로봇 관절 수에 맞는 기본 역기구학 목표를 고른다.
 * @details 6개보다 적은 관절로는 위치와 세 방향을 모두 자유롭게 맞출 수 없으므로 위치만 맞춘다.
 * 6개 이상이면 위치와 방향을 함께 맞춘다. 호출자가 더 구체적인 목표를 원하면 IkOptions를 직접 지정한다.
 */
[[nodiscard]] IkOptions RobotDefaultIkOptions(const models::RobotSpecification& specification);

/**
 * @brief IK의 성공 여부와 가장 가까워진 관절각, 잔여 오차를 값으로 반환한다.
 * @details jointPositions는 RobotSpecification 순서다. 회전 관절은 [rad], 직선 관절은 [m]다.
 * 실패 결과의 관절각은 진단용 후보이므로 Controller 목표로 실행하지 않는다.
 * iterations는 수행한 반복 수이며 오차는 Robot base 기준 위치 거리 [m]와 최단 회전각 [rad]다.
 * terminationReason은 관절 제한, 정체, 반복 한도, 수치 오류 중 계산을 끝낸 직접 원인을 나타낸다.
 * collectFailureJacobianDiagnostics가 true이면 weightedJacobianMinimumSingularValue와 weightedJacobianConditionNumber를 실패 시 가중 Jacobian에서 계산한다. 값은 위치·방향 scaling에 따라 달라진다.
 * 입력이나 거리 사전검사에서 반환되어 FK 오차를 계산하지 않았다면 두 오차 값은 infinity다.
 */
struct IkResult
{
    IkStatus status = IkStatus::InvalidInput;
    JointVector jointPositions;
    std::size_t iterations = 0;
    double positionErrorMeters = 0.0;
    double orientationErrorRadians = 0.0;
    IkTerminationReason terminationReason = IkTerminationReason::None;
    double weightedJacobianMinimumSingularValue = 0.0;
    double weightedJacobianConditionNumber = 0.0;
    std::string message;
    [[nodiscard]] bool Ok() const noexcept { return status == IkStatus::Success; }
    explicit operator bool() const noexcept { return Ok(); }
};

enum class IkSessionState { Running, Completed, Cancelled };

/**
 * @brief TCP 목표 위치·방향과 현재 관절각에서 목표 관절각을 찾는다.
 * @details 역기구학(IK)은 공구 끝이 목표 위치와 방향에 도달하도록 관절각을 찾는다. 순기구학(FK)은 반대로 관절각에서 공구 끝의 자세를 계산한다.
 * ToolFrame은 공구가 로봇에 붙는 기준점이고 TCP는 실제 작업에 쓰는 공구 끝점이다. tcpInToolFrame으로 두 점 사이의 위치와 방향을 지정한다.
 * 기본값은 두 점이 같은 위치라는 뜻이다. 공구가 길면 실제 공구 끝에 맞는 변환을 전달해야 한다.
 * 목표와 결과 위치는 Robot base 기준이다. Scene에서 로봇 전체를 옮긴 값은 포함하지 않는다.
 * 해석 IK는 특정 로봇 모양에 맞춘 공식을 사용한다. LM은 관절을 조금씩 움직이는 계산에서 목표 오차가 줄었는지 확인해 감쇠값을 조절하므로, 관절 수가 다른 직렬 로봇에도 같은 절차를 쓸 수 있다.
 * 이 계산기는 회전 관절과 직선 관절로 이루어진 직렬 관절 사양을 받는다. URDF를 읽는 일은 모델 로더가 맡으며, 여러 갈래로 나뉘는 팔과 듀얼 암 계산은 지원하지 않는다.
 * 현재 관절각에서 출발해 한 경로만 찾으므로, 계산을 마쳐도 목표에 닿지 못할 수 있다.
 * 로봇 사양은 복사하지 않고 빌려 쓴다. 따라서 사양 자료는 계산기보다 오래 살아 있어야 하며, 같은 계산기를 여러 스레드에서 동시에 사용하면 안 된다.
 */
class LevenbergMarquardtIk final
{
public:
    using SessionId = std::uint64_t;

    /** @brief 사양과 ToolFrame 기준 TCP 고정 변환 [m], quaternion [w,x,y,z]를 검사해 계산기를 만든다. 잘못된 사양이나 변환에는 std::invalid_argument를 던진다. */
    explicit LevenbergMarquardtIk(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame = {});
    /// IK 계산기 수명 동안 사양을 빌리므로 임시 사양은 받을 수 없다.
    LevenbergMarquardtIk(models::RobotSpecification&&, models::Pose3 = {}) = delete;

    /**
     * @brief 현재 관절각에 가까운 목표 관절각을 관절 한계 안에서 반복 계산한다.
     * @param targetInBase TCP 목표 위치 [m]와 quaternion [x,y,z,w]다. 유한하고 길이가 0이 아닌 quaternion은 정규화한다.
     * @param currentSeed 사양 순서의 시작 관절 위치다. 회전 관절은 [rad], 직선 관절은 [m]이며 관절 제한 안에 있어야 한다.
     * @param options 반복 한도와 위치·방향 오차, damping 설정이다. 모든 실수 설정은 유한한 양수여야 한다.
     * @return 성공 시 실행 가능한 목표 관절각이다. 실패 시 상태를 변경하지 않고 실패 이유와 진단 후보를 반환한다.
     */
    [[nodiscard]] IkResult Solve(const CartesianPose& targetInBase, const JointVector& currentSeed, const IkOptions& options = {});

    /**
     * @brief 지정한 시작 관절각 하나에서만 IK를 계산한다.
     * @details 다른 자세 가지를 추가로 탐색하지 않는다. Controller가 각 후보를 따로 충돌 검사할 때 중복 탐색을 막는 용도다.
     */
    [[nodiscard]] IkResult SolveSingleSeed(const CartesianPose& targetInBase,
        const JointVector& seed, const IkOptions& options = {});

    /**
     * @brief 한 관절각 시작점의 IK 계산을 시작하고 식별자를 반환한다.
     * @details 한 번에 반복을 모두 수행하지 않으므로 호출자는 각 프레임에서 StepSingleSeed를 제한된 횟수만큼 호출할 수 있다.
     * 같은 계산기에서 새 세션을 시작하면 이전 세션은 취소되며, 이 계산기는 기존처럼 동시에 여러 스레드에서 사용할 수 없다.
     */
    [[nodiscard]] SessionId BeginSingleSeed(const CartesianPose& targetInBase,
        const JointVector& seed, const IkOptions& options = {});

    /**
     * @brief 지정한 반복 횟수만큼 이어서 계산하고 현재 세션 상태를 반환한다.
     * @param session 세션 식별자다. 새 세션을 시작한 뒤에는 이전 식별자를 사용할 수 없다.
     * @param iterationBudget 이번 호출에서 수행할 LM 갱신 반복의 최대 횟수다. 0이면 상태만 반환한다.
     */
    [[nodiscard]] IkSessionState StepSingleSeed(SessionId session, std::size_t iterationBudget);

    /** @brief 계산 중인 세션을 중단한다. 이미 끝난 세션은 결과를 바꾸지 않는다. */
    void CancelSingleSeed(SessionId session);

    /** @brief 진행 중이거나 끝난 세션의 현재 결과를 읽는다. 새 세션을 시작하면 이전 결과는 더 이상 유효하지 않다. */
    [[nodiscard]] const IkResult& GetSingleSeedResult(SessionId session) const;

    /**
     * @brief 관절각의 FK에 TCP offset을 적용해 Robot base 기준 TCP 자세를 계산한다.
     * @param jointPositions 사양 순서의 관절 위치다. 회전 관절은 [rad], 직선 관절은 [m]다. FK처럼 한계는 검사하지 않는다.
     * @return 모델로 계산한 TCP 위치 [m]와 quaternion [x,y,z,w]다. 물리 장치에서 측정한 feedback은 아니다.
     * @throws std::invalid_argument ToolFrame이 없거나 관절 수·각도가 잘못된 경우다.
     */
    [[nodiscard]] CartesianPose EvaluateTcp(const JointVector& jointPositions);

private:
    void FinishSession(IkStatus status, IkTerminationReason reason, const char* message);
    void FinishAtIterationLimit();

    const models::RobotSpecification& specification_;
    RobotKinematics forward_;
    models::Pose3 tcpInToolFrame_;
    JointVector jointPositionsScratch_;
    JointVector jointStepScratch_;
    JointVector candidateScratch_;
    std::vector<bool> activeJointScratch_;
    std::vector<std::array<double, detail::taskSpaceDimension>> jacobianColumnsScratch_;
    double maximumReachMeters_ = 0.0;
    models::Pose3 sessionTarget_;
    IkOptions sessionOptions_;
    IkResult sessionResult_;
    SessionId nextSessionId_ = 0;
    SessionId activeSessionId_ = 0;
    std::size_t sessionIteration_ = 0;
    double sessionDamping_ = 0.0;
    bool sessionActiveJointLimit_ = false;
    std::size_t sessionStagnantIterations_ = 0;
    IkSessionState sessionState_ = IkSessionState::Cancelled;
};
}
