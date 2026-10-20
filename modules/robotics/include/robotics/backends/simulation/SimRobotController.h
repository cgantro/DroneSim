#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/planning/JointPathPlanner.h"
#include "robotics/planning/LinearPathPlanner.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/RobotSpecification.h"

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace grasplink::robotics::backends::simulation
{

/**
 * @brief 관절 목표 위치를 향해 위치와 속도를 계산하는 시뮬레이션 제어기다.
 * @details Controller는 관절 상태만 갱신하며 화면 메시, Scene Entity, 물리 Body는 직접 움직이지 않는다.
 * 앱은 RobotState를 RobotKinematics에 보내 FK를 계산한다.
 * FK는 관절 위치에서 Link 위치와 방향을 구하며 어댑터가 이 결과를 화면 계층에 적용한다.
 * MoveJoint는 검증한 관절 공간 직선을 하나의 진행률로 따라가며, 가장 느린 관절에 맞춰 모든 관절을 동기화한다.
 * 각 관절 속도는 모델의 관절별 최대 속도에 velocityScale을 곱한 한계를 넘지 않는다.
 * MoveJoint 가속도는 모델 사양에 없는 값을 장비 한계로 간주하지 않고, 최대 속도에 0.20초 동안 도달하는 Simulation 정책으로 제한한다.
 * 역기구학(IK)은 목표 TCP 자세를 만드는 관절 위치를 찾는 계산이다.
 * MovePose는 IK 결과를 MoveJoint에 보내는 동기 호출이고, 충돌 검사 함수가 있으면 대체 해도 시험한다.
 * BeginPosePlanning은 직전 관절 위치를 먼저 IK 시작값으로 사용하고, 그 계산이 실패할 때만 제한된 대체 자세를 시험한다.
 * MoveLinear은 TCP 직선 위치와 최단 회전 경로의 IK를 계산하고 각 경로 표본에서 안전한 해를 찾는다.
 * TCP는 모델 ToolFrame에 생성자에서 정한 고정 변환을 더한 작업 기준점이며 기본 변환은 항등이다.
 * 목표와 tcpPose feedback은 Robot base 기준이고 Scene의 robotRoot World 변환을 포함하지 않는다.
 * ToolFrame이 있으면 tcpPoseValid는 true이며 이 feedback은 모델 FK 결과이지 물리 장치 측정값이 아니다.
 * ToolFrame이 없으면 관절 이동은 지원하지만 MovePose와 MoveLinear은 Unsupported다.
 * 사양 문자열과 배열은 빌려 쓰므로 원본은 Controller보다 오래 살아야 한다.
 */
class SimRobotController final : public IRobotController
{
public:
    /**
     * @brief 관절 이름·각도 제한·최대 속도를 제공하는 로봇 사양을 참조하는 Controller를 만든다.
     * @param specification 관절 제한과 최대 속도를 제공하는 사양. 배열과 문자열 저장소도 수명 동안 유효해야 한다.
     * @param tcpInToolFrame 모델 ToolFrame 기준 TCP 고정 위치 [m]와 quaternion [w,x,y,z]다. 기본값은 항등이다.
     * @throws std::invalid_argument 로봇 사양이나 공구 변환이 잘못됐거나 관절 최대 속도가 양수가 아닐 때.
     * @details 내부 IK 계산기가 관절 배열과 위치 제한, FK geometry를 검사한다. 각 관절의 최대 속도는 시간에 따른 움직임을 계산하기 위해 Controller가 확인한다.
     */
    explicit SimRobotController(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame = {});
    /// 제어기 수명 동안 사양을 빌리므로 임시 사양은 받을 수 없다.
    SimRobotController(models::RobotSpecification&&, models::Pose3 = {}) = delete;

    /**
     * @brief IK가 만든 관절 자세 후보를 검사하는 함수를 등록한다.
     * @param checker 자세를 허용하면 None을, Viewer의 고정 환경과 겹치면 EnvironmentCollision을 반환한다.
     * @details 경로 계획기는 이 검사기를 부르기 전에 관절 수와 각도의 유한성, 각 관절의 허용 범위를 확인한다.
     * 검사기는 명령을 처리하는 호출 흐름 안에서 실행되므로 등록한 객체가 검사기가 사용되는 동안 살아 있어야 한다.
     * 빈 함수를 등록하면 환경 충돌 검사를 생략한다.
     */
    void SetJointStateValidityChecker(planning::StateValidityChecker checker);
    /** @brief 유효성 검사 실패 원인을 UI에 함께 표시할 설명 함수를 등록한다. */
    void SetJointStateValidityDiagnosticProvider(std::function<std::string()> provider);

    /** @brief 관절을 허용 범위 안의 시작 위치에 두고 유효한 시뮬레이션 상태로 연결한다.
     * @return 성공하며 ToolFrame이 있으면 설정된 TCP의 모델 FK feedback을 제공한다.
     * @details 각 관절은 0이 허용되면 0에서 시작하고, 0이 범위 밖이면 가장 가까운 경계에서 시작한다. 속도는 0으로 둔다.
     */
    Result Connect() override;

    /** @brief 연결을 해제하고 논리 상태를 Disconnected/invalid로 표시하며 속도를 0으로 만든다. */
    void Disconnect() noexcept override;

    /** @brief 현재 software 연결 여부를 반환한다. */
    [[nodiscard]] bool IsConnected() const noexcept override;

    /**
     * @brief 모델에 적힌 순서대로 관절 목표 위치를 받아 다음 Update부터 움직인다.
     * @param command 사양과 같은 순서의 목표 위치다. 회전 관절은 [rad], 직선 관절은 [m]이며 속도·가속도 비율도 포함한다.
     * @return 연결되지 않았거나 관절 수, 값, 비율, 범위가 잘못되면 오류를 반환한다.
     * @details 두 비율은 유한한 (0,1] 값이어야 한다.
     * velocityScale은 모델의 관절별 최대 속도에 곱해 속도 상한을 정한다.
     * 모든 관절은 같은 0..1 진행률을 공유하므로 충돌 검증 때 확인한 직선 관절 경로를 따라간다.
     * accelerationScale은 최대 속도에 도달하는 Simulation ramp 시간을 조절한다. 기본값 1.0은 최대 속도까지 0.20초를 사용한다.
     * 움직이는 중 새 목표를 받으면 검증된 기존 관절 경로에서 감속한 뒤 새 경로를 시작한다.
     * 성공은 목표를 받았다는 뜻이며 도달 여부는 GetState에서 확인한다.
     */
    Result MoveJoint(const JointMoveCommand& command) override;

    /**
     * @brief Robot base 기준 TCP 목표를 IK로 관절 위치로 바꾼 뒤 MoveJoint로 실행한다.
     * @param targetInBase TCP 위치 [m]와 quaternion [x,y,z,w]다. World 배치를 포함하지 않는다.
     * @param velocityScale 모델의 관절별 최대 속도에 곱하는 유한한 (0,1] 비율이다.
     * @param accelerationScale 유한한 (0,1] 비율이며 최대 속도까지의 Simulation ramp 시간을 조절한다.
     * @return IK와 명령 수락 결과다. 실패하면 기존 진행 목표와 현재 상태를 바꾸지 않는다.
     * @details 현재 자세를 먼저 IK 시작값으로 사용하고, 해나 관절 이동 경로가 충돌하면 다른 시작값을 시험한다.
     * 검사에 통과한 첫 해를 사용하며 모든 가능한 IK 해를 열거하지는 않는다.
     * 충돌 검사 함수가 없으면 현재 관절 위치를 시작점으로 구한 해를 사용한다. TCP 이동 경로는 관절 이동에 따른 곡선일 수 있다.
     */
    Result MovePose(const CartesianPose& targetInBase, double velocityScale = 1.0,
        double accelerationScale = 1.0) override;
    /**
     * @brief 목표 TCP 자세의 관절 경로를 계산하고 검증하는 작업을 시작한다.
     * @details IK 반복과 직접 관절 경로 충돌 검사를 작은 작업 단위로 나누어 AdvanceMotionPlanning에서 진행한다. 막힌 경로는 우회하지 않고 실패한다.
     * 검증 callback은 이 Controller를 진행하는 호출 스레드에서 실행하므로 Viewer의 Jolt/ECS 객체를 다른 thread에서 호출하지 않는다.
     * 결과는 경로가 검증된 뒤 MoveJoint 궤적으로 실행하며 관절 waypoint마다 멈춘다. TCP 직선 경로는 보장하지 않는다.
     */
    Result BeginPosePlanning(const CartesianPose& targetInBase, double velocityScale = 1.0,
        double accelerationScale = 1.0) override;
    Result BeginPosePlanningWithLinearContinuation(const CartesianPose& approach,
        const LinearPathMoveCommand& continuation, double velocityScale = 1.0,
        double accelerationScale = 1.0) override;

    /**
     * @brief TCP 목표까지 직선 이동을 요청한다.
     * @param command 공통 API의 TCP 목표 위치·방향, 선속도·각속도 상한과 선·각 가속도 상한이다.
     * @return 연결, 입력, ToolFrame 또는 전체 경로의 IK 검증이 실패하면 오류를 반환하고 기존 동작을 유지한다.
     * @details 목표는 Robot base 기준이며 TCP 위치는 직선, 방향은 최단 quaternion 보간으로 진행한다.
     * TCP 직선 경로를 1 cm 간격으로 나누어 IK로 계획하고 표본 사이 관절 위치를 보간해 실행한다.
     * 유한한 양수 선속도·각속도·가속도와 모델 관절 속도 상한을 적용하며 관절이 느리면 TCP 진행을 늦춘다.
     * 특이 자세에서는 같은 TCP 경로를 유지하며 관절 위치를 작은 폭으로 조정할 수 있다. 이때도 관절/TCP 속도와 FK 오차를 검사한다.
     * 최대 4096개 구간을 넘는 요청은 InvalidCommand이며 표본 사이 모든 자세의 도달 가능성을 수학적으로 보장하지는 않는다.
     * 실행 중 보조 IK가 필요한 특이 자세에서 해를 찾지 못하면 마지막 유효한 관절 위치에서 Fault로 멈추고 속도를 0으로 만든다.
     * TCP 속도는 주어진 가속도 제한에 따라 빠르게 상승하고 목표 전에 감속한다. 충돌 검사 함수가 등록된 경우에만 표본 관절 경로를 충돌 확인한다.
     * 이 확인은 환경 장애물을 돌아가는 대체 TCP 경로를 계획하지 않는다.
     */
    Result MoveLinear(const LinearMoveCommand& command) override;
    Result MoveLinearPath(const LinearPathMoveCommand& command) override;
    Result BeginLinearPathPlanning(const LinearPathMoveCommand& command) override;
    void AdvanceMotionPlanning(std::size_t workBudget) override;
    /** @brief 마지막 MovePose 요청에서 시험한 서로 다른 IK 시작 자세 수를 반환한다. */
    [[nodiscard]] std::size_t GetLastPoseIkSeedAttemptCount() const noexcept;
    [[nodiscard]] bool IsMotionPlanning() const noexcept override;
    std::optional<Result> TakeMotionPlanningResult() override;

    /**
     * @brief 현재 관절 위치에서 Simulation 동작을 멈춘다.
     * @return 연결되지 않았으면 NotConnected를 반환하고 연결 상태에서는 성공한다.
     * @details 현재 관절 위치를 새 목표로 저장하고 속도를 0, mode를 Stopped로 바꾼다.
     * 이는 Simulation 상태 변경이며 장비 보호 정지나 비상 정지가 아니다.
     */
    Result Stop() override;

    /**
     * @brief Controller 상태의 독립된 값 복사본을 반환한다.
     * @return 관절 위치와 속도를 담는다. 회전 관절은 [rad], [rad/s], 직선 관절은 [m], [m/s]다.
     * @details ToolFrame이 정의되어 있으면 관절 FK에 고정 공구 변환을 더한 Robot base TCP를 제공한다.
     * Simulation feedback이므로 실제 장치에서 측정한 TCP는 아니며 기본 offset은 TCP를 ToolFrame에 놓는다.
     * 반환 vector는 내부 저장소를 빌리지 않는다.
     */
    [[nodiscard]] RobotState GetState() const override;

    /** @brief 같은 스레드에서 즉시 읽을 Controller 내부 상태를 const 참조로 반환한다.
     * @details GetState()와 달리 관절 vector를 복사하지 않는다. 다음 Controller 갱신 뒤에도 참조 객체는 유효하지만 값은 새 상태로 바뀌므로 snapshot 보관에는 GetState()를 사용한다.
     */
    [[nodiscard]] const RobotState& GetStateView() const noexcept;

    /**
     * @brief 경과 시간 [s]만큼 각 관절을 목표 위치 쪽으로 움직인다.
     * @details 연결되고 Moving 상태이며 시간이 유한한 양수일 때만 적용한다.
     * MoveJoint는 검증한 관절 공간 직선을 따라 공유 진행률을 올리고, 관절별 속도는 해당 관절의 최대 속도에 velocityScale을 곱한 값 이하다.
     * MoveLinear은 계획한 TCP 직선 자세에 IK를 적용하고 관절 또는 TCP 속도 상한을 넘으면 경로 진행을 줄인다.
     * 회전 관절 속도는 [rad/s], 직선 관절 속도는 [m/s]로 상태에 기록한다. 관절 위치는 가속도 프로파일을 적분해 갱신한다.
     * MoveJoint는 최대 속도까지의 기본 0.20초 ramp를 accelerationScale로 조절해 가속과 감속을 제한한다.
     * MoveJoint가 끝나면 관절 위치를 요청한 값에 맞추고, MoveLinear가 끝나면 TCP를 IK 허용 오차 안에 맞춘다.
     * 동작을 완료하면 속도를 0으로 만든 뒤 Idle로 바꾼다.
     * 잘못된 시간은 오류 없이 무시한다.
     */
    void Update(double dtSeconds) override;

    /** @brief 생성 때 빌린 RobotSpecification을 const 참조로 반환한다.
     * @return Controller 수명 동안 참조 가능한 사양. 소유권은 호출자에게 이전되지 않는다.
     */
    [[nodiscard]] const models::RobotSpecification& GetSpecification() const noexcept;

    /**
     * @brief 물리 충돌 뒤 저장한 안전 관절 자세로 즉시 돌아가고 새 명령을 받을 수 있게 멈춘다.
     * @param safePositions 직전 충돌 검사까지 유지된 관절 위치다. 회전 관절은 [rad], 직선 관절은 [m]다.
     * @param collisionReason 복원하게 된 EnvironmentContact, SelfCollision 또는 AttachedObjectCollision 원인이다.
     * @return 입력 자세가 전부 유한하고 관절 한계 안이면 복원 성공이다. 잘못된 입력은 상태를 바꾸지 않는다.
     * @details 현재 q와 새 q 사이를 보간하지 않는다. 호출자는 새로 저장한 자세가 충돌 검사에 통과한 경우에만 이 함수를 사용한다.
     * 이는 Kinematic Body의 마지막 틱 침투를 화면 기준으로 되돌리는 복구이고 연속 속도 제한 궤적이 아니다.
     * 상태는 Idle이 되며 errorCode에는 전달한 충돌 원인이 남는다. 상위 작업은 이 오류를 보고 안전한 후퇴 동작을 제출할 수 있다.
     */
    bool RestoreCollisionSafeState(const JointVector& safePositions);
    bool RestoreCollisionSafeState(const JointVector& safePositions, ErrorCode collisionReason);

private:
    using LinearPathPoint = planning::LinearPathPoint;

    struct PendingJointMove
    {
        JointVector targetPositions;
        double velocityScale = 1.0;
        double accelerationScale = 1.0;
    };

    // 관절 이동 목표, 가감속 진행 상태, 감속 뒤 실행할 대기 명령을 한 묶음으로 보관한다.
    struct JointMotionState
    {
        JointVector targetPositions;
        JointVector startPositions;
        double progress = 0.0;
        double progressVelocity = 0.0;
        double progressAcceleration = 0.0;
        double progressRate = 0.0;
        double elapsedSeconds = 0.0;
        double accelerationTimeSeconds = 0.0;
        double cruiseTimeSeconds = 0.0;
        double profileDurationSeconds = 0.0;
        double peakProgressVelocity = 0.0;
        double stopProgress = 1.0;
        bool braking = false;
        std::optional<PendingJointMove> pendingMove;

        // 현재 자세를 시작점과 목표로 두고, 이전 이동의 진행·감속·대기 상태를 지운다.
        void Reset(const JointVector& currentPositions)
        {
            targetPositions = currentPositions;
            startPositions = currentPositions;
            ResetProgress();
        }

        // 새 관절 경로를 설정할 때 벡터 저장소를 다시 복사하지 않고 진행 상태만 초기화한다.
        void ResetProgress()
        {
            progress = 0.0;
            progressVelocity = 0.0;
            progressAcceleration = 0.0;
            progressRate = 0.0;
            elapsedSeconds = 0.0;
            accelerationTimeSeconds = 0.0;
            cruiseTimeSeconds = 0.0;
            profileDurationSeconds = 0.0;
            peakProgressVelocity = 0.0;
            stopProgress = 1.0;
            braking = false;
            pendingMove.reset();
        }
    };

    enum class JointMoveOrigin { ExternalJointCommand, PoseTargetCommand, PlannedPoseSegment };
    Result MoveJointImpl(const JointMoveCommand& command, JointMoveOrigin origin);
    Result BeginPosePlanningImpl(const CartesianPose& targetInBase,
        std::optional<LinearPathMoveCommand> continuation,
        double velocityScale, double accelerationScale);
    void ResetMotionPlanningWork();
    void ClearPosePlanningState();
    void FinishPosePlanning(Result result);
    void RetryPosePlanningOrFinish(Result failure);
    void AdvancePoseIkPlanning(std::size_t workBudget);
    void StartPoseLinearContinuation();
    void AdvancePoseLinearContinuation(std::size_t workBudget);
    void AdvancePoseJointPath(std::size_t workBudget);
    void AdvanceLinearPath(std::size_t workBudget);
    [[nodiscard]] bool StartNextPoseIkSeed();
    [[nodiscard]] bool StartPoseCandidateJointPath();
    Result StartPosePathSegment();
    void ConfigureJointMove(JointVector target, double velocityScale, double accelerationScale);
    void AdvanceJointMove(double dtSeconds);
    void FinishJointMoveIfNeeded();
    void RefreshTcp();
    void UpdateLinear(double dtSeconds);
    [[nodiscard]] bool TryAdvanceLinearSegment(double availableSeconds,
        double& pathTimeBudget, double& usedSeconds);
    bool TryAdjustJointsAlongLinearPath(const JointVector& plannedJoints, double availableSeconds);
    std::optional<Result> CommitCachedLinearContinuation(const LinearPathMoveCommand& command);
    Result CommitLinearPathPlan(const LinearPathMoveCommand& command, planning::LinearPathPlan plan);
    Result ValidateLinearPathRequest(const LinearPathMoveCommand& command) const;

    // 로봇 사양을 복사하거나 소유하지 않고 빌려 쓴다. 원본은 이 제어기보다 오래 살아야 한다.
    const models::RobotSpecification& specification_;

    // 현재 관절 위치·속도와 제어 상태를 저장한다.
    RobotState state_{};

    JointMotionState jointMotion_;

    // MoveLinear tick에서 사용할 관절 보간 공간이다. Connect 때 크기를 정해 실행 중 vector 할당을 피한다.
    JointVector linearInterpolationBuffer_;

    // 각 tick 시작 시 관절 위치를 보관해 실제 관절 속도를 계산한다.
    JointVector previousJointPositions_;

    // Simulation backend가 Connect되어 사용 가능한 상태인지 나타낸다.
    bool connected_ = false;

    // IK 계산기는 같은 사양을 빌리며 모델 ToolFrame에 고정 공구 변환을 적용해 TCP를 계산한다.
    kinematics::LevenbergMarquardtIk inverse_;
    planning::StateValidityChecker jointStateValidityChecker_;
    std::function<std::string()> jointStateValidityDiagnosticProvider_;
    planning::LinearPathPlanningJob linearPathPlanningJob_;
    LinearPathMoveCommand pendingLinearPathCommand_;
    std::optional<Result> motionPlanningResult_;
    enum class PosePlanningPhase { None, Ik, LinearContinuation, JointPath };
    // IK 대체 시작점과 이어서 계획할 경로, 관절 경로 계산 작업을 함께 보관한다.
    struct PosePlanningState
    {
        PosePlanningPhase phase = PosePlanningPhase::None;
        kinematics::LevenbergMarquardtIk::SessionId ikSession = 0;
        CartesianPose target{};
        std::vector<JointVector> alternativeSeeds;
        std::size_t alternativeSeedIndex = 0;
        std::size_t ikSeedAttempts = 0;
        Result pathFailure{};
        std::optional<LinearPathMoveCommand> continuationCommand;
        std::optional<planning::LinearPathPlan> continuationPlan;
        JointVector candidateJoints;
        planning::JointPathPlanningJob jointPathJob;
    };
    PosePlanningState posePlanning_;
    JointMoveCommand pendingPoseMove_;
    std::vector<JointVector> posePathPoints_;
    std::size_t posePathSegment_ = 0;
    // TCP 경로 점, 현재 구간, 속도·가속도 한계와 시간 프로파일을 함께 보관한다.
    struct LinearMotionState
    {
        std::vector<LinearPathPoint> path;
        std::size_t segment = 1;
        double segmentFraction = 0.0;
        // 앞선 구간의 시간을 기억해 매 제어 틱마다 경로 처음부터 다시 더하지 않는다.
        double completedPathDurationSeconds = 0.0;
        double linearVelocityLimit = 0.0;
        double angularVelocityLimit = 0.0;
        // 선형 및 회전 가속도 상한으로 사다리꼴 프로파일의 가속·감속 구간 길이를 계산한다.
        double linearAccelerationLimit = 0.0;
        double angularAccelerationLimit = 0.0;
        // 감속과 가속 구간을 포함한 경로의 명목 시간, 실제 프로파일 시간 및 진행 시간을 기록한다.
        double plannedDurationSeconds = 0.0;
        double profileDurationSeconds = 0.0;
        double profileRampFraction = 0.0;
        double profileElapsedSeconds = 0.0;

        void ClearPath() noexcept
        {
            path.clear();
            segment = 1;
            segmentFraction = 0.0;
            completedPathDurationSeconds = 0.0;
            profileElapsedSeconds = 0.0;
        }
    };
    LinearMotionState linearMotion_;
};

} // namespace grasplink::robotics::backends::simulation
