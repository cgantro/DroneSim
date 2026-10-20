#pragma once

#include "robotics/core/ControlTypes.h"

#include <cstddef>
#include <optional>

namespace grasplink::robotics
{

/**
 * @brief 실제 장치와 Simulation에 공통 관절·공구 목표를 요청하는 인터페이스다.
 * @details RobotSpecification은 관절 이름, 순서, 허용각을 제공한다.
 * TCP는 공구 끝에서 위치와 방향을 지정하는 작업 기준점이다.
 * Joint 명령은 각 관절각을 직접 요청하고 Cartesian 명령은 TCP 위치와 방향을 요청한다.
 * TCP 목표를 관절 움직임으로 바꾸려면 역기구학(IK)이 필요하다.
 * IK는 원하는 공구 위치를 만드는 관절각을 찾는 계산이며 정방향 기구학(FK)의 반대다.
 * 기능 지원, 연결, 오류와 새 명령 처리 방식은 구현마다 다르다.
 * 성공은 요청을 받았다는 뜻일 수 있으므로 목표 도달은 GetState로 확인한다.
 * GetState는 내부 저장소와 분리된 시점별 값 복사본을 반환한다.
 * Stop은 소프트웨어 명령이며 장비 보호 정지나 비상 정지를 대신하지 않는다.
 */
class IRobotController
{
public:
    virtual ~IRobotController() = default;

    /**
     * @brief 구현 연결 또는 초기화를 요청한다.
     * @return 성공 또는 구체적인 실패 분류와 진단. 성공만으로 물리 장비의 안전 상태를 보장하지 않는다.
     */
    virtual Result Connect() = 0;

    /** @brief 구현이 사용한 자원을 정리하고 장치 또는 Simulation 연결을 해제한다. */
    virtual void Disconnect() noexcept = 0;

    /** @brief 새 제어 요청을 처리할 수 있는 연결 상태인지 알려준다. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief 각 관절을 지정한 절대 목표 위치로 움직이도록 요청한다.
     * @param command 모델 순서의 목표 위치다. 회전 관절은 [rad], 직선 관절은 [m]이며 속도·가속도 비율도 포함한다.
     * @return 요청을 받아들였는지 나타낸다. 성공은 목표 도달이나 동작 완료가 아니다.
     * @details 관절 개수·범위와 Busy 처리, 실행 중 명령 교체 여부는 구현 계약에 따른다.
     */
    virtual Result MoveJoint(const JointMoveCommand& command) = 0;

    /**
     * @brief 목표 TCP 자세에 대응하는 관절각을 찾아 관절 공간 경로로 이동하도록 요청한다.
     * @param targetInBase Robot base 기준 TCP 위치 [m]와 방향 quaternion [x,y,z,w]다.
     * @param velocityScale 관절 최대 속도에 적용할 구현별 비율이다.
     * @param accelerationScale 관절 가속도 설정에 적용할 구현별 비율이다.
     * @return 요청 수락 여부와 IK 또는 경로 검증 실패 분류다.
     * @details 목표점만 역기구학으로 계산하므로 TCP가 직선을 따라간다고 보장하지 않는다.
     * 장거리 자유 공간 이동에 사용하고, 직선 접근이 필요하면 MoveLinear을 사용한다.
     * 구현은 이 기능을 지원하지 않으면 Unsupported를 반환할 수 있다.
     */
    virtual Result MovePose(const CartesianPose& targetInBase,
        double velocityScale = 1.0, double accelerationScale = 1.0)
    {
        (void)targetInBase;
        (void)velocityScale;
        (void)accelerationScale;
        return {ErrorCode::Unsupported, "IRobotController: joint-space pose motion is not supported"};
    }

    /** @brief 관절 공간 목표 자세 계획을 시작하며, 증분 실행이 없으면 MovePose를 호출한다. */
    virtual Result BeginPosePlanning(const CartesianPose& targetInBase,
        double velocityScale = 1.0, double accelerationScale = 1.0)
    {
        return MovePose(targetInBase, velocityScale, accelerationScale);
    }

    /**
     * @brief TCP 접근 자세와 그 뒤의 직선 경로를 함께 검사한 뒤 접근 동작을 시작한다.
     * @param approach Robot base 기준 접근 TCP 위치 [m]와 방향 quaternion [x,y,z,w]다.
     * @param continuation 접근 자세에 도달한 뒤 실행할 TCP 직선 경로다.
     * @param velocityScale 접근을 위한 관절 최대 속도에 적용할 비율이다.
     * @param accelerationScale 접근을 위한 관절 가속도에 적용할 비율이다.
     * @return 복합 경로 계획과 접근 동작의 시작 요청 결과다. 기능이 없으면 Unsupported를 반환한다.
     * @details Controller는 접근 자세와 continuation 전체가 유효한 경우에만 관절 공간 접근 동작을 시작한다.
     * 접근 동작이 끝난 뒤 호출자가 continuation을 별도로 요청하면, 구현은 사전 검사한 경로를 재사용할 수 있다.
     */
    virtual Result BeginPosePlanningWithLinearContinuation(const CartesianPose& approach,
        const LinearPathMoveCommand& continuation, double velocityScale = 1.0, double accelerationScale = 1.0)
    {
        (void)approach;
        (void)continuation;
        (void)velocityScale;
        (void)accelerationScale;
        return {ErrorCode::Unsupported, "IRobotController: approach with linear continuation is not supported"};
    }

    /**
     * @brief TCP가 목표 위치와 방향에 이르도록 직선 경로 이동을 요청한다.
     * @param command TCP 위치 [m], quaternion [x,y,z,w], 선속도 [m/s], 각속도 [rad/s], 선가속도 [m/s²], 각가속도 [rad/s²] 상한이다.
     * @return 요청 수락 여부와 실패 분류다. 기능이 없으면 Unsupported를 반환할 수 있다.
     * @details TCP는 공구 끝의 작업 기준점이다.
     * TCP 목표를 만드는 관절각을 찾는 역기구학(IK)과 경로 생성 기능이 있어야 실행할 수 있다.
     * FK는 관절각에서 위치를 계산하므로 FK만 구현한 Controller는 이 명령을 실행할 수 없다.
     * 목표 좌표계는 공통 자료형에서 정하지 않는다.
     */
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /**
     * @brief 여러 TCP 자세를 한 경로로 실행하도록 요청한다.
     * @details 구현은 이 기능을 지원하지 않으면 Unsupported를 반환한다. Simulation처럼 지원하는 구현은 전체 경로에 하나의 속도 프로파일을 적용할 수 있다.
     */
    virtual Result MoveLinearPath(const LinearPathMoveCommand& command)
    {
        if (command.targetPoses.empty())
            return {ErrorCode::InvalidCommand, "IRobotController: linear path requires at least one target pose"};
        return {ErrorCode::Unsupported, "IRobotController: continuous multi-waypoint paths are not supported"};
    }

    /** @brief 선형 경로 계획을 시작한다. 증분 계획을 지원하지 않는 컨트롤러는 기존 동기 실행 방식을 사용한다. */
    virtual Result BeginLinearPathPlanning(const LinearPathMoveCommand& command)
    {
        return MoveLinearPath(command);
    }

    /** @brief 지정한 작업 단위만큼 계획을 진행한다. 한 단위는 IK 반복, 직선성 검사 또는 충돌 검사 한 번이다. */
    virtual void AdvanceMotionPlanning(std::size_t workBudget) { (void)workBudget; }
    [[nodiscard]] virtual bool IsMotionPlanning() const noexcept { return false; }
    virtual std::optional<Result> TakeMotionPlanningResult() { return std::nullopt; }

    /**
     * @brief 구현 안에서 실행 중인 동작을 소프트웨어 방식으로 정지하도록 요청한다.
     * @return 요청 처리 결과. 성공은 구현 수준에서 요청을 처리했다는 뜻이다.
     * @details 물리 E-Stop이나 protective stop 회로의 동작을 보장하거나 대체하지 않는다.
     */
    virtual Result Stop() = 0;

    /**
     * @brief Controller가 보고한 상태의 독립된 복사본을 반환한다.
     * @return 관절 위치와 속도를 담는다. 회전 관절은 [rad], [rad/s], 직선 관절은 [m], [m/s]다.
     * @details valid는 관절을 포함한 전체 상태의 유효성을 나타낸다.
     * tcpPoseValid는 해당 구현의 TCP 위치·방향 feedback만 유효한지 나타낸다. Simulation은 관절 FK와 설정된 공구 offset으로 계산할 수 있다.
     * 오류 code의 뜻은 장치나 Simulation 구현에 따라 다르다.
     * 실제 TCP feedback과 모델 ToolFrame은 같은 위치라고 가정할 수 없다.
     */
    [[nodiscard]] virtual RobotState GetState() const = 0;

    /**
     * @brief Simulation 상태를 경과 시간만큼 진행하거나 실제 장치의 최신 상태를 갱신한다.
     * @param dtSeconds 경과 시간 [s]다. 유효 범위와 처리 방식은 구현이 정한다.
     * @details 실제 장치는 자체 제어 주기로 움직일 수 있다.
     * 따라서 이 호출은 고정된 관절 이동량이나 동작 완료를 보장하지 않는다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
