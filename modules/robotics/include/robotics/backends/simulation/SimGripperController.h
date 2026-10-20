#pragma once

#include "robotics/core/IGripperController.h"
#include "robotics/models/GripperSpecification.h"

namespace grasplink::robotics::backends::simulation
{

/**
 * @brief Gripper speed code를 Simulation의 기준 관절 각속도 [rad/s]로 바꾸는 설정이다.
 * @details raw code는 장치가 주고받는 0..255 정수다.
 * 이 Simulation은 raw 위치 범위의 양 끝을 열린 비율 0과 닫힌 비율 1로 선형 대응시킨다.
 * 기구 계산은 연속 비율을 사용하고 actualPosition만 표시용 정수로 반올림한다.
 * raw speed는 아래 프로젝트 각속도 범위에 대응하며 제조사 속도 사양이 아니다.
 * 힘과 전류는 계산하지 않으며 접촉 검출은 Simulation 물리 adapter가 제공한다.
 */
struct SimGripperMotionSettings
{
    /// 시뮬레이션에서 raw speed 최솟값에 대응하는 master 각속도 [rad/s].
    double minimumMasterVelocityRadiansPerSecond = 0.1;

    /// 시뮬레이션에서 raw speed 최댓값에 대응하는 master 각속도 [rad/s].
    double maximumMasterVelocityRadiansPerSecond = 1.0;
};

/**
 * @brief 위치·속도 code로 Gripper를 움직이고 외부 물리 접촉 feedback으로 개폐를 멈춘다.
 * @details GetState는 내부 저장소와 분리된 복사본을 반환한다.
 * 연속 개폐 비율을 쓰려면 valid와 closureFractionValid를 확인한다.
 * 목표 도달은 장애물 없는 공간에서 요청 위치에 왔다는 뜻이지 접촉이나 파지 성공이 아니다.
 * GripperSpecification의 관절 배열과 문자열은 빌려 쓰므로 원본이 이 Controller보다 오래 살아야 한다.
 */
class SimGripperController final : public IGripperController
{
public:
    /**
     * @brief 사양을 빌리고 시뮬레이션 속도 설정을 복사해 Controller를 만든다.
     * @param specification raw 범위, nominal closed master 각도와 joint 배열을 제공하는 모델 사양.
     * @param settings raw speed를 대응시킬 양의 master 각속도 범위 [rad/s].
     * @throws std::invalid_argument 사양 범위가 비었거나 분모가 0이고, nominal 각도 또는 설정이 유효하지 않은 경우.
     * @details position/speed raw 범위는 min < max, force 범위는 min <= max여야 한다.
     * nominalMasterClosedRadians와 설정 각속도는 유한한 양수이며 설정 최솟값은 최댓값보다 클 수 없다.
     */
    explicit SimGripperController(
        const models::GripperSpecification& specification,
        SimGripperMotionSettings settings = {});

    /** @brief 연결 전이면 열린 위치의 유효한 Inactive snapshot을 만들고, 연결 상태면 현재 상태를 보존한다. */
    Result Connect() override;

    /** @brief 연결을 끊고 snapshot을 무효화한다. 다음 Connect는 열린 위치에서 새 상태를 시작한다. */
    void Disconnect() noexcept override;

    /** @brief 연결되어 유효한 시뮬레이션 상태인지 반환한다. */
    [[nodiscard]] bool IsConnected() const noexcept override;

    /**
     * @brief 연결된 상태를 즉시 활성화하며, 이미 활성화된 경우 위치와 동작을 보존한다.
     * @details Reset 뒤에는 mode가 Inactive이므로 Activate만으로 이전 목표를 재개하지 않는다. 새 Command가 필요하다.
     */
    Result Activate() override;

    /**
     * @brief 현재 위치를 보존하고 이동을 중단한 뒤 비활성 상태로 전환한다.
     * @details 마지막 요청 echo와 미도달 분류는 보존한다. 다시 활성화해도 이전 요청은 자동 재개되지 않는다.
     */
    Result Reset() override;

    /**
     * @brief 활성화된 상태에서 raw 위치·속도·힘 범위가 유효한 요청을 수락한다.
     * @param command specification 범위 내 raw 요청. 물리 거리·속도·힘 단위가 아니다.
     * @return 연결/활성 상태 오류 또는 범위 오류, 아니면 목표 수락 결과.
     * @details 잘못된 요청은 기존 상태와 목표를 바꾸지 않는다. 유효한 새 요청은 현재 목표를 교체한다.
     */
    Result Command(const GripperCommand& command) override;

    /**
     * @brief 현재 위치를 보존하고 software 이동을 멈춘다.
     * @details 마지막 요청 echo를 유지하고, 미도달 위치는 objectStatus=Moving으로 분류한다.
     * 실제 E-Stop이나 접촉 정지는 아니다.
     */
    Result Stop() override;

    /** @brief 독립된 GripperState 복사본을 반환한다. 연결/closure 유효성 flag를 함께 확인한다. */
    [[nodiscard]] GripperState GetState() const override;

    /**
     * @brief 유효한 양의 시간 동안 연속 개폐 위치를 목표 방향으로 진행한다.
     * @param dtSeconds 경과 시간 [s]. 유한한 양수일 때만 반영한다.
     * @details 목표에 가까워지면 정확히 목표에서 멈추고 raw 위치는 표시용으로만 반올림한다.
     */
    void Update(double dtSeconds) override;

    /**
     * @brief 물리 계산 뒤 발견한 접촉 방향을 기록하고 현재 개폐 위치에서 멈춘다.
     * @details opening은 열면서 닿았는지 뜻한다. 닫으며 닿으면 같은 닫힘 요청을 반복해도 다시 밀어 넣지 않는다.
     * 접촉 여부는 Jolt adapter가 계산하며 이 함수는 파지 성공이나 힘 [N]을 판정하지 않는다.
     */
    void ApplyContactFeedback(bool opening);

    /** @brief 마지막 수락한 명령이 현재 위치보다 닫힌 위치를 요청했는지 반환한다. 접촉 정지 뒤에도 유지된다. */
    [[nodiscard]] bool IsClosingRequested() const noexcept;

    /** @brief 마지막 수락한 명령이 현재 위치보다 열린 위치를 요청했는지 반환한다. */
    [[nodiscard]] bool IsOpeningRequested() const noexcept;

    /** @brief 열기, Reset, Disconnect로 이전 파지를 해제해야 할 때 증가하는 번호를 반환한다. */
    [[nodiscard]] std::uint64_t GetReleaseRevision() const noexcept { return releaseRevision_; }
    /** @brief 범위 검사를 통과한 새 명령을 수락할 때 증가하는 번호를 반환한다. */
    [[nodiscard]] std::uint64_t GetCommandRevision() const noexcept { return commandRevision_; }

private:
    bool IsAtTarget() const noexcept;
    void UpdateRawPosition() noexcept;

    // 그리퍼 사양을 복사하거나 소유하지 않고 빌려 쓴다. 원본은 이 제어기보다 오래 살아야 한다.
    const models::GripperSpecification& specification_;

    // 이 속도 범위는 제조사 사양이 아니라 프로젝트 Simulation backend가 사용하는 가정값이다.
    SimGripperMotionSettings settings_{};
    GripperState state_{};
    bool connected_ = false;

    // 프로토콜 8-bit raw 위치로 반올림하지 않고 기구 계산에 사용하는 연속 위치와 목표다. 값은 0에서 1 사이의 무차원 비율이다.
    double targetClosureFraction_ = 0.0;

    // 현재 명령의 raw speed를 프로젝트 설정값에 대응시켜 정한 자유공간 master 관절의 각속도 [rad/s]다.
    double masterVelocityRadiansPerSecond_ = 0.0;
    bool contactStopped_ = false;
    int requestedDirection_ = 0;
    std::uint64_t releaseRevision_ = 0;
    std::uint64_t commandRevision_ = 0;
};

} // namespace grasplink::robotics::backends::simulation
