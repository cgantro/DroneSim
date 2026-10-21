#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::robotics::kinematics
{

/**
 * @brief 관절 위치에서 계산한 각 관절과 Link의 Robot base 기준 위치·방향을 담는다.
 * @details FK(정방향 기구학)는 관절 위치를 입력받아 로봇 관절과 Link의 위치·방향을 계산한다. 회전 관절 위치는 [rad], 직선 관절 위치는 [m]다.
 * Robot base는 로봇 바닥에 고정된 기준이고 World는 장면 전체의 기준이다.
 * 이 결과에는 robotRoot가 Scene에서 갖는 World 위치와 회전이 아직 더해지지 않았다.
 * 화면 모델과 충돌 계산용 proxy가 같은 결과를 사용하므로 Entity 변환을 FK 입력으로 다시 읽지 않는다.
 * Proxy는 화면 메시 대신 충돌 계산에 쓸 별도 물체다.
 * 위치 단위는 [m]이고 quaternion은 회전 방향을 네 숫자로 저장하며 성분 순서는 [w,x,y,z]다.
 * ToolFrame은 모델에 정한 공구 장착 기준점이며 실제 공구 끝의 TCP나 Controller feedback과는 다른 값이다.
 */
struct RobotKinematicState
{
    /// 초기 모델 자세에 더할 관절별 Local 회전이다. 직선 관절은 항등 회전을 유지한다.
    std::vector<models::QuaternionWxyz> jointLocalRotations;
    /// URDF의 초기 관절값에 더할 관절별 움직임이다. 회전 관절은 [rad], 직선 관절은 [m]다.
    JointVector jointPositions;
    /// RobotSpecification 순서대로 관절 회전 중심의 Robot base 기준 위치 [m]와 누적 회전을 저장한다. Link는 이 회전 중심 사이에 이어지는 로봇 팔 부분이다.
    std::vector<models::Pose3> linkPosesInBaseFrame;
    /// 각 관절의 회전축을 Robot base 좌표계에서 길이 1인 방향으로 저장한다. IK는 이 축과 회전 중심에서 TCP 변화율을 계산한다.
    std::vector<models::Axis3> jointAxesInBaseFrame;
    /// 마지막 관절에 고정 변환을 적용한 ToolFrame의 위치 [m]와 방향이다. Robot base 기준이며 실제 공구 끝 TCP와 다를 수 있다.
    models::Pose3 toolFrameInBaseFrame{};
    /// RobotSpecification에 ToolFrame이 정의되어 있으면 true다. Controller가 실제 TCP feedback을 제공하는지는 별도로 판정한다.
    bool toolFrameValid = false;
};

/**
 * @brief 관절 위치에서 각 관절과 Link의 위치·방향을 로봇 기준 좌표로 계산한다.
 * @details FK(정방향 기구학)는 사양 순서의 관절 위치에서 팔의 관절과 Link 위치·방향을 구한다. 회전 관절은 [rad], 직선 관절은 [m]를 사용한다.
 * Robot base는 로봇 바닥에 고정된 기준 좌표이며 World는 장면 전체의 기준 좌표다.
 * 계산 결과는 Robot base 기준이고 Scene에서 robotRoot에 적용한 위치와 회전은 아직 포함하지 않는다.
 * joints 배열 순서대로 관절이 한 줄로 이어지는 팔 구조를 계산한다.
 * bindPivotMeters는 모델 초기 자세에서 Robot base를 기준으로 측정한 각 관절의 회전 중심 [m]이다.
 * axis는 초기 관절 회전이 항등일 때 관절 Local 좌표계에서 본 회전축 방향이다.
 * Local은 바로 위 부모를 기준으로 하고 World는 부모 회전을 누적한 장면 기준이다.
 * 계산할 때 앞 관절의 회전을 다음 관절 중심까지의 간격과 자식 Link 방향에 차례로 적용한다.
 * 예를 들어 어깨를 돌리면 팔꿈치 중심과 그 아래 Link도 함께 움직인다.
 * specification과 관절 배열·이름은 빌려 쓰므로 이 계산기보다 오래 살아야 한다.
 */
class RobotKinematics final
{
public:
    /**
     * @brief 로봇 관절 사양이 유효한지 확인하고 FK 계산 결과를 저장할 공간을 준비한다.
     * @param specification 유효한 joints 배열과 선택적 ToolFrame을 제공하는 빌린 모델 참조.
     * @throws std::invalid_argument 관절 배열이 비었거나 pivot, 축 또는 지정된 ToolFrame이 유효하지 않을 때.
     */
    explicit RobotKinematics(const models::RobotSpecification& specification);
    /// FK 계산기 수명 동안 사양을 빌리므로 임시 사양은 받을 수 없다.
    RobotKinematics(models::RobotSpecification&&) = delete;

    /**
     * @brief 유효한 Controller 상태 복사본의 관절 위치를 사용해 각 Link 자세를 다시 계산한다.
     * @param state 모델 순서의 관절 위치다. 회전 관절은 [rad], 직선 관절은 [m]이며 TCP feedback은 사용하지 않는다.
     * @return 이 객체가 소유하는 Robot base 기준 결과. 다음 Update 호출에서 같은 저장 공간을 덮어쓴다.
     * @throws std::invalid_argument 상태가 유효하지 않거나 관절 수가 다르거나 관절 위치가 유한수가 아닐 때.
     */
    const RobotKinematicState& Update(const RobotState& state);

    /**
     * @brief 상태 feedback 없이 관절 위치 배열만으로 Robot base 기준 FK를 계산한다.
     * @details IK는 목표 자세에 가까워지는 후보 관절각을 반복 평가하므로 Controller 상태를 만들지 않고 이 함수에 전달한다.
     * 관절 제한은 여기서 적용하지 않으며 Controller와 IK가 각각 검사한다.
     * @param jointPositions RobotSpecification 순서의 관절 위치다. 회전 관절은 [rad], 직선 관절은 [m]다.
     * @return 이 객체가 소유하며 다음 Update에서 덮어쓰는 FK 결과다.
     * @throws std::invalid_argument 관절 수가 다르거나 유한하지 않은 관절 위치가 있을 때.
     */
    const RobotKinematicState& Update(const JointVector& jointPositions);

private:
    const models::RobotSpecification& specification_;
    std::vector<models::Axis3> normalizedJointAxes_;
    RobotKinematicState state_;
};

}
