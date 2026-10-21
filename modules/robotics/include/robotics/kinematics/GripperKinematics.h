#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/GripperSpecification.h"

#include <vector>

namespace grasplink::robotics::kinematics
{

/**
 * @brief 한 Gripper의 개폐 위치에서 계산한 관절각과 Local 회전을 담는다.
 * @details master는 움직임 기준 관절을 뜻한다. 이 관절 각도에 GripperSpecification의 nominalMasterClosedRadians와 closureFraction을 곱해 자유공간 닫힘 위치를 계산한다.
 * nominalMasterClosedRadians는 모델이 정한 닫힘 기준 각도 [rad]이며 실제 장치 모터 축 각도가 아니다.
 * 나머지 mimic 관절은 기준 관절 각도에 각자의 masterMultiplier를 곱해 따라 움직인다.
 * 각 관절의 Local 값은 바로 위 부모를 기준으로 한 회전이고, World 값은 부모 회전을 합친 장면 기준이다.
 * quaternion은 회전 방향을 네 숫자 [w,x,y,z]로 저장한다.
 * 이 결과에는 관절의 위치가 없으며 물체 접촉에 맞춰 손가락이 따로 움직이는 파지도 계산하지 않는다.
 */
struct GripperKinematicState
{
    /// mimic 관절의 각도를 정하는 기준 관절의 각도 [rad]다.
    double masterAngleRadians = 0.0;

    /// GripperSpecification 순서대로 저장한 연결 관절의 각도 [rad]다.
    std::vector<double> jointAnglesRadians;

    /// 사양 순서대로 관절에 적용하는 Local 회전 변화다. quaternion은 회전 방향을 네 숫자 [w,x,y,z]로 저장한다.
    std::vector<models::QuaternionWxyz> jointLocalRotations;
};

/**
 * @brief 열린 기준에서 닫힌 기준까지의 비율을 Gripper 관절 회전으로 바꾼다.
 * @details closureFraction 0은 열린 기준이고 1은 물체에 닿지 않았을 때 모델이 정한 닫힘 기준이다.
 * Gripper linkage는 여러 손가락을 잇는 관절 구조이며 각 mimic 관절은 master 관절각에 정해진 배율을 곱해 움직인다.
 * 계산식은 각 관절각 = masterMultiplier × masterAngleRadians다.
 * bind pivot은 초기 모델에서 측정한 각 관절의 회전 중심 위치다.
 * 이 계산기는 그 위치를 사용해 관절 World 위치를 구하지 않고 회전만 반환한다.
 * 물체에 닿은 뒤 손가락이 접촉면에 맞춰 따로 움직이는 적응 파지는 계산하지 않는다.
 * GripperState.valid가 true이면 Inactive 상태에서도 보유한 개폐 위치로 계산할 수 있다.
 * @throws std::invalid_argument 사양 배열, 이름, 축, 각도 계수, 기준 각도 또는 관절 한계가 유효하지 않을 때.
 */
class GripperKinematics final
{
public:
    /**
     * @brief Gripper 관절 사양을 확인하고 결과를 저장할 공간을 만든다.
     * @param specification 관절 배열과 nominalMasterClosedRadians를 제공하는 모델 사양이다.
     * 이 계산기는 사양을 빌려 쓰므로 원본과 그 배열은 계산기보다 오래 살아야 한다.
     * @throws std::invalid_argument 이름 중복, 길이가 0인 축, 유효하지 않은 배율 또는 관절 제한이 기준 각을 포함하지 않을 때 발생한다.
     */
    explicit GripperKinematics(const models::GripperSpecification& specification);
    /// 계산기 수명 동안 사양을 빌리므로 임시 사양은 받을 수 없다.
    GripperKinematics(models::GripperSpecification&&) = delete;

    /**
     * @brief Controller 상태의 개폐 비율로 각 관절의 각도와 Local 회전을 계산한다.
     * @param state closureFraction은 열린 기준 0부터 닫힌 기준 1까지의 비율이다.
     * valid와 closureFractionValid가 모두 true여야 하며 장치의 8-bit raw 위치 code는 사용하지 않는다.
     * @return 이 계산기가 소유한 결과다. 다음 정상 Update가 같은 저장 공간을 덮어쓴다.
     * @throws std::invalid_argument 상태나 비율이 유효하지 않을 때 발생하며 이전 결과는 유지된다.
     */
    const GripperKinematicState& Update(const GripperState& state);

private:
    const models::GripperSpecification& specification_;
    GripperKinematicState state_;
    // 계산 결과를 임시 버퍼에서 먼저 만든 뒤 교환한다. 계산이 실패하면 직전 결과를 유지하고, 성공하면 vector 저장 공간을 다음 update에서 재사용한다.
    GripperKinematicState pending_;
};

} // namespace grasplink::robotics::kinematics
