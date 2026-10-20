#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <optional>
#include <string_view>

namespace grasplink::robotics::models
{

/** @brief 로봇 관절의 움직임과 그 값의 단위를 나타낸다. */
enum class JointMotionType
{
    Revolute,
    Continuous,
    Prismatic
};

inline constexpr double kFullTurnRadians = 6.28318530717958647692;

/**
 * @brief 로봇 관절의 이름, 원점, 축, 위치 제한과 최대 속도를 제공한다.
 * @details bindPivotMeters는 모델 초기 자세에서 Robot base를 기준으로 잰 회전 중심 [m]이다.
 * Robot base는 로봇 바닥에 고정된 기준 좌표다.
 * axis는 관절 원점의 Local 좌표계에서 본 단위 방향이다.
 * Local은 바로 위 부모 기준이고 World는 부모 변환을 합친 장면 기준이다.
 * 회전 관절의 위치와 속도 단위는 [rad], [rad/s]이며 직선 관절은 [m], [m/s]다.
 */
struct JointSpecification
{
    /// GLB node와 Flecs Entity에서 같은 관절을 찾을 때 쓰는 이름이다. 예를 들어 "J1"은 첫 번째 관절을 가리킨다.
    std::string_view name;

    /// 모델 초기 상태(bind pose)에서 Robot base를 기준으로 측정한 관절 회전 중심 위치 [m]다.
    Vec3 bindPivotMeters;

    // 관절 Local 좌표계에서 회전이 일어나는 축의 방향 벡터다. 방향 벡터이므로 물리 단위는 없다.
    Axis3 axis;

    /// 제조사가 정한 관절 위치 범위다. 회전 관절은 [rad], 직선 관절은 [m]다.
    double minimumPosition = 0.0;
    double maximumPosition = 0.0;

    /// 이 관절의 최대 속도다. 회전 관절은 [rad/s], 직선 관절은 [m/s]다.
    double maximumVelocity = 0.0;

    /// 있으면 직전 관절의 자식 링크 좌표계에서 이 관절 원점까지의 고정 위치 [m]와 방향이다. 첫 관절은 base_link 기준이다.
    std::optional<Pose3> parentToJointOrigin;

    /// 관절이 회전하는지, 연속 회전하는지, 축을 따라 움직이는지 나타낸다.
    JointMotionType motionType = JointMotionType::Revolute;

    /// 연속 회전 관절은 끝 각도가 없고, 나머지 관절은 위치 한계를 따른다.
    [[nodiscard]] bool HasPositionLimits() const noexcept
    {
        return motionType != JointMotionType::Continuous;
    }
};

/** @brief Link 이름을 그 Link를 움직이는 joints 배열 원소에 연결한다. */
struct LinkSpecification
{
    /// GLB와 Flecs Entity에서 Link를 찾는 이름이다. 같은 이름이 양쪽에서 대응해야 한다.
    std::string_view name;
    /// joints 배열에서 이 Link를 움직이는 관절의 0부터 시작하는 위치다.
    std::size_t jointIndex = 0;
};

/**
 * @brief 로봇 이름과 관절·Link·ToolFrame 계산에 필요한 모델 기준값을 제공한다.
 * @details 이 구조체는 문자열과 배열을 복사하거나 소유하지 않고 원본을 가리킨다.
 * 원본 데이터는 이를 쓰는 Controller와 기구학 계산기보다 오래 살아야 한다.
 * joints 배열 순서는 RobotState와 관절 명령의 순서다.
 * Link는 인접한 관절 사이의 팔 부분이며 links 배열은 각 Link를 움직이는 관절 번호와 이름을 연결한다.
 * FK(정방향 기구학)는 관절각에서 각 관절과 Link의 위치·방향을 계산한다.
 * hasToolFrame이 true이면 마지막 관절에 고정 변환 toolFrameInLastJoint를 더해 모델 ToolFrame을 계산한다.
 * ToolFrame은 공구 장착 기준점이며 실제 공구 끝 TCP나 Controller feedback과 같다고 보장되지 않는다.
 */
struct RobotSpecification
{

    /// UI나 로그에서 보여 줄 제조사 이름이다.
    std::string_view manufacturer;

    /// 장치 또는 로봇 제품을 식별하는 모델명이다.
    std::string_view model;

    /// RobotState 관절 값과 같은 순서의 관절 배열.
    const JointSpecification* joints = nullptr;

    /// 상태·명령 관절 배열 길이의 기준.
    std::size_t jointCount = 0;

    /// 각 Link를 어느 관절이 움직이는지 나타내는 배열이다. 이 배열 데이터는 빌리지 않으므로 specification 사용이 끝날 때까지 원본이 살아 있어야 한다.
    const LinkSpecification* links = nullptr;
    /// links가 가리키는 Link 대응 배열의 원소 개수다.
    std::size_t linkCount = 0;

    /// 마지막 관절 원점에서 모델의 ToolFrame까지의 변하지 않는 위치 [m]와 회전이다. ToolFrame은 장착 공구의 실제 TCP와 다를 수 있다.
    Pose3 toolFrameInLastJoint{};
    /// true이면 FK가 마지막 관절 자세에 toolFrameInLastJoint를 적용해 ToolFrame 자세를 계산한다. 이 값은 Controller의 TCP feedback 유효성과 무관하다.
    bool hasToolFrame = false;
};

} // namespace grasplink::robotics::models
