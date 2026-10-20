#pragma once

#include <glm/vec3.hpp>

#include <chrono>
#include <cstdint>

namespace grasplink::application::pick_place::config
{

/** @brief 시연 바닥의 충돌 상자 크기와 Entity 원점에서 옮길 위치를 정한다. 단위는 [m]다. */
inline constexpr glm::vec3 floorHalfExtentsMeters{3.0F, 0.02F, 3.0F};
inline constexpr glm::vec3 floorColliderCenterOffsetMeters{0.0F, -0.015F, 0.0F};

/**
 * @brief 집기·놓기 임무가 Gripper에 보내는 원시 요청값을 모은다.
 * @details 위치·속도·힘 값은 장치에 보내는 code이며 미터, 속도, 힘 단위가 아니다.
 */
struct GripperCommandSettings final
{
    /** @brief 이 시연에서 Gripper를 열거나 닫을 때 보내는 위치 요청 code다. */
    enum class PositionRequest : std::uint8_t
    {
        Open = 0,
        Closed = 255
    };

    /// Gripper 장치에 보낼 속도 code [0,255]다.
    inline static constexpr std::uint8_t speedRequest = 255;
    /// Gripper 장치에 보낼 힘 code [0,255]다.
    inline static constexpr std::uint8_t forceRequest = 128;
};

/** @brief 놓기와 사각형 배치에 필요한 시간·방향 설정을 모은다. */
struct PlacementSettings final
{
    /// 물체를 내려놓은 뒤 Gripper를 열기 전에 기다리는 시간이다.
    inline static constexpr std::chrono::milliseconds settleDuration{400};
    /// 정사각형 물체는 수직축을 기준으로 네 방향이 같은 배치로 취급된다.
    inline static constexpr int equivalentSquareOrientations = 4;
};

inline constexpr double approachHeightMeters = 0.25;
inline constexpr double transitBoxHeightMeters = 0.45;
/**
 * @brief 집기 전 상자를 향해 내려갈 때 유지할 TCP 높이 여유다 [m].
 * @details 현재 장면의 그리퍼 충돌 형상은 상자 중심보다 10 mm 위에서 겹치고, 20 mm에서 간격이 생긴다. 25 mm를 사용해 5 mm 여유를 둔다.
 */
inline constexpr double graspClearanceMeters = 0.025;
inline constexpr double linearVelocityMetersPerSecond = 2.0;
inline constexpr double angularVelocityRadiansPerSecond = 8.0;
inline constexpr double linearAccelerationMetersPerSecondSquared = 30.0;
inline constexpr double angularAccelerationRadiansPerSecondSquared = 120.0;
inline constexpr float boxSideMeters = 0.04F;
inline constexpr float placementAreaSideMeters = boxSideMeters * 1.7320508F;
inline constexpr float placementAreaThicknessMeters = 0.002F;
inline constexpr float boxPositionHeightMeters = 0.026F;
inline constexpr float placementPositionHeightMeters = 0.006F;
inline constexpr float minimumScenarioRadiusMeters = 0.50F;
/** @brief 무작위 상자 위치를 뽑을 XZ 평면 반경의 상한이다 [m]. */
inline constexpr float maximumBoxRadiusMeters = 1.05F;
/** @brief 무작위 목표 위치를 뽑을 XZ 평면 반경의 상한이다 [m]. 목표 도달 가능 여부는 IK와 경로 검사가 판단한다. */
inline constexpr float maximumPlacementRadiusMeters = 1.00F;
inline constexpr float minimumBoxRepeatSeparationMeters = 0.035F;
inline constexpr float minimumPlacementRepeatSeparationMeters = 0.05F;
inline constexpr float scenarioCoordinateLimitMeters = 1.05F;
inline constexpr float scenarioRotationLimitRadians = 1.57079632679F;

}
