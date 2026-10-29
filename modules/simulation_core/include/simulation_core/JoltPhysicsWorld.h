#pragma once

#include <glm/gtc/quaternion.hpp>
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace aerosimlab::terrain {
struct TerrainGrid;
}

namespace aerosimlab::simulation {
struct QuadrotorParameters;

/** @brief 한 Jolt 물리 월드 안의 강체를 가리킨다. */
struct JoltRigidBodyHandle {
  std::uint32_t value = 0;
};

/** @brief 상자 강체의 크기와 시작 상태다. 길이와 질량은 미터와 킬로그램 단위다.
 */
struct JoltBoxDefinition {
  glm::dvec3 positionMeters{0.0};
  glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 halfExtentsMeters{0.5};
  double massKilograms = 1.0;
  double friction = 0.5;
  double restitution = 0.0;
};

/** @brief 강체의 위치·자세와 ENU 기준 속도다. */
struct JoltRigidBodyState {
  glm::dvec3 positionMeters{0.0};
  glm::dquat orientation{1.0, 0.0, 0.0, 0.0};
  glm::dvec3 linearVelocityMetersPerSecond{0.0};
  glm::dvec3 angularVelocityRadiansPerSecond{0.0};
};

/**
 * @brief Jolt로 상자 강체와 바닥 충돌을 계산한다.
 * @details 공개 좌표는 동·북·위(ENU)이고 내부 계산은 Jolt의 북·위·서 좌표를
 * 쓴다. 한 번의 StepFixed는 1/60초를 진행한다.
 */
class JoltPhysicsWorld final {
public:
  JoltPhysicsWorld();
  ~JoltPhysicsWorld();

  JoltPhysicsWorld(const JoltPhysicsWorld &) = delete;
  JoltPhysicsWorld &operator=(const JoltPhysicsWorld &) = delete;

  /** @brief 고정된 상자 충돌체를 만든다. 중심 위치와 반크기를 지정한다. */
  [[nodiscard]] JoltRigidBodyHandle
  AddStaticBox(const glm::dvec3 &centerMeters,
               const glm::dvec3 &halfExtentsMeters);

  /** @brief 지형 격자와 같은 표면 높이를 가진 정적 충돌 지형을 만든다. */
  void SetTerrain(const aerosimlab::terrain::TerrainGrid &terrain);

  /** @brief 외곽선과 높이로 건물·구조물의 정적 충돌 메시를 만든다. */
  void AddStaticPrism(const std::vector<glm::dvec2> &outlineMeters,
                      double bottomMeters, double topMeters);

  /** @brief 모터에 작용할 월드 기준 바람 속도를 지정한다. */
  void SetWindVelocityMetersPerSecond(const glm::dvec3 &velocity);

  /** @brief 질량과 상자 모양을 가진 움직이는 강체를 만든다. */
  [[nodiscard]] JoltRigidBodyHandle
  CreateDynamicBox(const JoltBoxDefinition &definition);

  /** @brief X500 질량·관성·로터 배치로 움직이는 기체를 만든다. */
  [[nodiscard]] JoltRigidBodyHandle
  CreateQuadrotor(const JoltBoxDefinition &body,
                  const QuadrotorParameters &parameters);

  /** @brief 네 모터의 목표 회전 속도를 라디안/초로 지정한다. */
  void SetMotorTargets(JoltRigidBodyHandle body,
                       const std::array<double, 4> &speedRadiansPerSecond);

  /** @brief 첫 물리 단계 전에 모터의 현재 회전 속도를 설정한다. */
  void
  SetInitialMotorSpeeds(JoltRigidBodyHandle body,
                        const std::array<double, 4> &speedRadiansPerSecond);

  /** @brief 모터의 현재 회전 속도를 라디안/초로 읽는다. */
  [[nodiscard]] std::array<double, 4>
  MotorSpeedsRadiansPerSecond(JoltRigidBodyHandle body) const;

  /** @brief 현재 모터 속도에서 만들어진 각 로터의 추력을 뉴턴으로 읽는다. */
  [[nodiscard]] std::array<double, 4>
  MotorThrustsNewtons(JoltRigidBodyHandle body) const;

  /** @brief 강체에 한 번의 물리 단계 동안 작용할 힘을 가한다. */
  void ApplyForce(JoltRigidBodyHandle body, const glm::dvec3 &forceNewtons);

  /** @brief 강체에 한 번의 물리 단계 동안 작용할 토크를 가한다. */
  void ApplyTorque(JoltRigidBodyHandle body,
                   const glm::dvec3 &torqueNewtonMeters);

  /** @brief 기존 강체의 위치·자세·속도를 지정한 상태로 바꾼다. */
  void SetState(JoltRigidBodyHandle body, const JoltRigidBodyState &state);

  /** @brief 물리 시계를 1/60초 진행한다. */
  void StepFixed(double *stepDurationMicroseconds = nullptr);

  /** @brief 현재 위치·자세·속도를 읽는다. */
  [[nodiscard]] JoltRigidBodyState State(JoltRigidBodyHandle body) const;

  /** @brief 강체를 월드에서 제거하고 자원을 해제한다. */
  [[nodiscard]] bool DestroyBody(JoltRigidBodyHandle body);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace aerosimlab::simulation
