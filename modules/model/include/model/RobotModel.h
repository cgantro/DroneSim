#pragma once

#include "model/ModelResource.h"

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace grasplink::model
{

/**
 * @brief URDF 관절이 링크 사이를 어떤 방식으로 움직이는지 나타낸다.
 * @details Fixed는 움직이지 않는 연결이고, Revolute와 Continuous는 축을 중심으로 회전한다. Prismatic은 축 방향으로 미끄러진다.
 * Revolute와 Prismatic에는 URDF의 위치 한계가 적용되며, Continuous는 회전 범위를 제한하지 않는다.
 */
enum class RobotJointType
{
    Fixed,
    Revolute,
    Continuous,
    Prismatic
};

/**
 * @brief URDF 링크에 적힌 충돌 도형의 종류다.
 * @details 화면에 보이는 GLB 모양과 별도로 충돌 계산에 쓰는 단순한 모양을 고른다.
 */
enum class RobotCollisionType
{
    Box,
    Sphere,
    Cylinder
};

/**
 * @brief 링크 기준 충돌 도형의 모양과 배치를 저장한다.
 * @details 크기와 위치는 미터 단위다. 상자는 x·y·z 길이를, 구는 x에 반지름을, 원기둥은 x에 반지름과 y에 길이를 저장한다.
 * 회전과 위치는 링크 원점을 기준으로 하며, 물리 시뮬레이션이 이 값을 월드 좌표로 옮겨 충돌을 계산한다.
 */
struct RobotCollisionGeometry
{
    RobotCollisionType type = RobotCollisionType::Box;
    glm::dvec3 dimensionsMeters{1.0};
    glm::dvec3 positionMeters{0.0};
    glm::dquat rotation{1.0, 0.0, 0.0, 0.0};
};

/**
 * @brief URDF 링크 이름과 충돌 도형, 화면 모델 연결 정보를 저장한다.
 * @details URDF 링크는 로봇 구조와 충돌 모양을 정의하고, GLB 노드는 화면에 보이는 부품을 제공한다. `visualNodeIndex`는 두 자료를 연결한다.
 * `base_link`는 GLB에 대응 노드가 없어도 되지만 충돌 도형은 필요하다. 끝점인 `tool0`는 화면 노드와 충돌 도형이 없어도 된다.
 */
struct RobotLink
{
    std::string name;
    std::vector<RobotCollisionGeometry> collisionGeometry;
    /// 화면 모델의 부품을 찾을 때 쓰는 번호다. 대응하는 화면 노드가 없는 기준 링크에는 값이 없다.
    std::optional<std::size_t> visualNodeIndex;
};

/**
 * @brief 부모 링크와 자식 링크 사이의 관절 정보를 저장한다.
 * @details `originPositionMeters`와 `originRotation`은 부모 링크에서 관절 중심까지의 고정 변환이다. `axis`는 그 관절 중심 좌표계에서 본 단위 방향이다.
 * 회전 관절의 위치와 속도는 [rad], 직선 관절의 위치는 [m], 최대 속도는 각각 [rad/s] 또는 [m/s]를 쓴다.
 */
struct RobotJoint
{
    std::string name;
    RobotJointType type = RobotJointType::Fixed;
    std::size_t parentLink = 0;
    std::size_t childLink = 0;
    glm::dvec3 originPositionMeters{0.0};
    glm::dquat originRotation{1.0, 0.0, 0.0, 0.0};
    glm::dvec3 axis{1.0, 0.0, 0.0};
    double lowerLimit = 0.0;
    double upperLimit = 0.0;
    double maximumVelocity = 0.0;
    /// 움직이지 않는 고정 관절은 화면에서 별도 관절 축 노드와 연결하지 않는다.
    std::optional<std::size_t> visualNodeIndex;
};

/**
 * @brief 한 로봇 모델의 링크·관절 구조와 팔의 기준 링크를 보관한다.
 * @details `activeJoints`는 `baseLink`에서 `toolLink`까지 이어지는 움직일 수 있는 관절 번호를 시작점부터 차례로 담는다.
 * 이 경로를 FK와 IK가 사용한다. 팔이 갈라지는 구조나 듀얼 암은 이 모델 경로에서 지원하지 않는다.
 */
struct RobotModel
{
    std::string name;
    std::vector<RobotLink> links;
    std::vector<RobotJoint> joints;
    std::vector<std::size_t> activeJoints;
    std::size_t baseLink = 0;
    std::size_t toolLink = 0;
};

/**
 * @brief URDF 구조와 GLB 화면 모델을 함께 읽은 결과다.
 * @details `robot`은 관절 계산과 충돌 형상에, `visual`은 화면 그리기에 쓰며, `directory`는 불러온 자료가 있는 폴더를 가리킨다.
 */
struct LoadedRobotModel
{
    RobotModel robot;
    ModelResource visual;
    std::filesystem::path directory;
};

/**
 * @brief 로봇 폴더의 URDF 구조와 GLB 화면 모델을 읽어 링크 이름으로 연결한다.
 * @param directory 구조 설명과 화면 모델이 함께 들어 있는 폴더다.
 * @return 실행 중 관절 계산·충돌 검사·렌더링에 쓰는 모델 자료다.
 * @throws std::runtime_error 파일이 없거나 모델 연결이 잘못된 경우 발생한다. 축과 한계값은 추정하지 않는다.
 */
[[nodiscard]] LoadedRobotModel LoadRobotModel(const std::filesystem::path& directory);

/** @brief URDF에서 링크·관절 나무를 읽는다. GLB와의 연결 검사는 LoadRobotModel이 수행한다. */
[[nodiscard]] RobotModel LoadRobotUrdf(const std::filesystem::path& path);

} // namespace grasplink::model
