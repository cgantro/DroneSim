#pragma once

#include "model/RobotModel.h"
#include "robotics/models/ModelTypes.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <vector>

namespace grasplink::simulation::robotics
{

/**
 * @brief Pose3 위치와 방향을 어댑터가 사용하는 GLM 행렬로 바꾼다.
 * @details 위치는 미터, quaternion 성분은 w, x, y, z 순서다. GLM 행렬은 float을 쓰므로 변환 뒤 회전 길이를 1로 맞춘다.
 * @param pose 변환할 위치와 방향이다.
 * @return 점을 먼저 회전하고 마지막에 위치만큼 옮기는 행렬이다.
 */
[[nodiscard]] inline glm::mat4 PoseMatrix(const grasplink::robotics::models::Pose3& pose)
{
    const glm::quat rotation{static_cast<float>(pose.rotation.w), static_cast<float>(pose.rotation.x),
        static_cast<float>(pose.rotation.y), static_cast<float>(pose.rotation.z)};
    const glm::vec3 position{static_cast<float>(pose.positionMeters.x),
        static_cast<float>(pose.positionMeters.y), static_cast<float>(pose.positionMeters.z)};
    return glm::translate(glm::mat4{1.0F}, position) *
        glm::mat4_cast(glm::normalize(rotation));
}

/**
 * @brief 공통 기준으로 표현한 pose를 부모 기준 위치와 방향으로 바꾼다.
 * @param parent 부모 pose이며 위치는 [m] 단위다.
 * @param target 부모와 같은 기준으로 표현한 대상 pose다.
 * @return 대상 원점의 부모 기준 위치 [m]와 방향이다.
 */
[[nodiscard]] grasplink::robotics::models::Pose3 RelativeRobotPose(
    const grasplink::robotics::models::Pose3& parent,
    const grasplink::robotics::models::Pose3& target);

/**
 * @brief 관절을 모두 0으로 둔 상태에서 각 URDF 링크의 위치와 방향을 계산한다.
 * @details 결과 배열은 RobotModel::links와 같은 순서이며, 모든 위치와 방향은 base_link 기준이다.
 * 고정 관절과 움직이는 관절의 origin 변환을 함께 누적한다. 또한 activeJoints가 base_link부터 tool0까지 순서대로 이어지는지 확인해 FK와 충돌 도형 생성기가 같은 관절 경로를 사용하게 한다.
 * @param model 링크와 부모·자식 관절 관계를 가진 로봇 모델이다.
 * @return 링크마다 base_link 기준 위치 [m]와 quaternion [w,x,y,z]을 담은 배열이다.
 * @throws std::invalid_argument 기준 링크가 없거나, 일부 링크가 이어지지 않거나, 움직이는 관절 경로가 tool0까지 이어지지 않은 경우.
 */
[[nodiscard]] std::vector<grasplink::robotics::models::Pose3> BuildRobotLinkBindPoses(
    const grasplink::model::RobotModel& model);

} // namespace grasplink::simulation::robotics
