#pragma once

#include "model/RobotModel.h"
#include "physics/PhysicsTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <cstddef>
#include <string>
#include <vector>

namespace grasplink::simulation::robotics::collision_geometry
{

/** @brief 한 FK 관절에 연결된 URDF 링크와 해당 링크의 충돌 도형이다. */
struct LinkGeometry
{
    /// URDF에서 읽은 링크 이름이다.
    std::string name;
    /// 관절 상태 결과 배열에서 기준이 되는 움직이는 관절 번호다.
    std::size_t jointIndex = 0;
    /// 기준 관절의 자식 링크에서 이 링크까지의 고정 위치 [m]와 방향이다.
    grasplink::robotics::models::Pose3 poseInJointFrame{};
    /// 이 링크의 URDF 충돌 모양이다.
    std::vector<grasplink::physics::CollisionShapeDescription> shapes;
};

/**
 * @brief 로봇 베이스와 링크별 충돌 도형을 담는다.
 * @details 베이스 도형은 RobotRoot 기준으로 고정된다. 움직이는 관절과 고정 링크의 도형은 가장 가까운 움직이는 조상 링크를 기준으로 돌려준다.
 */
struct Result
{
    std::vector<grasplink::physics::CollisionShapeDescription> baseShapes;
    std::vector<LinkGeometry> links;
};

/** @brief URDF 링크의 충돌 도형을 물리 엔진이 쓰는 모양으로 바꾼다. */
[[nodiscard]] Result Build(
    const grasplink::model::RobotModel& model);

}
