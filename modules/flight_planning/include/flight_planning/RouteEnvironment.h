#pragma once

#include <span>
#include <vector>

#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

namespace dronesim::flight_planning
{
/** @brief 지도 자료를 경로 탐색에 필요한 간단한 장애물 정보로 표현한다.
 *  @details 외곽선은 ENU 평면의 동·북 좌표, 꼭대기는 같은 좌표계의 고도다.
 */
struct RouteObstacle
{
    std::vector<glm::dvec2> footprintMeters;
    double topAltitudeMeters = 0.0;
};

/** @brief 경로 탐색에 필요한 지도 정보만 제공하는 인터페이스다.
 *  @details 좌표는 동·북·상(ENU) 방향의 미터 단위다. 구현체는 경로 탐색이
 *           끝날 때까지 살아 있어야 하며, 반환한 장애물 목록도 유효해야 한다.
 */
class RouteEnvironment
{
public:
    virtual ~RouteEnvironment() = default;

    /** @brief 탐색 가능한 서쪽·남쪽·동쪽·북쪽 경계를 반환한다. */
    [[nodiscard]] virtual glm::dvec4 BoundsMeters() const noexcept = 0;
    /** @brief 위치가 지도 안에 있는지 확인한다. */
    [[nodiscard]] virtual bool Contains(glm::dvec2 positionMeters) const noexcept = 0;
    /** @brief 위치의 지면 고도를 반환한다. */
    [[nodiscard]] virtual double GroundAltitudeMeters(
        glm::dvec2 positionMeters) const = 0;
    /** @brief 경로를 막는 장애물 외곽선과 꼭대기 높이를 반환한다. */
    [[nodiscard]] virtual std::span<const RouteObstacle> Obstacles() const noexcept = 0;
};
}
