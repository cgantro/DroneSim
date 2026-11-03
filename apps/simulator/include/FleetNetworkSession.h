#pragma once

#include "communication/DroneSocketTransport.h"
#include "communication/VirtualMeshNetwork.h"
#include "flight_planning/BuildingObstacle.h"

#include <memory>
#include <span>
#include <vector>

namespace dronesim::simulator
{
/**
 * @brief 편대 통신에 필요한 소켓, 가상 연결망, 건물 가림 정보를 함께 소유한다.
 * @details 건물 외곽을 무선 장애물로 한 번 바꾸어 두고, 드론 위치가 갱신될 때
 *          같은 장애물 목록을 연결망에 전달한다. 임무 메시지의 의미는 이 객체가
 *          정하지 않으며, 임무 진행 쪽에서 처리한다.
 */
class FleetNetworkSession final
{
public:
    /** @brief 드론 위치와 건물 형상으로 통신 세션을 준비한다. */
    [[nodiscard]] bool Prepare(
        std::span<const communication::NetworkNodeSnapshot> nodes,
        std::span<const flight_planning::BuildingObstacle> buildings,
        communication::VirtualMeshSettings settings,
        bool reset);

    /** @brief 갱신된 드론 위치를 기존 건물 장애물과 함께 반영한다. */
    void UpdateNodes(std::span<const communication::NetworkNodeSnapshot> nodes);

    /** @brief 소켓과 가상 연결망이 준비되었는지 알려준다. */
    [[nodiscard]] bool IsReady() const noexcept;

    /** @brief 임무 계층에서 메시지 전달과 장애 상태를 다룰 연결망을 돌려준다. */
    [[nodiscard]] communication::VirtualMeshNetwork* VirtualNetwork() noexcept;
    [[nodiscard]] const communication::VirtualMeshNetwork* VirtualNetwork() const noexcept;

    /** @brief 시험과 상태 화면에서 드론별 UDP 포트를 확인할 전송 계층을 돌려준다. */
    [[nodiscard]] const communication::DroneSocketTransport& SocketTransport() const noexcept;

private:
    /** @brief 건물 바닥 높이와 높이로 무선 신호를 막는 3D 범위를 만든다. */
    void SetBuildingObstacles(
        std::span<const flight_planning::BuildingObstacle> buildings);

    communication::DroneSocketTransport transport_;
    std::unique_ptr<communication::VirtualMeshNetwork> link_;
    std::vector<communication::RadioObstacle> obstacles_;
};
}
