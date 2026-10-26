#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/EoCameraModel.h"

#include <cstdint>
#include <cstddef>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace aerosimlab::communication
{
inline constexpr std::uint16_t defaultPort = 27015;
inline constexpr std::size_t maximumDatagramBytes = 1200;
inline constexpr std::size_t maximumFleetDroneCount = 8;

/** @brief 지상통제가 카메라에 보낼 동작이다. */
enum class CameraCommand
{
    Enable,
    Disable,
    InjectFault,
    Recover
};

/** @brief 시뮬레이터의 게임 입력 기록과 재생을 제어한다. */
enum class SessionCommand
{
    StartRecording,
    StopRecording,
    ReplayRecording
};

/** @brief EO 영상을 사용자 데이터 폴더에 녹화한다. */
enum class VideoRecordingCommand
{
    Start,
    Stop
};

/** @brief 준비된 점검 구역 전체를 순서대로 수행한다. */
enum class InspectionCommand
{
    Start
};

/** @brief 시뮬레이터의 여러 드론을 추가하거나 상태를 조회한다. */
enum class FleetCommand
{
    AddDrone,
    GetStatus
};

/** @brief UDP로 받은 비행·카메라·기록·편대 명령 중 하나를 보관한다. */
struct CommandPacket
{
    using Command = std::variant<std::monostate, simulation::DroneCommand,
        CameraCommand, SessionCommand, VideoRecordingCommand, InspectionCommand,
        FleetCommand>;

    std::uint32_t commandId = 0;
    std::uint32_t sequence = 0;
    Command command;
};

/** @brief 편대 목록에 표시할 드론의 간단한 상태다. */
struct DroneTelemetry
{
    std::uint64_t id = 0;
    std::string name;
    simulation::FlightMode mode = simulation::FlightMode::Grounded;
    glm::dvec3 positionMeters{0.0};
};

/** @brief 명령 응답에 실어 지상통제 프로그램으로 돌려보낼 모의 상태다. */
struct Telemetry
{
    std::uint64_t tick = 0;
    double simulationTimeSeconds = 0.0;
    simulation::DroneState drone;
    simulation::CameraState camera;
    std::vector<DroneTelemetry> drones;
    std::string inspectionMissionStatus;
};

/** @brief 지상통제 명령의 접수·적용 결과와 적용 시점이다. */
struct CommandReply
{
    std::uint32_t commandId = 0;
    std::uint32_t sequence = 0;
    bool accepted = false;
    std::string status;
    std::uint64_t receivedAtMilliseconds = 0;
    std::uint64_t appliedTick = 0;
    Telemetry telemetry;
};

/** @brief 고정 버전 JSON 형식으로 명령을 만들고 검사한다. */
[[nodiscard]] std::string EncodeCommand(const CommandPacket& packet);
[[nodiscard]] CommandPacket DecodeCommand(std::string_view datagram);
[[nodiscard]] std::string EncodeReply(const CommandReply& reply);
[[nodiscard]] CommandReply DecodeReply(std::string_view datagram);
[[nodiscard]] std::string EncodeError(std::string_view message);
[[nodiscard]] std::string_view ToString(simulation::CameraStatus status) noexcept;

/** @brief 한 지상국 연결에서 중복·역순 명령을 구분한다. */
class SequenceGuard
{
public:
    enum class Decision { New, Duplicate, OutOfOrder };

    [[nodiscard]] Decision Inspect(const CommandPacket& packet) const noexcept;
    void Commit(const CommandPacket& packet) noexcept;

private:
    bool hasLast_ = false;
    std::uint32_t lastCommandId_ = 0;
    std::uint32_t lastSequence_ = 0;
};
}
