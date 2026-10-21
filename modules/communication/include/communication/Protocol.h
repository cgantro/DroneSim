#pragma once

#include "simulation_core/DroneModel.h"
#include "simulation_core/EoCameraModel.h"

#include <cstdint>
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>

namespace aerosimlab::communication
{
inline constexpr std::uint16_t defaultPort = 27015;
inline constexpr std::size_t maximumDatagramBytes = 1200;

enum class CameraCommand
{
    Enable,
    Disable,
    InjectFault,
    Recover
};

/** @brief UDP로 받은 한 번의 드론 또는 카메라 명령을 보관한다. */
struct CommandPacket
{
    std::uint32_t commandId = 0;
    std::uint32_t sequence = 0;
    std::optional<simulation::DroneCommand> droneCommand;
    std::optional<CameraCommand> cameraCommand;
};

/** @brief 명령 응답에 실어 지상통제 프로그램으로 돌려보낼 모의 상태다. */
struct Telemetry
{
    std::uint64_t tick = 0;
    double simulationTimeSeconds = 0.0;
    simulation::DroneState drone;
    simulation::CameraState camera;
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
