#pragma once

#include "communication/Protocol.h"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>

namespace aerosimlab::ground_control
{
/**
 * @brief UDP 명령 응답과 왕복 시간을 함께 보관한다.
 * @details 화면과 자동 시험이 같은 통신 절차를 사용하도록, 재전송과 응답 확인을 한곳에서 처리한다.
 */
struct CommandResult
{
    communication::CommandReply reply;
    std::uint64_t responseTimeMilliseconds = 0;
};

/** @brief 제한 시간 안에 시뮬레이터 응답을 받지 못했음을 알린다. */
class CommandTimeout final : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

/**
 * @brief 지정한 시뮬레이터에 명령을 보내고 같은 명령의 응답을 기다린다.
 * @param host 숫자 형식의 IPv4 주소다.
 * @param port 시뮬레이터가 기다리는 UDP 포트다.
 * @param packet 재전송할 때도 식별자와 내용이 바뀌지 않는 명령이다.
 * @param timeout 응답을 기다리는 최대 시간이다.
 * @return 시뮬레이터가 돌려준 상태와 요청 시작부터 응답까지 걸린 시간이다.
 * @throws std::exception 주소 변환, 통신 또는 응답 해석에 실패하면 예외를 던진다.
 */
[[nodiscard]] CommandResult SendCommand(
    std::string_view host,
    std::uint16_t port,
    const communication::CommandPacket& packet,
    std::chrono::milliseconds timeout = std::chrono::seconds(4));
}
