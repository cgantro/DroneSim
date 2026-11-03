#pragma once

#include <cstdint>

namespace dronesim::communication
{
/** @brief 가상 통신망과 UDP 전송 계층이 함께 사용하는 드론 식별값이다. */
using NetworkNodeId = std::uint64_t;
}
