#pragma once

#include <string_view>

namespace dronesim::simulator
{
/** @brief 실행 파일을 만든 소스 버전을 돌려준다. */
std::string_view BuildRevision() noexcept;
}
