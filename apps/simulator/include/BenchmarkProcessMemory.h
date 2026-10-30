#pragma once

#include <cstddef>
#include <optional>

#ifdef _WIN32
#include <Windows.h>
#include <Psapi.h>
#endif

namespace dronesim::simulator::detail
{
/** @brief Windows 프로세스 메모리 표본이다. */
struct ProcessMemorySample
{
    std::size_t workingSetBytes = 0;
    std::size_t privateBytes = 0;
};

/** @brief 현재 프로세스 메모리를 읽는다. 지원하지 않는 환경에서는 값이 없다. */
inline std::optional<ProcessMemorySample> ReadProcessMemorySample()
{
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS_EX memory{};
    memory.cb = sizeof(memory);
    if (GetProcessMemoryInfo(GetCurrentProcess(),
            reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&memory), sizeof(memory)))
    {
        return ProcessMemorySample{
            static_cast<std::size_t>(memory.WorkingSetSize),
            static_cast<std::size_t>(memory.PrivateUsage)};
    }
#endif
    return std::nullopt;
}
}
