#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>

#ifdef _WIN32
#define NOMINMAX
#include <Windows.h>
#endif

namespace aerosimlab::simulator::detail
{
/** @brief 실행 파일이 있는 폴더를 돌려준다. */
inline std::filesystem::path ExecutableDirectory()
{
#ifdef _WIN32
    std::wstring executablePath(32768U, L'\0');
    const DWORD pathLength = GetModuleFileNameW(
        nullptr, executablePath.data(), static_cast<DWORD>(executablePath.size()));
    if (pathLength == 0U || pathLength >= executablePath.size())
        throw std::runtime_error("프로그램이 있는 폴더를 확인하지 못했습니다.");
    executablePath.resize(pathLength);
    return std::filesystem::path(executablePath).parent_path();
#elif defined(__EMSCRIPTEN__)
    return {};
#else
    std::error_code error;
    const auto executablePath = std::filesystem::read_symlink("/proc/self/exe", error);
    if (error)
        throw std::runtime_error("프로그램이 있는 폴더를 확인하지 못했습니다.");
    return executablePath.parent_path();
#endif
}
}
