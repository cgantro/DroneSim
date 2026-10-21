#include "AeroSimLabApp.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <shellapi.h>
#endif
#include <exception>
#include <charconv>
#include <iostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace
{
namespace command_line
{
inline constexpr std::string_view helpOption = "--help";
inline constexpr std::string_view smokeTestOption = "--smoke-test";
inline constexpr std::string_view demoOption = "--demo";
inline constexpr std::string_view benchmarkOption = "--benchmark";
inline constexpr std::string_view portOption = "--udp-port";
inline constexpr std::string_view usage =
    "시뮬레이터 실행 옵션\n"
    "--udp-port 번호: 지상통제 연결 포트 (기본값 27015)\n"
    "--demo: 자동 비행 시연\n"
    "--benchmark: 5초 성능 측정\n"
    "--smoke-test: 자동 화면 시험\n";
inline constexpr std::string_view unknownOptionPrefix = "알 수 없는 옵션: ";
inline constexpr std::string_view fatalErrorPrefix = "[오류] ";
}

enum class MessageKind
{
    Information,
    Error
};

void ShowStartupMessage(std::string_view message, MessageKind kind)
{
#ifdef _WIN32
    const std::string utf8(message);
    const int wideLength = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (wideLength <= 0)
        return;
    std::wstring text(static_cast<std::size_t>(wideLength), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, text.data(), wideLength);
    text.pop_back();
    const UINT icon = kind == MessageKind::Error ? MB_ICONERROR : MB_ICONINFORMATION;
    MessageBoxW(nullptr, text.c_str(), L"AeroSimLab", MB_OK | icon);
#else
    if (kind == MessageKind::Error)
        std::cerr << message << '\n';
    else
        std::cout << message;
#endif
}

int RunSimulator(int argc, char** argv)
{
    bool smokeTest = false;
    try
    {
#ifdef __EMSCRIPTEN__
        bool demoMode = true;
        bool networkEnabled = false;
#else
        bool demoMode = false;
        bool networkEnabled = true;
#endif
        bool benchmarkMode = false;
        std::uint16_t udpPort = aerosimlab::communication::defaultPort;
        for (int i = 1; i < argc; ++i)
        {
            const std::string_view argument = argv[i];
            if (argument == command_line::helpOption)
            {
                ShowStartupMessage(command_line::usage, MessageKind::Information);
                return 0;
            }

            if (argument == command_line::smokeTestOption)
            {
                smokeTest = true;
                networkEnabled = false;
                continue;
            }

            if (argument == command_line::demoOption)
            {
                demoMode = true;
                continue;
            }

            if (argument == command_line::benchmarkOption)
            {
                benchmarkMode = true;
                networkEnabled = false;
                continue;
            }

            if (argument == command_line::portOption && i + 1 < argc)
            {
                const std::string_view value(argv[++i]);
                unsigned int port = 0;
                const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), port);
                if (error != std::errc{} || end != value.data() + value.size() || port == 0 || port > 65535)
                {
                    ShowStartupMessage("UDP 포트는 1부터 65535 사이여야 합니다.", MessageKind::Error);
                    return 2;
                }
                udpPort = static_cast<std::uint16_t>(port);
                networkEnabled = true;
                continue;
            }

            ShowStartupMessage(std::string(command_line::unknownOptionPrefix) + std::string(argument),
                MessageKind::Error);
            return 2;
        }
        aerosimlab::simulator::AeroSimLabApp::Options options{
            .smokeTest = smokeTest,
            .udpPort = udpPort,
            .demoMode = demoMode,
            .networkEnabled = networkEnabled,
            .benchmarkMode = benchmarkMode
        };
        aerosimlab::simulator::AeroSimLabApp app(options);
        return app.Run();
    }
    catch (const std::exception& error)
    {
        if (smokeTest)
        {
#ifdef _WIN32
            const std::string message = std::string(command_line::fatalErrorPrefix) + error.what() + "\n";
            OutputDebugStringA(message.c_str());
            std::cerr << message;
#else
            std::cerr << command_line::fatalErrorPrefix << error.what() << '\n';
#endif
            return -1;
        }
        ShowStartupMessage(std::string(command_line::fatalErrorPrefix) + error.what(), MessageKind::Error);
        return -1;
    }
}
}

#ifdef _WIN32
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
    int argumentCount = 0;
    LPWSTR* wideArguments = CommandLineToArgvW(GetCommandLineW(), &argumentCount);
    if (wideArguments == nullptr)
        return RunSimulator(0, nullptr);

    std::vector<std::string> arguments;
    arguments.reserve(static_cast<std::size_t>(argumentCount));
    for (int index = 0; index < argumentCount; ++index)
    {
        const int requiredSize = WideCharToMultiByte(CP_UTF8, 0, wideArguments[index], -1,
            nullptr, 0, nullptr, nullptr);
        std::string argument(static_cast<std::size_t>(requiredSize), '\0');
        WideCharToMultiByte(CP_UTF8, 0, wideArguments[index], -1,
            argument.data(), requiredSize, nullptr, nullptr);
        argument.pop_back();
        arguments.push_back(std::move(argument));
    }
    LocalFree(wideArguments);

    std::vector<char*> argumentPointers;
    argumentPointers.reserve(arguments.size());
    for (std::string& argument : arguments)
        argumentPointers.push_back(argument.data());
    return RunSimulator(static_cast<int>(argumentPointers.size()), argumentPointers.data());
}
#else
int main(int argc, char** argv)
{
    return RunSimulator(argc, argv);
}
#endif
