#include "DroneSimApp.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <shellapi.h>
#endif
#include <exception>
#include <iostream>
#ifdef __EMSCRIPTEN__
#include <memory>
#include <emscripten.h>
#endif
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
inline constexpr std::string_view visibleBenchmarkOption = "--benchmark-visible";
inline constexpr std::string_view usage =
    "시뮬레이터 실행 옵션\n"
    "--demo: 자동 비행 시연\n"
    "--benchmark: 창을 숨기고 화면 성능을 5초 측정\n"
    "--benchmark-visible: 실제 창을 띄워 화면만 5초 측정 (--demo를 함께 쓰면 자동 비행)\n"
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
#ifdef __EMSCRIPTEN__
    if (kind == MessageKind::Error)
    {
        const std::string text(message);
        EM_ASM({ window.showSimulatorStartupError(UTF8ToString($0)); }, text.c_str());
    }
#elif defined(_WIN32)
    const std::string utf8(message);
    const int wideLength = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (wideLength <= 0)
        return;
    std::wstring text(static_cast<std::size_t>(wideLength), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, text.data(), wideLength);
    text.pop_back();
    const UINT icon = kind == MessageKind::Error ? MB_ICONERROR : MB_ICONINFORMATION;
    MessageBoxW(nullptr, text.c_str(), L"DroneSim", MB_OK | icon);
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
        bool demoMode = false;
        bool benchmarkMode = false;
        bool visibleBenchmarkMode = false;
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
                continue;
            }

            if (argument == command_line::visibleBenchmarkOption)
            {
                visibleBenchmarkMode = true;
                continue;
            }

            ShowStartupMessage(std::string(command_line::unknownOptionPrefix) + std::string(argument),
                MessageKind::Error);
            return 2;
        }
        dronesim::simulator::DroneSimApp::Options options{
            .smokeTest = smokeTest,
            .demoMode = demoMode,
            .benchmarkMode = benchmarkMode,
            .visibleBenchmarkMode = visibleBenchmarkMode
        };
#ifdef __EMSCRIPTEN__
        // 브라우저 프레임 함수가 계속 실행되므로 앱을 프로그램이 끝날 때까지 보관한다.
        static std::unique_ptr<dronesim::simulator::DroneSimApp> app;
        app = std::make_unique<dronesim::simulator::DroneSimApp>(options);
        return app->Run();
#else
        dronesim::simulator::DroneSimApp app(options);
        return app.Run();
#endif
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
