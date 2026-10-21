#include "AeroSimLabApp.h"

#include <exception>
#include <charconv>
#include <iostream>
#include <string_view>

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
    "사용법: aerosimlab_simulator [--udp-port 번호] [--demo] [--benchmark] [--smoke-test] [--help]\n"
    "기본 UDP 포트는 27015이며, --demo를 주면 자동 비행을 시연하고 --benchmark는 5초 성능을 측정합니다.\n";
inline constexpr std::string_view unknownOptionPrefix = "알 수 없는 옵션: ";
inline constexpr std::string_view fatalErrorPrefix = "[오류] ";
}
}

int main(int argc, char** argv)
{
    try
    {
        bool smokeTest = false;
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
                std::cout << command_line::usage;
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
                    std::cerr << "UDP 포트는 1부터 65535 사이여야 합니다.\n";
                    return 2;
                }
                udpPort = static_cast<std::uint16_t>(port);
                networkEnabled = true;
                continue;
            }

            std::cerr << command_line::unknownOptionPrefix << argument << '\n';
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
    catch (const std::exception& e)
    {
        std::cerr
            << command_line::fatalErrorPrefix
            << e.what()
            << std::endl;

        return -1;
    }
}
