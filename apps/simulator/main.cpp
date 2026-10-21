#include "AeroSimLabApp.h"

#include <exception>
#include <iostream>
#include <string_view>

namespace
{
namespace command_line
{
inline constexpr std::string_view helpOption = "--help";
inline constexpr std::string_view smokeTestOption = "--smoke-test";
inline constexpr std::string_view usage =
    "사용법: aerosimlab_simulator [--smoke-test] [--help]\n";
inline constexpr std::string_view unknownOptionPrefix = "알 수 없는 옵션: ";
inline constexpr std::string_view fatalErrorPrefix = "[오류] ";
}
}

int main(int argc, char** argv)
{
    try
    {
        bool smokeTest = false;
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
                continue;
            }

            std::cerr << command_line::unknownOptionPrefix << argument << '\n';
            return 2;
        }
        aerosimlab::simulator::AeroSimLabApp app(smokeTest);
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
