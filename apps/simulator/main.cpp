#include "ViewerApp.h"

#include <exception>
#include <iostream>
#include <string_view>

namespace
{
namespace command_line
{
inline constexpr std::string_view helpOption = "--help";
inline constexpr std::string_view smokeTestOption = "--smoke-test";
inline constexpr std::string_view robotDirectoryOption = "--robot";
inline constexpr std::string_view usage =
    "사용법: grasplink_simulator [--robot <모델 폴더>] [--smoke-test] [--help]\n";
inline constexpr std::string_view unknownOptionPrefix = "알 수 없는 옵션: ";
inline constexpr std::string_view missingRobotDirectory =
    "--robot 뒤에 모델 폴더 경로를 지정해 주세요.\n";
inline constexpr std::string_view fatalErrorPrefix = "[오류] ";
}
}

int main(int argc, char** argv)
{
    try
    {
        grasplink::simulator::ViewerOptions options;
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
                options.smokeTest = true;
                continue;
            }

            if (argument == command_line::robotDirectoryOption)
            {
                if (i + 1 >= argc)
                {
                    std::cerr << command_line::missingRobotDirectory;
                    return 2;
                }
                options.robotDirectory = argv[++i];
                continue;
            }

            std::cerr << command_line::unknownOptionPrefix << argument << '\n';
            return 2;
        }
        grasplink::simulator::ViewerApp app(options);
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
