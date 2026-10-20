#include "ViewerApp.h"

#include <exception>
#include <iostream>
#include <string_view>

namespace
{
struct SimulatorCommandLine final
{
    inline static constexpr std::string_view helpOption = "--help";
    inline static constexpr std::string_view smokeTestOption = "--smoke-test";
    inline static constexpr std::string_view robotDirectoryOption = "--robot";
    inline static constexpr std::string_view usage =
        "사용법: grasplink_simulator [--robot <모델 폴더>] [--smoke-test] [--help]\n";
    inline static constexpr std::string_view unknownOptionPrefix = "알 수 없는 옵션: ";
    inline static constexpr std::string_view missingRobotDirectory =
        "--robot 뒤에 모델 폴더 경로를 지정해 주세요.\n";
    inline static constexpr std::string_view fatalErrorPrefix = "[오류] ";
};
}

int main(int argc, char** argv)
{
    try
    {
        grasplink::simulator::ViewerOptions options;
        for (int i = 1; i < argc; ++i)
        {
            const std::string_view argument = argv[i];
            if (argument == SimulatorCommandLine::helpOption)
            {
                std::cout << SimulatorCommandLine::usage;
                return 0;
            }

            if (argument == SimulatorCommandLine::smokeTestOption)
            {
                options.smokeTest = true;
                continue;
            }

            if (argument == SimulatorCommandLine::robotDirectoryOption)
            {
                if (i + 1 >= argc)
                {
                    std::cerr << SimulatorCommandLine::missingRobotDirectory;
                    return 2;
                }
                options.robotDirectory = argv[++i];
                continue;
            }

            std::cerr << SimulatorCommandLine::unknownOptionPrefix << argument << '\n';
            return 2;
        }
        grasplink::simulator::ViewerApp app(options);
        return app.Run();
    }
    catch (const std::exception& e)
    {
        std::cerr
            << SimulatorCommandLine::fatalErrorPrefix
            << e.what()
            << std::endl;

        return -1;
    }
}
