#include "ViewerApp.h"

#include <iostream>
#include <charconv>
#include <cstdint>
#include <string>

int main(int argc, char** argv)
{
    try
    {
        grasplink::simulator::ViewerOptions options;
        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];
            if (argument == "--smoke-test") options.smokeTest = true;
            else if (argument == "--pick-place-stress" && i + 1 < argc)
            {
                const std::string value = argv[++i];
                const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), options.pickPlaceStressRuns);
                if (error != std::errc{} || end != value.data() + value.size() || options.pickPlaceStressRuns == 0)
                {
                    std::cerr << "--pick-place-stress requires a positive attempt count\n";
                    return 2;
                }
            }
            else if (argument == "--seed" && i + 1 < argc)
            {
                const std::string value = argv[++i];
                const auto [end, error] = std::from_chars(value.data(), value.data() + value.size(), options.pickPlaceStressSeed);
                if (error != std::errc{} || end != value.data() + value.size())
                {
                    std::cerr << "--seed requires a 32-bit unsigned integer\n";
                    return 2;
                }
            }
            else if (argument == "--failure-log" && i + 1 < argc)
            {
                options.pickPlaceFailureLogPath = argv[++i];
                if (options.pickPlaceFailureLogPath.empty())
                {
                    std::cerr << "--failure-log requires a file path\n";
                    return 2;
                }
            }
            else if (argument == "--help")
            {
                std::cout << "Usage: grasplink_simulator [--smoke-test] [--pick-place-stress COUNT] [--seed UINT32] [--failure-log PATH]\n";
                return 0;
            }
            else
            {
                std::cerr << "Unknown option: " << argument << '\n';
                return 2;
            }
        }
        grasplink::simulator::ViewerApp app(options);
        return app.Run();
    }
    catch (const std::exception& e)
    {
        std::cerr
            << "[Fatal Error] "
            << e.what()
            << std::endl;

        return -1;
    }
}
