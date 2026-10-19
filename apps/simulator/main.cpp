#include "ViewerApp.h"

#include <iostream>
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
            else if (argument == "--help")
            {
                std::cout << "Usage: grasplink_simulator [--smoke-test]\n";
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
