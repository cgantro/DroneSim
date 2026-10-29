#include "terrain/TerrainGrid.h"

#include <charconv>
#include <iostream>
#include <stdexcept>
#include <string_view>

namespace
{
double ReadNumber(std::string_view text, const char* label)
{
    double value = 0.0;
    const auto [end, error] = std::from_chars(
        text.data(), text.data() + text.size(), value);
    if (error != std::errc{} || end != text.data() + text.size())
        throw std::invalid_argument(std::string("숫자를 읽지 못했습니다: ") + label);
    return value;
}
}

int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cerr << "사용법: terrain_grid_converter 입력.asc 출력.aerotrn "
            "기준높이\n";
        return 2;
    }

    try
    {
        const auto grid = aerosimlab::terrain::TerrainGrid::LoadAsciiGrid(
            argv[1], static_cast<float>(ReadNumber(argv[3], "기준 높이")));
        grid.Save(argv[2]);
        std::cout << "지형 격자를 저장했습니다: " << grid.sampleCount << " x "
            << grid.sampleCount << ", " << grid.spacingXMeters << " m 간격\n";
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
