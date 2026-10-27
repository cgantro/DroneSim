#include "terrain/MapTrees.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>
#include <string>

namespace aerosimlab::terrain
{
std::vector<MapTree> LoadMapTrees(
    const std::filesystem::path& path, const TerrainGrid& terrain)
{
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("나무 위치 자료를 열지 못했습니다: " + path.string());

    nlohmann::json document;
    try
    {
        input >> document;
    }
    catch (const nlohmann::json::exception&)
    {
        throw std::runtime_error("나무 위치 자료의 GeoJSON을 읽지 못했습니다.");
    }

    const auto features = document.find("features");
    if (features == document.end() || !features->is_array())
        throw std::runtime_error("나무 위치 자료에 공간 형상이 없습니다.");

    std::vector<MapTree> trees;
    for (const auto& feature : *features)
    {
        const auto geometry = feature.find("geometry");
        const auto properties = feature.find("properties");
        if (geometry == feature.end() || properties == feature.end()
            || !geometry->is_object() || !properties->is_object()
            || geometry->value("type", std::string{}) != "Point")
            continue;

        const auto tags = properties->find("tags");
        if (tags == properties->end() || !tags->is_object()
            || tags->value("natural", std::string{}) != "tree")
            continue;

        const auto coordinates = geometry->find("coordinates");
        if (coordinates == geometry->end() || !coordinates->is_array()
            || coordinates->size() < 2U
            || !(*coordinates)[0].is_number() || !(*coordinates)[1].is_number())
            continue;

        const glm::dvec2 local = terrain.ToLocalMeters(
            (*coordinates)[1].get<double>(), (*coordinates)[0].get<double>());
        if (terrain.ContainsLocalPositionMeters(local))
            trees.push_back({local});
    }
    return trees;
}
}
