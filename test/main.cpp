#include "../src/path_searcher/map.hpp"
#include "../src/path_searcher/search.hpp"
#include "../src/utility/utility.hpp"

int main()
{
    auto map = GridMap<2, 2>({ {
        { 11, 22 },
        { 1, 2 },
    } });

    auto search = search::JpsSearch(map);

    utility::info("Hello World!");
    return 0;
}