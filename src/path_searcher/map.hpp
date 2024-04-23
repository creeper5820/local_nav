#pragma once

#include <Eigen/Eigen>

#include <cassert>
#include <cstdint>
#include <vector>

class GridMap {
public:
    using DataType = std::vector<std::vector<uint8_t>>;
    using PointType = Eigen::Vector2i;
    using PathType = std::vector<PointType>;

    explicit GridMap(const std::vector<std::vector<uint8_t>>& map)
    {
        assert(!map.empty());
        map_ = map;
    }

    std::vector<uint8_t>& operator[](int index)
    {
        assert(index < map_.size());
        return map_[index];
    }

    int width()
    {
        return map_.size();
    }

    int height()
    {
        return map_[0].size();
    }

private:
    std::vector<std::vector<uint8_t>> map_;
};
