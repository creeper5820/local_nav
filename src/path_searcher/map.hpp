/// @note TODO

#pragma once

#include <Eigen/Eigen>

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <vector>

template <size_t _width, size_t _height>
class GridMap {
public:
    using PointType = Eigen::Vector2i;
    using DataType  = std::array<std::array<int8_t, _width>, _height>;
    using PathType  = std::vector<PointType>;

    GridMap() = default;

    explicit GridMap(const DataType& map) {
        assert(!map.empty());
        map_ = map;
    }

    std::array<int8_t, _width>& operator[](int index) {
        assert(index < _height);
        return map_[index];
    }

    constexpr size_t width() { return _width; }

    constexpr size_t height() { return _height; }

private:
    DataType map_;
};
