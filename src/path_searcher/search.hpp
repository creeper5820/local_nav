#pragma once

#include "../utility/utility.hpp"
#include "./map.hpp"

#include <Eigen/Eigen>

#include <memory>

class JpsSearch {
public:
    JpsSearch(const GridMap::DataType& map)
        : map_(std::make_unique<GridMap>(map))
    {
        utility::info("Make a map",
            " ", map_->width(),
            " ", map_->height());

        initialized_ = true;
    }

    JpsSearch(const GridMap& map)
        : map_(std::make_unique<GridMap>(map))
    {
        utility::info("Make a map",
            " ", map_->width(),
            " ", map_->height());
    }

    bool search(const GridMap::PointType& start, const GridMap::PointType& end, GridMap::PathType& path)
    {
        if (!initialized_) {
            return false;
        }

        return true;
    }

private:
    std::unique_ptr<GridMap> map_;

    bool initialized_ = false;
};
