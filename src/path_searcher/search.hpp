#pragma once

#include "../utility/utility.hpp"
#include "./map.hpp"

#include <Eigen/Eigen>

#include <array>
#include <list>

namespace search {

template <size_t _width, size_t _height>
class JpsSearch {
public:
    using MapType = GridMap<_width, _height>;

    static inline Eigen::Vector2i UP = { 0, 1 },
                                  DOWN = { 0, -1 },
                                  LEFT = { -1, 0 },
                                  RIGHT = { 1, 0 },
                                  UP_LEFT = { -1, 1 },
                                  UP_RIGHT = { 1, 1 },
                                  DOWN_LEFT = { -1, -1 },
                                  DOWN_RIGHT = { 1, -1 };

    struct SearchNode {
    public:
        MapType::PointType position_;

        SearchNode(const MapType::PointType& position)
            : position_(position)
        {
        }

        SearchNode(int x, int y)
            : position_({ x, y })
        {
        }

        template <size_t n>
        void move(std::array<Eigen::Vector2i, n> directions)
        {
            for (auto& direction : directions) {
                position_.x() += direction.x();
                position_.y() += direction.y();
            }
        }
    };

public:
    JpsSearch(const MapType::DataType& map)
        : map_(map)
    {
        initialized_ = true;
    }

    JpsSearch(const MapType& map)
        : map_(map)
    {
        initialized_ = true;
    }

    bool search(
        MapType::PathType& path,
        const MapType::PointType& start,
        const MapType::PointType& end)
    {
        if (!initialized_) {
            return false;
        }

        open_list_.push_back(start);

        return true;
    }

private:
    MapType map_;

    std::list<SearchNode> open_list_;
    std::list<SearchNode> closed_list_;

    bool initialized_ = false;

private:
    SearchNode find_jump_point(const SearchNode& node)
    {
        auto result = SearchNode { -1, -1 };

        return result;
    }

    int calculate_cost()
    {
        auto result = -1;

        return result;
    }
};

}
