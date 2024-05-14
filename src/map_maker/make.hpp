#pragma once

#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>

#include "../utility/param.hpp"

inline nav_msgs::msg::OccupancyGrid make(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud) {
    auto grid = nav_msgs::msg::OccupancyGrid();
}