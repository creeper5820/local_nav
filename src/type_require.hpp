#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rclcpp/rclcpp.hpp>

namespace creeper {

using PointType = pcl::PointXYZ;
using PointCloudType = pcl::PointCloud<PointType>;

using LivoxType = livox_ros_driver2::msg::CustomMsg;
using GridType = nav_msgs::msg::OccupancyGrid;

} // namespace creeper