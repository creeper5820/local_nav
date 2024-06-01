#pragma once

#include "node.hpp"

#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <vector>

class Process {
public:
    using LivoxCloudType    = std::vector<livox_ros_driver2::msg::CustomPoint>;
    using StandardCloudType = sensor_msgs::msg::PointCloud2;
    using PCLCloudType      = pcl::PointCloud<pcl::PointXYZ>;

    Process();

    void set(double blind);
    void set(double resolution, double width);
    void set(Eigen::Affine3d& transform);

    std::unique_ptr<std::vector<type::Node>> generate_grid_map(
        const pcl::PointCloud<pcl::PointXYZ>& pointcloud, const Eigen::Affine3d& transform);
    std::unique_ptr<std::vector<type::Node>> generate_grid_map(LivoxCloudType& points);
    std::unique_ptr<std::vector<type::Node>> generate_grid_map(StandardCloudType& points);

    std::unique_ptr<std::vector<type::Node>>
        generate_cost_map(const pcl::PointCloud<pcl::PointXYZ>& points);
    std::unique_ptr<std::vector<type::Node>> generate_cost_map(LivoxCloudType& points);

private:
    Eigen::Affine3d transform_;
    rclcpp::Logger logger_;

    double width_;
    double resolution_;
    size_t grid_width_;
    double blind_;

private:
    template <typename T, typename ValT>
    bool in(const T&& left, const ValT& val, const T&& right)
    {
        assert(left < right);
        return (val > left && val < right);
    }

    static void
        transform_from_lidar_link(const pcl::PointXYZ& point_origin, pcl::PointXYZ& point_goal);
    static pcl::PointXYZ transform_from_lidar_link(const pcl::PointXYZ& point_origin);
};
