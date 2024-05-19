#pragma once

#include "node.hpp"

#include <Eigen/Eigen>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_types.h>
#include <rclcpp/logger.hpp>

#include <vector>

class Preprocess {
public:
    using LivoxCloudType = std::vector<livox_ros_driver2::msg::CustomPoint>;

    Preprocess();

    void set(double blind);
    void set(double resolution, double width);
    void set(Eigen::Affine3d& transform);

    std::unique_ptr<std::vector<type::Node>> livox_preprocess(LivoxCloudType& points);
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
    bool in(const T&& left, const ValT& val, const T&& right) {
        assert(left < right);
        return (val > left && val < right);
    }

    static void
        transform_from_lidar_link(const pcl::PointXYZ& point_origin, pcl::PointXYZ& point_goal);
    static pcl::PointXYZ transform_from_lidar_link(const pcl::PointXYZ& point_origin);
};
