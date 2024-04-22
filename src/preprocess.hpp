#pragma once

#include "./param.hpp"
#include "./type_require.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <rclcpp/rclcpp.hpp>

#include <cassert>
#include <memory>

namespace creeper {

class Preprocess {
public:
    Preprocess() { }

    std::unique_ptr<GridType> get_grid(const LivoxType::SharedPtr& msg)
    {

        const auto range = resolution_ * static_cast<float>(width_);

        static auto point_goal_link = PointType();

        auto cloud = std::make_shared<PointCloudType>();
        auto grid = std::make_unique<GridType>();
        auto data = std::vector<int8_t>(width_ * width_, -1);

        grid->header.frame_id = "local_link";
        grid->header.stamp = msg->header.stamp;
        grid->info.resolution = resolution_;
        grid->info.width = width_;
        grid->info.height = width_;

        // Push all points in area to a point cloud valuable
        for (auto point : msg->points) {

            // transformed link
            transform_from_lidar_link({ point.x, point.y, point.z }, point_goal_link);

            // Remove ground
            if (point_goal_link.z < param::ground_height)
                continue;

            if (in(-(range / 2.0f), point_goal_link.x, (range / 2.0f))
                && in(-(range / 2.0f), point_goal_link.y, (range / 2.0f))) {

                cloud->push_back(point_goal_link);
            }
        }

        // Make grid map now
        for (auto point : *cloud) {

            auto x = static_cast<int>((point.x + (range / 2.0f)) / resolution_);
            auto y = static_cast<int>((point.y + (range / 2.0f)) / resolution_);

            data[x + width_ * y] = std::max(
                data[x + width_ * y],
                static_cast<int8_t>(point.z * param::z_weight));
        }

        grid->data = data;

        return grid;
    }

    void set_grid(float resolution, int width)
    {
        resolution_ = resolution;
        width_ = width;
    }

    void set_transform(Eigen::Affine3d& transform)
    {
        transform_ = transform;

        auto point_origin_link = PointType(0.1, 0.1, 0.2);

        auto point_goal_link = transform_from_lidar_link(point_origin_link);

        RCLCPP_INFO(logger_, "(%.2f,%.2f,%.2f)>(%.2f,%.2f,%.2f)",
            point_origin_link.x, point_origin_link.y, point_origin_link.z,
            point_goal_link.x, point_goal_link.y, point_goal_link.z);
    }

private:
    Eigen::Affine3d transform_;
    rclcpp::Logger logger_ = rclcpp::get_logger("preprocess");

    float resolution_;
    int width_;

private:
    template <typename T>
    bool in(const T&& left, const T& val, const T&& right)
    {
        assert(left < right);
        return (val > left && val < right);
    }

    void transform_from_lidar_link(const PointType& point_origin, PointType& point_goal)
    {
        point_goal = {
            point_origin.x,
            -point_origin.y,
            static_cast<float>(-param::transform_translation_z - point_origin.z)
        };
    }

    PointType transform_from_lidar_link(const PointType& point_origin)
    {
        auto point_goal = PointType();
        transform_from_lidar_link(point_origin, point_goal);

        return point_goal;
    }
};
} // namespace creeper