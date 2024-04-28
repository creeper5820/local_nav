#pragma once

// Private
#include "map_maker/preprocess.hpp"
#include "utility/param.hpp"
#include "utility/type_require.hpp"

// Third party
#include <Eigen/Eigen>
#include <livox_ros_driver2/msg/detail/custom_msg__struct.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Std
#include <functional>
#include <memory>
#include <string>

class MainProcessNode : public rclcpp::Node {
public:
    explicit MainProcessNode()
        : Node("local_nav")
    {
        RCLCPP_INFO(this->get_logger(), "map compressor start");

        auto param = std::make_unique<ParamServer>();
        param->read_param();

        this->read_param();

        livox_subscriber_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", 10,
            [this](livox_ros_driver2::msg::CustomMsg::SharedPtr msg) -> void { livox_subscriber_callback(msg); });

        map_publisher_
            = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_2d", 10);
    }

private:
    rclcpp::Subscription<type::LivoxType>::SharedPtr livox_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;

    std::unique_ptr<Preprocess> preprocess_;
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> map_;

    bool debug_;

private:
    void read_param()
    {
        auto translation = Eigen::Translation3d {
            param::transform_translation_x,
            param::transform_translation_y,
            param::transform_translation_z
        };

        auto quaternion = Eigen::Quaterniond(
            param::transform_quaternion_x,
            param::transform_quaternion_y,
            param::transform_quaternion_z,
            param::transform_quaternion_w);

        auto transform = Eigen::Affine3d(translation * quaternion);

        debug_ = param::debug;

        RCLCPP_INFO(this->get_logger(),
            "translation[ %.2lf, %.2lf, %.2lf ]",
            translation.x(),
            translation.y(),
            translation.z());

        RCLCPP_INFO(this->get_logger(),
            "quaternion[ %.2lf, %.2lf, %.2lf, %.2lf ]",
            quaternion.x(),
            quaternion.y(),
            quaternion.z(),
            quaternion.w());

        RCLCPP_INFO(this->get_logger(), "grid[ %.2f, %d ]", param::resolution,
            param::width);

        preprocess_ = std::make_unique<Preprocess>();
        preprocess_->set_grid(param::resolution, param::width);
        preprocess_->set_transform(transform);
    }

    void livox_subscriber_callback(livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        auto grid = preprocess_->get_grid(msg);

        if (!grid->data.empty())
            map_publisher_->publish(*grid);
    }
};
