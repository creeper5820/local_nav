#pragma once

// Private
#include "map_maker/preprocess.hpp"
#include "utility/param.hpp"

// Third party
#include <Eigen/Eigen>
#include <cstddef>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Std
#include <memory>
#include <string>
#include <utility>

class MainProcessNode : public rclcpp::Node {
public:
    explicit MainProcessNode()
        : Node("local_nav") {
        RCLCPP_INFO(this->get_logger(), "map compressor start");

        auto param = std::make_unique<ParamServer>();
        param->read_param();

        this->read_param();

        livox_subscriber_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10,
            [this](std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) -> void {
                livox_subscriber_callback(std::move(msg));
            });

        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map_2d", 10);
    }

private:
    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscriber_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> map_publisher_;
    std::unique_ptr<Preprocess> preprocess_;
    bool debug_;

private:
    void read_param() {
        auto translation = Eigen::Translation3d{
            param::transform_translation_x, param::transform_translation_y,
            param::transform_translation_z};

        auto quaternion = Eigen::Quaterniond(
            param::transform_quaternion_x, param::transform_quaternion_y,
            param::transform_quaternion_z, param::transform_quaternion_w);

        auto transform = Eigen::Affine3d(translation * quaternion);

        debug_ = param::debug;

        preprocess_ = std::make_unique<Preprocess>();
        preprocess_->set(param::resolution, param::width);
        preprocess_->set(param::blind);
        preprocess_->set(transform);
    }

    void livox_subscriber_callback(std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) {

        // RCLCPP_INFO(this->get_logger(), "livox callback");

        if (!debug_) {
            auto grid = preprocess_->make(msg);

            if (!grid->data.empty())
                map_publisher_->publish(*grid);
        }

        if (debug_) {
            auto grid_base = nav_msgs::msg::OccupancyGrid();

            auto const grid_width = static_cast<size_t>(param::width / param::resolution + 1);

            grid_base.header.frame_id = "local_link";
            grid_base.header.stamp    = msg->header.stamp;
            grid_base.info.resolution = param::resolution;
            grid_base.info.width      = grid_width;
            grid_base.info.height     = grid_width;
            grid_base.data            = std::vector<int8_t>(grid_width * grid_width);

            auto data = preprocess_->pointcloud_process(msg);

            for (auto i : data) {
                grid_base.data[i.x + i.y * grid_width] =
                    (i.type == Preprocess::NodeType::BLOCK) ? -1 : 0;
            }

            map_publisher_->publish(grid_base);
        }
    }
};
