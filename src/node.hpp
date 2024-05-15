#pragma once

// Private
#include "map_maker/preprocess.hpp"
#include "utility/param.hpp"

// Third party
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Std
#include <cstddef>
#include <memory>
#include <string>
#include <utility>

class MainProcessNode : public rclcpp::Node {
public:
    explicit MainProcessNode()
        : Node("local_nav") {
        RCLCPP_INFO(this->get_logger(), "map compressor start");

        this->read_param();
        this->publish_transform("unity");

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
    bool cost_map_;

private:
    void publish_transform(const std::string& frame_id) {
        auto tf_static_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        auto static_transform_stamp = geometry_msgs::msg::TransformStamped();

        static_transform_stamp.header.stamp    = this->get_clock()->now();
        static_transform_stamp.header.frame_id = "local_link";
        static_transform_stamp.child_frame_id  = frame_id;

        static_transform_stamp.transform.translation.x = 0;
        static_transform_stamp.transform.translation.y = 0;
        static_transform_stamp.transform.translation.z = 0;

        static_transform_stamp.transform.rotation.w = 1;
        static_transform_stamp.transform.rotation.x = 0;
        static_transform_stamp.transform.rotation.y = 0;
        static_transform_stamp.transform.rotation.z = 0;

        tf_static_broadcaster->sendTransform(static_transform_stamp);

        RCLCPP_INFO(
            this->get_logger(), "set transform from %s to %s",
            static_transform_stamp.header.frame_id.c_str(),
            static_transform_stamp.child_frame_id.c_str());
    }

    void read_param() {
        auto translation = Eigen::Translation3d{
            param::transform_translation_x, param::transform_translation_y,
            param::transform_translation_z};

        auto quaternion = Eigen::Quaterniond(
            param::transform_quaternion_x, param::transform_quaternion_y,
            param::transform_quaternion_z, param::transform_quaternion_w);

        auto transform = Eigen::Affine3d(translation * quaternion);

        cost_map_ = param::cost_map;

        preprocess_ = std::make_unique<Preprocess>();
        preprocess_->set(param::resolution, param::width);
        preprocess_->set(param::blind);
        preprocess_->set(transform);
    }

    void livox_subscriber_callback(std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) {

        if (cost_map_) {
            auto grid = preprocess_->make(msg);

            if (!grid->data.empty())
                map_publisher_->publish(*grid);
        }

        if (!cost_map_) {
            auto map = nav_msgs::msg::OccupancyGrid();

            auto const grid_width = static_cast<size_t>(param::width / param::resolution + 1);

            map.header.frame_id = "local_link";
            map.header.stamp    = msg->header.stamp;
            map.info.resolution = param::resolution;
            map.info.width      = grid_width;
            map.info.height     = grid_width;
            map.data            = std::vector<int8_t>(grid_width * grid_width);

            auto data = preprocess_->pointcloud_process(msg);

            for (auto i : data) {
                map.data[i.x + i.y * grid_width] = (i.type == Preprocess::NodeType::BLOCK) ? -1 : 0;
            }

            map_publisher_->publish(map);
        }
    }
};
