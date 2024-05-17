#pragma once

// Private
#include "map_maker/preprocess.hpp"
#include "utility/param.hpp"

// Third party
#include <Eigen/Core>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

// Std
#include <memory>
#include <string>
#include <utility>

class MainProcessNode : public rclcpp::Node {
public:
    explicit MainProcessNode()
        : Node("local_nav") {
        RCLCPP_INFO(this->get_logger(), "map compressor start");

        livox_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10,
            [this](std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) -> void {
                livox_subscriber_callback(std::move(msg));
            });

        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_nav/map", 10);
        static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->read_param();
        this->publish_transform();
    }

private:
    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> position_subscription_;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> map_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

    std::unique_ptr<Preprocess> preprocess_;

    bool cost_map_;

private:
    void publish_transform() {

        auto transform  = geometry_msgs::msg::TransformStamped();
        auto rotation   = Eigen::AngleAxisd{};
        auto quaternion = Eigen::Quaterniond{};

        transform.header.stamp            = this->get_clock()->now();
        transform.header.frame_id         = "lidar_link";
        transform.child_frame_id          = "map_2d_link";
        transform.transform.translation.x = -param::width / 2;
        transform.transform.translation.y = param::width / 2;
        transform.transform.translation.z = 0.7;
        rotation   = Eigen::AngleAxisd{std::numbers::pi, Eigen::Vector3d::UnitX()};
        quaternion = Eigen::Quaterniond{rotation}.normalized();
        transform.transform.rotation.w = quaternion.w();
        transform.transform.rotation.x = quaternion.x();
        transform.transform.rotation.y = quaternion.y();
        transform.transform.rotation.z = quaternion.z();
        static_transform_broadcaster_->sendTransform(transform);
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

        static auto packages = std::vector<std::vector<livox_ros_driver2::msg::CustomPoint>>();
        static auto current  = std::vector<livox_ros_driver2::msg::CustomPoint>();

        packages.insert(packages.begin(), msg->points);

        if (packages.size() == param::livox_frames)
            packages.pop_back();

        current.clear();

        for (auto package : packages) {
            current.insert(current.end(), package.begin(), package.end());
        }

        if (cost_map_) {
            auto grid             = std::make_unique<nav_msgs::msg::OccupancyGrid>();
            auto data             = preprocess_->make(current);
            grid->header.frame_id = "map_2d_link";
            grid->header.stamp    = msg->header.stamp;
            grid->info.resolution = float(param::resolution);
            grid->info.width      = param::grid_width;
            grid->info.height     = param::grid_width;
            grid->data            = std::vector<int8_t>(param::grid_width * param::grid_width);
            for (auto& node : data) {
                grid->data[node.x + node.y * param::grid_width] = node.value;
            }
            map_publisher_->publish(*grid);

        } else {
            auto map            = nav_msgs::msg::OccupancyGrid();
            auto data           = preprocess_->livox_preprocess(current);
            map.header.frame_id = "map_2d_link";
            map.header.stamp    = msg->header.stamp;
            map.info.resolution = static_cast<float>(param::resolution);
            map.info.width      = param::grid_width;
            map.info.height     = param::grid_width;
            map.data            = std::vector<int8_t>(param::grid_width * param::grid_width);
            for (const auto i : data) {
                map.data[i.x + i.y * param::grid_width] = i.value;
            }
            map_publisher_->publish(map);
        }
    }
};
