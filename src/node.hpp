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
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

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

        livox_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10,
            [this](std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) -> void {
                livox_subscriber_callback(std::move(msg));
            });
        position_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fast_lio/position", 10, [this](std::unique_ptr<nav_msgs::msg::Odometry> msg) {
                auto q = Eigen::Quaterniond{
                    msg->pose.pose.orientation.w,
                    msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                };
                auto euler = q.matrix().eulerAngles(0, 1, 2);

                auto position  = geometry_msgs::msg::Pose2D{};
                position.x     = -msg->pose.pose.position.x;
                position.y     = msg->pose.pose.position.y;
                position.theta = -euler.z();

                position_publisher_->publish(position);
            });

        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_nav/map", 10);
        position_publisher_ =
            this->create_publisher<geometry_msgs::msg::Pose2D>("/local_nav/position", 10);
        static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->read_param();
        this->publish_transform();
    }

private:
    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> position_subscription_;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> map_publisher_;
    std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Pose2D>> position_publisher_;
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

        if (cost_map_) {
            auto grid = preprocess_->make(msg);

            if (!grid->data.empty())
                map_publisher_->publish(*grid);
        }

        if (!cost_map_) {
            auto map = nav_msgs::msg::OccupancyGrid();

            auto const grid_width = static_cast<size_t>(param::width / param::resolution + 1);

            map.header.frame_id = "map_2d_link";
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
