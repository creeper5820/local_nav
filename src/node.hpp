#pragma once

// Private
#include "map_maker/preprocess.hpp"
#include "utility/convert.hpp"
#include "utility/param.hpp"

// Third party
#include <Eigen/Core>
#include <pcl/common/transforms.h>
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
        : Node("local_nav")
        , preprocess_(std::make_unique<Preprocess>())
        , map_frame_id_("map_link") {

        RCLCPP_INFO(this->get_logger(), "map compressor start");

        if (true) {
            livox_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10,
                [this](std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) -> void {
                    pointcloud_subscriber_callback(std::move(msg));
                });
        } else {
            pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/fast_lio/cloud_registered", 10,
                [this](std::unique_ptr<sensor_msgs::msg::PointCloud2> msg) -> void {
                    pointcloud_subscriber_callback(std::move(msg));
                });
        }

        using GridT      = nav_msgs::msg::OccupancyGrid;
        using CloudT     = sensor_msgs::msg::PointCloud2;
        map_publisher_   = this->create_publisher<GridT>("/local_nav/map", 10);
        cloud_publisher_ = this->create_publisher<CloudT>("local_nav/cloud", 10);

        static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        this->read_param();
        this->publish_transform();
    }

private:
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> pointcloud_subscription_;
    std::shared_ptr<rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>> livox_subscription_;
    std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry>> position_subscription_;

    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> map_publisher_;
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> cloud_publisher_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

    std::unique_ptr<Preprocess> preprocess_;

    const std::string map_frame_id_;
    bool cost_map_;

private:
    void publish_transform() {

        auto transform  = geometry_msgs::msg::TransformStamped();
        auto rotation   = Eigen::AngleAxisd{};
        auto quaternion = Eigen::Quaterniond{};

        transform.header.stamp            = this->get_clock()->now();
        transform.header.frame_id         = "lidar_link";
        transform.child_frame_id          = map_frame_id_;
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

        preprocess_->set(param::resolution, param::width);
        preprocess_->set(param::blind);
        preprocess_->set(transform);
    }

    void callback_for_test(std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) {
        auto cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
        utility::livox_to_pcl(msg->points, *cloud);

        auto header     = std_msgs::msg::Header{};
        header.frame_id = map_frame_id_;
        header.stamp    = msg->header.stamp;

        auto transform = Eigen::Affine3d{
            Eigen::AngleAxisd{std::numbers::pi, Eigen::Vector3d::UnitX()}
            * Eigen::Translation3d{0, 0, param::ground_height}
        };

        auto pointcloud_publish = pcl::PointCloud<pcl::PointXYZ>{};
        pcl::transformPointCloud(*cloud, pointcloud_publish, transform, true);

        auto pointcloud2 = sensor_msgs::msg::PointCloud2{};
        pcl::toROSMsg(pointcloud_publish, pointcloud2);

        this->cloud_publisher_->publish(pointcloud2);

        this->process(cloud, header, transform);
    }

    void pointcloud_subscriber_callback(std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) {
        static auto packages = std::vector<std::vector<livox_ros_driver2::msg::CustomPoint>>();

        packages.insert(packages.begin(), msg->points);

        auto packed_cloud = std::vector<livox_ros_driver2::msg::CustomPoint>();
        for (auto package : packages) {
            packed_cloud.insert(packed_cloud.end(), package.begin(), package.end());
        }

        if (cost_map_) {
            auto grid             = std::make_unique<nav_msgs::msg::OccupancyGrid>();
            auto data             = preprocess_->generate_cost_map(packed_cloud);
            grid->header.frame_id = map_frame_id_;
            grid->header.stamp    = msg->header.stamp;
            grid->info.resolution = float(param::resolution);
            grid->info.width      = param::grid_width;
            grid->info.height     = param::grid_width;
            grid->data            = std::vector<int8_t>(param::grid_width * param::grid_width);

            for (const auto& node : *data)
                grid->data[node.x + node.y * param::grid_width] = node.value;

            map_publisher_->publish(*grid);

        } else {
            auto map            = nav_msgs::msg::OccupancyGrid();
            auto data           = preprocess_->generate_grid_map(packed_cloud);
            map.header.frame_id = map_frame_id_;
            map.header.stamp    = msg->header.stamp;
            map.info.resolution = float(param::resolution);
            map.info.width      = param::grid_width;
            map.info.height     = param::grid_width;
            map.data            = std::vector<int8_t>(param::grid_width * param::grid_width);

            for (const auto i : *data)
                map.data[i.x + i.y * param::grid_width] = i.value;

            map_publisher_->publish(map);
        }

        if (packages.size() == param::livox_frames)
            packages.pop_back();
    }

    void pointcloud_subscriber_callback(std::unique_ptr<sensor_msgs::msg::PointCloud2> msg) {

        static auto packages = std::vector<sensor_msgs::msg::PointCloud2>();

        packages.insert(packages.begin(), *msg);

        auto packed_cloud = sensor_msgs::msg::PointCloud2();

        for (auto package : packages) {
            packed_cloud.data.insert(
                packed_cloud.data.end(), package.data.begin(), package.data.end());
        }

        auto map            = nav_msgs::msg::OccupancyGrid();
        auto data           = preprocess_->generate_grid_map(packed_cloud);
        map.header.frame_id = map_frame_id_;
        map.header.stamp    = msg->header.stamp;
        map.info.resolution = float(param::resolution);
        map.info.width      = param::grid_width;
        map.info.height     = param::grid_width;
        map.data            = std::vector<int8_t>(param::grid_width * param::grid_width);

        for (const auto i : *data)
            map.data[i.x + i.y * param::grid_width] = i.value;

        map_publisher_->publish(map);

        if (packages.size() == param::livox_frames)
            packages.pop_back();
    }

    void process(
        const std::unique_ptr<pcl::PointCloud<pcl::PointXYZ>>& pointcloud,
        const std_msgs::msg::Header& header, const Eigen::Affine3d& transform) {

        using PointCloudType = pcl::PointCloud<pcl::PointXYZ>;
        static auto packages = std::vector<PointCloudType>();

        packages.insert(packages.begin(), (*pointcloud));

        auto packed_cloud = PointCloudType();
        for (auto package : packages) {
            packed_cloud.insert(packed_cloud.end(), package.begin(), package.end());
        }

        if (cost_map_) {
            auto grid             = std::make_unique<nav_msgs::msg::OccupancyGrid>();
            auto data             = preprocess_->generate_cost_map(packed_cloud);
            grid->header.frame_id = header.frame_id;
            grid->header.stamp    = header.stamp;
            grid->info.resolution = float(param::resolution);
            grid->info.width      = param::grid_width;
            grid->info.height     = param::grid_width;
            grid->data            = std::vector<int8_t>(param::grid_width * param::grid_width);

            for (const auto& node : *data)
                grid->data[node.x + node.y * param::grid_width] = node.value;

            map_publisher_->publish(*grid);

        } else {
            auto map            = nav_msgs::msg::OccupancyGrid();
            auto data           = preprocess_->generate_grid_map(packed_cloud, transform);
            map.header.frame_id = header.frame_id;
            map.header.stamp    = header.stamp;
            map.info.resolution = float(param::resolution);
            map.info.width      = param::grid_width;
            map.info.height     = param::grid_width;
            map.data            = std::vector<int8_t>(param::grid_width * param::grid_width);

            for (const auto i : *data)
                map.data[i.x + i.y * param::grid_width] = i.value;

            map_publisher_->publish(map);
        }

        if (packages.size() == param::livox_frames)
            packages.pop_back();
    }
};
