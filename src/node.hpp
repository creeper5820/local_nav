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

        using GridT  = nav_msgs::msg::OccupancyGrid;
        using CloudT = sensor_msgs::msg::PointCloud2;
        RCLCPP_INFO(this->get_logger(), "map compressor start");

        this->read_param();
        this->publish_transform();

        livox_subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10,
            [this](std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) -> void {
                pointcloud_subscriber_callback(std::move(msg));
            });

        map_publisher_   = this->create_publisher<GridT>("/local_nav/map", 10);
        cloud_publisher_ = this->create_publisher<CloudT>("local_nav/cloud", 10);

        static_transform_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
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
        transform.header.frame_id         = map_frame_id_;
        transform.child_frame_id          = "lidar_link";
        transform.transform.translation.x = param::transform_translation_x;
        transform.transform.translation.y = param::transform_translation_y;
        transform.transform.translation.z = param::transform_translation_z;
        transform.transform.rotation.w    = param::transform_quaternion_w;
        transform.transform.rotation.x    = param::transform_quaternion_x;
        transform.transform.rotation.y    = param::transform_quaternion_y;
        transform.transform.rotation.z    = param::transform_quaternion_z;
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
        // map generate process
        auto cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
        utility::livox_to_pcl(msg->points, *cloud);

        auto header     = std_msgs::msg::Header{};
        header.frame_id = map_frame_id_;
        header.stamp    = msg->header.stamp;

        auto q = Eigen::Quaterniond{
            param::transform_quaternion_w, param::transform_quaternion_x,
            param::transform_quaternion_y, param::transform_quaternion_z};
        auto t = Eigen::Translation3d{
            param::transform_translation_x, param::transform_translation_y,
            param::transform_translation_z};

        auto transform = Eigen::Affine3d{q * t};

        this->process(cloud, header, transform);

        // check transformed pointcloud
        auto pointcloud2 = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*cloud, *pointcloud2);
        pointcloud2->header.frame_id = "map_link";
        pointcloud2->header.stamp    = msg->header.stamp;

        this->cloud_publisher_->publish(*pointcloud2);
    }

    void pointcloud_subscriber_callback(std::unique_ptr<livox_ros_driver2::msg::CustomMsg> msg) {
        static auto packages = std::vector<std::vector<livox_ros_driver2::msg::CustomPoint>>();

        packages.insert(packages.begin(), msg->points);

        auto packed_cloud = std::vector<livox_ros_driver2::msg::CustomPoint>();
        for (auto package : packages) {
            packed_cloud.insert(packed_cloud.end(), package.begin(), package.end());
        }

        if (cost_map_) {
            auto grid                    = std::make_unique<nav_msgs::msg::OccupancyGrid>();
            auto data                    = preprocess_->generate_cost_map(packed_cloud);
            grid->header.frame_id        = map_frame_id_;
            grid->header.stamp           = msg->header.stamp;
            grid->info.origin.position.x = -param::width / 2;
            grid->info.origin.position.y = -param::width / 2;
            grid->info.origin.position.z = 0;
            grid->info.resolution        = float(param::resolution);
            grid->info.width             = param::grid_width;
            grid->info.height            = param::grid_width;
            grid->data = std::vector<int8_t>(param::grid_width * param::grid_width);

            for (const auto& node : *data)
                grid->data[node.x + node.y * param::grid_width] = node.value;

            map_publisher_->publish(*grid);

        } else {
            auto map = nav_msgs::msg::OccupancyGrid();

            map.header.frame_id = map_frame_id_;
            map.header.stamp    = msg->header.stamp;

            map.info.origin.position.x = -param::width / 2;
            map.info.origin.position.y = -param::width / 2;
            map.info.origin.position.z = 0;

            map.info.resolution = float(param::resolution);
            map.info.width      = param::grid_width;
            map.info.height     = param::grid_width;

            map.data = std::vector<int8_t>(param::grid_width * param::grid_width);

            auto data = preprocess_->generate_grid_map(packed_cloud);
            for (const auto i : *data)
                map.data[i.x + i.y * param::grid_width] = i.value;

            map_publisher_->publish(map);
        }

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

        auto map                   = nav_msgs::msg::OccupancyGrid();
        map.data                   = std::vector<int8_t>(param::grid_width * param::grid_width);
        map.header.frame_id        = header.frame_id;
        map.header.stamp           = header.stamp;
        map.info.resolution        = float(param::resolution);
        map.info.width             = param::grid_width;
        map.info.height            = param::grid_width;
        map.info.origin.position.x = -param::width / 2;
        map.info.origin.position.y = -param::width / 2;
        map.info.origin.position.z = 0;

        if (cost_map_) {
            auto data = preprocess_->generate_cost_map(packed_cloud);
            for (const auto& node : *data)
                map.data[node.x + node.y * param::grid_width] = node.value;
        } else {
            auto data = preprocess_->generate_grid_map(packed_cloud, transform);
            for (const auto i : *data)
                map.data[i.x + i.y * param::grid_width] = i.value;
        }

        map_publisher_->publish(map);

        if (packages.size() == param::livox_frames)
            packages.pop_back();
    }
};
