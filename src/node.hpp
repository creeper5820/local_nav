#pragma once

// Private
#include "./preprocess.hpp"
#include "./type_require.hpp"

// Third party
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Translation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// Std
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

namespace creeper {

class MapCompressor : public rclcpp::Node {
public:
  explicit MapCompressor()
      : Node("map_compressor",
             rclcpp::NodeOptions()
                 .allow_undeclared_parameters(true)
                 .automatically_declare_parameters_from_overrides(true)) {

    read_param();

    using namespace std::chrono_literals;

    livox_subscriber_ = this->create_subscription<LivoxType>(
        "/livox/lidar", 10,
        std::bind(&MapCompressor::livox_subscriber_callback, this,
                  std::placeholders::_1));

    map_publisher_ = this->create_publisher<GridType>("/compressed_map", 10);

    point_cloud2_publisher_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/point_cloud_filtered", 10);
  }

private:
  rclcpp::Subscription<LivoxType>::SharedPtr livox_subscriber_;

  rclcpp::Publisher<GridType>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud2_publisher_;

  bool debug_;
  Preprocess preprocess_;

private:
  void read_param() {
    this->get_parameter<bool>("debug", debug_);

    auto translate = Eigen::Translation3d();
    auto quaternion = Eigen::Quaterniond();

    this->get_parameter<double>("transform.translate.x", translate.x());
    this->get_parameter<double>("transform.translate.y", translate.y());
    this->get_parameter<double>("transform.translate.z", translate.z());

    this->get_parameter<double>("transform.quaternion.x", quaternion.x());
    this->get_parameter<double>("transform.quaternion.y", quaternion.y());
    this->get_parameter<double>("transform.quaternion.z", quaternion.z());
    this->get_parameter<double>("transform.quaternion.w", quaternion.w());

    auto transform = Eigen::Affine3d(translate * quaternion);

    float resolution;
    int width;

    this->get_parameter<float>("grid.resolution", resolution);
    this->get_parameter<int>("grid.width", width);

    preprocess_.set_grid(resolution, width);
    preprocess_.set_transform(transform);
  }

  void livox_subscriber_callback(LivoxType::SharedPtr msg) {

    map_publisher_->publish(*preprocess_.get_grid(msg));

    if (!debug_)
      return;

    auto cloud = std::make_shared<PointCloudType>();
    auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

    for (auto point : msg->points) {
      cloud->push_back(PointType(point.x, point.y, point.z));
    }

    preprocess_.filter(cloud, cloud);

    pcl::toROSMsg(*cloud, *cloud_msg);

    point_cloud2_publisher_->publish(*cloud_msg);
  }
};
} // namespace creeper