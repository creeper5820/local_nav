#pragma once

#include "./type_require.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <rclcpp/rclcpp.hpp>

#include <cassert>
#include <memory>

namespace creeper {

class Preprocess {
public:
  Preprocess() {

    transform_ = Eigen::Quaterniond(0, 0, 0, 1) * Eigen::Translation3d(0, 0, 0);

    resolution_ = 0.1;
    width_ = 30;
  }

  GridType::UniquePtr get_grid(const LivoxType::SharedPtr &msg) {

    if (msg->point_num < 1000) {
      return nullptr;
      RCLCPP_WARN(logger_, "msg->point_num < 1000");
    }

    const auto range = resolution_ * static_cast<float>(width_);

    auto grid = std::make_unique<GridType>();
    auto cloud = std::make_shared<PointCloudType>();
    auto data = std::vector<int8_t>(width_ * width_, -1);

    grid->header.frame_id = "local_grid";
    grid->header.stamp = msg->header.stamp;
    grid->info.resolution = resolution_;
    grid->info.width = width_;
    grid->info.height = width_;

    // Push all points in area to a point cloud valuable
    for (auto point : msg->points) {

      if (in(-(range / 2), point.x, (range / 2)) &&
          in(-(range / 2), point.y, (range / 2))) {

        auto point_lidar_link = PointType(point.x, point.y, point.z);
        auto point_origin_link =
            pcl::transformPoint(point_lidar_link, transform_);

        cloud->push_back(point_origin_link);
      }
    }

    // Maybe you need a filter to make cloud clearer
    filter(cloud, cloud);

    for (auto point : *cloud) {

      auto x = static_cast<int>((point.x + (range / 2)) * 10);
      auto y = static_cast<int>((point.y + (range / 2)) * 10);

      data[x + width_ * y] += static_cast<int8_t>(point.z * 10);
    }

    grid->data.swap(data);

    return grid;
  }

  static void filter(const std::shared_ptr<PointCloudType> &from,
                     const std::shared_ptr<PointCloudType> &to) {

    auto filter = pcl::StatisticalOutlierRemoval<pcl::PointXYZ>();
    filter.setInputCloud(from);
    filter.setMeanK(10);
    filter.setStddevMulThresh(1.0);
    filter.filter(*to);
  }

  void set_grid(float resolution, int width) {
    resolution_ = resolution;
    width_ = width;
  }

  void set_transform(Eigen::Affine3d &transform) {
    //
    transform_ = transform;
  }

private:
  Eigen::Affine3d transform_;
  rclcpp::Logger logger_ = rclcpp::get_logger("preprocess");

  float resolution_;
  int width_;

private:
  template <typename T>
  bool in(const T &&left, const T &val, const T &&right) noexcept {
    assert(left < right);
    return (val > left && val < right);
  }
};
} // namespace creeper