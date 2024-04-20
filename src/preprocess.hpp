#pragma once

#include "./type_require.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cassert>
#include <memory>

namespace creeper {

class Preprocess {
public:
  Preprocess() {}

  GridType::UniquePtr get_grid(const LivoxType::SharedPtr &msg) {

    if (msg->point_num < 1000) {
      return nullptr;
      RCLCPP_WARN(logger_, "msg->point_num < 1000");
    }

    const auto range = resolution_ * static_cast<float>(width_);

    auto cloud = std::make_shared<PointCloudType>();
    auto grid = std::make_unique<GridType>();
    auto data = std::vector<int8_t>(width_ * width_, -1);

    grid->header.frame_id = "origin_link";
    grid->header.stamp = msg->header.stamp;
    grid->info.resolution = resolution_;
    grid->info.width = width_;
    grid->info.height = width_;

    // Push all points in area to a point cloud valuable
    for (auto point : msg->points) {

      auto point_goal_link =
          PointType{point.x, -point.y, static_cast<float>(0.55 - point.z)};

      if (in(-(range / 2.0f), point_goal_link.x, (range / 2.0f)) &&
          in(-(range / 2.0f), point_goal_link.y, (range / 2.0f))) {

        cloud->push_back(point_goal_link);
      }
    }

    // Maybe you need a filter to make cloud clearer
    filter(cloud, cloud);

    // Make grid map now
    for (auto point : *cloud) {

      auto x = static_cast<int>((point.x + (range / 2.0f)) / resolution_);
      auto y = static_cast<int>((point.y + (range / 2.0f)) / resolution_);

      data[x + width_ * y] += (static_cast<int8_t>((point.z + 0.1) * 10));
    }

    grid->data = data;

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

    auto point_origin_link = PointType(0.1, 0.1, 0.2);

    // auto point_goal = pcl::transformPoint(point_origin,
    // transform_.inverse());

    auto point_goal_link =
        PointType{point_origin_link.x, -point_origin_link.y,
                  static_cast<float>(0.55 - point_origin_link.z)};

    RCLCPP_INFO(logger_, "\n( %.2f, %.2f, %.2f ) -> ( %.2f, %.2f, %.2f )",
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
  bool in(const T &&left, const T &val, const T &&right) noexcept {
    assert(left < right);
    return (val > left && val < right);
  }
};
} // namespace creeper