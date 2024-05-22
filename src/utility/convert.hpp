#pragma once

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace utility {
void livox_to_pcl(
    const std::vector<livox_ros_driver2::msg::CustomPoint>& livox,
    pcl::PointCloud<pcl::PointXYZ>& pcl) {
    for (const auto point : livox) {
        pcl.points.emplace_back(point.x, point.y, point.z);
    }
}
void pc2_to_pcl(const sensor_msgs::msg::PointCloud2& pc2, pcl::PointCloud<pcl::PointXYZ>& pcl) {
    pcl::fromROSMsg(pc2, pcl);
}

template <typename PointCloudT>
requires(
    std::is_same_v<PointCloudT, std::vector<livox_ros_driver2::msg::CustomPoint>>
    || std::is_same_v<PointCloudT, sensor_msgs::msg::PointCloud2>)
void pack_pointcloud(pcl::PointCloud<pcl::PointXYZ>& package, const PointCloudT& cloud) {

    auto cloud_converted = pcl::PointCloud<pcl::PointXYZ>();

    if (std::is_same_v<PointCloudT, std::vector<livox_ros_driver2::msg::CustomPoint>>) {
        livox_to_pcl(cloud, cloud_converted);
    } else if (std::is_same_v<PointCloudT, sensor_msgs::msg::PointCloud2>) {
        pc2_to_pcl(cloud, cloud_converted);
    }

    package.insert(package.end(), cloud_converted.begin(), cloud_converted.end());
}
}; // namespace utility