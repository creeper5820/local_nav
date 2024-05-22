#include "preprocess.hpp"
#include "../utility/param.hpp"
#include "filter.hpp"
#include "node.hpp"

#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/utilities.hpp>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

Preprocess::Preprocess()
    : logger_(rclcpp::get_logger("local_nav")) {}

void Preprocess::set(double blind) {
    RCLCPP_INFO(logger_, "blind: %.2lf", blind);
    blind_ = blind;
}

void Preprocess::set(double resolution, double width) {
    resolution_ = resolution;
    width_      = width;
    grid_width_ = param::grid_width;

    RCLCPP_INFO(
        logger_, "resolution: %.2lf width: %.2lf grid width: %zu", resolution_, width_,
        grid_width_);
}

void Preprocess::set(Eigen::Affine3d& transform) {
    transform_ = transform;

    auto point_origin_link = pcl::PointXYZ(0.1, 0.1, 0.2);
    auto point_goal_link   = transform_from_lidar_link(point_origin_link);

    RCLCPP_INFO(
        logger_, "(%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)", point_origin_link.x, point_origin_link.y,
        point_origin_link.z, point_goal_link.x, point_goal_link.y, point_goal_link.z);
}

std::unique_ptr<std::vector<type::Node>> Preprocess::generate_grid_map(
    const pcl::PointCloud<pcl::PointXYZ>& pointcloud, const Eigen::Affine3d& transform) {

    auto pointcloud_goal_link = pcl::PointCloud<pcl::PointXYZ>();
    auto pointcloud_filtered  = pcl::PointCloud<pcl::PointXYZ>();

    pcl::transformPointCloud(pointcloud, pointcloud_goal_link, transform, true);

    for (auto point : pointcloud_goal_link) {
        if (point.z < param::ground_height)
            continue;

        auto in_width = in(-(width_ / 2.0), point.x, (width_ / 2.0))
                     && in(-(width_ / 2.0), point.y, (width_ / 2.0));
        auto in_blind = in(-(blind_ / 2.0), point.x, (blind_ / 2.0))
                     && in(-(blind_ / 2.0), point.y, (blind_ / 2.0));

        if (in_width && !in_blind) {
            pointcloud_filtered.push_back(point);
        }
    }

    auto data = std::make_unique<std::vector<type::Node>>(grid_width_ * grid_width_);
    for (int x = 0; x < grid_width_; x++)
        for (int y = 0; y < grid_width_; y++) {
            auto& temp = (*data)[x + y * grid_width_];
            temp.type  = type::NodeType::NONE;
            temp.value = 0;
            temp.x     = x;
            temp.y     = y;
        }

    for (const auto point : pointcloud) {
        auto x = static_cast<int>((point.x + (width_ / 2.0)) / resolution_);
        auto y = static_cast<int>((point.y + (width_ / 2.0)) / resolution_);

        if (!in(int(0), x, int(grid_width_)) || !in(int(0), y, int(grid_width_)))
            continue;

        auto& temp = (*data)[x + grid_width_ * y];
        temp.value = std::max(temp.value, static_cast<int8_t>(point.z * param::z_weight));
    }

    filter::handle(*data);

    for (auto& temp : *data) {
        if (temp.value > param::ground_height) {
            temp.type  = type::NodeType::BLOCK;
            temp.value = 0;
        } else {
            temp.value = -1;
            temp.type  = type::NodeType::AVAILABLE;
        }
    }

    return data;
}

std::unique_ptr<std::vector<type::Node>>
    Preprocess::generate_grid_map(std::vector<livox_ros_driver2::msg::CustomPoint>& points) {

    auto cloud_goal_link = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();
    auto point_goal_link = std::make_unique<pcl::PointXYZ>();

    for (auto point : points) {
        // transformed link
        transform_from_lidar_link({point.x, point.y, point.z}, *point_goal_link);

        // Remove ground
        if (point_goal_link->z < param::ground_height)
            continue;

        auto in_width = in(-(width_ / 2.0), point_goal_link->x, (width_ / 2.0))
                     && in(-(width_ / 2.0), point_goal_link->y, (width_ / 2.0));
        auto in_blind = in(-(blind_ / 2.0), point_goal_link->x, (blind_ / 2.0))
                     && in(-(blind_ / 2.0), point_goal_link->y, (blind_ / 2.0));

        if (in_width && !in_blind) {
            cloud_goal_link->push_back(*point_goal_link);
        }
    }

    auto data = std::make_unique<std::vector<type::Node>>(grid_width_ * grid_width_);
    for (int x = 0; x < grid_width_; x++)
        for (int y = 0; y < grid_width_; y++) {
            auto& temp = (*data)[x + y * grid_width_];
            temp.type  = type::NodeType::NONE;
            temp.value = 0;
            temp.x     = x;
            temp.y     = y;
        }

    for (const auto point : *cloud_goal_link) {
        auto x = static_cast<int>((point.x + (width_ / 2.0)) / resolution_);
        auto y = static_cast<int>((point.y + (width_ / 2.0)) / resolution_);

        if (!in(int(0), x, int(grid_width_)) || !in(int(0), y, int(grid_width_)))
            continue;

        auto& temp = (*data)[x + grid_width_ * y];
        temp.value = std::max(temp.value, static_cast<int8_t>(point.z * param::z_weight));
    }

    filter::handle(*data);

    for (auto& temp : *data) {
        if (temp.value > param::ground_height) {
            temp.type  = type::NodeType::BLOCK;
            temp.value = 0;
        } else {
            temp.value = -1;
            temp.type  = type::NodeType::AVAILABLE;
        }
    }

    return data;
}

std::unique_ptr<std::vector<type::Node>>
    Preprocess::generate_grid_map(sensor_msgs::msg::PointCloud2& points) {

    auto pcl_pointcloud2 = std::make_unique<pcl::PCLPointCloud2>();
    auto pointcloud      = std::make_unique<pcl::PointCloud<pcl::PointXYZINormal>>();

    pcl_conversions::toPCL(points, *pcl_pointcloud2);
    pcl::fromPCLPointCloud2(*pcl_pointcloud2, *pointcloud);

    auto data = std::make_unique<std::vector<type::Node>>(grid_width_ * grid_width_);
    for (int x = 0; x < grid_width_; x++)
        for (int y = 0; y < grid_width_; y++) {
            auto& temp = (*data)[x + y * grid_width_];
            temp.type  = type::NodeType::NONE;
            temp.value = 0;
            temp.x     = x;
            temp.y     = y;
        }

    for (const auto point : *pointcloud) {
        auto x = static_cast<int>((point.x + (width_ / 2.0)) / resolution_);
        auto y = static_cast<int>((point.y + (width_ / 2.0)) / resolution_);

        if (!in(int(0), x, int(grid_width_)) || !in(int(0), y, int(grid_width_)))
            continue;

        auto& temp = (*data)[x + grid_width_ * y];
        temp.value = std::max(temp.value, static_cast<int8_t>(point.z * param::z_weight));
    }

    filter::handle(*data);

    for (auto& temp : *data) {
        if (temp.value > param::ground_height) {
            temp.type  = type::NodeType::BLOCK;
            temp.value = 0;
        } else {
            temp.value = -1;
            temp.type  = type::NodeType::AVAILABLE;
        }
    }

    return data;
}

std::unique_ptr<std::vector<type::Node>>
    Preprocess::generate_cost_map(const pcl::PointCloud<pcl::PointXYZ>& points) {
    (void)points;
    (void)this;
    return nullptr;
}

std::unique_ptr<std::vector<type::Node>>
    Preprocess::generate_cost_map(std::vector<livox_ros_driver2::msg::CustomPoint>& points) {

    auto node_map = this->generate_grid_map(points);

    auto search_queue = std::queue<type::Node*>();

    // select all the block node to queue as begin of search
    for (auto& node : *node_map) {
        if (node.type == type::NodeType::BLOCK) {
            search_queue.push(&node);
        }
    }

    // search start
    while (!search_queue.empty()) {
        auto node = search_queue.front();
        search_queue.pop();

        auto value = static_cast<int8_t>(node->value + 1);
        int x_data = 0;
        int y_data = 0;

        // up
        x_data = node->x;
        y_data = node->y + 1;
        if (y_data < grid_width_) {
            auto select_node = &(*node_map)[x_data + y_data * grid_width_];
            if (select_node->type == type::NodeType::AVAILABLE) {
                select_node->value = value;
                select_node->type  = type::NodeType::USED;

                search_queue.push(select_node);
            }
        }
        // right
        x_data = node->x + 1;
        y_data = node->y;
        if (x_data < grid_width_) {
            auto select_node = &(*node_map)[x_data + y_data * grid_width_];
            if (select_node->type == type::NodeType::AVAILABLE) {
                select_node->value = value;
                select_node->type  = type::NodeType::USED;

                search_queue.push(select_node);
            }
        }
        // down
        x_data = node->x;
        y_data = node->y - 1;
        if (y_data > -1) {
            auto select_node = &(*node_map)[x_data + y_data * grid_width_];
            if (select_node->type == type::NodeType::AVAILABLE) {
                select_node->value = value;
                select_node->type  = type::NodeType::USED;

                search_queue.push(select_node);
            }
        }
        // left
        x_data = node->x - 1;
        y_data = node->y;
        if (x_data > -1) {
            auto select_node = &(*node_map)[x_data + y_data * grid_width_];
            if (select_node->type == type::NodeType::AVAILABLE) {
                select_node->value = value;
                select_node->type  = type::NodeType::USED;

                search_queue.push(select_node);
            }
        }
    }

    return node_map;
}

void Preprocess::transform_from_lidar_link(
    const pcl::PointXYZ& point_origin, pcl::PointXYZ& point_goal) {
    point_goal = {
        point_origin.x, -point_origin.y,
        static_cast<float>(-param::transform_translation_z - point_origin.z)};
}

pcl::PointXYZ Preprocess::transform_from_lidar_link(const pcl::PointXYZ& point_origin) {
    auto point_goal = pcl::PointXYZ();
    transform_from_lidar_link(point_origin, point_goal);

    return point_goal;
}
