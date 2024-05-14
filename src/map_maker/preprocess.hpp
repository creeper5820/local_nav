#pragma once

#include "../utility/param.hpp"

#include <Eigen/Core>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <rclcpp/logger.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <rclcpp/utilities.hpp>
#include <vector>

class Preprocess {
public:
    enum class NodeType {
        NONE,
        BLOCK,
        USED,
        AVAILABLE,
    };

    struct Node {
        int x;
        int y;
        int8_t value;
        NodeType type;
    };

    Preprocess() = default;

    void set(double blind) {
        RCLCPP_INFO(rclcpp::get_logger("make"), "blind: %lf", blind);
        blind_ = blind;
    }

    void set(double resolution, double width) {
        RCLCPP_INFO(rclcpp::get_logger("make"), "resolution: %lf, width: %lf", resolution, width);
        resolution_ = resolution;
        width_      = width;
    }

    void set(Eigen::Affine3d& transform) {
        transform_ = transform;

        auto point_origin_link = pcl::PointXYZ(0.1, 0.1, 0.2);
        auto point_goal_link   = transform_from_lidar_link(point_origin_link);

        RCLCPP_INFO(
            logger_, "(%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)", point_origin_link.x, point_origin_link.y,
            point_origin_link.z, point_goal_link.x, point_goal_link.y, point_goal_link.z);
    }

    std::vector<Node>
        pointcloud_process(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {

        static const auto grid_width = size_t(width_ / resolution_) + 1;

        auto cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

        auto point_goal_link = pcl::PointXYZ();

        for (auto point : msg->points) {
            // transformed link
            transform_from_lidar_link({point.x, point.y, point.z}, point_goal_link);

            // Remove ground
            if (point_goal_link.z < param::ground_height)
                continue;

            auto is_in_width = in(-(width_ / 2.0), point_goal_link.x, (width_ / 2.0))
                            && in(-(width_ / 2.0), point_goal_link.y, (width_ / 2.0));
            auto is_in_blind = in(-(blind_ / 2.0), point_goal_link.x, (blind_ / 2.0))
                            && in(-(blind_ / 2.0), point_goal_link.y, (blind_ / 2.0));

            if (is_in_width && !is_in_blind) {
                cloud->push_back(point_goal_link);
            }
        }

        auto data = std::vector<Node>(grid_width * grid_width);

        for (int x = 0; x < grid_width; x++)
            for (int y = 0; y < grid_width; y++) {
                auto& temp = data[x + y * grid_width];
                temp.type  = NodeType::NONE;
                temp.value = -1;
                temp.x     = x;
                temp.y     = y;
            }

        for (const auto point : *cloud) {
            auto x = static_cast<int>((point.x + (width_ / 2.0)) / resolution_);
            auto y = static_cast<int>((point.y + (width_ / 2.0)) / resolution_);

            auto& temp = data[x + grid_width * y];
            temp.value = std::max(temp.value, static_cast<int8_t>(point.z * param::z_weight));
        }

        for (auto& temp : data) {
            if (temp.value > param::ground_height) {
                temp.type  = NodeType::BLOCK;
                temp.value = 0;
            } else {
                temp.value = -1;
                temp.type  = NodeType::AVAILABLE;
            }
        }

        return data;
    }

    std::unique_ptr<nav_msgs::msg::OccupancyGrid>
        make(const std::unique_ptr<livox_ros_driver2::msg::CustomMsg>& msg) {

        static const auto grid_width = size_t(width_ / resolution_) + 1;

        auto node_map     = this->pointcloud_process(msg);
        auto search_queue = std::queue<std::shared_ptr<Node>>();

        // select all the block node to queue as begin of search
        for (auto& node : node_map) {
            if (node.type == NodeType::BLOCK) {
                node.value = 0;
                node.type  = NodeType::AVAILABLE;
                search_queue.push(std::make_shared<Node>(node));
            }
        }

        // search start
        while (!search_queue.empty()) {
            auto node = search_queue.front();
            search_queue.pop();

            RCLCPP_INFO(
                rclcpp::get_logger("make"), "x: %d, y: %d, type: %d, value: %d", node->x, node->y,
                static_cast<int>(node->type), node->value);

            auto x_data = int{0};
            auto y_data = int{0};
            auto value  = node->value;

            // up
            x_data = node->x;
            y_data = node->y + 1;
            if (y_data < grid_width && node->type == NodeType::AVAILABLE) {
                auto& select_node = node_map[x_data + y_data * grid_width];
                select_node.value = static_cast<int8_t>(value + 1);
                select_node.type  = NodeType::USED;

                search_queue.push(std::make_shared<Node>(select_node));
            }
            // right
            x_data = node->x + 1;
            y_data = node->y;
            if (x_data < grid_width && node->type == NodeType::AVAILABLE) {
                auto& select_node = node_map[x_data + y_data * grid_width];
                select_node.value = static_cast<int8_t>(value + 1);
                select_node.type  = NodeType::USED;

                search_queue.push(std::make_shared<Node>(select_node));
            }
            // down
            x_data = node->x;
            y_data = node->y - 1;
            if (y_data > -1 && node->type == NodeType::AVAILABLE) {
                auto& select_node = node_map[x_data + y_data * grid_width];
                select_node.value = static_cast<int8_t>(value + 1);
                select_node.type  = NodeType::USED;

                search_queue.push(std::make_shared<Node>(select_node));
            }
            // left
            x_data = node->x - 1;
            y_data = node->y;
            if (x_data > -1 && node->type == NodeType::AVAILABLE) {
                auto& select_node = node_map[x_data + y_data * grid_width];
                select_node.value = static_cast<int8_t>(value + 1);
                select_node.type  = NodeType::USED;

                search_queue.push(std::make_shared<Node>(select_node));
            }
        }
        using namespace std::chrono_literals;

        while (true)
            rclcpp::sleep_for(10ms);

        // make grid message to return
        auto grid = std::make_unique<nav_msgs::msg::OccupancyGrid>();

        grid->header.frame_id = "local_link";
        grid->header.stamp    = msg->header.stamp;
        grid->info.resolution = float(resolution_);
        grid->info.width      = grid_width;
        grid->info.height     = grid_width;
        grid->data            = std::vector<int8_t>(grid_width * grid_width);

        for (auto& node : node_map) {
            grid->data[node.x + node.y * grid_width] = node.value;
        }

        return grid;
    }

private:
    Eigen::Affine3d transform_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>> map_publisher_;
    rclcpp::Logger logger_ = rclcpp::get_logger("preprocess");

    double resolution_;
    double blind_;
    double width_;

private:
    template <typename T, typename ValT>
    bool in(const T&& left, const ValT& val, const T&& right) {
        assert(left < right);
        return (val > left && val < right);
    }

    static void
        transform_from_lidar_link(const pcl::PointXYZ& point_origin, pcl::PointXYZ& point_goal) {
        point_goal = {
            point_origin.x, -point_origin.y,
            static_cast<float>(-param::transform_translation_z - point_origin.z)};
    }

    static pcl::PointXYZ transform_from_lidar_link(const pcl::PointXYZ& point_origin) {
        auto point_goal = pcl::PointXYZ();
        transform_from_lidar_link(point_origin, point_goal);

        return point_goal;
    }
};
