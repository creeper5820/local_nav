#pragma once

#include "../utility/param.hpp"
#include "filter.hpp"
#include "node.hpp"

#include <Eigen/Core>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/point_cloud.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/utilities.hpp>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <queue>
#include <vector>

class Preprocess {
public:
    Preprocess() = default;

    void set(double blind) {
        RCLCPP_INFO(logger_, "blind: %.2lf", blind);
        blind_ = blind;
    }

    void set(double resolution, double width) {
        resolution_       = resolution;
        width_            = width;
        param::grid_width = size_t(width_ / resolution_);

        RCLCPP_INFO(
            logger_, "resolution: %.2lf, width: %.2lf, grid width: %zu", resolution_, width_,
            param::grid_width);
    }

    void set(Eigen::Affine3d& transform) {
        transform_ = transform;

        auto point_origin_link = pcl::PointXYZ(0.1, 0.1, 0.2);
        auto point_goal_link   = transform_from_lidar_link(point_origin_link);

        RCLCPP_INFO(
            logger_, "(%.2f,%.2f,%.2f)->(%.2f,%.2f,%.2f)", point_origin_link.x, point_origin_link.y,
            point_origin_link.z, point_goal_link.x, point_goal_link.y, point_goal_link.z);
    }

    std::vector<type::Node>
        livox_preprocess(std::vector<livox_ros_driver2::msg::CustomPoint>& points) {

        auto cloud = std::make_unique<pcl::PointCloud<pcl::PointXYZ>>();

        auto point_goal_link = pcl::PointXYZ();

        for (auto point : points) {
            // transformed link
            transform_from_lidar_link({point.x, point.y, point.z}, point_goal_link);

            // Remove ground
            if (point_goal_link.z < param::ground_height)
                continue;

            auto in_width = in(-(width_ / 2.0), point_goal_link.x, (width_ / 2.0))
                         && in(-(width_ / 2.0), point_goal_link.y, (width_ / 2.0));
            auto in_blind = in(-(blind_ / 2.0), point_goal_link.x, (blind_ / 2.0))
                         && in(-(blind_ / 2.0), point_goal_link.y, (blind_ / 2.0));

            if (in_width && !in_blind) {
                cloud->push_back(point_goal_link);
            }
        }

        auto data = std::vector<type::Node>(param::grid_width * param::grid_width);

        for (int x = 0; x < param::grid_width; x++)
            for (int y = 0; y < param::grid_width; y++) {
                auto& temp = data[x + y * param::grid_width];
                temp.type  = type::NodeType::NONE;
                temp.value = 0;
                temp.x     = x;
                temp.y     = y;
            }

        for (const auto point : *cloud) {
            auto x = static_cast<int>((point.x + (width_ / 2.0)) / resolution_);
            auto y = static_cast<int>((point.y + (width_ / 2.0)) / resolution_);

            auto& temp = data[x + param::grid_width * y];
            temp.value = std::max(temp.value, static_cast<int8_t>(point.z * param::z_weight));
        }

        filter::handle(data);

        for (auto& temp : data) {
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

    void expand_block(std::vector<type::Node>& map) const {
        for (auto& grid : map) {

            if (grid.type != type::NodeType::BLOCK)
                continue;

            int x;
            int y;

            x = grid.x + 1;
            y = grid.y;
            if (x < width_) {
                map[x + y * param::grid_width].type  = type::NodeType::BLOCK;
                map[x + y * param::grid_width].value = 0;
            }
            x = grid.x - 1;
            y = grid.y;
            if (x > -1) {
                map[x + y * param::grid_width].type  = type::NodeType::BLOCK;
                map[x + y * param::grid_width].value = 0;
            }
            x = grid.x;
            y = grid.y + 1;
            if (y < width_) {
                map[x + y * param::grid_width].type  = type::NodeType::BLOCK;
                map[x + y * param::grid_width].value = 0;
            }
            x = grid.x;
            y = grid.y - 1;
            if (y > -1) {
                map[x + y * param::grid_width].type  = type::NodeType::BLOCK;
                map[x + y * param::grid_width].value = 0;
            }
        }
    }

    static void remove_discrete_point(std::vector<type::Node>& map) {
        for (auto& grid : map) {
            if (grid.type != type::NodeType::BLOCK)
                continue;

            int discrete_count = 0;

            for (int x = 0; x < 3; x++)
                for (int y = 0; y < 3; y++) {

                    auto x_grid = grid.x + x - 1;
                    auto y_grid = grid.y + y - 1;

                    if (x_grid < 0 || x_grid >= param::grid_width || y_grid < 0
                        || y_grid >= param::grid_width)
                        continue;

                    if (map[x_grid + y_grid * param::grid_width].type == type::NodeType::BLOCK)
                        discrete_count++;
                }

            if (discrete_count < param::discrete_point) {
                grid.type = type::NodeType::AVAILABLE;
            }
        }
    }

    std::vector<type::Node> make(std::vector<livox_ros_driver2::msg::CustomPoint>& points) {

        auto node_map = this->livox_preprocess(points);

        auto search_queue = std::queue<type::Node*>();

        // select all the block node to queue as begin of search
        for (auto& node : node_map) {
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
            if (y_data < param::grid_width) {
                auto select_node = &node_map[x_data + y_data * param::grid_width];
                if (select_node->type == type::NodeType::AVAILABLE) {
                    select_node->value = value;
                    select_node->type  = type::NodeType::USED;

                    search_queue.push(select_node);
                }
            }
            // right
            x_data = node->x + 1;
            y_data = node->y;
            if (x_data < param::grid_width) {
                auto select_node = &node_map[x_data + y_data * param::grid_width];
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
                auto select_node = &node_map[x_data + y_data * param::grid_width];
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
                auto select_node = &node_map[x_data + y_data * param::grid_width];
                if (select_node->type == type::NodeType::AVAILABLE) {
                    select_node->value = value;
                    select_node->type  = type::NodeType::USED;

                    search_queue.push(select_node);
                }
            }
        }

        return node_map;
    }

private:
    Eigen::Affine3d transform_;
    rclcpp::Logger logger_ = rclcpp::get_logger("local_nav");

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
