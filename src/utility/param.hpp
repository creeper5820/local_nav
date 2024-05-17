#pragma once

#include <rclcpp/rclcpp.hpp>

namespace param {

const auto node_option = rclcpp::NodeOptions()
                             .allow_undeclared_parameters(true)
                             .automatically_declare_parameters_from_overrides(true);

// debug
inline bool cost_map;
inline int livox_frames;

// grid config
inline double resolution;
inline double width;
inline size_t grid_width;
inline double blind;
inline int expand_size;
inline int discrete_point;

// transform
inline double transform_translation_x;
inline double transform_translation_y;
inline double transform_translation_z;

inline double transform_quaternion_x;
inline double transform_quaternion_y;
inline double transform_quaternion_z;
inline double transform_quaternion_w;

// preprocess
inline double z_weight;

// grid filter
inline double ground_height;

} // namespace param

class ParamServer : public rclcpp::Node {
public:
    ParamServer()
        : Node("param_server", param::node_option) {}

    void read_param() {
        this->get_parameter<bool>("cost_map", param::cost_map);
        this->get_parameter<int>("livox_frames", param::livox_frames);

        this->get_parameter<double>("transform.translation.x", param::transform_translation_x);
        this->get_parameter<double>("transform.translation.y", param::transform_translation_y);
        this->get_parameter<double>("transform.translation.z", param::transform_translation_z);

        this->get_parameter<double>("transform.quaternion.x", param::transform_quaternion_x);
        this->get_parameter<double>("transform.quaternion.y", param::transform_quaternion_x);
        this->get_parameter<double>("transform.quaternion.z", param::transform_quaternion_x);
        this->get_parameter<double>("transform.quaternion.w", param::transform_quaternion_x);

        this->get_parameter<double>("grid.resolution", param::resolution);
        this->get_parameter<double>("grid.width", param::width);
        this->get_parameter<double>("grid.blind", param::blind);
        this->get_parameter<int>("grid.expand_size", param::expand_size);
        this->get_parameter<int>("grid.discrete_point", param::discrete_point);

        this->get_parameter<double>("grid.z_wight", param::z_weight);

        this->get_parameter<double>("grid_filter.ground_height", param::ground_height);

        param::grid_width = static_cast<size_t>(param::width / param::resolution);
    }
};
