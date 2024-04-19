#include "./node.hpp"

int main(int argc, char** argv) {

    rclcpp::init(argc, argv);

    auto map_compressor_node = std::make_shared<creeper::MapCompressor>();
    RCLCPP_INFO(map_compressor_node->get_logger(), "map compressor start");

    rclcpp::spin(map_compressor_node);

    rclcpp::shutdown();

    return 0;
}