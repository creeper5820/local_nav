#include "node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    ParamServer().read_param();

    rclcpp::spin(std::make_shared<MainProcessNode>());

    rclcpp::shutdown();

    return 0;
}
