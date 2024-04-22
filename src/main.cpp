#include "./node.hpp"

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<creeper::MainProcessNode>());

  rclcpp::shutdown();

  return 0;
}
