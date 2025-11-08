#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "stack_detector/stack_detector_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<stack_detector::StackDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
