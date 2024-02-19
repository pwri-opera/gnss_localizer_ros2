#include <rclcpp/rclcpp.hpp>
#include "gnss_localizer_ros2.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  int plane;

  {
    auto param_node = rclcpp::Node::make_shared("get_parameters");
    param_node->declare_parameter("plane", 9);
    
    plane = param_node->get_parameter("plane").get_parameter_value().get<int>();
  }


  auto node_ = std::make_shared<gnss_localizer_ros2>(plane);

  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}
