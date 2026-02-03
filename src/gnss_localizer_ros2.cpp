#include <rclcpp/rclcpp.hpp>
#include "gnss_localizer_ros2.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  int plane;
  std::string base_link_name;
  std::string gnss_link_name;

  {
    auto param_node = rclcpp::Node::make_shared("get_parameters");
    param_node->declare_parameter("plane", 9);
    param_node->declare_parameter("base_link_tf", "base_link");
    param_node->declare_parameter("gnss_link_tf", "gnss_temp_link");
    plane = param_node->get_parameter("plane").get_parameter_value().get<int>();
    base_link_name = param_node->get_parameter("base_link_tf").get_parameter_value().get<std::string>();
    gnss_link_name = param_node->get_parameter("gnss_link_tf").get_parameter_value().get<std::string>();
    
  }

  auto node_ = std::make_shared<gnss_localizer_ros2>(plane, base_link_name, gnss_link_name);

  rclcpp::spin(node_);

  rclcpp::shutdown();
  return 0;
}
