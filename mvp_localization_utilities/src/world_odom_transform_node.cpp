#include "rclcpp/rclcpp.hpp"

#include "mvp_localization_utilities/world_odom_transform.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<WorldOdomTransform> node = std::make_shared<WorldOdomTransform>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}