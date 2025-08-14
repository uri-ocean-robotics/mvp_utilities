#include "rclcpp/rclcpp.hpp"

#include "mvp_localization_utilities/gps_odom_publisher.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<GpsOdomPublisher> node = std::make_shared<GpsOdomPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}