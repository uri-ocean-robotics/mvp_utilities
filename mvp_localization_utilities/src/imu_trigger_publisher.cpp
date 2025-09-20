#include "rclcpp/rclcpp.hpp"

#include "mvp_localization_utilities/imu_on_demand_publisher.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<ImuDemandPublisher> node = std::make_shared<ImuDemandPublisher>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}