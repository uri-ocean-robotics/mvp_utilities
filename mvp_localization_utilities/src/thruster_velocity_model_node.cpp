#include "rclcpp/rclcpp.hpp"
#include "mvp_localization_utilities/thruster_velocity_model.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ThrusterVelocityModel>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
