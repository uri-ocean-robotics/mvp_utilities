#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

using namespace std::chrono_literals;

class ZeroOdomPublisher : public rclcpp::Node
{
public:
  ZeroOdomPublisher()
  : Node("zero_odom_publisher")
  {
    this->declare_parameter<std::string>("frame_id", "odom");
    this->declare_parameter<std::string>("child_frame_id", "base_link");

    // Get parameter values
    this->get_parameter("frame_id", m_frame_id_);
    this->get_parameter("child_frame_id", m_child_frame_id_);

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("zero_odom", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&ZeroOdomPublisher::publish_odom, this));
    RCLCPP_INFO(this->get_logger(), "Publishing zero odometry on /odom");
  }

private:
  void publish_odom()
  {
    auto msg = nav_msgs::msg::Odometry();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = m_frame_id_;
    msg.child_frame_id = m_child_frame_id_;

    // Zero position
    msg.pose.pose.position.x = 0.0;
    msg.pose.pose.position.y = 0.0;
    msg.pose.pose.position.z = 0.0;

    // Identity quaternion (no rotation)
    msg.pose.pose.orientation.w = 1.0;

    // Zero velocities
    msg.twist.twist.linear.x = 0.0;
    msg.twist.twist.linear.y = 0.0;
    msg.twist.twist.angular.z = 0.0;

    publisher_->publish(msg);
  }

  std::string m_frame_id_;
  std::string m_child_frame_id_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ZeroOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
