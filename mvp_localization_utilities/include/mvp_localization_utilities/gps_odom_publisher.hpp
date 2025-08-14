#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "robot_localization/srv/from_ll.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;




class GpsOdomPublisher : public rclcpp::Node
{
public:
    GpsOdomPublisher(std::string name = "gps_odom_publisher");
  

private:

    std::string m_world_frame;

    std::string m_child_frame;

    std::string m_tf_prefix;

    double m_acceptable_var;

    double m_manual_position_covariance;


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_gps_odom_publisher;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gps_fix_subscriber;

    rclcpp::Client<robot_localization::srv::FromLL>::SharedPtr fromLL_client;

    void f_cb_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};
