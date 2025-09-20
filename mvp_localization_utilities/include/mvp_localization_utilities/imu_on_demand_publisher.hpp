#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;




class ImuDemandPublisher : public rclcpp::Node
{
public:
    ImuDemandPublisher(std::string name = "imu_on_demand_publisher");
  

private:

    double m_head_on_demand_period;
    sensor_msgs::msg::Imu m_latest_imu;  // stores the most recent IMU message
    
    bool m_has_imu = false;              // flag to check if we have received data yet
    bool m_publish_on_demand = false;
    
    rclcpp::Time m_publish_end_time;        // when to stop publishing


    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr m_imu_on_demand_pub;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_on_demand_service;

    void f_cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

    void f_cb_trigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

};
