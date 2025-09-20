#include <chrono>
#include <functional>
#include <memory>


#include <rclcpp/rclcpp.hpp>
#include "imu_on_demand_publisher.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;


ImuDemandPublisher::ImuDemandPublisher(std::string name) : Node(name)
{
   

    this->declare_parameter("imu_heading_on_demand_period", 0.0);
    this->get_parameter("imu_heading_on_demand_period", m_head_on_demand_period);

  
    m_imu_on_demand_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/on_demand/data", 10);
    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("imu/data", 10, 
                                                                std::bind(&ImuDemandPublisher::f_cb_imu, 
                                                                this, _1));

    imu_on_demand_service = this->create_service<std_srvs::srv::Trigger>(
      "~/triger_imu_heading",
      std::bind(&ImuDemandPublisher::f_cb_trigger, this, _1, _2)
    );

}

void ImuDemandPublisher::f_cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    m_latest_imu = *msg;
    m_has_imu = true;

    if (m_publish_on_demand) {
        if (this->now() < m_publish_end_time) 
        {
            m_imu_on_demand_pub->publish(m_latest_imu);
            // RCLCPP_DEBUG(this->get_logger(), "Published IMU on demand");
        } 
        else {

            m_publish_on_demand = false;
            RCLCPP_INFO(this->get_logger(), "On-demand publishing finished");
        }
    }

}

void ImuDemandPublisher::f_cb_trigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!m_has_imu) {
        response->success = false;
        response->message = "No IMU data available yet";
        return;
    }
    // Enable publishing for the configured duration
    rclcpp::Duration duration = rclcpp::Duration::from_seconds(m_head_on_demand_period);
    m_publish_end_time = this->now() + duration;
    m_publish_on_demand = true;

    response->success = true;
    response->message = "On-demand publishing enabled for " +
                        std::to_string(m_head_on_demand_period) + " seconds";
    RCLCPP_INFO(this->get_logger(), "IMU on-demand publishing enabled");
    
}

