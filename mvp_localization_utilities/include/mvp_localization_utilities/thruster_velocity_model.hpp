#ifndef THRUSTER_VELOCITY_MODEL_HPP_
#define THRUSTER_VELOCITY_MODEL_HPP_

#include <mutex>
#include <cmath>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

class ThrusterVelocityModel : public rclcpp::Node
{
public:
    ThrusterVelocityModel(std::string name = "thruster_velocity_model");

private:
    // surge = A * command + B * ax_gravity_corrected
    static constexpr double A = 1.37580358;
    static constexpr double B = 0.00647348;
    static constexpr double LIN_X_VAR = 0.11;

    std::string m_base_frame;
    std::string m_tf_prefix;

    double m_ax_gravity_corrected{0.0};
    std::mutex m_imu_mutex;

    rclcpp::CallbackGroup::SharedPtr m_imu_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_cmd_cb_group;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr m_thrust_cmd_sub;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr m_twist_pub;

    void f_cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void f_cb_thrust_cmd(const std_msgs::msg::Float64::SharedPtr msg);
};

#endif // THRUSTER_VELOCITY_MODEL_HPP_
