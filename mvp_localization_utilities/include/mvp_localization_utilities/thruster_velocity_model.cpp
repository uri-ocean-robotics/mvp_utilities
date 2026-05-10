#include "thruster_velocity_model.hpp"

using std::placeholders::_1;

ThrusterVelocityModel::ThrusterVelocityModel(std::string name) : Node(name)
{
    this->declare_parameter("base_frame", "base_link");
    this->get_parameter("base_frame", m_base_frame);

    this->declare_parameter("tf_prefix", "");
    this->get_parameter("tf_prefix", m_tf_prefix);

    if (!m_tf_prefix.empty()) {
        m_base_frame = m_tf_prefix + "/" + m_base_frame;
    }

    // Separate callback groups so IMU and command callbacks can run concurrently
    m_imu_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cmd_cb_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions imu_opts;
    imu_opts.callback_group = m_imu_cb_group;

    rclcpp::SubscriptionOptions cmd_opts;
    cmd_opts.callback_group = m_cmd_cb_group;

    m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10,
        std::bind(&ThrusterVelocityModel::f_cb_imu, this, _1),
        imu_opts);

    m_thrust_cmd_sub = this->create_subscription<std_msgs::msg::Float64>(
        "thrusters/surge/command", 10,
        std::bind(&ThrusterVelocityModel::f_cb_thrust_cmd, this, _1),
        cmd_opts);

    m_twist_pub = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
        "surge/twist", 10);
}

void ThrusterVelocityModel::f_cb_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double g_accel = 9.81 * std::sin(pitch);

    std::lock_guard<std::mutex> lock(m_imu_mutex);
    m_ax_gravity_corrected = msg->linear_acceleration.x + g_accel;
}

void ThrusterVelocityModel::f_cb_thrust_cmd(const std_msgs::msg::Float64::SharedPtr msg)
{
    double surge = 0.0;
    if (msg->data != 0.0) {
        double ax;
        {
            std::lock_guard<std::mutex> lock(m_imu_mutex);
            ax = m_ax_gravity_corrected;
        }
        surge = A * msg->data + B * ax;
    }

    geometry_msgs::msg::TwistWithCovarianceStamped twist;
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = m_base_frame;
    twist.twist.twist.linear.x = surge;
    twist.twist.covariance[0] = LIN_X_VAR;

    m_twist_pub->publish(twist);
}
