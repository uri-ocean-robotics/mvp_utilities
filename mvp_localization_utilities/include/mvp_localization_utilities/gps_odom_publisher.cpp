#include <chrono>
#include <functional>
#include <memory>


#include <rclcpp/rclcpp.hpp>
#include "gps_odom_publisher.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;


GpsOdomPublisher::GpsOdomPublisher(std::string name) : Node(name)
{
    this->declare_parameter("world", "world");
    this->get_parameter("world", m_world_frame);

    this->declare_parameter("gps_frame", "gps");
    this->get_parameter("gps_frame", m_child_frame);

    this->declare_parameter("acceptable_var", 0.0);
    this->get_parameter("acceptable_var", m_acceptable_var);

    this->declare_parameter("tf_prefix", "");
    this->get_parameter("tf_prefix", m_tf_prefix);

    m_world_frame = m_tf_prefix + "/" + m_world_frame;
    m_child_frame = m_tf_prefix + "/" + m_child_frame;
    
    m_gps_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("gps/world_odometry", 10);
    m_gps_fix_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", 10, 
                                                                std::bind(&GpsOdomPublisher::f_cb_gps_fix, 
                                                                this, _1));

    fromLL_client = this->create_client<robot_localization::srv::FromLL>("fromLL");

}

void GpsOdomPublisher::f_cb_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (!fromLL_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "FromLL service not available.");
        return;
    }

    if( msg->position_covariance[0]<m_acceptable_var 
                &&  msg->position_covariance[4]<m_acceptable_var
                && msg->status.status>-1)
    {

        auto request = std::make_shared<robot_localization::srv::FromLL::Request>();
        geometry_msgs::msg::Point map_point;

        
        request->ll_point.latitude = msg->latitude;
        request->ll_point.longitude = msg->longitude;
        request->ll_point.altitude = msg->altitude;


        // Send the request and wait for the result
        auto future = fromLL_client->async_send_request(request,
        [this, msg](rclcpp::Client<robot_localization::srv::FromLL>::SharedFuture future_response) {
        try {
            auto response = future_response.get();
            geometry_msgs::msg::Point map_point = response.get()->map_point;

            nav_msgs::msg::Odometry gps_world_odom;
            gps_world_odom.pose.pose.position.x = map_point.x;
            gps_world_odom.pose.pose.position.y = map_point.y;
            gps_world_odom.pose.pose.position.z = map_point.z;
            gps_world_odom.header.frame_id = m_world_frame;
            gps_world_odom.child_frame_id = m_child_frame;
            gps_world_odom.header.stamp = msg->header.stamp;

            gps_world_odom.pose.covariance[0] = msg->position_covariance[0];
            gps_world_odom.pose.covariance[7] = msg->position_covariance[4];
            gps_world_odom.pose.covariance[14] = msg->position_covariance[8];

            m_gps_odom_publisher->publish(gps_world_odom);
            }
            catch (const std::exception & e) {
            RCLCPP_WARN(this->get_logger(), "FromLL service call failed: %s", e.what());
            }
        }
    );

    }
    else{
        RCLCPP_WARN(this->get_logger(), "GPS is not good");
        RCLCPP_WARN(this->get_logger(), "GPS covariance = %lf, %lf; status=%d", msg->position_covariance[0], msg->position_covariance[4], msg->status.status);

        return;
    }
}
