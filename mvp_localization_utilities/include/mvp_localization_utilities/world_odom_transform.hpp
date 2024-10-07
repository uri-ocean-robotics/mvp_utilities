#ifndef WORLD_ODOM_TRANSFORM_HPP_
#define WORLD_ODOM_TRANSFORM_HPP_

#include "rclcpp/rclcpp.hpp"
#include <string>
#include "nav_msgs/msg/odometry.hpp"
#include "geographic_msgs/msg/geo_point.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "robot_localization/srv/from_ll.hpp"
#include "robot_localization/srv/to_ll.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
// #include <tf2_ros/transform_broadcaster.h>

class WorldOdomTransform : public rclcpp::Node
{
    public:
        WorldOdomTransform(std::string name = "world_odom_transform");

    private:
        //publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_gps_odom_publisher;
        rclcpp::Publisher<geographic_msgs::msg::GeoPoint>::SharedPtr m_datum_publisher;
        rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr m_geopose_publisher;
        


        //subscriber
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr m_gps_fix_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscriber;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_depth_subscriber;

        void f_cb_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

        void f_cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    
        void f_cb_depth(const nav_msgs::msg::Odometry::SharedPtr msg);


        //service
        rclcpp::Service<robot_localization::srv::FromLL>::SharedPtr fromLL_server;
        rclcpp::Service<robot_localization::srv::ToLL>::SharedPtr toLL_server;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_tf_server;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_datum_server;

        //service call
        bool f_cb_fromLL_srv(
            const std::shared_ptr<robot_localization::srv::FromLL::Request> request,
            const std::shared_ptr<robot_localization::srv::FromLL::Response> response);

        bool f_cb_toLL_srv(
            const std::shared_ptr<robot_localization::srv::ToLL::Request> request,
            const std::shared_ptr<robot_localization::srv::ToLL::Response> response);


        bool f_cb_reset_tf_srv(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            const std::shared_ptr<std_srvs::srv::Trigger::Response> response);

        bool f_cb_reset_datum_srv(
            const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
            const std::shared_ptr<std_srvs::srv::Trigger::Response> response);


        //other variables
        geographic_msgs::msg::GeoPoint m_datum;

        nav_msgs::msg::Odometry m_odom, m_odom_gps;

        nav_msgs::msg::Odometry m_depth, m_depth_gps;

        sensor_msgs::msg::NavSatFix m_gps;

        sensor_msgs::msg::NavSatFix m_gps_for_datum;

        std::string m_world_frame;

        std::string m_odom_frame;

        std::string m_gps_frame;
        
        std::string m_tf_prefix;

        std::string m_mag_model_path;

        bool m_datum_set = false;

        bool m_tf_set = false;

        bool m_use_depth_for_tf = true;

        bool m_mag_declination_auto;

        double m_earthR = 6371000;

        double m_mag_declination;

        double m_acceptable_var;

        double m_gps_wait_time;

        double m_datum_latitude;

        double m_datum_longitude;

        double m_datum_altitude;

        double m_position_accuracy;

        bool m_publish_tf;

        void f_ll2dis(geographic_msgs::msg::GeoPoint ll_point, geometry_msgs::msg::Point::SharedPtr map_point);

        void f_dis2ll(geometry_msgs::msg::Point map_point, geographic_msgs::msg::GeoPoint::SharedPtr ll_point);

        bool f_set_tf();

        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> br;
        geometry_msgs::msg::TransformStamped transformStamped;

        std::unique_ptr<tf2_ros::Buffer> m_transform_buffer;
        std::unique_ptr<tf2_ros::TransformListener> m_transform_listener;

};



#endif
