#include <chrono>
#include <functional>
#include <memory>


#include <rclcpp/rclcpp.hpp>
#include "world_odom_transform.hpp"
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/MagneticModel.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

WorldOdomTransform::WorldOdomTransform(std::string name) : Node(name)
{

    this->declare_parameter("world", "world");
    this->get_parameter("world", m_world_frame);

    this->declare_parameter("odom", "odom");
    this->get_parameter("world", m_odom_frame);

    this->declare_parameter("tf_prefix", "");
    this->get_parameter("tf_prefix", m_tf_prefix);

    this->declare_parameter("mag_declination", 0.0);
    this->get_parameter("mag_declination", m_mag_declination);

    this->declare_parameter("mag_declination_auto", true);
    this->get_parameter("mag_declination_auto", m_mag_declination_auto);

    this->declare_parameter("use_depth_for_tf", true);
    this->get_parameter("use_depth_for_tf", m_use_depth_for_tf);
    
    this->declare_parameter("acceptable_var", 0.0);
    this->get_parameter("acceptable_var", m_acceptable_var);

    this->declare_parameter("position_accuracy", 0.0);
    this->get_parameter("position_accuracy", m_position_accuracy);

    this->declare_parameter("max_gps_wait_time", 60.0);
    this->get_parameter("max_gps_wait_time", m_gps_wait_time);


    this->declare_parameter("max_gps_wait_time", 60.0);
    this->get_parameter("max_gps_wait_time", m_gps_wait_time);

    this->declare_parameter("datum_latitude", 0.0);
    this->get_parameter("datum_latitude", m_datum_latitude);

    this->declare_parameter("datum_longitude", 0.0);
    this->get_parameter("datum_longitude", m_datum_longitude);

    this->declare_parameter("datum_altitude", 0.0);
    this->get_parameter("datum_altitude", m_datum_altitude);

    this->declare_parameter("mag_model_path", "");
    this->get_parameter("mag_model_path", m_mag_model_path);

    this->declare_parameter("publish_tf", true);
    this->get_parameter("publish_tf", m_publish_tf);


    m_datum.latitude = m_datum_latitude;
    m_datum.longitude = m_datum_longitude;
    m_datum.altitude = m_datum_altitude;


    if(m_datum.latitude==0){
        RCLCPP_WARN(get_logger(), "Datum is not set please set the parameters");
        m_datum_set = false;
    }
    else{
        m_datum_set = true;
    }
    

    m_tf_set = false;
    m_world_frame = m_tf_prefix + "/" + m_world_frame;
    m_odom_frame = m_tf_prefix + "/" + m_odom_frame;

    //publisher
    m_gps_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("gps/odometry", 10);
    m_datum_publisher = this->create_publisher<geographic_msgs::msg::GeoPoint>("gps/datum", 10);
    m_geopose_publisher = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("odometry/geopose", 10);

    //subscriber
    m_gps_fix_subscriber = this->create_subscription<sensor_msgs::msg::NavSatFix>("gps/fix", 10, 
                                                                std::bind(&WorldOdomTransform::f_cb_gps_fix, 
                                                                this, _1));
    m_odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odometry", 10, 
                                                                std::bind(&WorldOdomTransform::f_cb_odom, 
                                                                this, _1));
    m_depth_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("depth/odometry", 10, 
                                                                std::bind(&WorldOdomTransform::f_cb_depth, 
                                                                this, _1));
    
    /**
     * Initialize services
     */

    fromLL_server = this->create_service<robot_localization::srv::FromLL>("fromLL",
        std::bind(&WorldOdomTransform::f_cb_fromLL_srv, this, _1, _2));

    toLL_server = this->create_service<robot_localization::srv::ToLL>("toLL",
        std::bind(&WorldOdomTransform::f_cb_toLL_srv, this, _1, _2));

    reset_tf_server = this->create_service<std_srvs::srv::Trigger>("reset_tf",
        std::bind(&WorldOdomTransform::f_cb_reset_tf_srv, this, _1, _2));

    reset_datum_server = this->create_service<std_srvs::srv::Trigger>("reset_datum",
        std::bind(&WorldOdomTransform::f_cb_reset_datum_srv, this, _1, _2));


    m_transform_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    m_transform_listener = std::make_unique<tf2_ros::TransformListener>(*m_transform_buffer);
        
}

void WorldOdomTransform::f_cb_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
     //compute the latitude longitude in the world frame using datum.
    geometry_msgs::msg::Point::SharedPtr map_point;
    nav_msgs::msg::Odometry gps_odom;
    geographic_msgs::msg::GeoPoint ll_point;

    m_gps = *msg;
    m_gps_for_datum = *msg;
    m_odom_gps = m_odom; //map the most recent odom;
    m_depth_gps = m_depth;

    if(m_datum_set)
    {
        m_datum_publisher->publish(m_datum); 
    }

    if(m_tf_set && m_datum_set)
    {

        if(m_gps.position_covariance[0]<m_acceptable_var 
                && m_gps.position_covariance[4]<m_acceptable_var
                && m_gps.status.status>-1)
        {
            ll_point.latitude = msg->latitude;
            ll_point.longitude = msg->longitude;
            ll_point.altitude = msg->altitude;

            f_ll2dis(ll_point, map_point);
            try {        

                geometry_msgs::msg::TransformStamped tf_w2o = m_transform_buffer->lookupTransform(
                    m_odom_frame,
                    m_world_frame,
                    tf2::TimePointZero,
                    10ms
                );

                geometry_msgs::msg::PoseStamped odom_pose, world_pose;
                world_pose.header = msg->header;
                world_pose.header.frame_id = m_world_frame;
                world_pose.pose.position.x = map_point->x;
                world_pose.pose.position.y = map_point->y;
                world_pose.pose.position.z = map_point->z;
                world_pose.pose.orientation.x = 0;
                world_pose.pose.orientation.y = 0;
                world_pose.pose.orientation.z = 0;
                world_pose.pose.orientation.w = 1.0;
                // Transform the pose from odom frame to world frame
                tf2::doTransform(world_pose, odom_pose, tf_w2o);

                gps_odom.pose.pose.position.x = odom_pose.pose.position.x;
                gps_odom.pose.pose.position.y = odom_pose.pose.position.y;
                gps_odom.pose.pose.position.z = odom_pose.pose.position.z;
                gps_odom.header.frame_id = m_odom_frame;
                gps_odom.header.stamp = msg->header.stamp;
                gps_odom.pose.covariance[0] = msg->position_covariance[0];
                gps_odom.pose.covariance[1] = 0;
                gps_odom.pose.covariance[2] = 0;
                gps_odom.pose.covariance[6] =0;
                gps_odom.pose.covariance[7] = msg->position_covariance[4];
                gps_odom.pose.covariance[8] = 0;
                gps_odom.pose.covariance[12] = 0;
                gps_odom.pose.covariance[13] = 0;
                gps_odom.pose.covariance[14] =  msg->position_covariance[8];
                m_gps_odom_publisher->publish(gps_odom);  

            } 
            catch(tf2::TransformException &e) {
               RCLCPP_WARN(get_logger(), "Can't get the tf from world to odom");
            }
        }
    }
    else
    {
         if(m_datum_set)
        {
            if(m_gps.position_covariance[0]<m_acceptable_var 
                && m_gps.position_covariance[4]<m_acceptable_var
                && m_gps.status.status>-1)
            {
                // f_set_tf();
            }
            else{
                // ROS_INFO("GPS fix covariance is not good");
                RCLCPP_INFO(get_logger(), "GPS fix covariance is not good");
                return;
            }
        }
    }

}

bool WorldOdomTransform::f_set_tf()
{

}


void WorldOdomTransform::f_cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{

}
    
void WorldOdomTransform::f_cb_depth(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{

}

void WorldOdomTransform::f_cb_fromLL_srv(
            const std::shared_ptr<robot_localization::srv::FromLL::Request> request,
            const std::shared_ptr<robot_localization::srv::FromLL::Response> response)
{

}


void WorldOdomTransform::f_cb_toLL_srv(
            const std::shared_ptr<robot_localization::srv::ToLL::Request> request,
            const std::shared_ptr<robot_localization::srv::ToLL::Response> response)
{

}


void WorldOdomTransform::f_cb_reset_tf_srv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{

}

void WorldOdomTransform::f_cb_reset_datum_srv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{

}


void WorldOdomTransform::f_ll2dis(geographic_msgs::msg::GeoPoint ll_point, geometry_msgs::msg::Point::SharedPtr map_point)
{
    // double north = m_earthR*(ll_point.latitude - m_datum.latitude)/180.0*M_PI;
    // double east = m_earthR*cos(m_datum.latitude/180.0*M_PI) * (ll_point.longitude - m_datum.longitude)/180.0*M_PI;
    
    //in world frame.
    //Geographic lib implementation
    double distance, azimuth1, azimuth2;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
    geod.Inverse(m_datum.latitude, m_datum.longitude, 
                 ll_point.latitude, ll_point.longitude, 
                 distance, azimuth1, azimuth2);
    double north = distance *cos(azimuth1/180.0*M_PI);
    double east = distance * sin(azimuth1/180.0*M_PI);

    map_point->x = east;
    map_point->y = north;
    map_point->z = ll_point.altitude;
}

void WorldOdomTransform::f_dis2ll(geometry_msgs::msg::Point map_point, geographic_msgs::msg::GeoPoint::SharedPtr ll_point)
{
    //from world frame
    // double lat = m_datum.latitude + map_point.y/m_earthR * 180.0/M_PI;
    // double lon = m_datum.longitude + map_point.x/(m_earthR*cos(m_datum.latitude/180.0*M_PI)) * 180.0/M_PI;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
    double lat, lon;
     // Calculate latitude and longitude of target point
    geod.Direct(m_datum.latitude, m_datum.longitude, 0.0, map_point.y, lat, lon); // Move north
    geod.Direct(lat, lon, 90.0, map_point.x, lat, lon); // Move east
    
    ll_point->latitude = lat;
    ll_point->longitude = lon;
    ll_point->altitude = map_point.z;

}
