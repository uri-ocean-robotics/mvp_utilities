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
    this->get_parameter("odom", m_odom_frame);

    this->declare_parameter("gps_frame", "gps");
    this->get_parameter("gps_frame", m_gps_frame);

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
    m_gps_frame = m_tf_prefix + "/" + m_gps_frame;
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
    m_depth_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("depth", 10, 
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
    br = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    printf("initialization done \r\n");
        
}

void WorldOdomTransform::f_cb_gps_fix(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    //  compute the latitude longitude in the world frame using datum.
    geometry_msgs::msg::Point::SharedPtr map_point = std::make_shared<geometry_msgs::msg::Point>();
    nav_msgs::msg::Odometry gps_odom;
    geographic_msgs::msg::GeoPoint ll_point;

    m_gps = *msg;

    m_gps_for_datum = *msg;

    m_depth_gps = m_depth;

    // m_odom_gps = m_odom; //map the most recent odom;

        // m_odom_gps = m_odom; //map the most recent odom;
    //get gps location in the odom
    geometry_msgs::msg::TransformStamped tf_odom_gps = m_transform_buffer->lookupTransform(
                    m_odom_frame,
                    msg->header.frame_id,
                    tf2::TimePointZero,
                    10ms
                );
    m_odom_gps.header = m_odom.header;
    m_odom_gps.child_frame_id = msg->header.frame_id;
    m_odom_gps.pose.pose.position.x = tf_odom_gps.transform.translation.x;
    m_odom_gps.pose.pose.position.y = tf_odom_gps.transform.translation.y;
    m_odom_gps.pose.pose.position.z = tf_odom_gps.transform.translation.z;
    // Optional: fill the orientation
    m_odom_gps.pose.pose.orientation = tf_odom_gps.transform.rotation;


    

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
                gps_odom.child_frame_id = msg->header.frame_id;
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
                // printf("good gps \r\n");
                f_set_tf();
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
 //check the quality of the gps if the x and y variance is good enough?

    geographic_msgs::msg::GeoPoint ll_point;
    // geometry_msgs::msg::Point::SharedPtr map_point;
    nav_msgs::msg::Odometry m_odom_gps_temp = m_odom_gps;
    nav_msgs::msg::Odometry m_depth_gps_temp = m_depth_gps;
    sensor_msgs::msg::NavSatFix m_gps_temp = m_gps;

    geometry_msgs::msg::Point::SharedPtr map_point = std::make_shared<geometry_msgs::msg::Point>();

    GeographicLib::MagneticModel magModel("wmm2020", m_mag_model_path);
    printf("set tf function \r\n");
    
    ll_point.latitude = m_gps_temp.latitude;
    ll_point.longitude = m_gps_temp.longitude;
    ll_point.altitude = m_gps_temp.altitude;
    //Get x and y from lattiude and longitude. x->east, y->north
    f_ll2dis(ll_point, map_point);
    // //get mag declination
    double h, Bx, By, Bz;
    double H, F, D, I;
    magModel(2024, ll_point.latitude,ll_point.longitude, h, Bx, By, Bz);
    GeographicLib::MagneticModel::FieldComponents(Bx, By, Bz, H, F, D, I);
    //D is negative to west but we are in ENU frame.
    if (m_mag_declination_auto)
    {
        m_mag_declination = -D *M_PI/180.0;
    }
    // Print the magnetic field components
    //  printf("declination = %lf\r\n", D);
    // printf("tf update\r\n");

    transformStamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    transformStamped.header.frame_id = m_world_frame;
    transformStamped.child_frame_id = m_odom_frame;


    //convert odom xy into world first using magnetic declination
    double odom_world_x = m_odom_gps_temp.pose.pose.position.x*cos(m_mag_declination) - m_odom_gps_temp.pose.pose.position.y*sin(m_mag_declination);
    double odom_world_y = m_odom_gps_temp.pose.pose.position.x*sin(m_mag_declination) + m_odom_gps_temp.pose.pose.position.y*cos(m_mag_declination);

    double dx =  map_point->x -odom_world_x;
    double dy =  map_point->y - odom_world_y;
    printf("gps fix in world =%lf, %lf\r\n", map_point->x, map_point->y);
    printf("matching odom position= %lf, %lf\r\n", m_odom_gps_temp.pose.pose.position.x, m_odom_gps_temp.pose.pose.position.y);

    transformStamped.transform.translation.x = dx;
    transformStamped.transform.translation.y = dy;

    if (m_use_depth_for_tf)
    {
    transformStamped.transform.translation.z = -m_depth_gps_temp.pose.pose.position.z + 0.0;
    }
    else{
    transformStamped.transform.translation.z = -m_gps_temp.altitude + 0.0;
    }

    printf("translation in z =%lf\r\n", transformStamped.transform.translation.z);
    tf2::Quaternion q;
    q.setRPY(0, 0, m_mag_declination);

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    br->sendTransform(transformStamped);
    // printf("%s to %s\r\n", m_world_frame.c_str(), m_odom_frame.c_str());
    RCLCPP_INFO(get_logger(), "TF Between world and odom is set!");
    m_tf_set = true;
    return true;
}


void WorldOdomTransform::f_cb_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // printf("got odometry\r\n");
    m_odom = *msg;
    // printf("got odometry\r\n");
    if(m_tf_set)
    {   
        
        //publishtf
        transformStamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
        br->sendTransform(transformStamped);
        //convert odom from odom to world 
        try {        
            geometry_msgs::msg::TransformStamped tf_o2w = m_transform_buffer->lookupTransform(
                    m_world_frame,
                    m_odom_frame,
                    tf2::TimePointZero,
                    10ms
                );
            
            geometry_msgs::msg::PoseStamped odom_pose, world_pose;
            odom_pose.header = msg->header;
            odom_pose.pose = msg->pose.pose;

            // Transform the pose from odom frame to world frame
            tf2::doTransform(odom_pose, world_pose, tf_o2w);
            geometry_msgs::msg::Point map_point;
            geographic_msgs::msg::GeoPoint::SharedPtr ll_point = std::make_shared<geographic_msgs::msg::GeoPoint>();
            world_pose.header= msg->header;

            map_point.x = world_pose.pose.position.x;
            map_point.y = world_pose.pose.position.y;
            map_point.z = world_pose.pose.position.z;

            f_dis2ll(map_point, ll_point);
            geographic_msgs::msg::GeoPoseStamped geopose;
            geopose.pose.position.latitude = ll_point->latitude;
            geopose.pose.position.longitude = ll_point->longitude;
            geopose.pose.position.altitude = ll_point->altitude;
            geopose.pose.orientation.x = world_pose.pose.orientation.x;
            geopose.pose.orientation.y = world_pose.pose.orientation.y;
            geopose.pose.orientation.z = world_pose.pose.orientation.z;
            geopose.pose.orientation.w = world_pose.pose.orientation.w;
            geopose.header= world_pose.header;
            
            m_geopose_publisher->publish(geopose);

        } catch(tf2::TransformException &e) {
            RCLCPP_WARN(get_logger(), "Can't get the tf from world to odom when computing geopose");
            
        }
    }

}
    
void WorldOdomTransform::f_cb_depth(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    m_depth = *msg;
}

bool WorldOdomTransform::f_cb_fromLL_srv(
            const std::shared_ptr<robot_localization::srv::FromLL::Request> request,
            const std::shared_ptr<robot_localization::srv::FromLL::Response> response)
{
    geometry_msgs::msg::Point::SharedPtr map_point = std::make_shared<geometry_msgs::msg::Point>();
    f_ll2dis(request->ll_point, map_point);
    response->map_point = *map_point;
    return true;
}


bool WorldOdomTransform::f_cb_toLL_srv(
            const std::shared_ptr<robot_localization::srv::ToLL::Request> request,
            const std::shared_ptr<robot_localization::srv::ToLL::Response> response)
{   
    geographic_msgs::msg::GeoPoint::SharedPtr ll_point = std::make_shared<geographic_msgs::msg::GeoPoint>();
    f_dis2ll(request->map_point, ll_point);
    response->ll_point = *ll_point;
    return true;
}


bool WorldOdomTransform::f_cb_reset_tf_srv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    m_tf_set = false;
    response->success = true;
    response->message = "tf reset triggered";
    return true;
}

bool WorldOdomTransform::f_cb_reset_datum_srv(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    const std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    m_datum_set = false;
    m_tf_set = false;
    //if the gps covariance is not small we will wait.
    double  datum_reset_request_time = rclcpp::Clock(RCL_ROS_TIME).now().seconds();
    while(rclcpp::Clock(RCL_ROS_TIME).now().seconds() - datum_reset_request_time < m_gps_wait_time)
    {
        RCLCPP_INFO(get_logger(), "Waiting for a good gps fix for datum");
        if(m_gps_for_datum.position_covariance[0]<m_acceptable_var 
            && m_gps_for_datum.position_covariance[4]<m_acceptable_var
            && m_gps_for_datum.status.status>-1)
        {
            m_datum.latitude = m_gps_for_datum.latitude;
            m_datum.longitude = m_gps_for_datum.longitude;
            m_datum.altitude = m_gps_for_datum.altitude;
            m_datum_set = true;
            break;
        }
        sleep(1);
    }

    if(m_datum_set){
        response->success = true;
        response->message = "Datum reset done";
        return true;
    }
    else{
        response->success = false;
        response->message = "Datum failed due to lack of good gps fix within the defined time";
        return true;
    }
    return true;

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
