/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Mingxi Zhou
    Email: mzhou@uri.edu
    Year: 2024

    Copyright (C) 2024 Smart Ocean Systems Laboratory
*/

#pragma once

#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include "mvp_msgs/Float64Stamped.h"
#include "geographic_msgs/GeoPoint.h"
#include "geographic_msgs/GeoPoseStamped.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_srvs/Trigger.h"
#include "robot_localization/FromLL.h"
#include "robot_localization/ToLL.h"
#include "nav_msgs/Odometry.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_eigen/tf2_eigen.h"
#include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/doTransform.h>

#include "memory"
#include "vector"
#include "thread"
#include "functional"
#include "cmath"

class WorldOdomTransform{

private:

    ros::NodeHandlePtr m_nh;

    ros::NodeHandlePtr m_pnh;


    ros::Publisher m_gps_odom_publisher;

    ros::Publisher m_datum_publisher;

    ros::Publisher m_geopose_publisher;


    ros::Subscriber m_gps_fix_subscriber;

    ros::Subscriber m_odom_subscriber;

    ros::Subscriber m_depth_subscriber;


    ros::ServiceServer fromLL_server;

    ros::ServiceServer toLL_server;

    ros::ServiceServer reset_tf_server;

    ros::ServiceServer reset_datum_server;

    geographic_msgs::GeoPoint m_datum;

    nav_msgs::Odometry m_odom, m_odom_gps;

    nav_msgs::Odometry m_depth, m_depth_gps;

    sensor_msgs::NavSatFix m_gps;

    sensor_msgs::NavSatFix m_gps_for_datum;

    std::string m_world_frame;

    std::string m_odom_frame;
    
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

    void f_cb_gps_fix(const sensor_msgs::NavSatFix& msg);

    void f_cb_odom(const nav_msgs::Odometry& msg);
    
    void f_cb_depth(const nav_msgs::Odometry& msg);

    bool f_cb_reset_datum_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    bool f_cb_reset_tf_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    bool f_cb_fromLL_srv(robot_localization::FromLL::Request &req, robot_localization::FromLL::Response &resp);
    
    bool f_cb_toLL_srv(robot_localization::ToLL::Request &req, robot_localization::ToLL::Response &resp);
    
    void f_ll2dis(geographic_msgs::GeoPoint ll_point, geometry_msgs::Point& map_point);

    void f_dis2ll(geometry_msgs::Point map_point, geographic_msgs::GeoPoint& ll_point);

    bool f_set_tf();

    geometry_msgs::TransformStamped transformStamped;
    tf2_ros::StaticTransformBroadcaster br;

    tf2_ros::Buffer m_transform_buffer;

    std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;

    
public:

    WorldOdomTransform();
    
    // void f_check_tf();



};

