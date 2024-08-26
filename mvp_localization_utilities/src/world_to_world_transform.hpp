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

#include "geographic_msgs/GeoPoint.h"
#include "std_srvs/Trigger.h"
#include "robot_localization/FromLL.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

class WorldToWorldTransform{

private:

    ros::NodeHandlePtr m_nh;

    ros::NodeHandlePtr m_pnh;

    ros::Subscriber m_auvdatum_subscriber;

    ros::ServiceClient asvfrLL;

    ros::ServiceServer reset_tf_server;

    ros::ServiceServer reset_datum_server;

    geographic_msgs::GeoPoint m_auvdatum;

    std::string m_world_frame;

    std::string m_asv_world_frame;

    std::string m_auv_world_frame;

    std::string m_odom_frame;

    std::string m_asvodom_frame;
    
    std::string m_auv_tf_prefix;
    
    std::string m_asv_tf_prefix;

    std::string m_auvdatum_topic;

    std::string m_asvfromLL_srvname;
    
    bool m_datum_set = false;

    bool m_tf_set = false;

    bool m_publish_tf;

    void f_cb_auvdatum(const geographic_msgs::GeoPoint& msg);

    bool f_cb_reset_tf_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    bool f_set_tf();

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped m_asvworld_to_auvworld;
    geometry_msgs::TransformStamped m_asvworld_to_asvodom;
    geometry_msgs::TransformStamped m_asvodom_to_auvworld;
    tf2_ros::StaticTransformBroadcaster br;

    tf2_ros::Buffer m_transform_buffer;

    std::shared_ptr<tf2_ros::TransformListener> m_transform_listener;
    
public:

    WorldToWorldTransform();
    
    // void f_check_tf();

};

