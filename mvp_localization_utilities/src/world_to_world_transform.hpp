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

    Author: Ansel Austin
    Email: ansel.austin@uri.edu
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
#include <tf2_ros/static_transform_broadcaster.h>

class WorldToWorldTransform{

private:

    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;
    ros::Subscriber m_child_datum_subscriber;
    ros::ServiceClient m_parent_fromLL_client;
    ros::ServiceServer reset_tf_server;

    geographic_msgs::GeoPoint m_child_datum;

    std::string m_world_frame;
    std::string m_parent_world_frame;
    std::string m_child_world_frame;
    std::string m_child_name;
    std::string m_parent_name;
    std::string m_child_datum_topic;
    std::string m_parent_fromLL_srv_name;

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped m_parent_world_to_child_world;

    bool m_tf_set = false;
    bool m_publish_tf;

    tf2_ros::StaticTransformBroadcaster br;

    void f_cb_child_datum(const geographic_msgs::GeoPoint& msg);
    bool f_cb_reset_tf_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);
    bool f_set_tf();

public:
    WorldToWorldTransform();
};