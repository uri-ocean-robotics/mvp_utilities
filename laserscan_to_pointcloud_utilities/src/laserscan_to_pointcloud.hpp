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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class LaserScanToPointCloud {

private:

    ros::NodeHandlePtr m_nh;
    ros::NodeHandlePtr m_pnh;
    
    ros::Subscriber m_scan_sub;
    ros::Publisher m_scan_pub;
    ros::Publisher m_cloud_pub;

    std::string m_topic_name;
    std::string m_robot_name;
    std::string m_world_name;
    std::string m_world_frame;

    std::string m_laserscan_name;
    std::string m_pointcloud_name;
    
    std::string m_laserscan_sub_topic;
    std::string m_laserscan_pub_topic;
    std::string m_pointcloud_pub_topic;

    sensor_msgs::LaserScan m_scan_msg;

    laser_geometry::LaserProjection m_projector;
    tf2_ros::Buffer m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    bool f_laserscan_to_pointcloud();
    void f_cb_scan(const sensor_msgs::LaserScan& scan_msg);

public:

    LaserScanToPointCloud();

};