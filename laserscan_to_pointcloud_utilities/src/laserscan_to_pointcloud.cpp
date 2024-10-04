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

#include "laserscan_to_pointcloud.hpp"

LaserScanToPointCloud::LaserScanToPointCloud() {
// Initialize ROS node handles
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

// Load parameters from the parameter server
    m_pnh->param<std::string>("robot_name", m_robot_name, "");
    m_pnh->param<std::string>("world", m_world_name, "world");
    m_pnh->param<std::string>("topic_name", m_topic_name, "wamv/vlp16/stonefish/data");
    m_pnh->param<std::string>("laserscan_name", m_laserscan_name, "scan");
    m_pnh->param<std::string>("pointcloud_name", m_pointcloud_name, "pointcloud");

// Initialize frame and topic names
    m_world_frame = m_robot_name + "/" + m_world_name;
    m_laserscan_sub_topic = "/" + m_topic_name;
    m_laserscan_pub_topic = "/" + m_topic_name + "/" + m_laserscan_name;
    m_pointcloud_pub_topic = "/" + m_topic_name + "/" + m_pointcloud_name;

// Initialize subscriber and publisher
    m_scan_sub = m_nh->subscribe(m_laserscan_sub_topic, 10, &LaserScanToPointCloud::f_cb_scan, this);
    m_scan_pub = m_nh->advertise<sensor_msgs::LaserScan>(m_laserscan_pub_topic, 10);
    m_cloud_pub = m_nh->advertise<sensor_msgs::PointCloud2>(m_pointcloud_pub_topic, 10);

// Initialize transform listener
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(m_tf_buffer);
}

bool LaserScanToPointCloud::f_laserscan_to_pointcloud() {
    try
    {
        sensor_msgs::PointCloud2 cloud;
        geometry_msgs::TransformStamped transform_stamped = m_tf_buffer.lookupTransform(m_world_frame, m_scan_msg.header.frame_id, ros::Time::now(), ros::Duration(1.0));

        // Convert LaserScan to PointCloud2
        m_projector.transformLaserScanToPointCloud(m_world_frame, m_scan_msg, cloud, m_tf_buffer);

        // Publish the point cloud data
        m_cloud_pub.publish(cloud);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
    }
    return true;
}

void LaserScanToPointCloud::f_cb_scan(const sensor_msgs::LaserScan& msg) {
// Try to lookup the transform between the laser frame and base frame
    m_scan_msg = msg;
    f_laserscan_to_pointcloud();
}

int main(int argc, char* argv[]) {
    // Initialize the ROS node
    ros::init(argc, argv, "laserscan_to_pointcloud");

    LaserScanToPointCloud d;

    ros::spin();

    return 0;
}