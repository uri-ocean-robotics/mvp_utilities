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

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#include "depth_odometry_publisher.hpp"

DepthOdomNode::DepthOdomNode() {
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));

    m_pnh->param<std::string>("frame_id", m_frame_id, "world");
    m_pnh->param<double>("depth_covariance", m_depth_covariance, 0.0);

    m_depth_subscriber = m_nh->subscribe("depth", 10, &DepthOdomNode::f_depth_callback, this);
    m_depth_publisher = m_nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("depth/odometry", 10);

}

void DepthOdomNode::f_depth_callback(const mvp_msgs::Float64StampedConstPtr &msg) {

    geometry_msgs::PoseWithCovarianceStamped out;

    out.header = msg->header;
    out.header.frame_id = m_frame_id;
    out.pose.pose.position.x = 0.0;
    out.pose.pose.position.y = 0.0;
    out.pose.pose.position.z = -msg->data;

    out.pose.covariance[6 * 2 + 2] = m_depth_covariance;

    m_depth_publisher.publish(out);

}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "depth_odometry_node");

    DepthOdomNode d;

    ros::spin();

    return 0;
}