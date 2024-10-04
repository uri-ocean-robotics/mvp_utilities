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

#include "world_to_world_transform.hpp"

WorldToWorldTransform::WorldToWorldTransform() {

    // Initialize ROS node handles
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));
    
    // Load parameters from the parameter server
    m_pnh->param<std::string>("world", m_world_frame, "world");
    m_pnh->param<std::string>("child_name", m_child_name, "");
    m_pnh->param<std::string>("parent_name", m_parent_name, "");
    m_pnh->param<bool>("publish_tf", m_publish_tf, true);

    // Initialize frames and flags
    m_tf_set = false;
    m_parent_world_frame = m_parent_name + "/" + m_world_frame;
    m_child_world_frame = m_child_name + "/" + m_world_frame;
    m_child_datum_topic = "/" + m_child_name + "/gps/datum";

    // Subscribe to the child datum topic
    m_child_datum_subscriber = m_nh->subscribe(m_child_datum_topic, 10, 
                                               &WorldToWorldTransform::f_cb_child_datum, this);

    // Create a client for parent/fromLL service
    m_parent_fromLL_srv_name = "/" + m_parent_name + "/fromLL";
    m_parent_fromLL_client = m_nh->serviceClient<robot_localization::FromLL>(m_parent_fromLL_srv_name);

    // Initialize reset tf service
    reset_tf_server = m_pnh->advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response> (
            "reset_tf", std::bind(&WorldToWorldTransform::f_cb_reset_tf_srv, this,std::placeholders::_1,std::placeholders::_2)
        );
}

bool WorldToWorldTransform::f_set_tf() {
    // Setup current time
    ros::Time current_time = ros::Time::now();

    // Request parent/fromLL to get child/world coordinates in parent/world
    robot_localization::FromLL srv;
    srv.request.ll_point = m_child_datum;

    if (!m_parent_fromLL_client.call(srv)) {
        ROS_ERROR("Failed to call service: %s", m_parent_fromLL_srv_name.c_str());
    }

    // Setup [parent/world to child/world] transform
    m_parent_world_to_child_world.header.stamp = current_time;
    m_parent_world_to_child_world.header.frame_id = m_parent_world_frame;
    m_parent_world_to_child_world.child_frame_id = m_child_world_frame;
    m_parent_world_to_child_world.transform.translation.x = srv.response.map_point.x;
    m_parent_world_to_child_world.transform.translation.y = srv.response.map_point.y;
    m_parent_world_to_child_world.transform.translation.z = srv.response.map_point.z;
    m_parent_world_to_child_world.transform.rotation.w = 1.0;
    m_parent_world_to_child_world.transform.rotation.x = 0.0;
    m_parent_world_to_child_world.transform.rotation.y = 0.0;
    m_parent_world_to_child_world.transform.rotation.z = 0.0;

    // printf("map_point.x: %f | map_point.y: %f | map_point.z: %f \n", srv.response.map_point.x, srv.response.map_point.y, srv.response.map_point.z);

    // Publish Transform if it has changed
    if ((m_tf_set == false) || (m_parent_world_to_child_world.transform.translation.x != transformStamped.transform.translation.x || m_parent_world_to_child_world.transform.translation.y != transformStamped.transform.translation.y || m_parent_world_to_child_world.transform.translation.z != transformStamped.transform.translation.z || m_parent_world_to_child_world.transform.rotation.w != transformStamped.transform.rotation.w || m_parent_world_to_child_world.transform.rotation.x != transformStamped.transform.rotation.x || m_parent_world_to_child_world.transform.rotation.y != transformStamped.transform.rotation.y || m_parent_world_to_child_world.transform.rotation.z != transformStamped.transform.rotation.z)) {
        transformStamped = m_parent_world_to_child_world;
        br.sendTransform(transformStamped);
        ROS_INFO("tf between %s and %s is set\r\n", m_parent_world_frame.c_str(), m_child_world_frame.c_str());
        m_tf_set = true;
    }
    
    return true;
}

void WorldToWorldTransform::f_cb_child_datum(const geographic_msgs::GeoPoint& msg) {
    m_child_datum = msg;
    f_set_tf();
}

bool WorldToWorldTransform::f_cb_reset_tf_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    m_tf_set = false;
    resp.success = true;
    resp.message = "tf reset triggered";
    return true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "world_to_world_tf");
    WorldToWorldTransform d;
    ros::spin();
    return 0;
}