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
    Email: mzhou
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
    m_pnh->param<std::string>("odom", m_odom_frame, "odom");
    m_pnh->param<std::string>("child_tf_prefix", m_child_tf_prefix, "");
    m_pnh->param<std::string>("parent_tf_prefix", m_parent_tf_prefix, "");
    m_pnh->param<bool>("publish_tf", m_publish_tf, true);

    // Initialize frames and flags
    m_tf_set = false;
    m_parent_world_frame = m_parent_tf_prefix + "/" + m_world_frame;
    m_child_world_frame = m_child_tf_prefix + "/" + m_world_frame;
    m_parent_odom_frame = m_parent_tf_prefix + "/" + m_odom_frame;
    m_child_datum_topic = "/" + m_child_tf_prefix + "/gps/datum";

    // Subscribe to the child datum topic
    m_child_datum_subscriber = m_nh->subscribe(m_child_datum_topic, 10, 
                                               &WorldToWorldTransform::f_cb_childdatum, this);

    // Create a client for parent/fromLL service
    m_parent_fromLL_srv_name = "/" + m_parent_tf_prefix + "/fromLL";
    m_parent_fromLL_client = m_nh->serviceClient<robot_localization::FromLL>(m_parent_fromLL_srv_name);

    // Initialize reset tf service
    reset_tf_server = m_pnh->advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response> (
            "reset_tf", std::bind(&WorldToWorldTransform::f_cb_reset_tf_srv, this,std::placeholders::_1,std::placeholders::_2)
        );

    // Initialize tf listener
    m_transform_listener.reset(
            new tf2_ros::TransformListener(m_transform_buffer)
        );
}

bool WorldToWorldTransform::f_set_tf() {
    // Setup current time
    ros::Time current_time = ros::Time::now();

    // Request parent/fromLL to get child/world coordinates in parent/odom
    robot_localization::FromLL::Request parent_fromLL_client_req;
    robot_localization::FromLL::Response parent_fromLL_client_res;

    parent_fromLL_client_req.ll_point = m_child_datum;

    if (!m_parent_fromLL_client.call(parent_fromLL_client_req, parent_fromLL_client_res)) {
        ROS_ERROR("Failed to call service: %s", m_parent_fromLL_srv_name.c_str());
    }

    // Get [parent/world to parent/odom] transform
    try
    {
        m_parent_world_to_parent_odom = m_transform_buffer.lookupTransform(m_parent_world_frame, m_parent_odom_frame, current_time, ros::Duration(10));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s to %s not available within the timeout period.\r\n", m_parent_world_frame.c_str(), m_parent_odom_frame.c_str());
    }

    // Setup [parent/odom to child/world] transform
    m_parent_odom_to_child_world.header.stamp = current_time;
    m_parent_odom_to_child_world.header.frame_id = m_parent_odom_frame;
    m_parent_odom_to_child_world.child_frame_id = m_child_world_frame;
    m_parent_odom_to_child_world.transform.translation.x = parent_fromLL_client_res.map_point.x;
    m_parent_odom_to_child_world.transform.translation.y = parent_fromLL_client_res.map_point.y;
    m_parent_odom_to_child_world.transform.translation.z = parent_fromLL_client_res.map_point.z;
    m_parent_odom_to_child_world.transform.rotation.w = 1.0;
    m_parent_odom_to_child_world.transform.rotation.x = 0.0;
    m_parent_odom_to_child_world.transform.rotation.y = 0.0;
    m_parent_odom_to_child_world.transform.rotation.z = 0.0;

    // Compute [parent/world to child/world] = [parent/world to parent/odom] * [parent/odom to child/world]
    tf2::Transform parent_world_to_parent_odom_tf, parent_odom_to_child_world_tf, parent_world_to_child_world_tf;
    tf2::fromMsg(m_parent_world_to_parent_odom.transform, parent_world_to_parent_odom_tf);
    tf2::fromMsg(m_parent_odom_to_child_world.transform, parent_odom_to_child_world_tf);
    parent_world_to_child_world_tf = parent_world_to_parent_odom_tf * parent_odom_to_child_world_tf;

    m_parent_world_to_child_world.header.stamp = current_time;
    m_parent_world_to_child_world.header.frame_id = m_parent_world_frame;
    m_parent_world_to_child_world.child_frame_id = m_child_world_frame;
    m_parent_world_to_child_world.transform = tf2::toMsg(parent_world_to_child_world_tf);

    // Publish Transform if it has changed
    if (m_parent_world_to_child_world.transform.translation.x != transformStamped.transform.translation.x || m_parent_world_to_child_world.transform.translation.y != transformStamped.transform.translation.y || m_parent_world_to_child_world.transform.translation.z != transformStamped.transform.translation.z || m_parent_world_to_child_world.transform.rotation.w != transformStamped.transform.rotation.w || m_parent_world_to_child_world.transform.rotation.x != transformStamped.transform.rotation.x || m_parent_world_to_child_world.transform.rotation.y != transformStamped.transform.rotation.y || m_parent_world_to_child_world.transform.rotation.z != transformStamped.transform.rotation.z) {
        transformStamped = m_parent_world_to_child_world;
        br.sendTransform(transformStamped);
        ROS_INFO("tf between %s and %s is set\r\n", m_parent_world_frame.c_str(), m_child_world_frame.c_str());
        m_tf_set = true;
    }
    
    return true;
}

void WorldToWorldTransform::f_cb_childdatum(const geographic_msgs::GeoPoint& msg) {
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