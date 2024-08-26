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
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));
    
    m_pnh->param<std::string>("world", m_world_frame, "world");

    m_pnh->param<std::string>("odom", m_odom_frame, "odom");

    m_pnh->param<std::string>("auv_tf_prefix", m_auv_tf_prefix, "");

    m_pnh->param<std::string>("asv_tf_prefix", m_asv_tf_prefix, "");

    m_pnh->param<bool>("publish_tf", m_publish_tf, true);

    m_tf_set = false;

    m_asv_world_frame = m_asv_tf_prefix + "/" + m_world_frame;

    m_auv_world_frame = m_auv_tf_prefix + "/" + m_world_frame;

    m_asvodom_frame = m_asv_tf_prefix + "/" + m_odom_frame;



    m_auvdatum_topic = "/" + m_auv_tf_prefix + "/gps/datum";
    m_auvdatum_subscriber = m_nh->subscribe(m_auvdatum_topic, 10, 
                                &WorldToWorldTransform::f_cb_auvdatum, this);

    // Create a client for wamv/fromLL service
    m_asvfromLL_srvname = "/" + m_asv_tf_prefix + "/fromLL";
    asvfrLL = m_nh->serviceClient<robot_localization::FromLL>(m_asvfromLL_srvname);

    // Initialize Services
    reset_tf_server = m_pnh->advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response> (
            "reset_tf", std::bind(&WorldToWorldTransform::f_cb_reset_tf_srv, this,std::placeholders::_1,std::placeholders::_2)
        );

    m_transform_listener.reset(
            new tf2_ros::TransformListener(m_transform_buffer)
        );
}

bool WorldToWorldTransform::f_set_tf() {
    // 0: SETUP
    ros::Time current_time = ros::Time::now();

    // 1: use wamv/fromLL to get coordinates of alpha_rise/world in wamv/odom
    robot_localization::FromLL::Request asvfrLL_req;
    robot_localization::FromLL::Response asvfrLL_res;

    asvfrLL_req.ll_point.latitude = m_auvdatum.latitude;
    asvfrLL_req.ll_point.longitude = m_auvdatum.longitude;
    asvfrLL_req.ll_point.altitude = m_auvdatum.altitude;
    if (asvfrLL.call(asvfrLL_req, asvfrLL_res)) {
        // ROS_INFO("wamv/fromLL response: %f %f %f", asvfrLL_res.map_point.x, asvfrLL_res.map_point.y, asvfrLL_res.map_point.z);
    }
    else {
        ROS_ERROR("Failed to call service wamv/fromLL");
    }
    // 2: get [wamv/world to wamv/odom] and [wamv/odom to alpha_rise/world]
    // 2.1 Get [wamv/world to wamv/odom]
    try
    {
            // ROS_INFO("%s->%s\r\n", m_asv_world_frame.c_str(), m_asvodom_frame.c_str());
            m_asvworld_to_asvodom = m_transform_buffer.lookupTransform(m_asv_world_frame, m_asvodom_frame, current_time, ros::Duration(10));
            // ROS_INFO("wamv/world to wamv/odom tf received");
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s to %s not available within the timeout period.\r\n", m_asv_world_frame.c_str(), m_asvodom_frame.c_str());
    }

    // 2.2 Get [wamv/odom to alpha_rise/world]
    m_asvodom_to_auvworld.header.stamp = current_time;
    m_asvodom_to_auvworld.header.frame_id = m_asvodom_frame;
    m_asvodom_to_auvworld.child_frame_id = m_auv_world_frame;
    m_asvodom_to_auvworld.transform.translation.x = asvfrLL_res.map_point.x;
    m_asvodom_to_auvworld.transform.translation.y = asvfrLL_res.map_point.y;
    m_asvodom_to_auvworld.transform.translation.z = asvfrLL_res.map_point.z;
    m_asvodom_to_auvworld.transform.rotation.w = 1;
    m_asvodom_to_auvworld.transform.rotation.x = 0;
    m_asvodom_to_auvworld.transform.rotation.y = 0;
    m_asvodom_to_auvworld.transform.rotation.z = 0;

    // 3: compute [wamv/world to alpha_rise/world] = [wamv/world to wamv/odom] * [wamv/odom to alpha_rise/world]
    tf2::Transform asvworld_to_asvodom_tf, asvodom_to_auvworld_tf, asvworld_to_auvworld_tf;
    tf2::fromMsg(m_asvworld_to_asvodom.transform, asvworld_to_asvodom_tf);
    tf2::fromMsg(m_asvodom_to_auvworld.transform, asvodom_to_auvworld_tf);
    asvworld_to_auvworld_tf = asvworld_to_asvodom_tf * asvodom_to_auvworld_tf;

    m_asvworld_to_auvworld.header.stamp = current_time;
    m_asvworld_to_auvworld.header.frame_id = m_asv_world_frame;
    m_asvworld_to_auvworld.child_frame_id = m_auv_world_frame;
    m_asvworld_to_auvworld.transform = tf2::toMsg(asvworld_to_auvworld_tf);

    // 4: Publish Transform (unless it is identical to the existing tf)
    if (m_asvworld_to_auvworld.transform.translation.x != transformStamped.transform.translation.x || m_asvworld_to_auvworld.transform.translation.y != transformStamped.transform.translation.y || m_asvworld_to_auvworld.transform.translation.z != transformStamped.transform.translation.z || m_asvworld_to_auvworld.transform.rotation.w != transformStamped.transform.rotation.w || m_asvworld_to_auvworld.transform.rotation.x != transformStamped.transform.rotation.x || m_asvworld_to_auvworld.transform.rotation.y != transformStamped.transform.rotation.y || m_asvworld_to_auvworld.transform.rotation.z != transformStamped.transform.rotation.z) {
        transformStamped = m_asvworld_to_auvworld;
        br.sendTransform(transformStamped);
        ROS_INFO("TF Between %s and %s is set\r\n", m_asv_world_frame.c_str(), m_auv_world_frame.c_str());
        m_tf_set = true;
    }
    
    return true;
}

void WorldToWorldTransform::f_cb_auvdatum(const geographic_msgs::GeoPoint& msg) {
    m_auvdatum = msg;

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