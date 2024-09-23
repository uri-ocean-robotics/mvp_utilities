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

#include "world_odom_transform.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/MagneticModel.hpp>


WorldOdomTransform::WorldOdomTransform(){
    m_nh.reset(new ros::NodeHandle(""));
    m_pnh.reset(new ros::NodeHandle("~"));
    
    m_pnh->param<std::string>("world", m_world_frame, "world");

    m_pnh->param<std::string>("odom", m_odom_frame, "odom");

    m_pnh->param<std::string>("tf_prefix", m_tf_prefix, "");

    m_pnh->param<double>("mag_declination", m_mag_declination, 0.0);

    m_pnh->param<bool>("mag_declination_auto", m_mag_declination_auto, true);
    //mag_north - true north in ENU frame.

    m_pnh->param<bool>("use_depth_for_tf", m_use_depth_for_tf, true);

    m_pnh->param<double>("acceptable_var", m_acceptable_var, 0.0);

    m_pnh->param<double>("position_accuracy", m_position_accuracy, 0.0);

    m_pnh->param<double>("max_gps_wait_time", m_gps_wait_time, 60.0);

    m_pnh->param<double>("datum_latitude", m_datum_latitude, 0.0);

    m_pnh->param<double>("datum_longitude", m_datum_longitude, 0.0);

    m_pnh->param<double>("datum_altitude", m_datum_altitude, 0.0);

    m_pnh->param<std::string>("mag_model_path", m_mag_model_path, "");

    m_pnh->param<bool>("publish_tf", m_publish_tf, true);
    
    m_datum.latitude = m_datum_latitude;
    m_datum.longitude = m_datum_longitude;
    m_datum.altitude = m_datum_altitude;


    if(m_datum.latitude==0){
        ROS_WARN("Datum is not set please set the parameters");
        m_datum_set = false;
    }
    else{
        m_datum_set = true;
    }
    m_tf_set = false;
    m_world_frame = m_tf_prefix + "/" + m_world_frame;

    m_odom_frame = m_tf_prefix + "/" + m_odom_frame;

    m_gps_odom_publisher = m_nh->advertise<nav_msgs::Odometry>("gps/odometry", 10);
    
    m_datum_publisher = m_nh->advertise<geographic_msgs::GeoPoint>("gps/datum",10);

    m_geopose_publisher = m_nh->advertise<geographic_msgs::GeoPoseStamped>("odometry/geopose",10);


    m_gps_fix_subscriber = m_nh->subscribe("gps/fix", 10, 
                                &WorldOdomTransform::f_cb_gps_fix, this);
    m_odom_subscriber = m_nh->subscribe("odometry", 10, 
                                &WorldOdomTransform::f_cb_odom, this);

    m_depth_subscriber=m_nh->subscribe("depth/odometry", 10, 
                                &WorldOdomTransform::f_cb_depth, this);
    /**
     * Initialize services
     */
    fromLL_server = m_nh->advertiseService<robot_localization::FromLL::Request,
        robot_localization::FromLL::Response>
        (
        "fromLL",
        std::bind(&WorldOdomTransform::f_cb_fromLL_srv,
            this,std::placeholders::_1,std::placeholders::_2
        )
        );

    toLL_server = m_nh->advertiseService<robot_localization::ToLL::Request,
        robot_localization::ToLL::Response>
        (
        "toLL",
        std::bind(&WorldOdomTransform::f_cb_toLL_srv,
            this,
            std::placeholders::_1,std::placeholders::_2
        )
        );

    reset_datum_server = m_pnh->advertiseService<std_srvs::Trigger::Request,
        std_srvs::Trigger::Response>
        (
        "reset_datum",
        std::bind(&WorldOdomTransform::f_cb_reset_datum_srv,
            this,std::placeholders::_1,std::placeholders::_2
        )
        );

    reset_tf_server = m_pnh->advertiseService<std_srvs::Trigger::Request,
        std_srvs::Trigger::Response>
        (
        "reset_tf",
        std::bind(&WorldOdomTransform::f_cb_reset_tf_srv,
            this,std::placeholders::_1,std::placeholders::_2
        )
        );

    m_transform_listener.reset(new
        tf2_ros::TransformListener(m_transform_buffer)
    );
    
}

void WorldOdomTransform::f_cb_gps_fix(const sensor_msgs::NavSatFix& msg)
{
    //compute the latitude longitude in the world frame using datum.
    geometry_msgs::Point map_point;
    nav_msgs::Odometry gps_odom;
    geographic_msgs::GeoPoint ll_point;

    m_gps = msg;
    m_gps_for_datum = msg;
    // m_odom_gps = m_odom; //map the most recent odom;
    //get gps location in the odom
    auto tf_odom_gps = m_transform_buffer.lookupTransform(
                    m_odom_frame,
                    msg.header.frame_id,
                    ros::Time(0),
                    ros::Duration(0.1)
                    );
    
    // auto tf_odom_gps = m_transform_buffer.lookupTransform(
    //                 msg.header.frame_id,
    //                 m_odom_frame,
    //                 ros::Time(0)
    //                 );

    m_odom_gps.header = m_odom.header;
    m_odom_gps.header.frame_id = msg.header.frame_id;
    m_odom_gps.pose.pose.position.x = tf_odom_gps.transform.translation.x;
    m_odom_gps.pose.pose.position.y = tf_odom_gps.transform.translation.y;
    m_odom_gps.pose.pose.position.z = tf_odom_gps.transform.translation.z;
    // Optional: fill the orientation
    m_odom_gps.pose.pose.orientation = tf_odom_gps.transform.rotation;

    m_depth_gps = m_depth;


    if(m_datum_set)
    {
        m_datum_publisher.publish(m_datum); 
    }
    // printf("Got GPS\r\n");
    //only do the following if the tf between world and odom are set.
    if(m_tf_set && m_datum_set)
    {
        if(m_gps.position_covariance[0]<m_acceptable_var 
                && m_gps.position_covariance[4]<m_acceptable_var
                && m_gps.status.status>-1)
        {
            ll_point.latitude = msg.latitude;
            ll_point.longitude = msg.longitude;
            ll_point.altitude = msg.altitude;

            //Get x and y from lattiude and longitude. x->east, y->north
            f_ll2dis(ll_point, map_point);
            //convert distance from gps into odom frame using mag_declination.
            try {        
                auto tf_w2o = m_transform_buffer.lookupTransform(
                    m_odom_frame,
                    m_world_frame,
                    ros::Time(0)
                );

                geometry_msgs::PoseStamped odom_pose, world_pose;
                world_pose.header = msg.header;
                world_pose.header.frame_id = m_world_frame;
                world_pose.pose.position.x = map_point.x;
                world_pose.pose.position.y = map_point.y;
                world_pose.pose.position.z = map_point.z;
                world_pose.pose.orientation.x = 0;
                world_pose.pose.orientation.y = 0;
                world_pose.pose.orientation.z = 0;
                world_pose.pose.orientation.w = 1.0;
                // Transform the pose from odom frame to world frame
                tf2::doTransform(world_pose, odom_pose, tf_w2o);

                // auto tf_eigen = tf2::transformToEigen(tf_w2o);

                gps_odom.pose.pose.position.x = odom_pose.pose.position.x;
                gps_odom.pose.pose.position.y = odom_pose.pose.position.y;
                gps_odom.pose.pose.position.z = odom_pose.pose.position.z;
                gps_odom.header.frame_id = m_odom_frame;
                gps_odom.child_frame_id = msg.header.frame_id;
                gps_odom.header.stamp = msg.header.stamp;
                // gps_odom.pose.covariance[0] = pow(m_position_accuracy,2);
                gps_odom.pose.covariance[0] = msg.position_covariance[0];
                gps_odom.pose.covariance[1] = 0;
                gps_odom.pose.covariance[2] = 0;
                gps_odom.pose.covariance[6] =0;
                // gps_odom.pose.covariance[7] =  pow(m_position_accuracy,2);
                gps_odom.pose.covariance[7] = msg.position_covariance[4];
                gps_odom.pose.covariance[8] = 0;
                gps_odom.pose.covariance[12] = 0;
                gps_odom.pose.covariance[13] = 0;
                // gps_odom.pose.covariance[14] =  pow(m_position_accuracy,2);
                gps_odom.pose.covariance[14] =  msg.position_covariance[8];
                m_gps_odom_publisher.publish(gps_odom);  

            } catch(tf2::TransformException &e) {
                ROS_WARN_STREAM_THROTTLE(10, std::string("Can't get the tf from world to odom") + e.what());
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
                f_set_tf();
            }
            else{
                ROS_INFO("GPS fix covariance is not good");
                return;
            }
        }
    }
}



bool WorldOdomTransform::f_set_tf()
{
    //check the quality of the gps if the x and y variance is good enough?

    geographic_msgs::GeoPoint ll_point;
    geometry_msgs::Point map_point;
    nav_msgs::Odometry m_odom_gps_temp = m_odom_gps;
    nav_msgs::Odometry m_depth_gps_temp = m_depth_gps;
    sensor_msgs::NavSatFix m_gps_temp = m_gps;

    GeographicLib::MagneticModel magModel("wmm2020", m_mag_model_path);

    
    ll_point.latitude = m_gps_temp.latitude;
    ll_point.longitude = m_gps_temp.longitude;
    ll_point.altitude = m_gps_temp.altitude;
    //Get x and y from lattiude and longitude. x->east, y->north
    f_ll2dis(ll_point, map_point);
    //get mag declination
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
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = m_world_frame;
    transformStamped.child_frame_id = m_odom_frame;

    //convert odom xy into world first using magnetic declination
    double odom_world_x = m_odom_gps_temp.pose.pose.position.x*cos(m_mag_declination) - m_odom_gps_temp.pose.pose.position.y*sin(m_mag_declination);
    double odom_world_y = m_odom_gps_temp.pose.pose.position.x*sin(m_mag_declination) + m_odom_gps_temp.pose.pose.position.y*cos(m_mag_declination);

    double dx =  map_point.x -odom_world_x;
    double dy =  map_point.y - odom_world_y;
    printf("gps fix in world =%lf, %lf\r\n", map_point.x, map_point.y);
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

    tf2::Quaternion q;
    q.setRPY(0, 0, m_mag_declination);

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.header.stamp = ros::Time::now();
    br.sendTransform(transformStamped);
    // sleep(0.1);
    
    // //checking the m_gps in odom should align with the m_odom_gps
    // try {        
    //     auto tf_w2o = m_transform_buffer.lookupTransform(
    //         m_odom_frame,
    //         m_world_frame,
    //         ros::Time(0),
    //         ros::Duration(1)
    //     );

    //     auto tf_eigen = tf2::transformToEigen(tf_w2o);
    //     Eigen::Vector3d p_odom;

    //     p_odom = tf_eigen.rotation() * Eigen::Vector3d(map_point.x, map_point.y, 0) + tf_eigen.translation();
    //     printf("gps fix points in odom with new tf=%lf,%lf\r\n\r\n", p_odom.x(), p_odom.y());


    // } catch(tf2::TransformException &e) {
    //         ROS_WARN_STREAM_THROTTLE(10, std::string("Can't get the tf from world to odom") + e.what());
    //     }

    ROS_INFO("TF Between world and odom is set\r\n");
    m_tf_set = true;
    return true;
}


void WorldOdomTransform::f_cb_odom(const nav_msgs::Odometry& msg)
{
    m_odom = msg;
    //if tf is set i will keep setting the tf
    if(m_tf_set)
    {   
        
        //publishtf
        transformStamped.header.stamp = ros::Time::now();
        br.sendTransform(transformStamped);
        //convert odom from odom to world 
        try {        
            auto tf_o2w = m_transform_buffer.lookupTransform(
                m_world_frame,
                m_odom_frame,
                ros::Time(0)
            );
            geometry_msgs::PoseStamped odom_pose, world_pose;
            odom_pose.header = msg.header;
            odom_pose.pose = msg.pose.pose;

            // Transform the pose from odom frame to world frame
            tf2::doTransform(odom_pose, world_pose, tf_o2w);
            geometry_msgs::Point map_point;
            geographic_msgs::GeoPoint ll_point;
            world_pose.header= msg.header;

            map_point.x = world_pose.pose.position.x;
            map_point.y = world_pose.pose.position.y;
            map_point.z = world_pose.pose.position.z;

            f_dis2ll(map_point, ll_point);
            geographic_msgs::GeoPoseStamped geopose;
            geopose.pose.position.latitude = ll_point.latitude;
            geopose.pose.position.longitude = ll_point.longitude;
            geopose.pose.position.altitude = ll_point.altitude;
            geopose.pose.orientation.x = world_pose.pose.orientation.x;
            geopose.pose.orientation.y = world_pose.pose.orientation.y;
            geopose.pose.orientation.z = world_pose.pose.orientation.z;
            geopose.pose.orientation.w = world_pose.pose.orientation.w;
            geopose.header= world_pose.header;
            
            m_geopose_publisher.publish(geopose);

        } catch(tf2::TransformException &e) {
            ROS_WARN_STREAM_THROTTLE(10, std::string("Can't get the tf from world to odom") + e.what());
        }
    }
}

// void WorldOdomTransform::f_cb_depth(const geometry_msgs::PoseWithCovarianceStamped& msg)
// {
//     m_depth = msg;
// }

void WorldOdomTransform::f_cb_depth(const nav_msgs::Odometry& msg)
{
    m_depth = msg;
}

bool WorldOdomTransform::f_cb_reset_datum_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    m_datum_set = false;
    m_tf_set = false;
    //if the gps covariance is not small we will wait.
    ros::Time datum_reset_request_time = ros::Time::now();
    while(ros::Time::now().toSec()-datum_reset_request_time.toSec()<m_gps_wait_time)
    {
        ROS_INFO("Waiting for a good gps fix for datum");
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
        resp.success = true;
        resp.message = "Datum reset done";
        return true;
    }
    else{
        resp.success = false;
        resp.message = "Datum failed due to lack of good gps fix within the defined time";
        return true;
    }
    return true;
}

bool WorldOdomTransform::f_cb_reset_tf_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    m_tf_set = false;
    resp.success = true;
    resp.message = "tf reset triggered";
    return true;
}

bool WorldOdomTransform::f_cb_fromLL_srv(robot_localization::FromLL::Request &req, robot_localization::FromLL::Response &resp)
{
    f_ll2dis(req.ll_point, resp.map_point);
    return true;

}

bool WorldOdomTransform::f_cb_toLL_srv(robot_localization::ToLL::Request &req, robot_localization::ToLL::Response &resp)
{
    f_dis2ll(req.map_point, resp.ll_point);
    return true;
}

void WorldOdomTransform::f_ll2dis(geographic_msgs::GeoPoint ll_point, geometry_msgs::Point& map_point)
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

    map_point.x = east;
    map_point.y = north;
    map_point.z = ll_point.altitude;
}

void WorldOdomTransform::f_dis2ll(geometry_msgs::Point map_point, geographic_msgs::GeoPoint& ll_point)
{
    //from world frame
    // double lat = m_datum.latitude + map_point.y/m_earthR * 180.0/M_PI;
    // double lon = m_datum.longitude + map_point.x/(m_earthR*cos(m_datum.latitude/180.0*M_PI)) * 180.0/M_PI;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();
    double lat, lon;
     // Calculate latitude and longitude of target point
    geod.Direct(m_datum.latitude, m_datum.longitude, 0.0, map_point.y, lat, lon); // Move north
    geod.Direct(lat, lon, 90.0, map_point.x, lat, lon); // Move east
    
    ll_point.latitude = lat;
    ll_point.longitude = lon;
    ll_point.altitude = map_point.z;


}


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "gps_transform");

    WorldOdomTransform d;
    
    ros::spin();

    return 0;
}