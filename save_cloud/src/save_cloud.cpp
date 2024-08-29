#include <fstream>
#include <iostream>
// ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>


class SavePC
{
public:
    SavePC():
       pnh("~"), transform_listener(transform_buffer)
    {
        pc_sub = nh.subscribe<sensor_msgs::PointCloud2> ("/pointcloud/in", 1, &SavePC::pointcloudCallback, this);

        odom_sub = nh.subscribe<nav_msgs::Odometry> ("/odom", 1, &SavePC::odomCallback, this);

        pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud/out",1);

        trigger = nh.advertiseService("/save_pointcloud", &SavePC::srvCallback, this);

        // param for cloud saving
        pnh.param<std::string>("frame_parent", frame_parent, "world");
        pnh.param<std::string>("frame_child", frame_child, "base");
        pnh.param<std::string>("cloud_path", cloud_path, "/tmp/test");
        pnh.param<bool>("general_mode", general_mode, true);
        pnh.param<int>("max_saved_frames", max_saved_frames, 500);

        // param for odom saving
        pnh.param<std::string>("odom_file", odom_file, "/tmp/odom.csv");
        pnh.param<bool>("save_odom", save_odom, false);

        save = false;
    }

    ~SavePC() {}

    void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input);

    void odomCallback(const nav_msgs::Odometry::ConstPtr& input);

    bool srvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);

private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber pc_sub;
    ros::Subscriber odom_sub;
    ros::Publisher pub;
    ros::ServiceServer trigger;

    pcl::PointCloud<pcl::PointXYZI> all_cloud;

    tf2_ros::Buffer transform_buffer;
    tf2_ros::TransformListener transform_listener;

    std::ofstream file;

    //// param
    std::string frame_parent;
    std::string frame_child;
    std::string cloud_path;
    int max_saved_frames;
    bool general_mode;

    std::string odom_file;
    bool save_odom;

    bool save;
    int cloud_frame_count = 0;
    int saved_cloud_num = 1;
};

bool SavePC::srvCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    if (req.data) {
        res.success = true;
        res.message = "True";
        ROS_INFO("Triggered Save Cloud: True!");
    }
    else {
        res.success = false;
        res.message = "False";
        ROS_INFO("False!");
        ROS_INFO("Triggered Save Cloud: False!");
    }

    // set flag
    save = true; 

    return true;
}


void SavePC::odomCallback(const nav_msgs::Odometry::ConstPtr& input) {
    // printf("%s Timestamp:%.9f\n", 
    //         input->header.frame_id.c_str(), input->header.stamp.toSec());    

    if (save_odom) {
        file.open(odom_file, std::ios_base::app);//std::ios_base::app
        file <<std::fixed << std::setprecision(9)<<input->header.stamp.toSec()<<",";
        file <<input->pose.pose.position.x<<",";
        file <<input->pose.pose.position.y<<",";
        file <<input->pose.pose.position.z<<std::endl;
        file.close();
    }
}

void SavePC::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input) {

    geometry_msgs::TransformStamped trans_tf;

    // get tf between world and odom
    try {
        trans_tf = transform_buffer.lookupTransform(
            frame_parent,
            frame_child,
            ros::Time(0));
    } 
    catch(tf2::TransformException& e) {
        ROS_WARN_STREAM_THROTTLE(10, std::string("Can't find TF") + e.what());
        return ;
    }

    // printf("%s Timestamp:%.9f\n", 
    //         input->header.frame_id.c_str(), input->header.stamp.toSec());

    // Transform the point cloud using the transform retrieved
    sensor_msgs::PointCloud2 transformed_cloud_msg;
    tf2::doTransform(*input, transformed_cloud_msg, trans_tf);
    pub.publish(transformed_cloud_msg);

    // transform
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(transformed_cloud_msg, *pcl_cloud);

    // add the pointclouds
    all_cloud += *pcl_cloud;

    // save by the frame counts
    cloud_frame_count++;
    if(cloud_frame_count > max_saved_frames && general_mode)
    {
        std::string path = cloud_path + std::to_string(saved_cloud_num)+".pcd";
        pcl::io::savePCDFileASCII (path, all_cloud);

        // info
        ROS_INFO("Time: %f, saved at: %s", input->header.stamp.toSec(), path.c_str()); 

        // update 
        all_cloud.clear();
        saved_cloud_num++;
        cloud_frame_count = 0;
    }

    // save by ros service
    if(save){
        save = false;

        // save
        std::string path = cloud_path + std::to_string(saved_cloud_num)+".pcd";
        pcl::io::savePCDFileASCII (path, all_cloud);

        // info
        ROS_INFO("Time: %f, saved at: %s", input->header.stamp.toSec(), path.c_str()); 

        // update 
        all_cloud.clear();
        saved_cloud_num++;
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Simple_Example_Node");

  SavePC  example;

  ros::spin();

  return 0;
}