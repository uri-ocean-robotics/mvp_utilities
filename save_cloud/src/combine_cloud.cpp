#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

#include <string>
#include <iostream>
#include <filesystem>

int main(int argc, char** argv)
{
    // ===================================================================== //
    // Launch our ros node to load parameters
    // ===================================================================== //

    ros::init(argc, argv, "combine_pointclouds");
    ros::NodeHandle nh("~");
    std::string saved_path;
    bool save;
    nh.param<std::string>("saved_path", saved_path, "temp/pcd/");
    nh.param<bool>("save", save, false);

    // ===================================================================== //
    // get all pcd and combined
    // ===================================================================== //

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);

    for (const auto & entry : std::filesystem::directory_iterator(saved_path))
    {
      // check if this is a pointcloud file, if not just try next one
      auto path_str = entry.path().string();
      auto suffix = path_str.substr(path_str.size() - 3);
      if(suffix!="pcd")
      {
        continue;
      }

      // Load the PCD file
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      if (pcl::io::loadPCDFile<pcl::PointXYZI>(entry.path(), *cloud) == -1)
      {
          PCL_ERROR("Couldn't read file \n");
          return -1;
      }
      std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << entry.path() << std::endl;

      // merge
      *cloud_all += *cloud;
    }

    // ===================================================================== //
    // Visualize and the point cloud
    // ===================================================================== //

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZI>(cloud_all, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    if (save)
    {
      std::string path = saved_path + "total_cloud.pcd";
        pcl::io::savePCDFileASCII (path, *cloud_all);
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
