cmake_minimum_required(VERSION 3.0.2)
project(save_cloud)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED)

find_package(PCL REQUIRED COMPONENTS)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp rospy nav_msgs pcl_ros tf2 nav_msgs
  sensor_msgs std_msgs tf2_ros tf2_eigen
  pcl_conversions message_filters tf2_sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp rospy nav_msgs pcl_ros tf2 nav_msgs
                sensor_msgs std_msgs tf2_ros tf2_eigen
                pcl_conversions message_filters tf2_sensor_msgs
 DEPENDS PCL EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(save_cloud_node src/save_cloud.cpp)
target_link_libraries(save_cloud_node ${thirdparty_libraries})

add_executable(combine_cloud_node src/combine_cloud.cpp)
target_link_libraries(combine_cloud_node ${thirdparty_libraries})