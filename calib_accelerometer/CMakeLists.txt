cmake_minimum_required(VERSION 3.0.2)
project(calib_accelerometer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs roscpp rospy
  sensor_msgs std_msgs tf tf_conversions
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES calib_accelerometer
 CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
 DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
#   include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

list(APPEND thirdparty_libraries
  ${catkin_LIBRARIES}
)

add_executable(calib_accel src/node.cpp)
target_link_libraries(calib_accel ${thirdparty_libraries})

