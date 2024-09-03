cmake_minimum_required(VERSION 3.8)
project(mvp_localization_utilities)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(robot_localization REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  geographic_msgs
  nav_msgs
  robot_localization
  tf2_ros
  tf2
  tf2_geometry_msgs

)

include_directories(include
  # ${EIGEN3_INCLUDE_DIR}
)
set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED 
  include/mvp_localization_utilities/world_odom_transform.cpp
)

ament_target_dependencies(
  ${library_name} 
  ${dependencies} 
)

add_executable(world_odom_transform_node src/world_odom_transform_node.cpp)
target_link_libraries(world_odom_transform_node ${library_name} ${GeographicLib_LIBRARIES})
ament_target_dependencies(world_odom_transform_node ${dependencies})

install(TARGETS world_odom_transform_node ${library_name}
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

##launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY 
  config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()