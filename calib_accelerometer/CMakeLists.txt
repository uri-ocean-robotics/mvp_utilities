cmake_minimum_required(VERSION 3.8)
project(calib_accelerometer)

####################################################################
## setup compiler                    
####################################################################

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

####################################################################
## setup packages                    
####################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(Eigen3 REQUIRED)

####################################################################
## build                  
####################################################################

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  std_srvs
  tf2_ros
  geometry_msgs
  builtin_interfaces
  tf2_eigen
)

include_directories(
  ${EIGEN3_INCLUDE_DIR} 
)

set(node_name ${PROJECT_NAME}_node)


# add exe
add_executable(${node_name}_node src/node.cpp)
# target_link_libraries(${node_name} ${library_name} ${Boost_LIBRARIES})
ament_target_dependencies(${node_name}_node ${dependencies})

####################################################################
## install all the files                    
####################################################################

# lib
install(TARGETS 
  ${node_name}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# include
# install(DIRECTORY include/
#   DESTINATION include/
# )

# launch
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# config
# install(DIRECTORY 
#   config 
#   DESTINATION share/${PROJECT_NAME}
# )

####################################################################
## export                    
####################################################################

# ament_export_include_directories(include)
# ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
