cmake_minimum_required(VERSION 3.8)
project(orne_box_lidar_with_mirror)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

# Executable
add_executable(convert_pointcloud_node src/convert_pointcloud_node.cpp src/convert_pointcloud.cpp)

# Link against required libraries
ament_target_dependencies(convert_pointcloud_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  laser_geometry
  Eigen3
)

# Install the executable
install(TARGETS convert_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Ament package macros
ament_package()