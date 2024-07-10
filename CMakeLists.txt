cmake_minimum_required(VERSION 3.0.2)
project(livox_msg_pointcloud)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  livox_ros_driver
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(livox_msg_pointcloud_node src/livox_msg_pointcloud_node.cpp)


target_link_libraries(livox_msg_pointcloud_node
  ${catkin_LIBRARIES}
)