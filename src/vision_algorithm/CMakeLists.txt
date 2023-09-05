cmake_minimum_required(VERSION 3.0.2)
project(vision_algorithm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS cv_bridge gazebo_msgs geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf2
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_3_INCLUDE_DIRS}
)

add_executable(camera_node src/vision_algorithm.cpp)
target_link_libraries(camera_node  ${catkin_LIBRARIES})
target_link_libraries(camera_node  ${OpenCV_LIBRARIES})


