cmake_minimum_required(VERSION 3.0.2)
project(fa_hex_ibvs)

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

add_executable(fahex_video_pub_node src/image_publisher.cpp)
target_link_libraries(fahex_video_pub_node  ${catkin_LIBRARIES})
target_link_libraries(fahex_video_pub_node  ${OpenCV_LIBRARIES})

add_executable(fahex_imfeat_node src/image_features.cpp)
target_link_libraries(fahex_imfeat_node  ${catkin_LIBRARIES})
target_link_libraries(fahex_imfeat_node  ${OpenCV_LIBRARIES})

add_executable(fahex_nftasmc_att_node src/attitude_nftasmc.cpp)
target_link_libraries(fahex_nftasmc_att_node ${catkin_LIBRARIES})

add_executable(fahex_ibvs_node src/ibvs_pos_ctrl.cpp)
target_link_libraries(fahex_ibvs_node ${catkin_LIBRARIES})

add_executable(fahex_dynamics_node src/fahex_dynamics.cpp)
target_link_libraries(fahex_dynamics_node ${catkin_LIBRARIES})

add_executable(fahex_tgt_pos_node src/target_position.cpp)
target_link_libraries(fahex_tgt_pos_node ${catkin_LIBRARIES})

add_executable(fahex_dist_node src/disturbances.cpp)
target_link_libraries(fahex_dist_node ${catkin_LIBRARIES})

add_executable(fahex_TD_node src/tracking_differentiator.cpp)
target_link_libraries(fahex_TD_node ${catkin_LIBRARIES})

add_executable(fahex_quad_broadcaster_node src/quad_gazebo_broadcaster.cpp)
target_link_libraries(fahex_quad_broadcaster_node ${catkin_LIBRARIES})

add_executable(fahex_target_broadcaster_node src/target_gazebo_broadcaster.cpp)
target_link_libraries(fahex_target_broadcaster_node ${catkin_LIBRARIES})
