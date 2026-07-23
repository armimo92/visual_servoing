cmake_minimum_required(VERSION 3.0.2)
project(fxteso_aibvs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_msgs
  mavros_msgs
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
CATKIN_DEPENDS cv_bridge gazebo_msgs geometry_msgs image_transport nav_msgs roscpp sensor_msgs std_msgs tf2 mavros_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_3_INCLUDE_DIRS}
)

#Image publisher
add_executable(video_publication_node src/image_publisher.cpp)
target_link_libraries(video_publication_node  ${catkin_LIBRARIES})
target_link_libraries(video_publication_node  ${OpenCV_LIBRARIES})

#Image Features processing
add_executable(img_feat_node src/image_features.cpp)
target_link_libraries(img_feat_node  ${catkin_LIBRARIES})
target_link_libraries(img_feat_node  ${OpenCV_LIBRARIES})

#IBVS-based FXTESO
add_executable(fixed_eso_node src/fixed_eso.cpp)
target_link_libraries(fixed_eso_node	${catkin_LIBRARIES})

#IBVS-Adaptive controller
add_executable(ibvs_controller_node src/ibvs_pos_ctrl.cpp)
target_link_libraries(ibvs_controller_node	${catkin_LIBRARIES})

#Attitude controller
add_executable(attitude_controller_node src/attitude_nftasmc.cpp)
target_link_libraries(attitude_controller_node	${catkin_LIBRARIES})

#QUAV dynamics
add_executable(quadrotor_dynamics_node src/uav_dynamics.cpp)
target_link_libraries(quadrotor_dynamics_node	${catkin_LIBRARIES})

#Target position
add_executable(target_position_node src/target_position.cpp)
target_link_libraries(target_position_node	${catkin_LIBRARIES})

#External Disturbances
add_executable(external_disturbances_node src/disturbances.cpp)
target_link_libraries(external_disturbances_node	${catkin_LIBRARIES})

#TD_Attitude
add_executable(td_attitude_node src/td_attitude.cpp)
target_link_libraries(td_attitude_node ${catkin_LIBRARIES})

#TD Attitude desired
add_executable(td_attitude_desired_node src/td_attitude_desired.cpp)
target_link_libraries(td_attitude_desired_node ${catkin_LIBRARIES})

#Send data to PX4
#add_executable(send2px4_node src/send2px4.cpp)
#target_link_libraries(send2px4_node ${catkin_LIBRARIES})

