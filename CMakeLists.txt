cmake_minimum_required(VERSION 2.8.3)
project(jetyak_dispatch)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  ar_track_alvar
  roscpp
  rospy
  std_msgs
  tf
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_filters
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  ar_track_alvar
  roscpp
  std_msgs
  tf
  geometry_msgs
  nav_msgs
  visualization_msgs
  sensor_msgs
  rospy 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller_node src/Controller.cpp src/PIDController.cpp)
add_executable(feature_finder_node src/FeatureFinder.cpp)

target_link_libraries(controller_node ${catkin_LIBRARIES})
target_link_libraries(feature_finder_node ${catkin_LIBRARIES})
