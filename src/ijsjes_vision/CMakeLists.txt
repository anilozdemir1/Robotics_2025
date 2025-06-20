cmake_minimum_required(VERSION 3.0.2)
project(ijsjes_vision)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
