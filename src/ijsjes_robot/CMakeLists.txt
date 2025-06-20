cmake_minimum_required(VERSION 3.0.2)
project(ijsjes_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  IceTarget.msg
  HmiToRobot.msg
  RobotToHmi.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rospy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
