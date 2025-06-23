cmake_minimum_required(VERSION 3.0.2)
project(cobot_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  urdf
  controller_manager
  joint_state_controller
  robot_state_publisher
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
