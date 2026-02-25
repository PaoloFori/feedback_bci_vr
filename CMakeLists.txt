cmake_minimum_required(VERSION 3.0.2)
project(feedback_bci_vr)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rosneuro_msgs
)


catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs rosneuro_msgs
)
