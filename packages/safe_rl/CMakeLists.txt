cmake_minimum_required(VERSION 2.8.3)
project(safe_rl)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  duckietown_msgs
)

catkin_package()

include_directories(
)

# Python nodes don't need building, they're just copied
# This is a placeholder for future C++ implementation
